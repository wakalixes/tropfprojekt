/**
 * TropfControl v0.1
 * 
 * used to control microfluid pumps for printing time/text into a river using 
 * food coloring (project zeitfluss by islandrabe 2013)
 *
 * Features - v0.1:
 *    + controls up to 16 microfluid pumps via two shift registers 74HC595
 *    + full configuration via serial
 *    + saves configuration in EEPROM
 *    + prints time in 7-segment text perpendicular to river
 *    + autoprint functionality
 *    + prints text along river
 *    + mirror text
 *    + realtime Clock DS3234 for reliable time information
 *    
 * TODOs:
 *    + implement individual ontime for each pump
 *    + measure water levels via static pressure gauges, 5 analog inputs
 * 
 * ----------------------------------------------------------------------------------
 * 
 * Copyright (C) 2013 wakalixes (albert.frisch@gmail.com)
 * 
 * ----------------------------------------------------------------------------------
 * 
 * LICENSE:
 * 
 * This file is part of TropfControl.
 * 
 * TropfControl is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * TropfControl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with TropfControl; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 * 
 * ----------------------------------------------------------------------------------
 * 
 * @copyright 2013 wakalixes.
 * @author    Albert Frisch, albert.frisch@gmail.com
 * @license   http://www.gnu.org/licenses/gpl.txt GNU General Public License v2
 **/
 
#include <stdio.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "Time.h"
#include "DS3234RTC.h"
#include "charsets.h"

#define PINRTCCS       8
#define PINPUMPDATA    2
#define PINPUMPCLK     3
#define PINPUMPLATCH   4
#define PINPUMPENABLE  5
#define PINTANKSOFFSET 0  // shifts ADC channels (0-(MAXNUMTANKS-1))+Offset
#define PINBATVOLTAGE  6  // measure battery voltage on ADC pin 6

#define MAXNUMPUMPS   16
#define MAXNUMTANKS   5

#define BATRESHIGH    1000
#define BATRESLOW     1000
#define ANALOGREF     5.0

#define CMDHIGH "on"
#define CMDLOW  "off"
#define CMDSEP  " "
#define CMDGETTIME      "gettime"
#define CMDSETTIME      "settime"
#define CMDGETCONFIG    "getconfig"
#define CMDCHGINTERVAL  "chginterval"
#define CMDCHGONTIME    "chgontime"
#define CMDCHGPUMPFACT  "chgpumpfact"
#define CMDPRINTTIME    "printtime"
#define CMDPRINTTEXT    "printtext"
#define CMDMIRRORTEXT   "mirrortext"
#define CMDAUTOPRINT    "autoprint"
#define CMDDEBUGPUMP    "debugpump"
#define CMDRESETCONFIG  "resetconfig"
#define CMDSTARTOPERAT  "startop"
#define CMDSTOPOPERAT   "stopop"
#define CMDAUTOOPERAT   "autoop"
#define CMDGETSTATUS    "getstatus"
#define CMDGETLEVELS    "t" //"getlevels"
#define CMDGETBATV      "t" //"getbat"

#define PUMPDEBUGINTERVAL  1000

#define RESETOPERATION  1
#define RESETINTERVAL   1000
#define RESETONTIME     300
#define RESETPUMPFACT   100
#define RESETAUTOPRINT  0
#define RESETAUTOTIME   60
#define RESETDEBUG      0
#define RESETMIRRORTEXT 0
#define RESETLEVELMIN   0
#define RESETLEVELMAX   1024

#define CHARMAXROWTIME   5
#define CHARMAXROWTEXT   5
#define CHARTEXTROWSPACE 1
#define LOADCONFIGEEPROM 1
#define EEPROMSTARTADDR  0
#define MINAUTOPRINTTIME 1

#define SENDWEBCAMTRIG   1
#define TIMEZONEOFFSET   2 //TODO: check function

String inputString = "";
String commandString = "";
String valueString = "";
String printTextString = "";
boolean stringComplete = false;
boolean commandAck = false;
unsigned long shiftSwitchTime;

struct flagsStruct {
  boolean sendconfig;
  boolean sendstatus;
  boolean sendlevels;
  boolean sendbatvolt;
  boolean sendtime;
  boolean synctime;
  boolean printtime;
  boolean printnextrow;
  boolean printtext;
  boolean autowaitfullminute;
  boolean pumpson;
  boolean pumpsoff;
} flags;

struct configStruct {
  boolean isoperating;
  unsigned int printinterval;
  unsigned int pumpontime;
  unsigned int pumpOnTimesFact[MAXNUMPUMPS];
  unsigned int levelmin[MAXNUMTANKS];
  unsigned int levelmax[MAXNUMTANKS];
  boolean autoprintenable;
  boolean autooperatenable;
  boolean debugpumpenable;
  unsigned int autoprinttime;
  boolean mirrortext;
  time_t timecode;
} config;

struct statusValStruct {
  unsigned int levels[MAXNUMTANKS];
  unsigned int levelsraw[MAXNUMTANKS];
  unsigned int batvoltage;
  unsigned int batvoltageraw;
} statusVal;

struct printTimeDataStruct {
  unsigned char hour;
  unsigned char minute;
  unsigned char rowIdx;
  unsigned long autoprintlast;
  int autoprintseconds;
} printTimeData;

struct printTextDataStruct {
  unsigned int charNum;
  unsigned int charIdx;
  unsigned int columnIdx;  
} printTextData;

void setup() {
  pinMode(PINPUMPDATA, OUTPUT);
  pinMode(PINPUMPCLK, OUTPUT);
  pinMode(PINPUMPLATCH, OUTPUT);
  pinMode(PINPUMPENABLE, OUTPUT);
  digitalWrite(PINPUMPENABLE, LOW);
  analogReference(DEFAULT);
  initVariables();
  Serial.begin(115200);
  inputString.reserve(25);
  commandString.reserve(20);
  valueString.reserve(12);
  initRTC();
}

void loop() {
  serialCommands();
  serialSendFlags();
  if (config.autooperatenable) checkOperatingHours();
  if (config.isoperating) {
    checkAutoPrint();
    printFlags();
    printTiming();
  }
  else shiftWrite(0);
}

void initVariables() {
  if (LOADCONFIGEEPROM) {
    readConfigEEPROM();
  } else {
    resetVariables();
  }
}

void resetVariables() {
  config.isoperating = RESETOPERATION;
  config.printinterval = RESETINTERVAL;
  config.pumpontime = RESETONTIME;
  for (int i=0;i<MAXNUMPUMPS;i++) config.pumpOnTimesFact[i] = RESETPUMPFACT;
  for (int i=0;i<MAXNUMTANKS;i++) {
    config.levelmin[i] = RESETLEVELMIN;
    config.levelmax[i] = RESETLEVELMAX;
  }
  config.debugpumpenable = RESETDEBUG;
  config.autoprintenable = RESETAUTOPRINT;
  config.autoprinttime = RESETAUTOTIME;
  config.mirrortext = RESETMIRRORTEXT;
  config.timecode = now();
  writeConfigEEPROM();
}

void initRTC() {  
  RTC.begin(PINRTCCS);
  if (RTC.isRunning()) {
    setSyncProvider(RTC.get);
    if (timeStatus()!= timeSet) 
      Serial.println("Unable to sync with the RTC");
    else
      Serial.println("RTC has set the system time");  
  }
  else
    Serial.println("ERROR: RTC is NOT connected!"); 
}

void checkAutoPrint() {
  if (config.autoprintenable) {
    if (flags.autowaitfullminute) {
      if (second()==0) {
        flags.autowaitfullminute = 0;
        printTimeData.autoprintseconds = 0;
      }
    } else {
      if (printTimeData.autoprintseconds<=0) {
        printTimeData.autoprintlast = now();
        printTime();
      }
      printTimeData.autoprintseconds = config.autoprinttime-(now()-printTimeData.autoprintlast);
    }
  }
}

void checkOperatingHours() {
  switch (weekday()) { //TODO: TIMEZONEOFFSET
    case 1:  //sunday
    case 2:  //monday
    case 7:  //saturday
      if ((hour()>=10)&&(hour()<16)) config.isoperating = true;
      else config.isoperating = false;
      break;
    case 3:  //tuesday
    case 4:  //wednesday
    case 5:  //thursday
    case 6:  //friday
      if ( (((hour()==11)&&(minute()>=30))||(hour()>=12) && ((hour()<=12)||((hour()==13)&&(minute()<=30)))) ||
         ((((hour()==16)&&(minute()>=30))||(hour()>=17)) && (hour()<19)) )
          config.isoperating = true;
      else config.isoperating = false;
  }  
}

void printFlags() {
  if (flags.printtime) {
    if (flags.printnextrow) {
      flags.printnextrow = 0;
      unsigned int shiftBits = getPrintTimeRow(printTimeData.rowIdx,printTimeData.hour,printTimeData.minute);
      shiftInitWrite(shiftBits);
      if (printTimeData.rowIdx==0) {
        Serial.println("printing start");
        if (SENDWEBCAMTRIG==1) Serial.println("trigger webcam");  
      }
      serialSendPrintTime(shiftBits);
      if (printTimeData.rowIdx++>=CHARMAXROWTIME-1) {
        flags.printtime = 0;
        printTimeData.rowIdx = 0;
        Serial.println("printing done");
      }
    }
  }
  if (flags.printtext) {
    if (flags.printnextrow) {
      flags.printnextrow = 0;
      unsigned int shiftBits = getPrintTextRow(printTextData.charIdx,printTextData.columnIdx);
      shiftInitWrite(shiftBits);
      serialSendPrintText(shiftBits);
      if (printTextData.columnIdx++>=(CHARMAXROWTEXT+CHARTEXTROWSPACE)-1) {
        printTextData.columnIdx = 0;
        if (printTextData.charIdx++>=printTextData.charNum-1) {
          flags.printtext = 0;
          printTextData.columnIdx = 0;
          printTextData.charIdx = 0;
          Serial.println("printing done");  
        }
      }
    }
  }
}

void printTiming() {
  if ((flags.pumpson)&&(!flags.pumpsoff)) {
    if ((millis()-shiftSwitchTime) >= config.pumpontime) { // TODO: ontime for every pump different, pumpontime*pumpfact/100
      shiftWrite(0);
      flags.pumpson = 0;
      flags.pumpsoff = 1;
      shiftSwitchTime = millis();
    }
  }
  if ((!flags.pumpson)&&(flags.pumpsoff)) {
    if ((millis()-shiftSwitchTime) >= config.printinterval-config.pumpontime) {
      flags.pumpsoff = 0;
      flags.printnextrow = 1;
    }
  }  
}

void printTime() {
  printTimeData.hour = hour(now()+TIMEZONEOFFSET*3600);
  printTimeData.minute = minute(now()+TIMEZONEOFFSET*3600);
  printTimeData.rowIdx = 0;
  flags.printtime = 1;
  flags.printnextrow = 1;
  flags.pumpson = 0;
  flags.pumpsoff = 0;
}

void printText() {
  printTextData.charNum = printTextString.length();
  printTextData.charIdx = 0;
  printTextData.columnIdx = 0;
  flags.printtext = 1;
  flags.printnextrow = 1;
  flags.pumpson = 0;
  flags.pumpsoff = 0;
}

unsigned int getPrintTimeRow(unsigned char i, unsigned char printhour, unsigned char printminute) {
  if (i>(CHARMAXROWTIME-1)) i = CHARMAXROWTIME-1;
  unsigned char hourtenths = printhour/10;
  unsigned char hourones = printhour%10;
  unsigned char minutetenths = printminute/10;
  unsigned char minuteones = printminute%10;
  int shiftBitsHour = (getDigitBits(hourtenths, i)<<3)|(getDigitBits(hourones, i)<<0);
  int shiftBitsMinute = (getDigitBits(minutetenths, i)<<3)|(getDigitBits(minuteones, i)<<0);
  int shiftBitsPoint = 0;
  if ((i==1)||(i==3)) shiftBitsPoint = 1;
  return (shiftBitsHour<<7)|(shiftBitsPoint<<6)|(shiftBitsMinute<<0);
}

unsigned char getDigitBits(unsigned char digit, unsigned char row) {
  return (digitMap[digit][row][0]<<2)|(digitMap[digit][row][1]<<1)|(digitMap[digit][row][2]<<0);
}

unsigned int getPrintTextRow(unsigned int charIdx, unsigned int colIdx) {
  if (colIdx>=CHARMAXROWTEXT) {
    return 0;
  }
  char printChar = printTextString[charIdx];
  return getCharBits(printChar, colIdx);
}

unsigned int getCharBits(unsigned char printChar, unsigned char column) {
  unsigned char dataBits;
  dataBits = pgm_read_byte(&(charSet[printChar-0x20][column]));
  if (config.mirrortext) dataBits = reverseBitOrder(dataBits);
  return (dataBits<<8);
}

unsigned char reverseBitOrder(unsigned char input) {
  unsigned char output = 0;
  int tmp = 0;
  for(output = tmp = 0; tmp < 8; tmp++) {
    output = (output << 1) + (input & 1); 
    input >>= 1; 
  }
  return output;
}

void printChar(char c) {
  for (int i=0;i<=4;i++) {
    char putChar = pgm_read_byte(&(charSet[c-0x20][i]));
    Serial.print(putChar,HEX);
    Serial.print(", ");
  }
  Serial.println();
}

void shiftInitWrite(int shiftOutput) {
  shiftWrite(shiftOutput);
  shiftSwitchTime = millis();
  flags.pumpson = 1;
}

void shiftWrite(int bitsToSend) {
  digitalWrite(PINPUMPLATCH, LOW);
  byte registerHigh = highByte(bitsToSend);
  byte registerLow = lowByte(bitsToSend);
  shiftOut(PINPUMPDATA, PINPUMPCLK, MSBFIRST, registerHigh);
  shiftOut(PINPUMPDATA, PINPUMPCLK, MSBFIRST, registerLow);
  digitalWrite(PINPUMPLATCH, HIGH);
}

void syncTimeCode() {
  RTC.set(config.timecode);
  setTime(config.timecode);
  setSyncProvider(RTC.get);
  Serial.println("RTC synced");
}

void measLevels() {
  for (int i=0;i<MAXNUMTANKS;i++) {
    int levelVal = analogRead(i+PINTANKSOFFSET);
    statusVal.levelsraw[i] = levelVal;
    statusVal.levels[i] = map(levelVal,config.levelmin[i],config.levelmax[i],0,100);
  }
}

void measBatVoltage() {
  int batVal = analogRead(PINBATVOLTAGE);
  statusVal.batvoltageraw = batVal;
  statusVal.batvoltage = (double)batVal*ANALOGREF/1024*(BATRESHIGH+BATRESLOW)/BATRESLOW*1000;
}

void serialSendFlags() {
  if (flags.sendconfig) {
    serialSendConfig();
    flags.sendconfig = false;
  }
  if (flags.sendstatus) {
    serialSendStatus();
    flags.sendstatus = false;
  }
  if (flags.sendlevels) {
    serialSendLevels();
    Serial.println();
    serialSendLevelsRaw();
    Serial.println();
    flags.sendlevels = false;  
  }
  if (flags.sendbatvolt) {
    serialSendBatVolt();
    Serial.println();
    serialSendBatVoltRaw();
    Serial.println();
    flags.sendbatvolt = false;
  }
  if (flags.sendtime) {
    serialSendTime();
    flags.sendtime = false; 
  }
  if (flags.synctime) {
    syncTimeCode();
    flags.synctime = false;
  }  
}

void serialSendConfig() {
  Serial.print("isoperating: ");
  Serial.println(config.isoperating);
  Serial.print("operatinghours: ");
  Serial.println(config.autooperatenable);
  Serial.print("printinterval (ms): ");
  Serial.println(config.printinterval);
  Serial.print("pumpontime (ms): ");
  Serial.println(config.pumpontime);
  Serial.print("pumponfactors (%): ");
  for (int i=0;i<MAXNUMPUMPS;i++) {
    if (i==8) {
      Serial.println();
      Serial.print("                   ");
    }
    Serial.print(i);
    Serial.print(":");
    Serial.print(config.pumpOnTimesFact[i]);
    if (i<MAXNUMPUMPS-1) Serial.print(", ");
  }
  Serial.println();
  Serial.print("tanklevelmin: ");
  for (int i=0;i<MAXNUMTANKS;i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(config.levelmin[i]);
    if (i<MAXNUMTANKS-1) Serial.print(", ");
  }
  Serial.println();
  Serial.print("tanklevelmax: ");
  for (int i=0;i<MAXNUMTANKS;i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(config.levelmax[i]);
    if (i<MAXNUMTANKS-1) Serial.print(", ");
  }
  Serial.println();
  Serial.print("debugpump: ");
  if (config.debugpumpenable) Serial.println("on");
  else Serial.println("off");
  Serial.print("autoprint (s): ");
  if (config.autoprintenable) Serial.print("on");
  else Serial.print("off");
  Serial.print(" ");
  Serial.println(config.autoprinttime);
  Serial.print("mirrortext: ");
  if (config.mirrortext) Serial.print("on");
  else Serial.print("off");
  Serial.println();
  Serial.print("timecode (s): ");
  Serial.println(now());
  Serial.print("date: ");
  Serial.print(year());
  Serial.print(".");
  if (month()<10) Serial.print("0");
  Serial.print(month());
  Serial.print(".");
  if (day()<10) Serial.print("0");
  Serial.print(day());
  Serial.print(" ");
  serialSendTimeString();
  Serial.println();
  Serial.println();
}

void serialSendStatus() {
  serialSendLevels();
  Serial.println();
  serialSendLevelsRaw();
  Serial.println();
  serialSendBatVolt();
  Serial.println();
  serialSendBatVoltRaw();
  Serial.println();
}

void serialSendBatVolt() {
  Serial.print("batteryvoltage (mV): ");
  Serial.print(statusVal.batvoltage);
}

void serialSendBatVoltRaw() {
  Serial.print("batteryvoltageraw: ");
  Serial.print(statusVal.batvoltageraw);  
}

void serialSendLevels() {
  Serial.print("tanklevels: ");
  for (int i=0;i<MAXNUMTANKS;i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(statusVal.levels[i]);
    Serial.print("%");
    if (i<MAXNUMTANKS-1) Serial.print(", ");
  }
}

void serialSendLevelsRaw() {
  Serial.print("tanklevelsraw: ");
  for (int i=0;i<MAXNUMTANKS;i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(statusVal.levelsraw[i]);
    if (i<MAXNUMTANKS-1) Serial.print(", ");
  }
}

void serialSendTimeString() {
  if (hour()<10) Serial.print("0");
  Serial.print(hour());
  Serial.print(":");
  if (minute()<10) Serial.print("0");
  Serial.print(minute());
  Serial.print(":");
  if (second()<10) Serial.print("0");
  Serial.print(second());
}

void serialSendPrintTime(unsigned int shiftBits) {
  Serial.print("printing time ");
  if (printTimeData.hour<10) Serial.print("0");
  Serial.print(printTimeData.hour);
  Serial.print(":");
  if (printTimeData.minute<10) Serial.print("0");
  Serial.print(printTimeData.minute);
  Serial.print(" - ");
  serialSendTimeString();
  Serial.print(" - row ");
  Serial.print(printTimeData.rowIdx);
  Serial.print(" - ");
  serialSendShiftBits(shiftBits);
  Serial.println();  
}

void serialSendPrintText(unsigned int shiftBits) {
  Serial.print("printing text - ");
  serialSendTimeString();
  Serial.print(" - char ");
  Serial.print(printTextData.charIdx);
  Serial.print(", ");
  Serial.print(printTextString[printTextData.charIdx]);
  Serial.print(" - column ");
  Serial.print(printTextData.columnIdx);
  Serial.print(" - ");
  serialSendShiftBits(shiftBits);
  Serial.println();
}

void serialSendTime() {
  Serial.print("time: ");
  Serial.println(now());  
}

void serialSendShiftBits(unsigned int shiftBits) {
  for (int i=15;i>0;i--) {
    if (shiftBits<pow(2,i)) Serial.print("0");
  }
  Serial.print(shiftBits,BIN);
}

void serialCommands() {
  if (stringComplete) {
    commandAck = false;
    Serial.println();
    if (inputString.startsWith(CMDSTARTOPERAT)) {
      config.isoperating = true;
      writeConfigEEPROM();
      commandAck = true;
    }
    if (inputString.startsWith(CMDSTOPOPERAT)) {
      config.isoperating = false;
      writeConfigEEPROM();
      commandAck = true;
    }
    if (inputString.startsWith(CMDAUTOOPERAT)) {
      commandString = inputString.substring(inputString.indexOf(CMDSEP)+1);
      if (commandString.startsWith(CMDHIGH)) {
        config.autooperatenable = true;
        writeConfigEEPROM();
        commandAck = true;
      }
      if (commandString.startsWith(CMDLOW)) {
        config.autooperatenable = false;
        writeConfigEEPROM();
        commandAck = true;
      }   
    }
    if (inputString.startsWith(CMDGETTIME)) {
      flags.sendtime = true;
      commandAck = true;  
    }
    if (inputString.startsWith(CMDSETTIME)) {
      valueString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      config.timecode = valueString.toInt();
      flags.synctime = true;
      commandAck = true;  
    }
    if (inputString.startsWith(CMDGETCONFIG)) {
      flags.sendconfig = true;
      commandAck = true;  
    }
    if (inputString.startsWith(CMDGETSTATUS)) {
      flags.sendstatus = true;
      commandAck = true;  
    }
    if (inputString.startsWith(CMDGETLEVELS)) {
      flags.sendlevels = true;
      commandAck = true; 
    }
    if (inputString.startsWith(CMDGETBATV)) {
      flags.sendbatvolt = true;
      commandAck = true; 
    }
    if (inputString.startsWith(CMDCHGINTERVAL)) {
      valueString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      int newInterval = valueString.toInt();
      if (newInterval<config.pumpontime) newInterval = config.pumpontime;
      config.printinterval = newInterval;
      writeConfigEEPROM();
      commandAck = true;
    }
    if (inputString.startsWith(CMDCHGONTIME)) {
      valueString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      config.pumpontime = valueString.toInt();
      writeConfigEEPROM();
      commandAck = true;  
    }
    if (inputString.startsWith(CMDCHGPUMPFACT)) {  //TODO doesnt work
      valueString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      valueString = valueString.substring(0,valueString.indexOf(CMDSEP));
      int pumpNum = valueString.toInt();
      if (pumpNum>MAXNUMPUMPS-1) pumpNum = MAXNUMPUMPS-1;
      valueString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      valueString = valueString.substring(valueString.indexOf(CMDSEP)+1,valueString.length());
      config.pumpOnTimesFact[pumpNum] = valueString.toInt();
      writeConfigEEPROM();
      commandAck = true;
    }
    if (inputString.startsWith(CMDPRINTTIME)) {
      printTime();
      commandAck = true;
    }      
    if (inputString.startsWith(CMDPRINTTEXT)) {
      printTextString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      printText();
      commandAck = true;
    }
    if (inputString.startsWith(CMDAUTOPRINT)) {
      commandString = inputString.substring(inputString.indexOf(CMDSEP)+1);
      if (commandString.startsWith(CMDHIGH)) {
        config.autoprintenable = true;
        valueString = commandString.substring(commandString.indexOf(CMDSEP)+1,commandString.length()-1);
        int printTime = valueString.toInt();
        if (printTime==0) printTime = RESETAUTOTIME;
        else if (printTime<=MINAUTOPRINTTIME) printTime = MINAUTOPRINTTIME;
        config.autoprinttime = printTime;
        flags.autowaitfullminute = true;
        writeConfigEEPROM();
        commandAck = true;
      }
      if (commandString.startsWith(CMDLOW)) {
        config.autoprintenable = false;
        writeConfigEEPROM();
        commandAck = true;
      }   
    }
    if (inputString.startsWith(CMDDEBUGPUMP)) {
      commandString = inputString.substring(inputString.indexOf(CMDSEP)+1);
      if (commandString.startsWith(CMDHIGH)) {
        config.debugpumpenable = true;
        writeConfigEEPROM();
        commandAck = true;
      }
      if (commandString.startsWith(CMDLOW)) {
        config.debugpumpenable = false;
        writeConfigEEPROM();
        commandAck = true;  
      }
    }
    if (inputString.startsWith(CMDRESETCONFIG)) {
      resetVariables();
      commandAck = true;  
    }
    if (inputString.startsWith(CMDMIRRORTEXT)) {
      commandString = inputString.substring(inputString.indexOf(CMDSEP)+1);
      if (commandString.startsWith(CMDHIGH)) {
        config.mirrortext = true;
        writeConfigEEPROM();
        commandAck = true;
      }
      if (commandString.startsWith(CMDLOW)) {
        config.mirrortext = false;
        writeConfigEEPROM();
        commandAck = true;
      }
    }
    Serial.print(inputString);
    if (commandAck) Serial.println("ACK");
    else Serial.println("ERR");
    Serial.println();
    inputString = "";
    stringComplete = false;
  }  
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); 
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}

void readConfigEEPROM() {
  eeprom_read_block((void*)&config, (void*)EEPROMSTARTADDR, sizeof(config));  
}

void writeConfigEEPROM() {
  eeprom_write_block((const void*)&config, (void*)EEPROMSTARTADDR, sizeof(config));
}
