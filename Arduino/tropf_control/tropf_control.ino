#include <stdio.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "Time.h"
#include "DS3234RTC.h"
#include "charsets.h"

#define PINRTCCS      8
#define PINPUMPDATA   2
#define PINPUMPCLK    3
#define PINPUMPLATCH  4
#define PINPUMPENABLE 5

#define PINPUMPTEST   2  // remove

#define CMDHIGH "on"
#define CMDLOW  "off"
#define CMDSEP  " "
#define CMDGETTIME     "gettime"
#define CMDSETTIME     "settime"
#define CMDGETCONFIG   "getconfig"
#define CMDCHGINTERVAL "changeinterval"
#define CMDCHGONTIME   "changeontime"
#define CMDPRINTTIME   "printtime"
#define CMDPRINTTEXT   "printtext"
#define CMDAUTOPRINT   "autoprint"
#define CMDDEBUGPUMP   "debugpump"

#define PUMPDEBUGINTERVAL  1000

#define CHARMAXROWTIME   5
#define CHARMAXROWTEXT   7
#define LOADCONFIGEEPROM 1
#define EEPROMSTARTADDR  0

String inputString = "";
String commandString = "";
String valueString = "";
boolean stringComplete = false;
boolean commandAck = false;

struct flagsStruct {
  boolean sendconfig;
  boolean sendtime;
  boolean synctime;
  boolean printtime;
  boolean printnextrow;
  boolean pumpson;
  boolean pumpsoff;
} flags;

struct configStruct {
  unsigned int printinterval;
  unsigned int pumpontime;
  boolean autoprintenable;
  boolean debugpumpenable;
  unsigned int autoprinttime;
  time_t timecode;
} config;

struct printTimeDataStruct {
  unsigned char hour;
  unsigned char minute;
  unsigned char rowIdx;
  unsigned long switchTime;
} printTimeData;

void setup() {
  pinMode(PINPUMPDATA, OUTPUT);
  pinMode(PINPUMPCLK, OUTPUT);
  pinMode(PINPUMPLATCH, OUTPUT);
  pinMode(PINPUMPENABLE, OUTPUT);
  pinMode(PINPUMPTEST, OUTPUT); // remove
  digitalWrite(PINPUMPENABLE, LOW);
  initVariables();
  Serial.begin(115200);
  inputString.reserve(50);
  commandString.reserve(20);
  valueString.reserve(12);
  initRTC();
}

void loop() {
  serialCommands();
  serialSendFlags();
  printFlags();
}

void initVariables() {
  if (LOADCONFIGEEPROM) {
    readConfigEEPROM();
  } else {
    config.printinterval = 1000;
    config.pumpontime = 300;
    config.debugpumpenable = false;
    config.autoprintenable = false;
    config.autoprinttime = 60;
    config.timecode = now();
  }
}

void initRTC() {  
  RTC.begin(PINRTCCS);
  if (RTC.isRunning()) {
    setSyncProvider(RTC.get);
    if(timeStatus()!= timeSet) 
      Serial.println("Unable to sync with the RTC");
    else
      Serial.println("RTC has set the system time");  
  }  
  else
    Serial.println("ERROR: RTC is NOT connected!"); 
}

void printFlags() {
  if (flags.printtime) {
    if (flags.printnextrow) {
      flags.printnextrow = 0;
      if (printTimeData.rowIdx>=CHARMAXROWTIME) {
        flags.printtime = 0;
        flags.pumpson = 0;
        flags.pumpsoff = 0;
        printTimeData.rowIdx = 0;
        Serial.println("printing done");
      } else {
        int shiftBits = getPrintTimeRow(printTimeData.rowIdx,printTimeData.hour,printTimeData.minute);
        printTimeData.rowIdx++;
        shiftWrite(shiftBits);
        flags.pumpson = 1;
        printTimeData.switchTime = millis();
        serialSendPrintTime(shiftBits);
      }
    }
    if ((flags.pumpson)&&(!flags.pumpsoff)) {
      if ((millis()-printTimeData.switchTime) >= config.pumpontime) {
        shiftWrite(0);
        flags.pumpson = 0;
        flags.pumpsoff = 1;
        printTimeData.switchTime = millis();
      }
    }
    if ((!flags.pumpson)&&(flags.pumpsoff)) {
      if ((millis()-printTimeData.switchTime) >= config.printinterval-config.pumpontime) {
        flags.pumpsoff = 0;
        flags.printnextrow = 1;
      }
    }
  }  
}

void printTime() {
  printTimeData.hour = hour();
  printTimeData.minute = minute();
  printTimeData.rowIdx = 0;
  flags.printtime = 1;
  flags.printnextrow = 1;
  flags.pumpson = 0;
  flags.pumpsoff = 0;
}

int getPrintTimeRow(unsigned char i, unsigned char printhour, unsigned char printminute) {
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

void printChar(char c) {
  for (int i=0;i<=4;i++) {
    char putChar = pgm_read_byte(&(charSet[c-0x20][i]));
    Serial.print(putChar,HEX);
    Serial.print(", ");
  }
  Serial.println();
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

void serialSendFlags() {
  if (flags.sendconfig) {
    serialSendConfig();
    flags.sendconfig = false;
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
  Serial.print("printinterval: ");
  Serial.println(config.printinterval);
  Serial.print("pumpontime: ");
  Serial.println(config.pumpontime);
  Serial.print("debugpump: ");
  if (config.debugpumpenable) Serial.println("on");
  else Serial.println("off");
  Serial.print("autoprint: ");
  if (config.autoprintenable) Serial.print("on");
  else Serial.print("off");
  Serial.print(" ");
  Serial.println(config.autoprinttime);
  Serial.print("timecode: ");
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
  if (hour()<10) Serial.print("0");
  Serial.print(hour());
  Serial.print(":");
  if (minute()<10) Serial.print("0");
  Serial.print(minute());
  Serial.print(":");
  if (second()<10) Serial.print("0");
  Serial.println(second());
}

void serialSendPrintTime(int shiftBits) {
  Serial.print("printing time - ");
  if (printTimeData.hour<10) Serial.print("0");
  Serial.print(printTimeData.hour);
  Serial.print(":");
  if (printTimeData.minute<10) Serial.print("0");
  Serial.print(printTimeData.minute);
  Serial.print(" - Row ");
  Serial.print(printTimeData.rowIdx-1);
  Serial.print(" - ");
  serialPrintShiftBits(shiftBits);
  Serial.println();  
}

void serialSendTime() {
  Serial.print("time: ");
  Serial.println(now());  
}

void serialPrintShiftBits(int shiftBits) {
  for (int i=0;i<16;i++)
  {
    if (shiftBits < pow(2,i))
    Serial.print("0");
  }
  Serial.print(shiftBits,BIN);
}

void serialCommands() {
  if (stringComplete) {
    commandAck = false;
    inputString.toLowerCase();
    if (inputString.startsWith(CMDGETTIME)) {
      flags.sendtime = true;
      commandAck = true;  
    }
    else if (inputString.startsWith(CMDSETTIME)) {
      valueString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      config.timecode = valueString.toInt();
      flags.synctime = true;
      commandAck = true;  
    }
    else if (inputString.startsWith(CMDGETCONFIG)) {
      flags.sendconfig = true;
      commandAck = true;  
    }
    else if (inputString.startsWith(CMDCHGINTERVAL)) {
      valueString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      config.printinterval = valueString.toInt();
      writeConfigEEPROM();
      commandAck = true;
    }
    else if (inputString.startsWith(CMDCHGONTIME)) {
      valueString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      config.pumpontime = valueString.toInt();
      writeConfigEEPROM();
      commandAck = true;  
    }
    else if (inputString.startsWith(CMDPRINTTIME)) {
      printTime();
      commandAck = true;
    }      
    else if (inputString.startsWith(CMDPRINTTEXT)) {
       //TODO
      commandAck = true;
    }
    else if (inputString.startsWith(CMDAUTOPRINT)) {
      commandString = inputString.substring(inputString.indexOf(CMDSEP)+1);
      if (commandString.startsWith(CMDHIGH)) {
        config.autoprintenable = true;
        valueString = commandString.substring(commandString.indexOf(CMDSEP)+1,commandString.length()-1);
        config.autoprinttime = valueString.toInt();
        writeConfigEEPROM();
        commandAck = true;
      }
      if (commandString.startsWith(CMDLOW)) {
        config.autoprintenable = false;
        writeConfigEEPROM();
        commandAck = true;
      }   
    }
    else if (inputString.startsWith(CMDDEBUGPUMP)) {
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
    Serial.print(inputString);
    if (commandAck) Serial.println("ACK");
    else Serial.println("ERR"); 
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
