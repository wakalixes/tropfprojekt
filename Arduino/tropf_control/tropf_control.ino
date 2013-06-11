#include <stdio.h>
#include <SPI.h>
#include <Wire.h>
#include "Time.h"
#include "DS3234RTC.h"

#define PINRTCCS     8

#define CMDHIGH "on"
#define CMDLOW  "off"
#define CMDSEP  " "
#define CMDGETTIME    "gettime"
#define CMDSETTIME    "settime"
#define CMDGETCONFIG  "getconfig"
#define CMDCHGSPEED   "changespeed"
#define CMDPRINTTIME  "printtime"
#define CMDPRINTTEXT  "printtext"
#define CMDAUTOPRINT  "autoprint"

String inputString = "";
String commandString = "";
String valueString = "";
boolean stringComplete = false;
boolean commandAck = false;

struct flagsStruct {
  boolean sendconfig;
  boolean sendtime;
  boolean synctime;
} flags;
struct configStruct {
  unsigned int printspeed;
  boolean autoprintenable;
  unsigned int autoprinttime;
  time_t timecode;
} config;

void setup() {
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
}

void initVariables() {
  config.printspeed = 10;
  config.autoprintenable = false;
  config.autoprinttime = 60;
  config.timecode = now();
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
  Serial.print("printspeed: ");
  Serial.println(config.printspeed);
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

void serialSendTime() {
  Serial.print("time: ");
  Serial.println(now());  
}

void syncTimeCode() {
  RTC.set(config.timecode);
  setTime(config.timecode);
  setSyncProvider(RTC.get);
  Serial.println("RTC synced");
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
    else if (inputString.startsWith(CMDCHGSPEED)) {
      valueString = inputString.substring(inputString.indexOf(CMDSEP)+1,inputString.length()-1);
      config.printspeed = valueString.toInt();
      commandAck = true;
    }
    else if (inputString.startsWith(CMDPRINTTIME)) {
       //TODO
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
        commandAck = true;
      }
      if (commandString.startsWith(CMDLOW)) {
        config.autoprintenable = false;
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


