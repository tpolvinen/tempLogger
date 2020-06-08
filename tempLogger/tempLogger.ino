// Just in case. See https://forum.arduino.cc/index.php?topic=158885.0
// BOF preprocessor bug prevent - insert me on top of your arduino-code
//#if 1
//__asm volatile ("nop");
//#endif

// To quickly switch on and off all Serial.prints,
// or choose between prints to serial monitor
// and serial plotter:
#define SILENT
#ifndef SILENT
#define DEBUG
#ifndef DEBUG
#define PLOTTER
#endif
#endif

// This makes it easy to turn debugging messages
// on or off, by defining DEBUG:
#include <DebugMacros.h>

#include <Wire.h>

// sdFat library, Adafruit fork:
#include <MinimumSerial.h>
#include <BlockDriver.h>
#include <FreeStack.h>
#include <SdFat.h>
#include <sdios.h>
#include <SysCall.h>
#include <SdFatConfig.h>

// statistic calculations with:
#include <Statistic.h>

// Real Time Clock with:
#include <RTClib.h>

// Adafruit ADS1x15 library for ADCs (ADS1115):
#include <Adafruit_ADS1015.h>

// Controllino libraries:
#include <SPI.h>
#include <Controllino.h>

// Watchdog timer:
#include <avr/wdt.h>

// For bus scanning from Wire library:
extern "C" {
#include "utility/twi.h"
}


void setup() {

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

#ifdef SILENT
  Serial.println("The rest is silence.");
#endif

#ifdef PLOTTER
  Serial.println("1.0");
#endif

  DPRINTLN("Debug messages are on!");

}

void loop() {
  DPRINTLN("Loop!");
  delay(2000);

}
