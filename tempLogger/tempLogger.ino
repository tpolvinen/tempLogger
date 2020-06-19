// Just in case. See https://forum.arduino.cc/index.php?topic=158885.0
// BOF preprocessor bug prevent - insert me on top of your arduino-code
//#if 1
//__asm volatile ("nop");
//#endif

// To quickly switch on and off all Serial.prints,
// or choose between prints to serial monitor
// and serial plotter:
//#define SILENT
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

/*
ToDo: initialise here all ADCs, write thermistor class, make array of thermistors,
read a set number of thermistor values as measurement round, calculate statistics,
parse measurement round statistics to a char array, write array to SD card.
 
Adafruit_ADS1115 port_0_ads_0(0x48);
Adafruit_ADS1115 port_0_ads_1(0x49);
Adafruit_ADS1115 port_0_ads_2(0x4A);

Adafruit_ADS1115 port_1_ads_0(0x48);
Adafruit_ADS1115 port_1_ads_1(0x49);
Adafruit_ADS1115 port_1_ads_2(0x4A);
Adafruit_ADS1115 port_1_ads_3(0x4B);

Adafruit_ADS1115 port_2_ads_0(0x48);

*/

const byte TCAADDR = 0x70; // this is the I2C address of the multiplexer (Adafruit TCA9548A)

const int8_t I2CPORT_0 = 0;
const int8_t I2CPORT_1 = 1;
const int8_t I2CPORT_2 = 2;
const int8_t I2CPORT_RTC = 3; 

RTC_DS3231 rtc;

/* Which of these would work better?

const int8_t dateAndTimeCase = 1;
const int8_t measurementFileCase = 2;
const int8_t logFileCase = 3;
const int8_t dirCase = 4;

#define DATEANDTIMECASE 1
#define MEASUREMENTFILECASE 2
#define LOGFILECASE 3
#define DIRCASE 4

*/

//------------------------------------------------------------------------------

// prototype for function for handling missing parameter
char * getDateAndTime(int8_t whichTime = 0);


char * getDateAndTime(int8_t whichTime) {

  tcaselect(I2CPORT_RTC);

  DPRINTLN("begin getDateAndTime()");
	uint16_t thisYear; 
	int8_t thisMonth, thisDay, thisHour, thisMinute, thisSecond;

	DateTime now = rtc.now();
	thisYear = now.year();
  thisMonth = now.month();
  thisDay = now.day();
  thisHour = now.hour();
  thisMinute = now.minute();
  thisSecond = now.second();

	char * timeBuffer = (char *) malloc (50);

	switch (whichTime) {
	case 1: // date and time
		sprintf (timeBuffer, ("%04d-%02d-%02dT%02d:%02d:%02d"), thisYear, thisMonth, thisDay, thisHour, thisMinute, thisSecond);
		break;
	case 2: // measurement file name
		sprintf (timeBuffer, ("%02d-%02d.csv"), thisMonth, thisDay);
		break;
	case 3: // log file name
		sprintf (timeBuffer, ("%02d-%02dlog.csv"), thisMonth, thisDay);
		break;
	case 4: // directory name
		sprintf (timeBuffer, ("%02d-%02d"), thisYear, thisMonth);
		break;
	default:
		sprintf (timeBuffer, ("NA"));
		DPRINTLN ("RTC time switch failed, check whichTime parameter!");
		break;
		
	}

	return timeBuffer;

	// see function returning char array here: https://forum.arduino.cc/index.php?topic=63659.0

  tcaselect(I2CPORT_0);

}

//------------------------------------------------------------------------------

void tcascan() {

  DPRINTLN("\nTCAScanner ready!");

  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    DPRINT("TCA Port #"); DPRINTLN(t);

    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;

      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
        DPRINT("Found I2C 0x");  DPRINTLN(addr, HEX);
      }
    }
  }
  DPRINTLN("\nScanning done...");
}

//------------------------------------------------------------------------------

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

//------------------------------------------------------------------------------

void initializeRTC() {

  tcaselect(I2CPORT_RTC);

  if (! rtc.begin()) {
    DPRINTLN("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    DPRINTLN("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  tcaselect(I2CPORT_0);

}

//------------------------------------------------------------------------------


void setup() {

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

	Wire.begin();

#ifdef SILENT
  Serial.println("The rest is silence.");
#endif

#ifdef PLOTTER
  Serial.println("1.0");
#endif

  DPRINTLN("Debug messages are on!");

  tcascan();
	
	initializeRTC();
	
}

void loop() {
  DPRINTLN("Loop!");

	// testing for RTC functionalities
	#ifdef DEBUG
	char * timeDataPtr1 = getDateAndTime (1);
	Serial.println (timeDataPtr1);
	free (timeDataPtr1);

	char * timeDataPtr2 = getDateAndTime (2);
	Serial.println (timeDataPtr2);
	free (timeDataPtr2);

	char * timeDataPtr3 = getDateAndTime (3);
	Serial.println (timeDataPtr3);
	free (timeDataPtr3);

	char * timeDataPtr4 = getDateAndTime (4);
	Serial.println (timeDataPtr4);
	free (timeDataPtr4);

	char * timeDataPtrDef = getDateAndTime ();
	Serial.println (timeDataPtrDef);
	free (timeDataPtrDef);
  #endif


#ifdef PLOTTER
  Serial.println("1.0");
#endif
  delay(2000);

}


/*
 * ToDo:
 * RTC
 * I2C multiplexer
 * ADCs in multiplexer ports 0, 1, and 2
 * RTC in multiplexer port 3
 * SD card writing
 * Timing for measurement rounds and intervals between rounds, or
 * define how many readings per measurement (easier)
 * Timing for relays (wait a moment after turning thermistors on)
 */
