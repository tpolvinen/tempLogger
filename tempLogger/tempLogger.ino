// Just in case. See https://forum.arduino.cc/index.php?topic=158885.0
// BOF preprocessor bug prevent - insert me on top of your arduino-code
#if 1
__asm volatile ("nop");
#endif

// To quickly switch on and off all Serial.prints,
// or choose between prints to serial monitor and serial plotter:

//#define SILENT
#ifndef SILENT
#define DEBUG
#ifndef DEBUG
#define PLOTTER
#endif
#endif

// To calibrate sensors: getting a measurement, comparing results (raw values)
// and giving each sensor (in this iteration all thermistors) a simple value
// that is added or subtracted from each reading value before calculations.
// In this iteration, just printing it out to serial.
#define CALIBRATION

// This makes it easy to turn debugging messages
// on or off, by defining DEBUG above:
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

Adafruit_ADS1115 port_0_adc_0(0x48);
Adafruit_ADS1115 port_0_adc_1(0x49);
Adafruit_ADS1115 port_0_adc_2(0x4A);

Adafruit_ADS1115 port_1_adc_0(0x48);
Adafruit_ADS1115 port_1_adc_1(0x49);
Adafruit_ADS1115 port_1_adc_2(0x4A);
Adafruit_ADS1115 port_1_adc_3(0x4B);

Adafruit_ADS1115 port_2_adc_0(0x48);

#define TCAADDR 0x70 // this is the I2C address of the multiplexer (Adafruit TCA9548A)

#define I2CPORT_0 0
#define I2CPORT_1 1
#define I2CPORT_2 2
#define I2CPORT_RTC 3

RTC_DS3231 rtc;

// definitions for relay pins
#define port0adc0Relay CONTROLLINO_R6
#define port0adc1Relay CONTROLLINO_R7
#define port0adc2Relay CONTROLLINO_R8
#define port1adc0Relay CONTROLLINO_R9
#define port1adc1Relay CONTROLLINO_R10
#define port1adc2Relay CONTROLLINO_R11
#define port1adc3Relay CONTROLLINO_R12
#define port2adc0Relay CONTROLLINO_R13

#define SDERRORLED CONTROLLINO_D23

uint16_t CHECK_SECONDS_INTERVAL_MS = 1000;
uint32_t PREVIOUS_SECOND_CHECK_MS;
uint32_t CURRENT_SECONDSTIME;
uint32_t PREVIOUS_MEASUREMENT_SECONDSTIME;
uint32_t NEXT_MEASUREMENT_SECONDSTIME;
uint32_t EVERY_X_SECONDS = 30;

uint16_t READINGS_PER_MEASUREMENT = 10;

const uint8_t SD1_CS = 7; //53;  // chip select for sd1

SdFat sd1;
SdFile sdMeasurementFile1;
SdFile sdLogFile1;

bool headerLine = false;

char sdMeasurementFileName[10]; // space for MM-DD.csv, plus the null char terminator
char sdLogFileName[13]; // space for MM-DDlog.csv, plus the null char terminator

char sdMeasurementDirName[10]; // space for /YY-MM, plus the null char terminator
char sdLogDirName[4] = {"log"};

char dateAndTimeData[20]; // space for YYYY-MM-DDTHH-MM-SS, plus the null char terminator
char sdDataLine[400]; // Could be smaller, but we've got memory. For now...well not any more we don't.

char delimiter[2] = {','};

int16_t thermistorArraySize;// = sizeof(thermistorArray) / sizeof(thermistorArray[0]);

/*------------------------------------------------------------------------------
  SD card errors:
  print error msg, any SD error codes, and halt. Stores messages in flash
  ------------------------------------------------------------------------------
*/
#define errorExit(msg) errorHalt(F(msg))
#define initError(msg) initErrorHalt(F(msg))

//------------------------------------------------------------------------------

/*------------------------------------------------------------------------------
  tcaSelect
  Opens communication to I2C address given as parameter.
  ------------------------------------------------------------------------------
*/
void tcaSelect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

//------------------------------------------------------------------------------

/*------------------------------------------------------------------------------
  tcaScan
  Scans over all 8 ports of the I2C multiplexer,
  and over all 128 I2C addresses,
  and prints responding addresses to serial.
  ------------------------------------------------------------------------------
*/
void tcaScan() {

  DPRINTLN("\nTCAScanner ready!");

  for (uint8_t t = 0; t < 8; t++) {
    tcaSelect(t);
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

/*------------------------------------------------------------------------------
  initializeRTC
  Initializes real time clock instance,
  resets the time if RTC has lost power (battery dead).
  ------------------------------------------------------------------------------
*/
void initializeRTC() {

  tcaSelect(I2CPORT_RTC);

  if (! rtc.begin()) {
    DPRINTLN("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    DPRINTLN("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

}

//------------------------------------------------------------------------------

/*==============================================================================
   Thermistor class
  ==============================================================================
*/
class Thermistor {
    int8_t i2cPort;
    Adafruit_ADS1115 adc;
    int8_t adcChannel;
    int8_t adcRelay;
    int16_t calibrationValue;

    //------------------------------------------------------------------------------

  public:
    Thermistor (int8_t port, Adafruit_ADS1115 &ads1115, int8_t channel, int8_t relay, int16_t calibration) {
      i2cPort = port;
      adc = ads1115;
      adcChannel = channel;
      adcRelay = relay;
      calibrationValue = calibration;
    }
    char * getMeasurementBuffer = (char*) malloc (30);
    int16_t numberOfReadings;

    //------------------------------------------------------------------------------

    bool getSingleReading() {

      tcaSelect(i2cPort);

      if ( int16_t reading = adc.readADC_SingleEnded(adcChannel) ) {

	reading += calibrationValue;

        numberOfReadings++;
        measurementStat.add(reading);

#ifdef PLOTTER
        Serial.print(readingTemperature);
#endif

        return true;

      } else {

        DPRINTLN("Could not get reading!");
        return false;

      }
    }

    //------------------------------------------------------------------------------

    char * getMeasurement() {

      //DPRINT("Begin getMeasurement...");

      float measurementAverage;
      char measurementValueBuffer[20];
      char integerBuffer[10];
      char noValueBuffer[13] = {"N/A,N/A,N/A,"};

      if (numberOfReadings >= 1) { // this makes sure that no 0 values in place of missing data are sent back...

        measurementAverage =  measurementStat.average();
        measurementTemperature = steinhartCalculation(measurementAverage);
        measurementPopStDev =  measurementStat.pop_stdev();

        //DPRINT("Begin writing getMeasurementBuffer...");

        dtostrf(measurementTemperature, 5, 2, measurementValueBuffer);
        strcpy(getMeasurementBuffer, measurementValueBuffer);
        strcat(getMeasurementBuffer, delimiter);

        dtostrf(measurementPopStDev, 5, 2, measurementValueBuffer);
        strcat(getMeasurementBuffer, measurementValueBuffer);
        strcat(getMeasurementBuffer, delimiter);

        itoa(numberOfReadings, integerBuffer, 10);
        strcat(getMeasurementBuffer, integerBuffer);
        strcat(getMeasurementBuffer, delimiter);

        measurementStat.clear();

        numberOfReadings = 0;

      } else { // ...but instead a marker saying "no readings were done, no data"
        strcpy(getMeasurementBuffer, noValueBuffer);
      }

      //DPRINT("Returning getMeasurementBuffer of: ");
      //DPRINTLN(getMeasurementBuffer);

      return getMeasurementBuffer;

    }

  private:

    float measurementTemperature;
    float measurementPopStDev;
    //float readingTemperature;
    int16_t reading;

    Statistic measurementStat;

    //------------------------------------------------------------------------------

    float steinhartCalculation(float reading) {

      float steinhart;
      float adcRange = 25890.0; //25970.8; //manually calibrated to 10 kOhm resistors = 25 oC (32767 / 6.144v * 5v = 2665.85)
      int16_t thermistorNominal = 10000; // resistance at 25 degrees C
      int8_t temperatureNominal = 25; // temperature per nominal resistance (almost always 25 deg.C)
      int16_t betaCoefficient = 3976; // beta coefficient of thermistor (usually 3000 - 4000)
      int16_t seriesResistor = 10000; // resistance of the 'other' resistor in voltage divider

      // convert the value to resistance
      reading = adcRange / reading - 1; //26665.85 / reading - 1; //adcRange / reading - 1;
      reading = 10000 / reading; // to get 10kOhm, needs 10000÷(26665.85÷13333−1)
      steinhart = reading; // ~6559 not 10000 !!?? --> range 32767 / 6.144v * 5v = 26665.85

      steinhart = reading / thermistorNominal;     // (R/Ro)
      steinhart = log(steinhart);                  // ln(R/Ro)
      steinhart /= betaCoefficient;                   // 1/B * ln(R/Ro)
      steinhart += 1.0 / (temperatureNominal + 273.15); // + (1/To)
      steinhart = 1.0 / steinhart;                 // Invert
      steinhart -= 273.15;                         // convert to C

      return steinhart;

    }
};

//==============================================================================



//------------------------------------------------------I2CPORT_0-------
Thermistor thermistor_0_0_1(I2CPORT_0, port_0_adc_0, 1, port0adc0Relay, 245);
Thermistor thermistor_0_0_3(I2CPORT_0, port_0_adc_0, 3, port0adc0Relay, 245);

Thermistor thermistor_0_1_1(I2CPORT_0, port_0_adc_1, 1, port0adc1Relay, 245);
Thermistor thermistor_0_1_3(I2CPORT_0, port_0_adc_1, 3, port0adc1Relay, 245);

Thermistor thermistor_0_2_1(I2CPORT_0, port_0_adc_2, 1, port0adc2Relay, 245);
Thermistor thermistor_0_2_3(I2CPORT_0, port_0_adc_2, 3, port0adc2Relay, 245);
//------------------------------------------------------I2CPORT_1-------
Thermistor thermistor_1_0_0(I2CPORT_1, port_1_adc_0, 0, port1adc0Relay, 245);
Thermistor thermistor_1_0_1(I2CPORT_1, port_1_adc_0, 1, port1adc0Relay, 245);
Thermistor thermistor_1_0_2(I2CPORT_1, port_1_adc_0, 2, port1adc0Relay, 245);
Thermistor thermistor_1_0_3(I2CPORT_1, port_1_adc_0, 3, port1adc0Relay, 245);

Thermistor thermistor_1_1_0(I2CPORT_1, port_1_adc_1, 0, port1adc1Relay, 245);
Thermistor thermistor_1_1_1(I2CPORT_1, port_1_adc_1, 1, port1adc1Relay, 245);
Thermistor thermistor_1_1_2(I2CPORT_1, port_1_adc_1, 2, port1adc1Relay, 245);
Thermistor thermistor_1_1_3(I2CPORT_1, port_1_adc_1, 3, port1adc1Relay, 245);

Thermistor thermistor_1_2_0(I2CPORT_1, port_1_adc_2, 0, port1adc2Relay, 245);
Thermistor thermistor_1_2_1(I2CPORT_1, port_1_adc_2, 1, port1adc2Relay, 245);
Thermistor thermistor_1_2_2(I2CPORT_1, port_1_adc_2, 2, port1adc2Relay, 245);
Thermistor thermistor_1_2_3(I2CPORT_1, port_1_adc_2, 3, port1adc2Relay, 245);

Thermistor thermistor_1_3_0(I2CPORT_1, port_1_adc_3, 0, port1adc3Relay, 245);
Thermistor thermistor_1_3_1(I2CPORT_1, port_1_adc_3, 1, port1adc3Relay, 245);
Thermistor thermistor_1_3_2(I2CPORT_1, port_1_adc_3, 2, port1adc3Relay, 245);
Thermistor thermistor_1_3_3(I2CPORT_1, port_1_adc_3, 3, port1adc3Relay, 245);
//------------------------------------------------------I2CPORT_2-------
Thermistor thermistor_2_0_0(I2CPORT_2, port_2_adc_0, 0, port2adc0Relay, 245);
Thermistor thermistor_2_0_1(I2CPORT_2, port_2_adc_0, 1, port2adc0Relay, 245);
Thermistor thermistor_2_0_2(I2CPORT_2, port_2_adc_0, 2, port2adc0Relay, 245);
Thermistor thermistor_2_0_3(I2CPORT_2, port_2_adc_0, 3, port2adc0Relay, 245);
//------------------------------------------------------I2CPORT_RTC----
// Only RTC in I2CPORT_RTC!

/*

//------------------------------------------------------I2CPORT_0-------
Thermistor thermistor_0_0_1(I2CPORT_0, port_0_adc_0, 1, port0adc0Relay);
Thermistor thermistor_0_0_3(I2CPORT_0, port_0_adc_0, 3, port0adc0Relay);

Thermistor thermistor_0_1_1(I2CPORT_0, port_0_adc_1, 1, port0adc1Relay);
Thermistor thermistor_0_1_3(I2CPORT_0, port_0_adc_1, 3, port0adc1Relay);

Thermistor thermistor_0_2_1(I2CPORT_0, port_0_adc_2, 1, port0adc2Relay);
Thermistor thermistor_0_2_3(I2CPORT_0, port_0_adc_2, 3, port0adc2Relay);
//------------------------------------------------------I2CPORT_1-------
Thermistor thermistor_1_0_0(I2CPORT_1, port_1_adc_0, 0, port1adc0Relay);
Thermistor thermistor_1_0_1(I2CPORT_1, port_1_adc_0, 1, port1adc0Relay);
Thermistor thermistor_1_0_2(I2CPORT_1, port_1_adc_0, 2, port1adc0Relay);
Thermistor thermistor_1_0_3(I2CPORT_1, port_1_adc_0, 3, port1adc0Relay);

Thermistor thermistor_1_1_0(I2CPORT_1, port_1_adc_1, 0, port1adc1Relay);
Thermistor thermistor_1_1_1(I2CPORT_1, port_1_adc_1, 1, port1adc1Relay);
Thermistor thermistor_1_1_2(I2CPORT_1, port_1_adc_1, 2, port1adc1Relay);
Thermistor thermistor_1_1_3(I2CPORT_1, port_1_adc_1, 3, port1adc1Relay);

Thermistor thermistor_1_2_0(I2CPORT_1, port_1_adc_2, 0, port1adc2Relay);
Thermistor thermistor_1_2_1(I2CPORT_1, port_1_adc_2, 1, port1adc2Relay);
Thermistor thermistor_1_2_2(I2CPORT_1, port_1_adc_2, 2, port1adc2Relay);
Thermistor thermistor_1_2_3(I2CPORT_1, port_1_adc_2, 3, port1adc2Relay);

Thermistor thermistor_1_3_0(I2CPORT_1, port_1_adc_3, 0, port1adc3Relay);
Thermistor thermistor_1_3_1(I2CPORT_1, port_1_adc_3, 1, port1adc3Relay);
Thermistor thermistor_1_3_2(I2CPORT_1, port_1_adc_3, 2, port1adc3Relay);
Thermistor thermistor_1_3_3(I2CPORT_1, port_1_adc_3, 3, port1adc3Relay);
//------------------------------------------------------I2CPORT_2-------
Thermistor thermistor_2_0_0(I2CPORT_2, port_2_adc_0, 0, port2adc0Relay);
Thermistor thermistor_2_0_1(I2CPORT_2, port_2_adc_0, 1, port2adc0Relay);
Thermistor thermistor_2_0_2(I2CPORT_2, port_2_adc_0, 2, port2adc0Relay);
Thermistor thermistor_2_0_3(I2CPORT_2, port_2_adc_0, 3, port2adc0Relay);
//------------------------------------------------------I2CPORT_RTC----
// Only RTC in I2CPORT_RTC!

*/

Thermistor thermistorArray[] = {
  thermistor_0_0_1,
  thermistor_0_0_3,

  thermistor_0_1_1,
  thermistor_0_1_3,

  thermistor_0_2_1,
  thermistor_0_2_3,

  thermistor_1_0_0,
  thermistor_1_0_1,
  thermistor_1_0_2,
  thermistor_1_0_3,

  thermistor_1_1_0,
  thermistor_1_1_1,
  thermistor_1_1_2,
  thermistor_1_1_3,

  thermistor_1_2_0,
  thermistor_1_2_1,
  thermistor_1_2_2,
  thermistor_1_2_3,

  thermistor_1_3_0,
  thermistor_1_3_1,
  thermistor_1_3_2,
  thermistor_1_3_3,

  thermistor_2_0_0,
  thermistor_2_0_1,
  thermistor_2_0_2,
  thermistor_2_0_3

};

/*------------------------------------------------------------------------------
  getDateAndTime
  Gets RTC time and date and puts them to char arrays,
  for file and folder naming, and
  for concatenation with sensor data.
  ------------------------------------------------------------------------------
*/
void getDateAndTime() {

  tcaSelect(I2CPORT_RTC);

  uint16_t thisYear;
  int8_t thisMonth, thisDay, thisHour, thisMinute, thisSecond;

  DateTime now = rtc.now();
  thisYear = now.year();
  thisMonth = now.month();
  thisDay = now.day();
  thisHour = now.hour();
  thisMinute = now.minute();
  thisSecond = now.second();

  sprintf(dateAndTimeData, ("%04d-%02d-%02dT%02d:%02d:%02d"), thisYear, thisMonth, thisDay, thisHour, thisMinute, thisSecond);
  sprintf(sdMeasurementFileName, ("%02d-%02d.csv"), thisMonth, thisDay);
  sprintf(sdLogFileName, ("%02d-%02dlog.csv"), thisMonth, thisDay);
  sprintf(sdMeasurementDirName, ("/%02d-%02d"), thisYear, thisMonth);
  CURRENT_SECONDSTIME = now.secondstime();
  
}

//------------------------------------------------------------------------------

/*------------------------------------------------------------------------------
  sdWrite
  Writes a data char array to SD card.
  Parameters: SD module's chip select, SdFat instance, directory name pointer,
  SdFile instance, filename pointer, data-to-write-pointer,
  boolean header (writing header line or not)
  ------------------------------------------------------------------------------
*/
void sdWrite(int8_t chipSelect, SdFat sd, char* dirName, SdFile sdFile, char* fileName, char* data, bool header) {

  //DPRINT("Begin sdWrite()...");

  if (!sd.begin(chipSelect)) {
    errorBlink(SDERRORLED);
    sd.errorExit("sd.begin(chipSelect)");
  }

  //DPRINT(" DIR: "); DPRINT(dirName);
  if (!sd.exists(dirName)) {
    if (!sd.mkdir(dirName)) {
      errorBlink(SDERRORLED);
      sd.errorExit("sd.mkdir(dirName)");
    }
  }

  // choose directory
  if (!sd.chdir(dirName)) {
    errorBlink(SDERRORLED);
    sd.errorExit("sd.chdir(dirName)");
  }

  //open file within Folder
  //DPRINT(" FILE: "); DPRINT(fileName);
  if (header) {
    if (!sdFile.open(fileName, O_RDWR | O_CREAT)) {
      errorBlink(SDERRORLED);
      sd.errorExit("sdFile.open");
    }
  } else {
    if (!sdFile.open(fileName, O_RDWR | O_CREAT | O_AT_END)) {
      errorBlink(SDERRORLED);
      sd.errorExit("sdFile.open");
    }
  }

  DPRINT(" DATA: "); DPRINTLN(data);
  if (! (sdFile.println(data)) ) {
    errorBlink(SDERRORLED);
    sd.errorExit("println(data)");
  }

  sdFile.close();

  //DPRINTLN(" ...end sdWrite().");
}

//------------------------------------------------------------------------------

/*------------------------------------------------------------------------------
  errorBlink
  Blinks D23 led 10 times if SD card fails.
  ------------------------------------------------------------------------------
*/
void errorBlink(byte led) {
  int8_t blinkCount = 0;
  while (blinkCount < 10) {
    wdt_reset();
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(500);
    blinkCount++;
  }
}

//------------------------------------------------------------------------------

void setup() {

  Wire.begin();

  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }

  pinMode(SDERRORLED, OUTPUT);

#ifdef SILENT
  Serial.println("The rest is silence.");
#endif

  DPRINTLN("Debug messages are on!");

  port_0_adc_0.setGain(GAIN_TWOTHIRDS);
  port_0_adc_1.setGain(GAIN_TWOTHIRDS);
  port_0_adc_2.setGain(GAIN_TWOTHIRDS);

  port_1_adc_0.setGain(GAIN_TWOTHIRDS);
  port_1_adc_1.setGain(GAIN_TWOTHIRDS);
  port_1_adc_2.setGain(GAIN_TWOTHIRDS);

  port_2_adc_0.setGain(GAIN_TWOTHIRDS);

  port_0_adc_0.begin();
  port_0_adc_1.begin();
  port_0_adc_2.begin();

  port_1_adc_0.begin();
  port_1_adc_1.begin();
  port_1_adc_2.begin();
  port_1_adc_3.begin();

  port_2_adc_0.begin();

  tcaScan();

  initializeRTC();

  pinMode(port0adc0Relay, OUTPUT);
  pinMode(port0adc1Relay, OUTPUT);
  pinMode(port0adc2Relay, OUTPUT);
  pinMode(port1adc0Relay, OUTPUT);
  pinMode(port1adc1Relay, OUTPUT);
  pinMode(port1adc2Relay, OUTPUT);
  pinMode(port1adc3Relay, OUTPUT);
  pinMode(port2adc0Relay, OUTPUT);

  digitalWrite(port0adc0Relay, HIGH);
  digitalWrite(port0adc1Relay, HIGH);
  digitalWrite(port0adc2Relay, HIGH);
  digitalWrite(port1adc0Relay, HIGH);
  digitalWrite(port1adc1Relay, HIGH);
  digitalWrite(port1adc2Relay, HIGH);
  digitalWrite(port1adc3Relay, HIGH);
  digitalWrite(port2adc0Relay, HIGH);

  delay(100); // delay to allow stabilization after relay connection

  getDateAndTime();

  thermistorArraySize = sizeof(thermistorArray) / sizeof(thermistorArray[0]);

  //----------------------------------------
  // writing header line to measurement file
  //----------------------------------------
  sprintf(sdDataLine, "Date and time,");
  for (int8_t i = 0; i < thermistorArraySize; i++) {
    strcat(sdDataLine, "Temp,StDev,R_per_M");
    if (i < (thermistorArraySize - 1)) {
      strcat(sdDataLine, delimiter);
    }
  }
  headerLine = true;
  sdWrite(SD1_CS, sd1, sdMeasurementDirName, sdMeasurementFile1, sdMeasurementFileName, sdDataLine, headerLine);

  //----------------------------------------
  // writing system start line to log file
  //----------------------------------------
  char howMany[20];
  strcpy(sdDataLine, dateAndTimeData);
  strcat(sdDataLine, delimiter);
  strcat(sdDataLine, "Hello world. I start now. Number of thermistors in program: ,");
  sprintf(howMany, "%d", thermistorArraySize);
  strcat(sdDataLine, howMany);
  strcat(sdDataLine, delimiter);
  strcat(sdDataLine, " Every x seconds: ,");
  sprintf(howMany, "%d", EVERY_X_SECONDS);
  strcat(sdDataLine, howMany);
  strcat(sdDataLine, delimiter);
  strcat(sdDataLine, "  Readings per measurement: ,");
  sprintf(howMany, "%d", READINGS_PER_MEASUREMENT);
  strcat(sdDataLine, howMany);
  strcat(sdDataLine, delimiter);  
  headerLine = false;
  sdWrite(SD1_CS, sd1, sdLogDirName, sdLogFile1, sdLogFileName, sdDataLine, headerLine);

  // This is to "prime the pump" and make sure first measurements are done
  // immediately at the first loop():
  PREVIOUS_SECOND_CHECK_MS = millis() - CHECK_SECONDS_INTERVAL_MS;
  PREVIOUS_MEASUREMENT_SECONDSTIME = CURRENT_SECONDSTIME - EVERY_X_SECONDS;
  NEXT_MEASUREMENT_SECONDSTIME = CURRENT_SECONDSTIME;

}

//------------------------------------------------------------------------------

void loop() {
  
  if (millis() - PREVIOUS_SECOND_CHECK_MS >= CHECK_SECONDS_INTERVAL_MS) {
    PREVIOUS_SECOND_CHECK_MS = millis();
    getDateAndTime();
  }

  if (CURRENT_SECONDSTIME >= NEXT_MEASUREMENT_SECONDSTIME
      && PREVIOUS_MEASUREMENT_SECONDSTIME != CURRENT_SECONDSTIME) {

    PREVIOUS_MEASUREMENT_SECONDSTIME = CURRENT_SECONDSTIME;
    NEXT_MEASUREMENT_SECONDSTIME = CURRENT_SECONDSTIME + EVERY_X_SECONDS;

    strcpy(sdDataLine, "NO_DATA"); // just to make sure that the buffer is empty from previous measurements

    strcpy(sdDataLine, dateAndTimeData);
    strcat(sdDataLine, delimiter);

    for (int16_t readings = 0; readings < READINGS_PER_MEASUREMENT; readings ++) {
      for (int8_t thermistor = 0; thermistor < thermistorArraySize; thermistor ++) {
        thermistorArray[thermistor].getSingleReading();
      }
    }

    for (int8_t thermistor = 0; thermistor < thermistorArraySize; thermistor ++) {
      char * singleMeasurementDataPtr;
      singleMeasurementDataPtr = thermistorArray[thermistor].getMeasurement();
      strcat(sdDataLine, singleMeasurementDataPtr);
    }

    headerLine = false;
    sdWrite(SD1_CS, sd1, sdMeasurementDirName, sdMeasurementFile1, sdMeasurementFileName, sdDataLine, headerLine);

  }
}

//------------------------------------------------------------------------------




// To keep the correct number of lines. :)
