// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
#include <SPI.h>
#include <Controllino.h> // Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch.
#include <Wire.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70

const int8_t I2CPORT0 = 0;
const int8_t I2CPORT1 = 1;
const int8_t I2CPORT2 = 2;
const int8_t I2CPORT3 = 3;

RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup () {

  Wire.begin();

  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("\nTCAScanner ready!");

  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;

      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
        Serial.print("Found I2C 0x");  Serial.println(addr, HEX);
      }
    }
  }
  Serial.print("\nScanning done...");


  tcaselect(I2CPORT3);

  Serial.println("...switched to port 3.");

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));


  //rtc.adjust(DateTime(2019, 12, 23, 18, 31, 20));

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

void loop () {
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print("\t");

  //  Serial.print(" since midnight 1/1/1970 = ");
  //  Serial.print(now.unixtime());
  //  Serial.print("s = ");
  //  Serial.print(now.unixtime() / 86400L);
  //  Serial.println("d");

  // calculate a date which is 7 days and 30 seconds into the future
  //  DateTime future (now + TimeSpan(7, 12, 30, 6));
  //
  //  Serial.print(" now + 7d + 30s: ");
  //  Serial.print(future.year(), DEC);
  //  Serial.print('/');
  //  Serial.print(future.month(), DEC);
  //  Serial.print('/');
  //  Serial.print(future.day(), DEC);
  //  Serial.print(' ');
  //  Serial.print(future.hour(), DEC);
  //  Serial.print(':');
  //  Serial.print(future.minute(), DEC);
  //  Serial.print(':');
  //  Serial.print(future.second(), DEC);
  //  Serial.println();

  Serial.print("Temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");

  Serial.println();
  delay(1000);
}
