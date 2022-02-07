#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>

#define sensorPin   2

OneWire oneWire(sensorPin);
DallasTemperature sensors(&oneWire);

float temp;
unsigned long TIME, oldTIME;

void setup() {
  
  Serial.begin(9600);
  sensors.begin();
  TIME = 0;
  oldTIME = 0;

  wdt_enable(WDTO_4S);  // watchdog timer with 2 second time out
}

void loop() {
  
  if ((millis() - oldTIME) > 3000)
  {
    wdt_reset();
    sensors.requestTemperatures(); 
    //print the temperature in Celsius
    Serial.print(sensors.getTempCByIndex(0));
    Serial.print(", ");
    TIME = TIME + 3;
    Serial.println(TIME);
    oldTIME = millis();
  }
}
