
#include <avr/wdt.h>

#define sensorPin   2

byte statusLed    = 13;

// The hall-effect flow sensor outputs approximately 1.67 pulses per second per
// litre/minute of flow.
float calibrationFactor = 1.67;

volatile byte pulseCount;

float flowRate;
//unsigned int flowMilliLitres;
//unsigned long totalMilliLitres;
double flowLitres;
double totalLitres;

unsigned long secNow, secLast;
int hours, minutes, seconds;

unsigned long oldTime;

void pulseCounter()
{
  pulseCount++;
}

void setup()
{

  // Initialize a serial connection for reporting values to the host
  Serial.begin(9600);
  
  // Set up the status LED line as an output
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, HIGH);  // We have an active-low LED attached

  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  pulseCount        = 0;
  flowRate          = 0.0;
  //flowMilliLitres   = 0;
  //totalMilliLitres  = 0;
  flowLitres        = 0;
  totalLitres       = 0;
  oldTime           = 0;

  wdt_enable(WDTO_2S);  // watchdog timer with 2 second time out
  
  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change (transition from HIGH
  // state to LOW state)
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
}

/**
   Main program loop
*/
void loop()
{

  if ((millis() - oldTime) > 1000)   // Only process counters once per second
  {
    // Disable the interrupt while calculating flow rate and sending the value to
    // the host
    detachInterrupt(digitalPinToInterrupt(sensorPin));

    wdt_reset();  // reset the watchdog timer
    
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;

    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTime = millis();

    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    // flowMilliLitres = (flowRate / 60) * 1000;
    flowLitres = (flowRate / 60.00);
    // Add the millilitres passed in this second to the cumulative total
    // totalMilliLitres += flowMilliLitres;
    totalLitres += flowLitres;

    secNow = (millis() / 1000);
    seconds = secNow - secLast;
    if (seconds == 60)
    {
        secLast = secNow;
        seconds = 0;
        minutes += 1;
    }
    if (minutes == 60)
    {
        minutes = 0;
        hours += 1;
    }
    Serial.print(hours);
    Serial.print(", ");
    Serial.print(minutes);
    Serial.print(", ");
    Serial.print(seconds);
    Serial.print(", ");
    // Print the flow rate for this second in litres / minute
    Serial.print(flowRate, 2);
    Serial.print(", ");
    // Print the cumulative total of litres flowed since starting
    Serial.println(totalLitres, 2);

    // Reset the pulse counter so we can start incrementing again
    pulseCount = 0;

    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
  }
}
