// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       diffPress_testing.ino
    Created:	2/4/2022 8:29:29 AM
    Author:     KERECIS\bkr
*/

// Define User Types below here or use a .h file
//
#include <DFRobot_LWLP.h>

// Define Function Prototypes that use User Types below here or use a .h file
//
// Differential pressure sensor
DFRobot_LWLP diffSensor;

// Define Functions below here or use other .ino or cpp files
//

// variables
float diff_press, diff_temp;

void getDiffPressure();

// The setup() function runs once each time the micro-controller starts
void setup()
{
    Serial.begin(9600);

    while (diffSensor.begin() != 0) {
        Serial.println("Failed to initialize the chip, please confirm the chip connection");
        delay(1000);
    }

    //Auto calibration differential pressure drift
    diffSensor.autoCorDrift();
    //Manual calibration differential pressure drift
    //lwlp.passiveCorDrift(/*Drift = */8.23);
}

// Add the main program code into the continuous loop() function
void loop()
{

    getDiffPressure();
    Serial.print(diff_press);
    Serial.print("\t");
    Serial.println(diff_temp);
    delay(1000);
}


void getDiffPressure()
{
    DFRobot_LWLP::sLwlp_t data;
    //Get data of single measurement 
    data = diffSensor.getData();
    //Get filter-processed data 
    //data = lwlp.getfilterData();
    diff_press = data.presure;
    diff_temp = data.temperature;

}
