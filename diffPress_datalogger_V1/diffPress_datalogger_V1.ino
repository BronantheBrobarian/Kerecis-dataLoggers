// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       diffPress_datalogger_V1.ino
    Created:	2/2/2022 9:14:11 AM
    Author:     KERECIS\bkr
*/

// Define User Types below here or use a .h file
//
#include <Wire.h>
#include <WiFi.h>
#include <DFRobot_LWLP.h>
#include <SparkFunBME280.h>

#include <esp_task_wdt.h>

// Define Function Prototypes that use User Types below here or use a .h file
//
#define LED_PIN         13

#define WDT_TIMEOUT     6       // 6 second WDT


// make sure when using WiFi to read sensors using ADC1 - 8 channel ADC available
// on Pins# 32, 33, A4, A3 and A2 


// DATA LOGGING SYSTEM INFO
// 
// MCU ID               MCU-01
// Sensor IDs
// Env-002-01            Temperature environment
// Env-002-02            Humidity environment
// Env-002-03            Barometric pressure environment
// Dpr-001-01            Differential pressure between rooms
// Dpr-001-02            Temperature used by compensation algirithm on sensor


float data_array[5];        // at least long enough for the longer transmission
String sensor_ids[5];
void initialize_sensor_ids()
{
    sensor_ids[0] = "Env-002-01";
    sensor_ids[1] = "Env-002-02";
    sensor_ids[2] = "Env-002-03";
    sensor_ids[3] = "Dpr-001-01";
    sensor_ids[4] = "Dpr-001-02";
}

// Differential pressure sensor
DFRobot_LWLP diffSensor;
// Environmental sensor - Bosch BME280
BME280 env_sensor;


// Network information
//
const char* ssid = "kerecis_staff";
const char* password = "0xt38Rpz71uiXG1qAFBTx126hNviS";

const char* server = "192.168.22.29";
const int hostPort = 80;


// Variables
float env_temp, env_humid, env_press;
float bsln_env_temp, bsln_env_humid, bsln_env_press;

float diff_press, diff_temp;

unsigned long envUpdate_oldTime, diffUpdate_oldTime, serverUpdate_oldTime;
int diffUpdateTime = 2000;    // 2 sec - diff pressure value update
int environmentUpdateTime = 5000;    // 5 sec - environment sensor value update
//int serverUpdateTime = 3600000;     // 1 hour - max delay between updates
int serverUpdateTime = 1800000;     // 30 min - max delay between updates
//int serverUpdateTime = 10000;     // 10 sec - max delay between updates

bool env_temp_trigger = false;
bool env_humid_trigger = false;
bool env_press_trigger = false;
bool diff_press_trigger = false;

// Error variable
float ERROR_CODE = -99.99;

// Environment thresholds for trigger warnings
float temp_threshold = 1.00;
float humid_threshold = 2.00;
float press_threshold = 1000.00;             // equals 10 mbar difference
float diff_press_threshold = 8.00;
// Normally pressure difference is between 8 and 12 pascals
// 8 pascal difference is considered too low... ? 


// Define Functions below here or use other .ino or cpp files
//
void readEnvironment();
void setEnvBaseLine();
void getDiffPressure();
void triggerCheck();

// The setup() function runs once each time the micro-controller starts
void setup()
{
    initialize_sensor_ids();
    esp_task_wdt_init(WDT_TIMEOUT, true);   // enable panic so ESP restarts
    esp_task_wdt_add(NULL);                 // add current thread to WDT watch
    Serial.begin(9600);

    esp_task_wdt_reset();
    connectToWiFi(ssid, password);
    esp_task_wdt_reset();
    
    Wire.begin();
    //Init chip 
    while (diffSensor.begin() != 0) {
        Serial.println("Failed to initialize the chip, please confirm the chip connection");
        delay(1000);
    }
    //Auto calibration differential pressure drift
    diffSensor.autoCorDrift();
    //Manual calibration differential pressure drift
    //lwlp.passiveCorDrift(/*Drift = */8.23);

    env_sensor.begin();
    env_sensor.setMode(MODE_SLEEP);
    // MODE_SLEEP - Sleep mode (Default): No operation, all registers accessible, 
    // lowest power, selected after startup.
    // MODE_FORCED - Forced mode (low power operation): Performs one measurement, 
    // store results and return to sleep mode.
    // MODE_NORMAL - Normal mode (active measurements): Perpetual cycling of 
    // measurements and inactive periods.
    // 
    // initalize environment variables for monitoring
    readEnvironment();
    setEnvBaseLine();
    getDiffPressure();
    data_array[0] = env_temp;
    data_array[1] = env_humid;
    data_array[2] = env_press;
    data_array[3] = diff_press;
    data_array[4] = diff_temp;
    transmit(data_array, sensor_ids, 5);
}

// Add the main program code into the continuous loop() function
void loop()
{
    esp_task_wdt_reset();
    if ((millis() - diffUpdate_oldTime) > diffUpdateTime)
    {
        getDiffPressure();
        if (abs(diff_press) < diff_press_threshold) // if the difference is smaller than threshold
        { diff_press_trigger = true; }
    }
    if ((millis() - envUpdate_oldTime) > environmentUpdateTime)
    {
        readEnvironment();
        if (abs(env_temp - bsln_env_temp) > temp_threshold) { env_temp_trigger = true; }
        if (abs(env_humid - bsln_env_humid) > humid_threshold) { env_humid_trigger = true; }
        if (abs(env_press - bsln_env_press) > press_threshold) { env_press_trigger = true; }
    }
    if ((millis() - serverUpdate_oldTime) > serverUpdateTime)
    {
        data_array[0] = env_temp;
        data_array[1] = env_humid;
        data_array[2] = env_press;
        data_array[3] = diff_press;
        data_array[4] = diff_temp;
        serverUpdate_oldTime = millis();
        transmit(data_array, sensor_ids, 5);
    }
    if ((env_temp_trigger == true) || (env_humid_trigger == true) ||
        (env_press_trigger == true) || (diff_press_trigger == true))
    {
        triggerCheck();
        env_temp_trigger = false;
        env_humid_trigger = false;
        env_press_trigger = false;
        diff_press_trigger = false;
    }

}


void readEnvironment()
{
    env_sensor.setMode(MODE_FORCED);
    // waking up the env sensor, reading and then putting it to sleep
    while (env_sensor.isMeasuring() == true);
    env_temp = env_sensor.readTempC();
    env_humid = env_sensor.readFloatHumidity();
    env_press = env_sensor.readFloatPressure();
    env_sensor.setMode(MODE_SLEEP);

    envUpdate_oldTime = millis();
}

void setEnvBaseLine()
{
    bsln_env_temp = env_temp;
    bsln_env_humid = env_humid;
    bsln_env_press = env_press;
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

    diffUpdate_oldTime = millis();
}

void triggerCheck()
{
    readEnvironment();
    // check if new sensor readings still exceed the threshold values
    // and transmit the out of bounds values
    if (abs(env_temp - bsln_env_temp) > temp_threshold) {
        data_array[0] = env_temp;
        bsln_env_temp = env_temp;
    }
    else {
        data_array[0] = ERROR_CODE;
    }
    if (abs(env_humid - bsln_env_humid) > humid_threshold) {
        data_array[1] = env_humid;
        bsln_env_humid = env_humid;
    }
    else {
        data_array[1] = ERROR_CODE;
    }
    if (abs(env_press - bsln_env_press) > press_threshold) {
        data_array[2] = env_press;
        bsln_env_press = env_press;
    }
    else {
        data_array[2] = ERROR_CODE;     // this is an error code..
    }
    if (abs(diff_press) < diff_press_threshold)  {
        data_array[3] = diff_press;
    }
    else {
        data_array[3] = ERROR_CODE;
    }

    if ((data_array[0] != ERROR_CODE) ||
        (data_array[1] != ERROR_CODE) ||
        (data_array[2] != ERROR_CODE) ||
        (data_array[3] != ERROR_CODE))
    {
        transmit(data_array, sensor_ids, 4);
    }
}


void transmit(float data_array[], String sensorIDs[], uint8_t varCount)
{
    esp_task_wdt_reset();
    printLine();
    Serial.println("Connecting to domain: " + String(server));
    WiFiClient client;
    if (!client.connect(server, hostPort))
    {
        Serial.println("connection failed");
        return;
    }
    Serial.println("Connected!");
    printLine();

    String initialization = "GET /DataSave.asmx/SaveSensorData?";
    String dataString = "";
    String end = " HTTP/1.1\r\nHost: " + String(server) + "\r\n"
        + "Connection: close\r\n\r\n";

    uint8_t stringVarCounter = 0;
    for (uint8_t i = 0; i < varCount; i++)
    {
        if (data_array[i] != ERROR_CODE)
        {
            // if not an error code proceed with making the string
            if (stringVarCounter > 0) { dataString += "&"; }
            dataString += "value=";
            dataString += sensorIDs[i];
            dataString += "&value=";
            dataString += String(data_array[i]);
            stringVarCounter++;
        }
        else {}
    }
    String transmission = initialization + dataString + end;

    Serial.println(transmission);

    client.print(transmission);
    unsigned long timeout = millis();
    while (client.available() == 0)
    {
        if (millis() - timeout > 5000)
        {
            Serial.println(">>> Client Timeout !");
            client.stop();
            return;
        }
    }

    esp_task_wdt_reset();
    // Read all the lines of the reply from server and print them to Serial
    while (client.available())
    {
        String line = client.readStringUntil('\r');
        Serial.print(line);
    }

    Serial.println();
    Serial.println("closing connection");
    client.stop();

    esp_task_wdt_reset();
}


void connectToWiFi(const char* ssid, const char* pwd)
{
    int ledState = 0;

    printLine();
    Serial.println("Connecting to WiFi network: " + String(ssid));

    WiFi.begin(ssid, pwd);

    while (WiFi.status() != WL_CONNECTED)
    {
        // Blink LED while we're connecting:
        digitalWrite(LED_PIN, ledState);
        ledState = (ledState + 1) % 2; // Flip ledState
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}


void printLine()
{
    Serial.println();
    for (int i = 0; i < 30; i++)
        Serial.print("-");
    Serial.println();
}
