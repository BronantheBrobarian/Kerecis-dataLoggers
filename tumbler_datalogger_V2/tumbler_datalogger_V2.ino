// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       tumbler_datalogger_V2.ino
    Created:	1/20/2022 11:04:41 AM
    Author:     KERECIS\bkr
*/

// Define User Types below here or use a .h file
//
#include <WiFi.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <esp_task_wdt.h>


// Define Function Prototypes that use User Types below here or use a .h file
//
#define FLOW_PIN        32      // was PIN 14 which is ADC2 and tied up by WiFi
#define PRESSURE_PIN    33      // was PIN 15 which is ADC2 and tied up by WiFi
#define TEMP_PIN        A5
// make sure when using WiFi to read sensors using ADC1 - 8 channel ADC available
// on Pins# 32, 33, A4, A3 and A2 
#define LED_PIN         13

#define ADC_RESOLUTION  4096
#define ADC_OFFSET      409
#define PSI_RANGE       100.00
#define BITS_PER_PSI    33.14

#define WDT_TIMEOUT     6       // 6 second WDT

// DATA LOGGING SYSTEM INFO
// 
// MCU ID               MCU-01
// Sensor IDs
// Env-001-01            Temperature environment
// Env-001-02            Humidity environment
// Env-001-03            Barometric pressure environment
// Flo-001-01            Flowrate of water
// Flo-001-02            Total water volume
// Pre-001-01            Water pressure
// Tmp-001-01            Water temperature


float data_array[4];        // at least long enough for the longer transmission
String tumbler_ids[4];
String env_ids[3];
void initialize_sensor_ids()
{
    env_ids[0] = "Env-001-01";
    env_ids[1] = "Env-001-02";
    env_ids[2] = "Env-001-03";
    tumbler_ids[0] = "Flo-001-01";
    tumbler_ids[1] = "Pre-001-01";
    tumbler_ids[2] = "Tmp-001-01";
    tumbler_ids[3] = "Flo-001-02";      // note the order of ids
}


// Environmental sensor - Bosch BME280
BME280 env_sensor;
// Temp sensor - DS18B20
// Setup onewire bus on temp pin
OneWire ONE_WIRE(TEMP_PIN);
DallasTemperature temp_sensor(&ONE_WIRE);


// Network information
//
const char* ssid = "kerecis_staff";
const char* password = "0xt38Rpz71uiXG1qAFBTx126hNviS";

const char* server = "192.168.22.29";
const int hostPort = 80;

// Variables
//
float env_temp, env_humid, env_press;
float bsln_env_temp, bsln_env_humid, bsln_env_press;
float calibrationFactor = 1.67;
volatile byte pulseCount;
float flowRate = 0.00, waterPress = 0.00, waterTemp = 0.00;
float flowLitres, flowRates_average;
float totalVolume_flow = 0.000;
uint8_t flow_counter = 0;
unsigned long flowUpdate_oldTime, envUpdate_oldTime, env_oldTime, flow_oldTime;
int waitTimeEnv = 3600000;       // 1 hour - max delay between environmental sensor transmissions
int waitTimeFlow = 60000;       // 60 sec - transmission delay for flow sensor
int environmentUpdateTime = 5000;    // 5 sec - environment sensor value update
int flowUpdateTime = 1000;    // 1 sec - flow sensor value update
int press_read;
bool env_temp_trigger = false;
bool env_humid_trigger = false;
bool env_press_trigger = false;
bool flow_trigger = false;

// Error variable
float ERROR_CODE = -99.99;

// Environment thresholds for trigger warnings
float temp_threshold = 1.00;
float humid_threshold = 2.00;
float press_threshold = 1000.00;             // equals 10 mbar difference

// Define Functions below here or use other .ino or cpp files
//
void triggerCheck();
void readEnvironment();
void setEnvBaseLine();
void getTotalFlow();
void getWaterPressure();
void getFlowReading();
void transmit(float data_array[], String sensorIDs[], uint8_t varCount); 
void connectToWiFi(const char* ssid, const char* pwd);
void printLine();

void pulseCounter()
{
    pulseCount++;
}


// The setup() function runs once each time the micro-controller starts
void setup()
{
    initialize_sensor_ids();
    esp_task_wdt_init(WDT_TIMEOUT, true);   // enable panic so ESP restarts
    esp_task_wdt_add(NULL);                 // add current thread to WDT watch

    Serial.begin(9600);
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());

    pinMode(FLOW_PIN, INPUT_PULLUP);
    pinMode(PRESSURE_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);

    Wire.begin();
    Wire.setClock(100000);  // 10kHz is slow - 400kHz is fast I2C speed
    // To allow placing the environment sensor at a longer distance
    // the bus speed is reduced to 10kHz which is supported by ESP32
    // default: 50kHz supports: 10kHz, 50kHz, 100kHz, 200kHz,... 800kHz

    // BME280 environment sensor only supports 100kHz, 400kHz or 3.4MHz - so 100kHz

    esp_task_wdt_reset();
    connectToWiFi(ssid, password);
    esp_task_wdt_reset();

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
    data_array[0] = env_temp;
    data_array[1] = env_humid;
    data_array[2] = env_press;
    transmit(data_array, env_ids, 3);

    temp_sensor.begin();                // start the temp sensor

    pulseCount = 0;
    flowRate = 0.00;
    flowRates_average = 0.00;
    flowUpdate_oldTime = 0;
    envUpdate_oldTime = 0;
    env_oldTime = 0;
    flow_oldTime = 0;

    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);

}

// Add the main program code into the continuous loop() function
void loop()
{
    esp_task_wdt_reset();
    if ((millis() - flowUpdate_oldTime) > flowUpdateTime)
    {
        getFlowReading();
        if (flowRate > 0.00) { flow_trigger = true; }
    }
    if ((millis() - envUpdate_oldTime) > environmentUpdateTime)
    {
        readEnvironment();
        if (abs(env_temp - bsln_env_temp) > temp_threshold) { env_temp_trigger = true; }
        if (abs(env_humid - bsln_env_humid) > humid_threshold) { env_humid_trigger = true; }
        if (abs(env_press - bsln_env_press) > press_threshold) { env_press_trigger = true; }
    }
    if ((millis() - env_oldTime) > waitTimeEnv)
    {
        readEnvironment();
        env_oldTime = millis();
        data_array[0] = env_temp;
        data_array[1] = env_humid;
        data_array[2] = env_press;
        transmit(data_array, env_ids, 3);
        setEnvBaseLine();
    }
    if ((flow_trigger == true) && ((millis() - flow_oldTime) > waitTimeFlow))
    {
        detachInterrupt(digitalPinToInterrupt(FLOW_PIN));
        // request temperature from temp sensor
        temp_sensor.requestTemperatures();              // give sensor heads-up before reading from the onewire bus
        getTotalFlow();
        getWaterPressure();
        flow_oldTime = millis();
        waterTemp = temp_sensor.getTempCByIndex(0);     // index "0" means the first sensor on the onewire bus
        data_array[0] = flowRate;
        data_array[1] = waterPress;
        data_array[2] = waterTemp;
        data_array[3] = totalVolume_flow;
        transmit(data_array, tumbler_ids, 4);

        flow_trigger = false;
    }
    if ((env_temp_trigger == true) || (env_humid_trigger == true) ||
        (env_press_trigger == true))
    {
        triggerCheck();
        env_temp_trigger = false;
        env_humid_trigger = false;
        env_press_trigger = false;
    }
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

    if ((data_array[0] != ERROR_CODE) ||
        (data_array[1] != ERROR_CODE) ||
        (data_array[2] != ERROR_CODE))
    {
        transmit(data_array, env_ids, 3);
    }
}

void readEnvironment() 
{
    detachInterrupt(digitalPinToInterrupt(FLOW_PIN));
    env_sensor.setMode(MODE_FORCED);
    // turn off the interrupts and reset the flow counter 
    // waking up the env sensor, reading and then putting it to sleep
    while (env_sensor.isMeasuring() == true);
    pulseCount = 0;
    flowUpdate_oldTime = millis();
    env_temp = env_sensor.readTempC();
    env_humid = env_sensor.readFloatHumidity();
    env_press = env_sensor.readFloatPressure();
    env_sensor.setMode(MODE_SLEEP);
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);

    envUpdate_oldTime = millis();
}

void setEnvBaseLine()
{
    bsln_env_temp = env_temp;
    bsln_env_humid = env_humid;
    bsln_env_press = env_press;
}

void getTotalFlow()
{
    flowRates_average = flowRates_average / (float)waitTimeFlow;
    totalVolume_flow += flowRates_average / (float)1000;   // estimated volume of water used in m3
    flowRates_average = 0.00;
}

void getWaterPressure()
{
    press_read = analogRead(PRESSURE_PIN);

    if (press_read > ADC_OFFSET)
    {
        press_read -= ADC_OFFSET;
        //int temp = ADC_RESOLUTION - (2 * ADC_OFFSET);
        //waterPress = ((float)press_read / (float)temp) * PSI_RANGE;
        waterPress = (float)press_read / (float)BITS_PER_PSI;
    }
    else
    {
        waterPress = 0.00;
    }
}

void getFlowReading() 
{
    detachInterrupt(digitalPinToInterrupt(FLOW_PIN));
    // calculate flow
    flowRate = (float)((1000.0 / (millis() - flowUpdate_oldTime)) * pulseCount) / calibrationFactor;
    flowRates_average += flowRate;
    pulseCount = 0;
    flowUpdate_oldTime = millis();
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);
}


void transmit(float data_array[], String sensorIDs[], uint8_t varCount)
{
    detachInterrupt(digitalPinToInterrupt(FLOW_PIN));
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

    // reset everything so to not mess up the reading of sensors
    pulseCount = 0;
    flowUpdate_oldTime = millis();
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);
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
