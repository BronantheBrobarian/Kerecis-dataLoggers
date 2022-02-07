// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       datalogger_test.ino
    Created:	1/6/2022 4:43:21 PM
    Author:     KERECIS\bkr
*/

// Define User Types below here or use a .h file
//
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


// Network information
const char* ssid = "kerecis_staff";
const char* password = "0xt38Rpz71uiXG1qAFBTx126hNviS";

const char* server = "192.168.22.29";
const int hostPort = 80;


#define BUTTON_PIN  0
#define LED_PIN     13
#define DHTPIN      A0     // Digital pin connected to the DHT sensor


#define DHTTYPE    DHT22     // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

float Temp = 0.00, Humid = 0.00;


// functions
void connectToWiFi(const char* ssid, const char* pwd);
void requestURL(const char* host, uint8_t port);
void printLine();



void setup()
{
    Serial.begin(9600);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    delay(500);
    // Connect to the WiFi network (see function below loop)
    connectToWiFi(ssid, password);

    digitalWrite(LED_PIN, LOW); // LED off
    Serial.print("Press button 0 to connect to ");
    Serial.println(server);

    dht.begin();
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    delayMS = sensor.min_delay / 100;
}


void loop()
{
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature))   {
        // send error message
        Serial.println("temp error\n");
    }
    else {
        // send temperature -> event.temperature
        Temp = event.temperature;
    }
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        // send error message
        Serial.println("humid error\n");
    }
    else {
        // send humidity -> event.relative_humidity
        Serial.print("\t");
        Humid = event.relative_humidity;
    }
    Serial.println("Sending T & H");
    Serial.print(Temp);
    Serial.print(" ,  ");
    Serial.println(Humid);
    digitalWrite(LED_PIN, HIGH); // Turn on LED
    requestURL(server, hostPort);
    digitalWrite(LED_PIN, LOW); // Turn off LED

    Serial.println("waiting for next transmission");
    delay(delayMS);



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

void requestURL(const char* host, uint8_t port)
{
    printLine();
    Serial.println("Connecting to domain: " + String(host));

    // Use WiFiClient class to create TCP connections
    WiFiClient client;
    if (!client.connect(host, port))
    {
        Serial.println("connection failed");
        return;
    }
    Serial.println("Connected!");
    printLine();

    // This will send the request to the server
    client.print((String)"GET /DataSave.asmx/SaveSensorData?" + 
        "value=100&value=" + 
        String(Humid) + "&value=Rh HTTP/1.1\r\n" +
        "Host: " + String(host) + "\r\n" +
        "Connection: close\r\n\r\n");
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

    // Read all the lines of the reply from server and print them to Serial
    while (client.available())
    {
        String line = client.readStringUntil('\r');
        Serial.print(line);
    }

    Serial.println();
    Serial.println("closing connection");
    client.stop();
}

void printLine()
{
    Serial.println();
    for (int i = 0; i < 30; i++)
        Serial.print("-");
    Serial.println();
}
