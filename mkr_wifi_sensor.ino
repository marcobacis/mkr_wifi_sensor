#include <Adafruit_BMP280.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <ArduinoLowPower.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <utility/wifi_drv.h>

#include "arduino_secrets.h"

char server_address[] = "192.168.1.100";
int port = 8000;

WiFiClient client;
HttpClient http = HttpClient(client, server_address, port);
Adafruit_BMP280 bmp;

enum LedColor { OFF = 0, RED = 1, GREEN = 2, BLUE = 3 };

char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)
int wifi_status = WL_IDLE_STATUS;

unsigned long lastConnectionTime = 0;              // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 30L * 1000L; // delay between updates, in milliseconds

void setup() {
    WiFiDrv::pinMode(25, OUTPUT);
    WiFiDrv::pinMode(26, OUTPUT);
    WiFiDrv::pinMode(27, OUTPUT);

    init_wifi();
    init_temp_sensor();

    http.setHttpResponseTimeout(5);
    http.setTimeout(5000);
}

void loop() {
    check_wifi();

    if (wifi_status == WL_CONNECTED) {
        float temperature = bmp.readTemperature();
        float pressure = bmp.readPressure();

        // read the input on analog pin 0:
        int sensorValue = analogRead(ADC_BATTERY);
        // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
        float voltage = sensorValue * (4.3 / 1023.0);

        byte status = getPowerState();
        Serial.print("Temperature ");
        Serial.print(temperature, 2);
        Serial.print("*C, Pressure ");
        Serial.print(pressure);
        Serial.print(" Pa, Voltage ");
        Serial.print(voltage);
        Serial.print("V, Power byte ");
        Serial.print(status);
        Serial.print("\n");

        httpRequest(temperature, pressure, voltage, status);
        Serial.println();

        lastConnectionTime = millis();
    }

    LowPower.sleep(30000);
}

void init_wifi() {
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        wifi_led(RED);
        // don't continue
        while (true)
            ;
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
        Serial.println("Please upgrade the firmware");
    }
}

#define PMIC_ADDRESS 0x6B
#define PMIC_REG08 0x08

byte getPowerState() {
    Wire.beginTransmission(PMIC_ADDRESS);
    Wire.write(PMIC_REG08);
    Wire.requestFrom(PMIC_ADDRESS, 1);
    byte val = Wire.read();
    Wire.endTransmission();
    return val;
}

void check_wifi() {
    if (wifi_status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        wifi_led(BLUE);
        wifi_status = WiFi.begin(ssid, pass);
        WiFi.lowPowerMode();
        wifi_led(OFF);

        if (wifi_status = WL_CONNECTED) {
            wifi_led(BLUE);
            delay(100);
            wifi_led(OFF);
            delay(100);
            wifi_led(BLUE);
            delay(100);
            wifi_led(OFF);
        }
    }

    wifi_status = WiFi.status();
}

void init_temp_sensor() {
    Serial.println("Initializing temperature sensor...");
    unsigned status;
    status = bmp.begin(BMP280_ADDRESS_ALT, 0x60);
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    Serial.println("Initialized temp sensor");
}

// this method makes a HTTP connection to the server:
void httpRequest(float temperature, float pressure, float voltage, byte status) {
    String postData;
    StaticJsonDocument<128> doc;

    doc["temperature"] = temperature;
    doc["voltage"] = voltage;
    doc["pressure"] = (int)pressure;
    doc["status"] = status;

    serializeJson(doc, postData);

    String contentType = "application/json";

    wifi_led(BLUE);

    http.post("/temperature", contentType, postData);
    int statusCode = http.responseStatusCode();
    String response = http.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode);
    Serial.print("Response: ");
    Serial.println(response);

    if (statusCode != 200)
        wifi_led(RED);
    delay(100);
    wifi_led(OFF);
}

void printWifiStatus() {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

void wifi_led(LedColor color) {
    switch (color) {
    case RED:
        wifi_led(50, 0, 0);
        break;
    case GREEN:
        wifi_led(50, 0, 0);
        break;
    case BLUE:
        wifi_led(0, 0, 50);
        break;
    case OFF:
        wifi_led(0, 0, 0);
        break;
    }
}

void wifi_led(int red, int green, int blue) {
    WiFiDrv::analogWrite(25, green); // GREEN
    WiFiDrv::analogWrite(26, red);   // RED
    WiFiDrv::analogWrite(27, blue);  // BLUE
}

float round2(float value) { return (int)(value * 100 + 0.5) / 100.0; }
float round3(float value) { return (int)(value * 1000 + 0.5) / 1000.0; }