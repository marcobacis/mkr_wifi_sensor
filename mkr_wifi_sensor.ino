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

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int wifi_status = WL_IDLE_STATUS;

// delay between updates, in milliseconds
const unsigned long postingInterval = 30L * 1000L;

void setup() {
    Serial.begin(57600);
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
        int sensorValue = analogRead(ADC_BATTERY);
        float voltage = sensorValue * (4.208 / 1024.0);
        byte status = getPowerState();

        httpRequest(temperature, pressure, voltage, status);
    }

    LowPower.sleep(postingInterval);
}

void init_wifi() {
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
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
        wifi_status = WiFi.begin(ssid, pass);
        WiFi.lowPowerMode();
        if (wifi_status == WL_CONNECTED) {
            Serial.println("Connected!");
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
    Serial.println(postData);

    String contentType = "application/json";

    http.post("/temperature", contentType, postData);
    int statusCode = http.responseStatusCode();
    String response = http.responseBody();
}