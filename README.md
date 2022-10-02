# Arduino MKR WiFi 1010 Experiments

Example project with an arduino MKR wifi 1010.

The application reads temperature and pressure from a BMP280 sensor and send them to an http server in the same network.
In addition, it reads and send the power status of the battery charger circuit (BQ24195) to check whether the board is powered by usb or battery.

## Requirements

- An Arduino MKR WiFi 1010
- BMP280 temperature/pressure sensor
- arduino-cli

## Arduino libs to install

These are the libraries used in the example sketch:

- ArduinoHttpClient
- "Adafruit BMP280 Library"
- WiFiNINA
- ArduinoJson
- "Arduino Low Power"

To install a library, run

```
arduino-cli lib install <name>
```

## Building and uploading the project on arduino

To compile the ino file, run

```
arduino-cli -b <board> mkr_wifi_sensor.ino
```

Where board is the fqbn (fully-qualified board name) of the arduino. In the case of a MKR1010 the fqbn is 'arduino:samd:mkrwifi1010'.

To upload the ino file to a board, first list the available devices:

```
arduino-cli board list
```

Then, compile (and upload) the sketch:

```
arduino-cli upload -b <board fqbn> -p <port> mkr_wifi_sensor.ino
```
