BOARD = arduino:samd:mkrwifi1010
PORT = /dev/tty.usbmodem14101
BUILD_OUTPUT=./out/

all: compile

compile: mkr_wifi_sensor.ino
	arduino-cli compile -b $(BOARD)

upload: compile
	arduino-cli upload -b $(BOARD) -p $(PORT)

clean:
	rm -rf $(BUILD_OUTPUT)