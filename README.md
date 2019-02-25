# arduino-led-controls
Arduino code to control an LED strip and visualize music

Instrutions to assemble the LED strip that can be controlled from an android phone and visualize music can be found here: https://www.instructables.com/id/Party-Lights-1/

LED_control/LED_control.ino is the main program that needs to be uploaded to Arduino.
WS2812B.cpp and WS2812B.h contains some assembly code used for efficient communication with the LED strip.
HC-06 setup folder contains code for setting up the Bluetooth module (baud rate, bluetooth name/password, etc.)
