#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#define BUTTON_FORWARD 5
#define BUTTON_BACK 6
#define BUTTON_LEFT 7
#define BUTTON_RIGHT 8

#define MODE_POINT_TURN 1
#define MODE_PIVOT_TURN 2

byte leftMotorPins[3]   = {10, 6, 5};  // B
byte rightMotorPins[3]  = {9, 3, 11};    // A
byte maxSpeed           = 255;
byte halfSpeed          = 127;
boolean verboseMode = true;

// Create the bluefruit object
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN, BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

void setup(void)
{
  for (byte i = 0 ; i < 3 ; i++) {
    pinMode(leftMotorPins[i], OUTPUT);
    pinMode(rightMotorPins[i], OUTPUT);
  }

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) ) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ) {
    error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  Serial.println(F("Setting device name to 'VaderTank': "));
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=VaderTank")) ) {
    error(F("Could not set device name."));
  }
  else Serial.println("name of device is now 'VaderTank'");

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  // Set Bluefruit to DATA mode
  Serial.println(F("*****************"));
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println(F("*****************"));
}

void loop(void) {
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    if (
      Serial.print ("Button ");
      Serial.print(buttnum);
      if (pressed) {
        Serial.println(" pressed");
      ] else {
        Serial.println(" released
      }
    }
    
    byte buttonNumber = (buttnum); // should this be a different type?

    // button conditionals
    if (buttonNumber <= 4) { //mode change button
      switch (buttonNumber) {
        case 1:  // full stop
          Serial.println("STOP");
          motorSpeeds(0, 0);
          break;
        case 2:
          break;
        case 3:
          break;
        case 4:
          break;
      }
    }
    else { // movement button
      switch (buttonNumber) {
        case BUTTON_FORWARD:
          Serial.println("FORWARD");
          motorSpeeds(maxSpeed, maxSpeed);
          motorDirections(true, true);
          break;
        case BUTTON_BACK:
          Serial.println("BACK");
          motorSpeeds(maxSpeed, maxSpeed);
          motorDirections(false, false);
          break;
        case BUTTON_LEFT:
          Serial.println("LEFT");
          motorSpeeds(maxSpeed, maxSpeed);
          motorDirections(false, true);
          break;
        case BUTTON_RIGHT:
          Serial.println("RIGHT");
          motorSpeeds(maxSpeed, maxSpeed);
          motorDirections(true, false);
          break;
      }
    }
  }
}

/* sets both motor speeds
   input: left and right PWM values (byte)
   input range: 0 to maxSpeed
   output: none
*/
void motorSpeeds(byte l, byte r) {
  analogWrite(leftMotorPins[0], l);
  analogWrite(rightMotorPins[0], r);
  
  /*Serial.print(analogRead(leftMotorPins[0]));
  Serial.print(" ");
  Serial.print(analogRead(rightMotorPins[0]));
  Serial.println();
}

/* sets both motor directions
   input: left and right direction (boolean)
   input range: true = forward, false = reverse
   output: none
*/
void motorDirections(boolean l, boolean r) {
  digitalWrite(leftMotorPins[1], l);
  digitalWrite(leftMotorPins[2], !l);
  digitalWrite(rightMotorPins[1], r);
  digitalWrite(rightMotorPins[2], !r);
}
