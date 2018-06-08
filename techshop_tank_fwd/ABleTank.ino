/*********************************************************************
 ABleTank 0.9.3
 21 August 2015
 Based on the nRF51822 library, used for bluetooth connection to RC tank
 Authors: Luca Angeleri and Kristof Aldenderfer
 For: Robotics Camp, TechShop San Francisco (Summer 2015)
 See readme.md for more information and hardware setup
*********************************************************************/
byte leftMotorPins[3]   = {10, 12, 11};
byte rightMotorPins[3]  = {9, 5, 6};
byte lightSensorPins[2] = {A0, A1};
byte maxSpeed           = 255;
String mode = "manual";
boolean verboseMode = true;

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#define BLUEFRUIT_HWSERIAL_NAME  Serial3 // check to see if this can be removed

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
  for (byte i=0 ; i<3 ; i++) {
    pinMode(leftMotorPins[i], OUTPUT);
    pinMode(rightMotorPins[i], OUTPUT);
  }
  pinMode(lightSensorPins[0], INPUT);
  pinMode(lightSensorPins[1], INPUT);

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

  Serial.println(F("Setting device name to 'ABleTankOne': "));
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=ABleTankOne")) ) {
    error(F("Could not set device name."));
  }
  else Serial.println("name of device is now ABleTankOne");

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
    if (verboseMode) {
      Serial.print ("Button ");
      Serial.print(buttnum);
      if (pressed) Serial.println(" pressed");
      else Serial.println("released");
    }
    byte buttonNumber = (buttnum); // should this be a different type?

    // button conditionals
    if (buttonNumber <= 4) { //mode change button
      if (buttonNumber == 1) { // full stop
        mode = "manual";
        motorSpeeds(0, 0);
      }
      else if (buttonNumber == 2) mode = "lightSeek";
      else mode = "null";
    }
    else { // movement button
      if (buttonNumber == 5) { // forward
        motorSpeeds(maxSpeed, maxSpeed);
        motorDirections(true, true);
      }
      else if (buttonNumber == 6) { // backward
        motorSpeeds(maxSpeed, maxSpeed);
        motorDirections(false, false);
      }
      else if (buttonNumber == 7) { // left
        motorSpeeds(maxSpeed, maxSpeed);
        motorDirections(false, true);
      }
      else if (buttonNumber == 8) { // right
        motorSpeeds(maxSpeed, maxSpeed);
        motorDirections(true, false);
      }
    }
  }
  else {
    if (mode == "lightSeek") {
      int lightSensorVals[2];
      for (byte i=0 ; i<2 ; i++) {
        lightSensorVals[i] = analogRead(lightSensorPins[i]);
      }
      if (abs(lightSensorVals[0] - lightSensorVals[1]) <= 40) {    // go straight
        motorSpeeds(maxSpeed, maxSpeed);
        motorDirections(true, true);
      }
      else if (lightSensorVals[0] > lightSensorVals[1]) {          // turn right
        motorSpeeds(maxSpeed, maxSpeed);
        motorDirections(true, false);
      }
      else {                                           // turn left
        motorSpeeds(maxSpeed, maxSpeed);
        motorDirections(false, true);
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
  byte leftMotorPins[3]   = {21, 18, 19};
byte rightMotorPins[3]  = {20, 17, 16};
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
