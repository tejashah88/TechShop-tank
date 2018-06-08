#include <string.h>
#include <Arduino.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <SoftwareSerial.h>
#include <SPI.h>
// LED
const int LED_PIN = 13;

// Motor controller pins
int leftMotorPins[3]    = {10, 12, 11};     // pins connected to the left motor controller
int rightMotorPins[3]   = {9, 5, 6};     // pins connected to the right motor controller
int maxSpeed            = 180;
int maxTurn             = 120;



// we are using pins 7 and 8 (for RX and TX respectively), which is Serial3
const int BLE_MODE_PIN = 4;
#define BLUEFRUIT_SERIAL_PORT Serial3
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_SERIAL_PORT, BLE_MODE_PIN);

// the packet buffer
extern uint8_t packetbuffer[];
#include <packetParser.h>
#include "packetParser.cpp"

// keep track of the last time we ran!
long last_run_time = millis();
bool blink_light_mode = true;
bool light_follow_mode = false;

int forward_speed = 0;
int turn_speed = 0;


void initialize_motors() {
  pinMode(leftMotorPins[2], OUTPUT);
  pinMode(leftMotorPins[1], OUTPUT);
  pinMode(leftMotorPins[0], OUTPUT);
  pinMode(rightMotorPins[2], OUTPUT);
  pinMode(rightMotorPins[1], OUTPUT);
  pinMode(rightMotorPins[0], OUTPUT);
}

void initialize_sensors() {
  // TODO: initialize your light sensors here!
  pinMode(A8, INPUT);
  pinMode(A2, INPUT);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  initialize_motors();
  initialize_sensors();

  //while (!Serial); // Wait for serial port to connect
  while (!BLUEFRUIT_SERIAL_PORT);

  Serial.begin(115200);

  initialize_bluetooth(ble, "T-1000");
}

void setMotorForwardAndTurn(int forward, int turn) {
  int left = forward + turn;
  int right = forward - turn;

  setMotorDirection(left >= 0, right >= 0);
  setMotorSpeeds(abs(left), abs(right));
}

void setMotorDirection(bool left, bool right) {
  if (left) {
    // TODO: set left motor to forward
    digitalWrite(leftMotorPins[1], LOW);     // digitalWrite to the next two pins will determine direction of the motor
    digitalWrite(leftMotorPins[2], HIGH);
  } else {
    // TODO: set left motor to backward
    digitalWrite(leftMotorPins[1], HIGH);
    digitalWrite(leftMotorPins[2], LOW);
  }

  if (right) {
    // TODO: set right motor to forward
    digitalWrite(rightMotorPins[1], LOW);
    digitalWrite(rightMotorPins[2], HIGH);
  } else {
    // TODO: set right motor to backward
    digitalWrite(rightMotorPins[1], HIGH);
    digitalWrite(rightMotorPins[2], LOW);
  }
}

void setMotorSpeeds(int left, int right) {
  if (left < 0) {
    left = 0;
  }
  if (left > 255) {
    left = 255;
  }
  if (right < 0) {
    right = 0;
  }
  if (right > 255) {
    right = 255;
  }

  // left and right are now between 0 and 255!
  // TODO: set left motor speed to left
  analogWrite(leftMotorPins[0], left);
  // TODO: set right motor speed to right
  analogWrite(rightMotorPins[0], right);
}

void doButton() {
  uint8_t buttnum = packetbuffer[2] - '0';
  boolean pressed = packetbuffer[3] - '0';
  Serial.print ("Button "); Serial.print(buttnum);
  if (pressed) {
    Serial.println(" pressed");
  } else {
    Serial.println(" released");
  }

  switch (buttnum) {
    case 1:
      blink_light_mode = !pressed;
      digitalWrite(LED_PIN, pressed);
      break;
    case 2:
      light_follow_mode = pressed;
      if (!pressed) {
        forward_speed = 0;
        turn_speed = 0;
        setMotorForwardAndTurn(forward_speed, turn_speed);
     
      
        
      }
      break;
    case 5: // forward
      if (pressed) {
        forward_speed = maxSpeed;
      } else {
        forward_speed = 0;
      }
      setMotorForwardAndTurn(forward_speed, turn_speed);
      break;
    case 6: // backward
      if (pressed) {
        forward_speed = -maxSpeed;
      } else {
        forward_speed = 0;
      }
      setMotorForwardAndTurn(forward_speed, turn_speed);
      break;
    case 7: // right
      if (pressed) {
        turn_speed = -maxTurn;
      } else {
        turn_speed = 0;
      }
      setMotorForwardAndTurn(forward_speed, turn_speed);
      break;
    case 8:
      if (pressed) {
        turn_speed = maxTurn;
      } else {
        turn_speed = 0;
      }
      setMotorForwardAndTurn(forward_speed, turn_speed);
      break;

    default:
      forward_speed = 0;
      turn_speed = 0;
      setMotorForwardAndTurn(forward_speed, turn_speed);
  }
}
void doMagnetometer() {
  float x, y, z;
  x = parsefloat(packetbuffer + 2);
  y = parsefloat(packetbuffer + 6);
  z = parsefloat(packetbuffer + 10);
  Serial.print("Mag\t");
  Serial.print(x); Serial.print('\t');
  Serial.print(y); Serial.print('\t');
  Serial.print(z); Serial.println();
}

void doGyroscope() {
  float x, y, z;
  x = parsefloat(packetbuffer + 2);
  y = parsefloat(packetbuffer + 6);
  z = parsefloat(packetbuffer + 10);
  Serial.print("Gyro\t");
  Serial.print(x); Serial.print('\t');
  Serial.print(y); Serial.print('\t');
  Serial.print(z); Serial.println();
}

void doAccelerometer() {
  float x, y, z;
  x = parsefloat(packetbuffer + 2);
  y = parsefloat(packetbuffer + 6);
  z = parsefloat(packetbuffer + 10);
  Serial.print("Accel\t");
  Serial.print(x); Serial.print('\t');
  Serial.print(y); Serial.print('\t');
  Serial.print(z); Serial.println();
}

void doLightFollow() {
  int sensorValue = analogRead(A8);
  double light = sensorValue;
  // print out the value you read:
  Serial.println(light);
  delay(500);        // delay in between reads for stability
  int sensorValue2 = analogRead(A2);
  double light2 = sensorValue2;
  // print out the value you read:
  Serial.println(light2);
  delay(1000);

  if (light < 400) {

    if (light2 < 400) {
      forward_speed = maxSpeed;
    }
    else {
      forward_speed = 0;
    }
  }
  else {
    forward_speed = 0;
  }
  if (light2 > light) {
    turn_speed = maxTurn;
  }
  else if (light > light2 + 200) {
    turn_speed = -maxTurn;
  }
  else {
    turn_speed = 0;
  }

   setMotorForwardAndTurn(forward_speed, turn_speed);
}

void loop(void)
{
  uint8_t len = readPacket(&ble, 20);
  if (len > 0) {
    // we got a packet!
    switch (packetbuffer[1]) {
      case 'B':
        doButton();
        break;
      case 'A':
        doAccelerometer();
        break;
      case 'G':
        doGyroscope();
        break;
      case 'M':
        doMagnetometer();
        break;
    }
  }

  long current_time = millis();
  if (current_time - last_run_time >= 100) {
    last_run_time = current_time;
    if (blink_light_mode) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    if (light_follow_mode) {
      doLightFollow();
 

    }
  }
}



