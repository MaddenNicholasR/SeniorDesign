#include "sensorbar.h"
#include <SoftwareSerial.h>
#include <Sabertooth.h>

// The Sabertooth library can be downloaded from the link below
// http://www.dimensionengineering.com/info/arduino

//Define pins for Sabertooth Controller
#define ADDRESS 128
#define SAB_ESTOP 12
#define SAB_SERIAL 13

//Define the states that the decision making machines uses:
#define IDLE_STATE 0
#define READ_STATE 1
#define EMERGENCY -1

int state;

//Uncomment one of the four lines to match SX1509's address pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const uint8_t SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const uint8_t SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const uint8_t SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);
SoftwareSerial SWSerial(NOT_A_PIN, SAB_SERIAL); // RX unused, TX on pin 13 (to S1).
Sabertooth motorController(ADDRESS, SWSerial);

void setup() {
  //Setup motor controller
  SWSerial.begin(9600);
  pinMode(SAB_ESTOP, OUTPUT);
  delay(200);
  motorController.autobaud();

  //Release emergency stop
  digitalWrite(SAB_ESTOP, HIGH);

  //Debug output
  Serial.begin(9600);
  Serial.println("Program started.");
  Serial.println();

  //Set up line reader
  //IR will only be turned on during reads.
  mySensorBar.setBarStrobe();
  //Dark on light
  mySensorBar.clearInvertBits();

  //Attempt connection
  uint8_t returnStatus = mySensorBar.begin();
  if (returnStatus)
  {
    Serial.println("sx1509 IC communication OK");
  }
  else
  {
    Serial.println("sx1509 IC communication FAILED!");
    while (1);
  }
  Serial.println();
}

void loop() {
  int power = 64;
  int turn = 0;

  switch (state) {
    case IDLE_STATE:
      power = 0;
      turn = 0;
      state = READ_STATE;
      break;
    case READ_STATE:
      int density = mySensorBar.getDensity();
      Serial.print("IR Density: ")
      Serial.println(density);
      if (density  < 7 ) {
        // Position (-127 - 127)
        if ( mySensorBar.getPosition() < -50 ) {
          power = 0;
          turn = -32;
        }
        else if ( mySensorBar.getPosition() > 50 ) {
          power = 0;
          turn = 32;
        }
      }
      else {
        state = IDLE_STATE;
      }
      break;
    case EMERGENCY:
      power = 0;
      turn = 0;
      digitalWrite(SAB_ESTOP, LOW);
      break;
  }

  motorController.drive(power);
  motorController.turn(turn);
  delay(75);
}
