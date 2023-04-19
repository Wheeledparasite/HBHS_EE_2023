/*
 * Author: Lucas Ruebsamen
 * Date: December 28, 2021
 * 
 * Version 1.1 - Updated Jan 14, 2022
 * Version 1.4 - Updated Feb 9, 2022
 * Version 3.0 - Updated March 16, 2022
 * Version 3.1 - Updated April 20, 2022
 * Version 4.1 - Updated April 10, 2023
 * Version 4.2 - Updated April 12, 2023
 * 
 * Changelog:
 * 1.0 - Original Version using Differential Steering
 * 1.1 - Remove reverse ability
 * 1.2 - Interrupt to count wheel rotation sensors (hall effect sensor)
 * 1.3 - Implement PID motor control
 * 1.4 - 500ms Timer2 interrupt to average wheel encoder counts.
 * 3.0 - Move to Cytron Driver board (serial) and steering encoder
 * 3.1 - Brake on Startup (Safety Feature)
 * 3.2 - Soft Acceleration
 * 4.1 - 4-line LCD, improved soft accelration, changed potentiometer values
 * 4.2 - Brake paddle, reverse w/ brake
 * 
 * Notes: Differential steering control using two BTS7960 motor drivers
 *        driving two 250W Scooter motors.
 * 
 * Differential Steering Library: https://github.com/edumardo/DifferentialSteering
 * LCD Library: https://github.com/mrkaleArduinoLib/LiquidCrystal_I2C
 * PID Lbirary: https://github.com/imax9000/Arduino-PID-Library
 */

// SmartDriveDuo-60 Simplied Serial Mode (115200 baud) DIP SW SETTINGS:
// 11011100

/*
 * Timer0 - used for millis() micros() delay()… and is on pin 5, 6
 * Timer1 - 16bit timer is on pin 9, 10
 * Timer2 - 8bit timer is on pin 3, 11
 * 
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//#include "QuickPID.h"
//#include <MsTimer2.h>
//#include "DifferentialSteering.h"
#include <Encoder.h>

//#define PIDCONT

// PWM PINS on NANO: 3,5,6,9,10,11
// INTERRUPT PINS ON NANO: 2,3

// STEERING SENSOR (Rotary Encoder)
const uint8_t STEER_A = 7;
const uint8_t STEER_B = 8;
const uint8_t STEER_SW = 4;

//Pedal sensor pin
const uint8_t PEDAL = A0;

//turbo pin
const uint8_t TURBO = 9;

//power switch
const uint8_t POWER = 6;

//direction switch
const uint8_t DIRSW = 5;

//brake light
const uint8_t BRAKELIGHT = 10;

const uint8_t MAXIMUM = 40;

// differential steering
//const int brakeThreshold = 0;
//const int fPivYLimit = 32;  // 0 - 127 (A greater value will assign more of the joystick's range to pivoting) - default 32
//DifferentialSteering DiffSteer;

// instantiate our objects
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 4 line display
Encoder steerEncoder(STEER_A, STEER_B);

const unsigned int sampleTime = 256;  // LCD UPDATE INTERVAL
const unsigned int accelTime = 500;   // ACCEL UPDATE INTERVAL
unsigned int MAX_POWER = 32;
unsigned long lastMillis = 0;
unsigned long lastMillisAccel = 0;
volatile uint16_t avgLeft = 0;
volatile uint16_t avgRight = 0;
int pedalPos;
int desiredPedalPos = 0;

void coast() {
 if (desiredPedalPos == 0 && pedalPos > 0) {
    if (desiredPedalPos > pedalPos) {
      pedalPos = desiredPedalPos;
  } else {
    int antidiff = floor( ((float)(desiredPedalPos + pedalPos))*(0.2f) );

    //if (pedalPos > 26) {
    //  diff = 1; // slow down acceleration at top speed.
    //}
    pedalPos -= antidiff;
    if (pedalPos < desiredPedalPos) {
      pedalPos = desiredPedalPos;
    }
    // check against max power
    //if (pedalPos > MAX_POWER) {
    //  pedalPos = MAX_POWER;
    //}
  }
 }
}


// Update Acceleration
void updateAccel()
{
  if (desiredPedalPos < pedalPos) {
    pedalPos = desiredPedalPos;
  } else {
    int diff = ceil( ((float)(desiredPedalPos - pedalPos))*(0.2f) );

    if (pedalPos > 26) {
      diff = 1; // slow down acceleration at top speed.
    }
    pedalPos += diff;
    if (pedalPos > desiredPedalPos) {
      pedalPos = desiredPedalPos;
    }
    // check against max power
    if (pedalPos > MAX_POWER) {
      pedalPos = MAX_POWER;
    }
  }
}

// Run Once - Setup
void setup()
{
  Serial.begin(115200);
  // setup pins for speed sensors
  //pinMode(R_SEN, INPUT);
  //pinMode(L_SEN, INPUT);

  // pin for rotary switch
  pinMode(STEER_SW, INPUT_PULLUP);

  // setup pin for analog accelerator pedal
  pinMode(PEDAL, INPUT);
  pinMode(TURBO, INPUT_PULLUP);

  // setup pin for Power Switch
  pinMode(POWER, INPUT_PULLUP);

  pinMode(DIRSW, INPUT_PULLUP);
  pinMode(BRAKELIGHT, OUTPUT);

  // set initial speed to 0 for motor driver
  Serial.write(0b01000000);
  Serial.write(0b11000000);
  pedalPos = 0;
  desiredPedalPos = 0;
  
  lcd.init();   // initialize the lcd 
  lcd.backlight();
  lcd.clear();
  lcd.print("GoKart v4.1");
  lcd.setCursor(0,1);
  lcd.print("Lucas Ruebsamen");
  delay(1000);
  lcd.clear();
  //DiffSteer.begin(fPivYLimit);
}

long positionSteering = -999;
long lastPosition = 0;
bool turboMode = true;
bool powerSwitch = false;
uint8_t direction = 1;  // current direction -- 1 is forward, 0 is reverse
uint8_t dirSwitch = 1;  // direction switch

// 0 = 0v to 1023 = 5v
// we need 0 - 127 
void loop()
{
  char sVal[5];

  // read steering encoder
  positionSteering = steerEncoder.read();

  if (positionSteering > lastPosition) {
    MAX_POWER++;
    lastPosition = positionSteering;
    if (MAX_POWER >= MAXIMUM  ) {
      MAX_POWER = MAXIMUM;
    }
  
  } else if (positionSteering < lastPosition) {
    MAX_POWER--;
    lastPosition = positionSteering;
    if (MAX_POWER <= 7) {
      MAX_POWER = 7;
    }
    
  }
  
  // reset encoder on button push
  if(digitalRead(STEER_SW) == LOW) {
    steerEncoder.write(0);
    MAX_POWER = 32;
  }

  // read accelerator pedal (185 - 855)
  // Note: MDC60 Cytron Driver - Serial mode takes 6 bit speed input (0 - 63)
  desiredPedalPos = analogRead(PEDAL);
  //desiredPedalPos = map(desiredPedalPos, 28, 63, 0, 63);

  // brake enabled
  if (desiredPedalPos <= 2) {
    desiredPedalPos = 0;
    digitalWrite(BRAKELIGHT, HIGH);
    direction = dirSwitch;
  } else {
    digitalWrite(BRAKELIGHT, LOW);
  }

  if (desiredPedalPos >= 63) {
    desiredPedalPos = 63;
  }
  
  //read dir switch
  if (digitalRead(DIRSW) == LOW) {
    dirSwitch = 1;  // forward
  } else {
    dirSwitch = 0; // reverse
  }
  
  // read turbo switch
  if (digitalRead(TURBO) == LOW) { 
    turboMode = true;
    MAX_POWER = 63;
  } else {
    //desiredPedalPos = floor(desiredPedalPos * 0.65);
    turboMode = false;
  }

  if (digitalRead(POWER) == LOW) {
    powerSwitch = false;
  } else {
    powerSwitch = true;
  }

  // if we want to go slower, immediately reduce speed
  if (desiredPedalPos < pedalPos) {
    pedalPos = desiredPedalPos;
  }
  
  // take pedal position and steering position and compute motor output

  // compute left motor
  unsigned char leftVal = pedalPos & 0b00111111;
  leftVal = (leftVal | 0b00000000) | (direction << 6);
  // compute right motor
  unsigned char rightVal = pedalPos & 0b00111111;
  rightVal = (rightVal | 0b10000000) | (direction << 6);

  // set motor direction
  //if (direction) {
  //  leftVal = leftVal | (1 << 6);
  //  rightVal = rightVal | (1 << 6);
  //}

  // Write serial data to motor driver
  if (powerSwitch) {
    // power is on, control motors
    Serial.write(leftVal);
    Serial.write(rightVal);
  } else {
    // power is off, motors disabled
    Serial.write(0b01000000);
    Serial.write(0b11000000);
  }

  // HANDLE ACCELERATION UPDATES HERE
  if (millis() - lastMillisAccel >= accelTime) {
    updateAccel();
    coast();
    lastMillisAccel = millis();
  }

  // HANDLE LCD UPDATES HERE
  if (millis() - lastMillis >= sampleTime) {
    // 500ms has elapsed since last lcd update, time to update again
    lcd.setCursor(0,0); // col 0, row 0
    if (powerSwitch) {
      lcd.print("S: ");
      sprintf(sVal, "%+04d", (int)MAX_POWER);
      lcd.printstr(sVal);
  
      lcd.setCursor(8,0); // col 0, row 1
      lcd.print("P: ");
  
      if (turboMode == true) {
        sprintf(sVal,"%+04d*", (int)pedalPos);
      }else {
        sprintf(sVal,"%+04d ", (int)pedalPos);
      }
      lcd.printstr(sVal);
  
      lcd.setCursor(0,1); // col 0, row 1
      lcd.print("D: ");
      //lcd.print(xValue);
      //sprintf(sVal,"%+04d", (int)pedalPos);
      if (direction == 1) {
        sprintf(sVal,"rev");
      } else {
        sprintf(sVal, "for");
      }
      
      lcd.printstr(sVal);
  
      lcd.setCursor(8,1);   // col 8, row 1
      lcd.print("V: ");
      sprintf(sVal,"%+04d", (int)desiredPedalPos);
      lcd.printstr(sVal);
    } else {
      lcd.print("Pow     Off");
    }
    lastMillis = millis();
  }
}
