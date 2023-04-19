/*
 * Author: Lucas Ruebsamen
 * Date: December 28, 2021
 * 
 * Version 1.1 - Updated Jan 14, 2022
 * Version 1.4 - Updated Feb 9, 2022
 * Version 3.0 - Updated March 16, 2022
 * Version 3.1 - Updated April 20, 2022
 * Version 4.0 - Updated Feb 11, 2023
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
 * 4.0 - MD30C Motor Driver Support
 *
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
#include "CytronMotorDriver.h"
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

// Configure the motor driver.
CytronMD motor(PWM_DIR, 11, 12);  // PWM = Pin 11, DIR = Pin 12.

const unsigned int sampleTime = 256;  // LCD UPDATE INTERVAL
const unsigned int accelTime = 500;   // ACCEL UPDATE INTERVAL
unsigned int MAX_POWER = 255;
unsigned long lastMillis = 0;
unsigned long lastMillisAccel = 0;
volatile uint16_t avgLeft = 0;
volatile uint16_t avgRight = 0;
int pedalPos;
int desiredPedalPos = 0;
bool powerSwitch = false;

// Update Acceleration
void updateAccel()
{
  if (powerSwitch) {
    if (desiredPedalPos < pedalPos) {
      pedalPos = desiredPedalPos;
    } else {
      int diff = ceil( ((float)(desiredPedalPos - pedalPos))*(0.2f) );
  
      if (pedalPos > 200) {
        diff = 4; // slow down acceleration at top speed.
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
  } else {
    //power off
    pedalPos = 0;
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
  motor.setSpeed(0);
  pedalPos = 0;
  desiredPedalPos = 0;
  
  lcd.init();   // initialize the lcd 
  lcd.backlight();
  lcd.clear();
  lcd.print("GoKart v4.0");
  lcd.setCursor(0,1);
  lcd.print("Lucas Ruebsamen");
  delay(1000);
  lcd.clear();
  //DiffSteer.begin(fPivYLimit);
}

long positionSteering = -999;
long lastPosition = 0;
bool turboMode = true;

uint8_t direction = 0;  // current direction -- 0 is forward, 1 is reverse
uint8_t dirSwitch = 0;  // direction switch

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
  // Note: Cytron Driver - PWM mode takes speed input (0 - 255)
  desiredPedalPos = analogRead(PEDAL);
  desiredPedalPos = map(desiredPedalPos, 187, 850, 0, 255);

  // brake enabled
  if (desiredPedalPos <= 10) {
    desiredPedalPos = 0;
    digitalWrite(BRAKELIGHT, HIGH);
    direction = dirSwitch;
  } else {
    digitalWrite(BRAKELIGHT, LOW);
  }

  // clamp to max
  if (desiredPedalPos >= 255) {
    desiredPedalPos = 255;
  }
  
  //read dir switch
  if (digitalRead(DIRSW) == LOW) {
    dirSwitch = 1;  // forward
    
  } else {
    dirSwitch = 0; // reverse
  }
  
  // read turbo switch
 /*
  if (digitalRead(TURBO) == LOW) { 
    turboMode = true;
    MAX_POWER = 255;
  } else {
    //desiredPedalPos = floor(desiredPedalPos * 0.65);
    turboMode = false;
  }
*/
  if (digitalRead(POWER) == LOW) {
    powerSwitch = true;
  } else {
    powerSwitch = false;
  }

  // if we want to go slower, immediately reduce speed
  if (desiredPedalPos < pedalPos) {
    pedalPos = desiredPedalPos;
  }
  
  // take pedal position and steering position and compute motor output
  if (direction == 1) {
    motor.setSpeed(pedalPos);
  } else {
    motor.setSpeed(-pedalPos);
  }

  // HANDLE ACCELERATION UPDATES HERE
  if (millis() - lastMillisAccel >= accelTime) {
    updateAccel();
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
      //sprintf(sVal,"%+04d", (int)avgLeft);
      if (direction == 1) {
        sprintf(sVal,"for");
      } else {
        sprintf(sVal, "rev");
      }
      
      lcd.printstr(sVal);
  
      lcd.setCursor(8,1);   // col 8, row 1
      lcd.print("R: ");
      sprintf(sVal,"%+04d", (int)avgRight);
      lcd.printstr(sVal);
      lcd.setCursor(0,2);
      lcd.print("...Problems?");
      lcd.setCursor(0,3);
      lcd.print("Call Lucas Ruebsamen");
    } else {
      lcd.print("Pow     Off");
    }
    lastMillis = millis();
  }
}
