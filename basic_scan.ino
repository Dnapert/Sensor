#include <Arduino.h>
#include "BasicStepperDriver.h"
#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>

LIDARLite_v3HP myLidarLite;

#define FAST_I2C
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 10

bool complete_flag = false;
bool old_flag = false;
int set_scan_x = 40;
int sweep_counter = 1;
float degrees_per_step = .1125;
float deg = .9;
float sweep_horizontal = 360;
float sweep_vertical = 210;
float maxPosY=220;
int CurrentYPosition;
int CurrentXPosition;

float positionX = CurrentXPosition * deg;
float positionY = CurrentYPosition * deg;
// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 16


#define DIR 5
#define STEP 6


// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper1(MOTOR_STEPS, DIR, STEP);
BasicStepperDriver stepper2(200, 7, 8);

void setup() {
  // Initialize Arduino serial port (for display of ASCII output to PC)
  Serial.begin(115200);

  // Initialize Arduino I2C (for communication to LidarLite)
  Wire.begin();
#ifdef FAST_I2C
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
#endif

  // Configure the LidarLite internal parameters so as to lend itself to
  // various modes of operation by altering 'configure' input integer to
  // anything in the range of 0 to 5. See LIDARLite_v3HP.cpp for details.
  myLidarLite.configure(0);
  delay(1000);
  stepper1.begin(RPM, MICROSTEPS);
  stepper1.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 100, 100);
  stepper2.begin(RPM, MICROSTEPS);
  Serial.begin(115200);
}

void Scan()
{
  uint16_t distance;
  uint8_t  newDistance = 0;
  if (sweep_counter  <= sweep_vertical) {



  { if (CurrentXPosition <= sweep_horizontal) {

        for (CurrentXPosition = 0; CurrentXPosition <= sweep_horizontal; CurrentXPosition++) {
          stepper1.move(8);
          myLidarLite.waitForBusy();
          myLidarLite.takeRange();
          myLidarLite.waitForBusy();
          distance = myLidarLite.readDistance();
          Serial.println(CurrentXPosition * deg);
          Serial.print(distance);

        }
      } else  {
        CurrentYPosition++;
        stepper2.move(8);
        myLidarLite.waitForBusy();
        myLidarLite.takeRange();
        myLidarLite.waitForBusy();
        distance = myLidarLite.readDistance();
        Serial.println(CurrentYPosition * deg);
        Serial.print(distance);
        sweep_counter++;

        for (CurrentXPosition = sweep_horizontal; CurrentXPosition >= 0; CurrentXPosition--) {
          stepper1.move(-8);
          myLidarLite.waitForBusy();
          myLidarLite.takeRange();
          myLidarLite.waitForBusy();
          distance = myLidarLite.readDistance();
          Serial.println(CurrentXPosition * deg);
          Serial.print(distance);

          if (CurrentXPosition == 0) {
            CurrentYPosition++;
            stepper2.move(8);
            myLidarLite.waitForBusy();
            myLidarLite.takeRange();
            myLidarLite.waitForBusy();
            distance = myLidarLite.readDistance();
            Serial.println(("Y ") + String(CurrentYPosition * degrees_per_step) + String(" Pass" ) + sweep_counter);
            sweep_counter++;
            Serial.print(distance);
          }
        }
      }
    }
  }
  else {
    complete_flag = true;
    stepper1.disable();
    stepper2.disable();

    if (complete_flag != old_flag) {
      Serial.println("Scan Complete");
    }
    old_flag = complete_flag;
    return;
  }
}
/* Math for coordinate datas
radius = lidar.distance();
float azimuth = CurrentXPosition * deg2rad;
    float elevation = (180 - maxPosY + CurrentYPosition) * deg2rad;
    double x = radius * sin(elevation) * cos(azimuth);
    double y = radius * sin(elevation) * sin(azimuth);
    double z = radius * cos(elevation);