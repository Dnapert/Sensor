#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 10
bool complete_flag = false;
bool old_flag = false;
int set_sweep_horizontal = 36;
int sweep_counter = 1;
float degrees_per_step = .9;
float sweep_horizontal = set_sweep_horizontal / degrees_per_step;
float set_sweep_vertical = 9;
float sweep_vertical = set_sweep_vertical / degrees_per_step;
float CurrentYPosition;
float CurrentXPosition;

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
  delay(1000);
  stepper1.begin(RPM, MICROSTEPS);
  stepper1.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 100, 100);
  stepper2.begin(RPM, MICROSTEPS);
  Serial.begin(115200);
}

void loop()
{ if (sweep_counter * degrees_per_step <= set_sweep_vertical) {



  { if (CurrentXPosition <= sweep_horizontal) {

        for (CurrentXPosition = 0; CurrentXPosition <= sweep_horizontal; CurrentXPosition++) {
          stepper1.move(8);
          delay(15);
          Serial.println(CurrentXPosition * degrees_per_step);

        }
      } else  {
        CurrentYPosition++;
        stepper2.move(8);
        Serial.println(("Y ") + String(CurrentYPosition * degrees_per_step) + String(" Pass ") + sweep_counter);
        sweep_counter++;

        for (CurrentXPosition = sweep_horizontal; CurrentXPosition >= 0; CurrentXPosition--) {
          stepper1.move(-8);
          delay(15);
          Serial.println(CurrentXPosition * degrees_per_step);

          if (CurrentXPosition == 0) {
            CurrentYPosition++;
            stepper2.move(8);
            Serial.println(("Y ") + String(CurrentYPosition * degrees_per_step) + String(" Pass" ) + sweep_counter);
            sweep_counter++;
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