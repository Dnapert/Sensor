#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include "BasicStepperDriver.h"
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define LOGO_HEIGHT 60
#define LOGO_WIDTH   51
#define MOTOR_STEPS 200
#define RPM 10
#define MICROSTEPS 16


#define DIR 5
#define STEP 6
static const unsigned char PROGMEM logo_bmp[] =
{ 0xff, 0xfe, 0xb9, 0xff, 0xff, 0xff, 0xe0,
  0xff, 0x51, 0xd6, 0x1f, 0xff, 0xff, 0xe0,
  0xfc, 0xf7, 0xff, 0xcc, 0x3f, 0xff, 0xe0,
  0xe5, 0xff, 0x8f, 0xc3, 0x3f, 0xff, 0xe0,
  0xdf, 0xe1, 0x00, 0x7e, 0x1f, 0xff, 0xe0,
  0xbf, 0x80, 0x00, 0x00, 0x0f, 0xff, 0xe0,
  0xbc, 0x00, 0x00, 0x00, 0x2f, 0xff, 0xe0,
  0x40, 0x00, 0xa8, 0x40, 0x8f, 0xff, 0xe0,
  0x40, 0x42, 0x00, 0x04, 0x2f, 0xff, 0xe0,
  0x00, 0x10, 0x05, 0x20, 0x9f, 0xff, 0xe0,
  0x80, 0xbf, 0xff, 0xff, 0x9f, 0xff, 0xe0,
  0x20, 0x7f, 0xff, 0xff, 0x95, 0x57, 0xe0,
  0xc2, 0xff, 0xff, 0xff, 0x82, 0x83, 0xe0,
  0x22, 0x22, 0x54, 0xaa, 0x9d, 0x79, 0xe0,
  0xa0, 0x00, 0x01, 0x55, 0xa0, 0x1d, 0xe0,
  0xc4, 0x89, 0x20, 0x02, 0x41, 0x0c, 0xe0,
  0xf0, 0x25, 0x56, 0xbe, 0x00, 0x0c, 0xe0,
  0xf5, 0xff, 0xff, 0xfe, 0x47, 0xc6, 0xe0,
  0xf5, 0xff, 0xff, 0xfe, 0x4f, 0xce, 0x60,
  0xf1, 0xff, 0xff, 0xfe, 0x5f, 0xe6, 0x60,
  0xf5, 0xff, 0xff, 0xfe, 0x1f, 0xe7, 0x60,
  0xf5, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0x60,
  0xf1, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0x60,
  0xf5, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0x20,
  0xf5, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0x60,
  0xf1, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0x20,
  0xf5, 0xff, 0xff, 0xfe, 0x1f, 0xf7, 0x20,
  0xf1, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0xa0,
  0xf5, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0x20,
  0xf1, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0xa0,
  0xf5, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0xa0,
  0xf5, 0xff, 0xff, 0xfe, 0x1f, 0xf7, 0xa0,
  0xf1, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0x20,
  0xfb, 0xff, 0xff, 0xfe, 0x5f, 0xf7, 0xa0,
  0xf1, 0xff, 0xff, 0xfe, 0x5f, 0xe7, 0x60,
  0xf9, 0xff, 0xff, 0xfe, 0x5f, 0xeb, 0x60,
  0xf3, 0xff, 0xff, 0xfe, 0x1f, 0xeb, 0x60,
  0xf9, 0xff, 0xff, 0xfe, 0x5f, 0xca, 0x60,
  0xf1, 0xff, 0xff, 0xfe, 0x4f, 0xd2, 0xe0,
  0xfb, 0xff, 0xff, 0xfe, 0x03, 0x34, 0xe0,
  0xf1, 0xff, 0xff, 0xfe, 0x40, 0x45, 0xe0,
  0xf9, 0xff, 0xff, 0xfe, 0x0f, 0x8b, 0xe0,
  0xf9, 0xff, 0xff, 0xfe, 0x50, 0x13, 0xe0,
  0xf3, 0xff, 0xff, 0xfe, 0x40, 0x27, 0xe0,
  0xf9, 0xff, 0xff, 0xfc, 0x00, 0x1f, 0xe0,
  0xf8, 0xff, 0xff, 0xe0, 0x40, 0xff, 0xe0,
  0xfa, 0x1f, 0xff, 0x01, 0x8f, 0xff, 0xe0,
  0xf9, 0x00, 0x00, 0x0e, 0x1f, 0xff, 0xe0,
  0xf0, 0x72, 0x00, 0xf0, 0x1f, 0xff, 0xe0,
  0xf8, 0x05, 0xfe, 0x00, 0x1f, 0xff, 0xe0,
  0xf3, 0x00, 0x00, 0x01, 0x5f, 0xff, 0xe0,
  0xf4, 0xf0, 0x00, 0x3e, 0x2f, 0xff, 0xe0,
  0xf0, 0x07, 0xff, 0xe0, 0x07, 0xff, 0x60,
  0xe0, 0x00, 0x00, 0x00, 0x07, 0xff, 0x00,
  0xe0, 0x14, 0x05, 0x00, 0x07, 0xff, 0x00,
  0xf2, 0x00, 0x00, 0x50, 0x0f, 0xff, 0xc0,
  0xf0, 0x07, 0xff, 0xe8, 0x1f, 0xff, 0xc0,
  0xfc, 0x00, 0x00, 0x00, 0xff, 0xff, 0x80,
  0xff, 0xa0, 0x00, 0x1f, 0xff, 0xff, 0x80,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80
};




static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
unsigned int encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
// Button reading, including debounce without delay function declarations
const byte buttonPin = 4; // this is the Arduino pin we are connecting the push button to
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
const unsigned long debounceTime = 10;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state
bool buttonPressed = 0; // a flag variable
// Menu and submenu/setting declarations
byte Mode = 0;   // This is which menu mode we are in at any given time (top level or one of the submenus)
const byte modeMax = 4; // This is the number of submenus/settings
int set_scan_x = 1;  // a variable which holds the value we set
int set_scan_y = 1;  // a variable which holds the value we set
float set_resolution = 8;
bool scan_flag = 0;
bool complete_flag = false;
bool old_flag = false;
int sweep_counter = 1;

float deg = set_resolution * .1125;
float sweep_horizontal = (set_scan_x * 9) / deg; // number of loops for scan()x
float sweep_vertical = (set_scan_y * 9) / deg; //number of loops for scan()y
float CurrentYPosition;
float CurrentXPosition;

float positionX = CurrentXPosition * deg;
float positionY = CurrentYPosition * deg;
/* Note: you may wish to change settingN etc to int, float or boolean to suit your application.
  Remember to change "void setAdmin(byte name,*BYTE* setting)" to match and probably add some
  "modeMax"-type overflow code in the "if(Mode == N && buttonPressed)" section*/
BasicStepperDriver stepper1(MOTOR_STEPS, DIR, STEP);
BasicStepperDriver stepper2(200, 7, 8);
void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  testdrawbitmap();
  Title();
  Menu();
  stepper1.begin(RPM, MICROSTEPS);
  stepper1.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 100, 100);
  stepper2.begin(RPM, MICROSTEPS);
  //Rotary encoder section of setup
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  // button section of setup
  pinMode (buttonPin, INPUT_PULLUP); // setup the button pin
  // DEBUGGING section of setup
  Serial.begin(9600);     // DEBUGGING: opens serial port, sets data rate to 9600 bps
}

void loop() {
  if (scan_flag) {
    Scan();
  } else {
    rotaryMenu();
    // carry out other loop code here
  }
}
  void rotaryMenu() { //This handles the bulk of the menu functions without needing to install/include/compile a menu library
    //DEBUGGING: Rotary encoder update display if turned
    if (oldEncPos != encoderPos && Mode != 0) {
      switch (Mode) {
        case 1:
          
          
          display.clearDisplay();
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(0, 0); //  Horizontal,Vertical
          display.println("Set Scan");
          display.println("Width");
          display.setCursor(24, 48);
          display.println(encoderPos * 9);
          display.display();
          break;
        case 2:
          
          display.clearDisplay();
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(0, 0); //  Horizontal,Vertical
          display.println("Set Scan");
          display.println("Height");
          display.setCursor(24, 48);
          display.println(encoderPos * 9);
          display.display();
          break;
        case 3:
        
         
          display.clearDisplay();
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(0, 0); //  Horizontal,Vertical
          display.println("Resolution");
          display.setCursor(24, 48);
          display.println(encoderPos * .1125);
          display.display();
          break;
        case 4:
          
         
          display.clearDisplay();
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(0, 32); //  Horizontal,Vertical
          display.println("Scanning!");
          display.display();
          break;
      }



      oldEncPos = encoderPos;

    }

    // Button reading with non-delay() debounce - thank you Nick Gammon!
    byte buttonState = digitalRead (buttonPin);
    if (buttonState != oldButtonState) {
      if (millis () - buttonPressTime >= debounceTime) { // debounce
        buttonPressTime = millis ();  // when we closed the switch
        oldButtonState =  buttonState;  // remember for next time
        if (buttonState == LOW) {
          buttonPressed = 1;
        }
        else {
          buttonPressed = 0;
        }
      }  // end if debounce time up
    } // end of state change

    //Main menu section
    if (Mode == 0) {
      if (encoderPos > (modeMax + 10)) encoderPos = modeMax; // check we haven't gone out of bounds below 0 and correct if we have
      else if (encoderPos > modeMax) encoderPos = 0;
      switch (encoderPos) {
        case 0: Menu();
          break;
        case 1: Menu1();
          break;
        case 2: Menu2();
          break;
        case 3: Menu3();
          break;
        case 4: Menu4();
      }
      if (buttonPressed) {
        Mode = encoderPos; // set the Mode to the current value of input if button has been pressed
        buttonPressed = 0; // reset the button status so one press results in one action
        if (Mode == 1) {
          encoderPos = set_scan_x;
          if (encoderPos > 40) {
            encoderPos = 40;
          }


          oldEncPos = encoderPos * 9;
        }
        if (Mode == 2) {
          encoderPos = set_scan_y;
          if (encoderPos > 40) {
            encoderPos = 40;
          }


          oldEncPos = encoderPos * 9;
        }
        if (Mode == 3) {

          encoderPos = set_resolution;
          if (encoderPos > 16 ) {
            encoderPos = 16;
          }


          oldEncPos = encoderPos;
        }
        if (Mode == 4 && buttonPressed) {

          

        }
      }

    }
    if (Mode == 1 && buttonPressed) {
      set_scan_x = encoderPos; // record whatever value your encoder has been turned to, to setting 1
      setAdmin(1, set_scan_x);
      //code to do other things with setting1 here, perhaps update display
    }
    if (Mode == 2 && buttonPressed) {
      set_scan_y = encoderPos; // record whatever value your encoder has been turned to, to setting 2
      setAdmin(2, set_scan_y);
      //code to do other things with setting2 here, perhaps update display
    }
    if (Mode == 3 && buttonPressed) {
      set_resolution = encoderPos; // record whatever value your encoder has been turned to, to setting 3
      setAdmin(3, set_resolution);
      //code to do other things with setting3 here, perhaps update display
    }
    if (Mode == 4 && buttonPressed) {
    scan_flag=!scan_flag;
    }
  }

  // Carry out common activities each time a setting is changed
  void setAdmin(int name, int setting) {
    encoderPos = 0; // reorientate the menu index - optional as we have overflow check code elsewhere
    buttonPressed = 0; // reset the button status so one press results in one action
    Mode = 0; // go back to top level of menu, now that we've set values
    Menu();
  }

  //Rotary encoder interrupt service routine for one encoder pin
  void PinA() {
    cli(); //stop interrupts happening before we read pin values
    reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
    if (reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
      encoderPos --; //decrement the encoder's position count
      bFlag = 0; //reset flags for the next turn
      aFlag = 0; //reset flags for the next turn
    }
    else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
    sei(); //restart interrupts
  }

  //Rotary encoder interrupt service routine for the other encoder pin
  void PinB() {
    cli(); //stop interrupts happening before we read pin values
    reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
    if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
      encoderPos ++; //increment the encoder's position count
      bFlag = 0; //reset flags for the next turn
      aFlag = 0; //reset flags for the next turn
    }
    else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
    sei(); //restart interrupts
  }
  void testdrawbitmap(void) {
    display.clearDisplay();

    display.drawBitmap(
      (display.width()  - LOGO_WIDTH ) / 2,
      (display.height() - LOGO_HEIGHT) / 2,
      logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
    display.display();
    delay(1500);
  }

  void Title(void) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(3, 32); //  Horizontal,Vertical
    display.println("DrunkWerks");
    display.display();
    delay(1000);
  }
  void Menu() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0); //  Horizontal,Vertical
    display.println("Main");
    display.println("Set X");
    display.println("Set Y");
    display.println("Res.");
    display.display();


  }
  void Menu1() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0); //  Horizontal,Vertical
    display.println("Main");
    display.println("Set X <");
    display.println("Set Y");
    display.println("Res.");
    display.display();


  }
  void Menu2() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0); //  Horizontal,Vertical
    display.println("Main");
    display.println("Set X ");
    display.println("Set Y<");
    display.println("Res.");
    display.display();


  }
  void Menu3() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0); //  Horizontal,Vertical
    display.println("Main");
    display.println("Set X");
    display.println("Set Y");
    display.println("Res.<");
    display.display();


  }
  void Menu4() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0); //  Horizontal,Vertical
    display.println("Set X");
    display.println("Set Y");
    display.println("Res.");
    display.println("Scan<");
    display.display();


  }
  void Scan()
{ if (sweep_counter  <= sweep_vertical) {



    { if (CurrentXPosition <= sweep_horizontal) {

          for (CurrentXPosition = 0; CurrentXPosition <= sweep_horizontal; CurrentXPosition++) {
            stepper1.move(8);
            delay(15);
            Serial.println(CurrentXPosition * deg);

          }
        } else  {
          sweep_counter++;
        CurrentYPosition++;
          stepper2.move(8);
          delay (15);
          Serial.print("Y");
          Serial.println( CurrentYPosition*deg);
         
          

          for (CurrentXPosition = sweep_horizontal; CurrentXPosition >= 0; CurrentXPosition--) {
            stepper1.move(-8);
            delay(15);
            Serial.println(CurrentXPosition * deg);

            if (CurrentXPosition == 0) {
              sweep_counter++;
              CurrentYPosition++;
              stepper2.move(8);
              delay(15);
              Serial.print("Y");
              Serial.println( CurrentYPosition*deg);
              
              
            }
          }
        }
      }
    }
    else {
      scan_flag = true;
      stepper1.disable();
      stepper2.disable();

      if (scan_flag != old_flag) {
        Serial.println("Scan Complete");
        scan_flag=false;
        rotaryMenu();
      }
      
    }
  }
