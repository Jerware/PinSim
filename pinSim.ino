/*
    PinSim Controller v20200517
    Controller for PC Pinball games
    https://www.youtube.com/watch?v=18EcIxywXHg
    
    Based on the excellent MSF_FightStick XINPUT project by Zack "Reaper" Littell
    https://github.com/zlittell/MSF-XINPUT
    
    Uses the Teensy-LC

    IMPORTANT PLUNGER NOTE:
    You MUST calibrate the plunger range at least once by holding down "A"
    when plugging in the USB cable. LED-1 should flash rapidly, and then you should
    pull the plunger all the way out and release it all the way back in. The LED1 should
    flash again, and normal operation resumes. The setting is saved between power cycles.

    If you're not using a plunger, ground Pin 15.
*/

//Includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Bounce.h>
#include "xinput.h"
#include <Average.h>
#include <EEPROMex.h>

int numSamples = 20;
Average<int> ave(numSamples);

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// GLOBAL VARIABLES
// configure these
boolean flipperL1R1 = true; //
boolean fourFlipperButtons = false; // FLIP_L & FLIP_R map to L1/R1 and pins 13 & 14 map to analog L2/R2 100%
boolean doubleContactFlippers = false; // FLIP_L & FLIP_R map to analog L2/R2 10% and pins 13 & 14 map to L2/R2 100% 
boolean analogFlippers = false; // use analog flipper buttons
boolean leftStickJoy = false; // joystick moves left analog stick instead of D-pad
boolean accelerometerEnabled = true;
boolean accelerometerCalibrated = false;
boolean plungerEnabled = true;
boolean currentlyPlunging = false;
int16_t nudgeMultiplier = 9000; // accelerometer multiplier (higher = more sensitive)
int16_t plungeTrigger = 60; // threshold to trigger a plunge (lower = more sensitive)
int16_t fourButtonModeThreshold = 250; // ms that pins 13/14 need to close WITHOUT FLIP_L/FLIP_R closing to trigger four flipper button mode.

// probably leave these alone
long fourButtonModeTriggeredLB = 0; // these two vars are used to check for 4 flipper buttons
long fourButtonModeTriggeredRB = 0;
int16_t zeroValue = 0; // xbox 360 value for neutral analog stick
int16_t zeroValueBuffer = 0; // save zero value during plunge
int16_t plungerReportDelay = 17; // delay in ms between reading ~60hz plunger updates from sensor
int16_t plungerMin = 200; // min plunger analog sensor value
int16_t plungerMax = 550; // max plunger analog sensor value
int16_t plungerMaxDistance = 0; // sensor value converted to actual distance
int16_t plungerMinDistance = 0;
uint32_t plungerReportTime = 0;
uint32_t tiltEnableTime = 0;
int16_t lastReading = 0;
int16_t lastDistance = 0;
int16_t distanceBuffer = 0;
float zeroX = 0;
float zeroY = 0;

////Pin Declarations
#define pinDpadL 0  //Left on DPAD
#define pinDpadR 1  //Right on DPAD
#define pinDpadU 2  //Up on DPAD
#define pinDpadD 3  //Down on DPAD
#define pinB1 4  //Button 1 (A) 
#define pinB2 5  //Button 2 (B) 
#define pinB3 6  //Button 3 (X) 
#define pinB4 7  //Button 4 (Y) 
#define pinLB 8  //Button 5 (LB)
#define pinRB 9  //Button 6 (RB) 
#define pinXB 10  //XBOX Guide Button
#define pinBK 11  //Button 7 (Back)
#define pinST 12  //Button 8 (Start)
#define pinLT 13  //Left Analog Trigger
#define pinRT 14  //Right Analog Trigger
#define pinPlunger 15 //IR distance for plunger
#define pinLED1 16  //Onboard LED 1
#define pinLED2 17  //Onboard LED 2
#define rumbleSmall 20 // Large Rumble Motor
#define rumbleLarge 22 // Large Rumble Motor
#define pinB9 21 //Button 9 (L3)
#define pinB10 23 //Button 10 (R3)

#define NUMBUTTONS 17  //Number of all buttons
#define MILLIDEBOUNCE 20  //Debounce time in milliseconds

//Position of a button in the button status array
#define POSUP 0
#define POSDN 1
#define POSLT 2
#define POSRT 3
#define POSB1 4
#define POSB2 5
#define POSB3 6
#define POSB4 7
#define POSL1 8
#define POSR1 9
#define POSL2 10
#define POSR2 11
#define POSST 12
#define POSBK 13
#define POSXB 14
#define POSB9 15
#define POSB10 16

// Dual flippers
uint8_t leftTrigger = 0;
uint8_t rightTrigger = 0;

//Global Variables
byte buttonStatus[NUMBUTTONS];  //array Holds a "Snapshot" of the button status to parse and manipulate

//LED Toggle Tracking Global Variables
uint8_t LEDState = LOW;	//used to set the pin for the LED
uint32_t previousMS = 0; //used to store the last time LED was updated
uint8_t LEDtracker = 0;	//used as an index to step through a pattern on interval

//LED Patterns
uint8_t patternAllOff[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t patternBlinkRotate[10] = {1,0,1,0,1,0,1,0,1,0};
uint8_t patternPlayer1[10] = {1,0,0,0,0,0,0,0,0,0};
uint8_t patternPlayer2[10] = {1,0,1,0,0,0,0,0,0,0};
uint8_t patternPlayer3[10] = {1,0,1,0,1,0,0,0,0,0};
uint8_t patternPlayer4[10] = {1,0,1,0,1,0,1,0,0,0};

//Variable to hold the current pattern selected by the host
uint8_t patternCurrent[10] = {0,0,0,0,0,0,0,0,0,0};

//Setup Button Debouncing
Bounce dpadUP = Bounce(pinDpadU, MILLIDEBOUNCE);
Bounce dpadDOWN = Bounce(pinDpadD, MILLIDEBOUNCE);
Bounce dpadLEFT = Bounce(pinDpadL, MILLIDEBOUNCE);
Bounce dpadRIGHT = Bounce(pinDpadR, MILLIDEBOUNCE);
Bounce button1 = Bounce(pinB1, MILLIDEBOUNCE);
Bounce button2 = Bounce(pinB2, MILLIDEBOUNCE);
Bounce button3 = Bounce(pinB3, MILLIDEBOUNCE);
Bounce button4 = Bounce(pinB4, MILLIDEBOUNCE);
Bounce buttonLB = Bounce(pinLB, MILLIDEBOUNCE);
Bounce buttonRB = Bounce(pinRB, MILLIDEBOUNCE);
Bounce buttonLT = Bounce(pinLT, MILLIDEBOUNCE);
Bounce buttonRT = Bounce(pinRT, MILLIDEBOUNCE);
Bounce buttonSTART = Bounce(pinST, MILLIDEBOUNCE);
Bounce buttonBACK = Bounce(pinBK, MILLIDEBOUNCE);
Bounce buttonXBOX = Bounce(pinXB, MILLIDEBOUNCE);
Bounce button9 = Bounce(pinB9, MILLIDEBOUNCE);
Bounce button10 = Bounce(pinB10, MILLIDEBOUNCE);

//Initiate the xinput class and setup the LED pin
XINPUT controller(LED_ENABLED, pinLED1);

//void Configure Inputs and Outputs
void setupPins()
{
    //Configure the direction of the pins
    //All inputs with internal pullups enabled
    pinMode(pinDpadU, INPUT_PULLUP);
    pinMode(pinDpadD, INPUT_PULLUP);
    pinMode(pinDpadL, INPUT_PULLUP);
    pinMode(pinDpadR, INPUT_PULLUP);
    pinMode(pinB1, INPUT_PULLUP);
    pinMode(pinB2, INPUT_PULLUP);
    pinMode(pinB3, INPUT_PULLUP);
    pinMode(pinB4, INPUT_PULLUP);
    pinMode(pinLB, INPUT_PULLUP);
    pinMode(pinRB, INPUT_PULLUP);
    pinMode(pinLT, INPUT_PULLUP);
    pinMode(pinRT, INPUT_PULLUP);
    pinMode(pinST, INPUT_PULLUP);
    pinMode(pinBK, INPUT_PULLUP);
    pinMode(pinXB, INPUT_PULLUP);
    pinMode(pinLED1, OUTPUT);
    pinMode(pinLED2, OUTPUT);
    //Set the LED to low to make sure it is off
    digitalWrite(pinLED1, LOW);
    //Set the LED to high to turn it on
    digitalWrite(pinLED2, HIGH);
    //Rumble
    pinMode(rumbleSmall, OUTPUT);
    pinMode(rumbleLarge, OUTPUT);
    //L3 & R3
    pinMode(pinB9, INPUT_PULLUP);
    pinMode(pinB10, INPUT_PULLUP);    
}

//Update the debounced button statuses
//We are looking for falling edges since the boards are built
//for common ground sticks
void buttonUpdate()
{
  if (dpadUP.update()) {buttonStatus[POSUP] = dpadUP.fallingEdge();}
  if (dpadDOWN.update()) {buttonStatus[POSDN] = dpadDOWN.fallingEdge();}
  if (dpadLEFT.update()) {buttonStatus[POSLT] = dpadLEFT.fallingEdge();}
  if (dpadRIGHT.update()) {buttonStatus[POSRT] = dpadRIGHT.fallingEdge();}
  if (button1.update()) {buttonStatus[POSB1] = button1.fallingEdge();}
  if (button2.update()) {buttonStatus[POSB2] = button2.fallingEdge();}
  if (button3.update()) {buttonStatus[POSB3] = button3.fallingEdge();}
  if (button4.update()) {buttonStatus[POSB4] = button4.fallingEdge();}
  if (buttonLB.update()) {buttonStatus[POSL1] = buttonLB.fallingEdge();}
  if (buttonRB.update()) {buttonStatus[POSR1] = buttonRB.fallingEdge();}
  if (buttonLT.update()) {buttonStatus[POSL2] = buttonLT.fallingEdge();}
  if (buttonRT.update()) {buttonStatus[POSR2] = buttonRT.fallingEdge();}
  if (buttonSTART.update()) {buttonStatus[POSST] = buttonSTART.fallingEdge();}
  if (buttonBACK.update()) {buttonStatus[POSBK] = buttonBACK.fallingEdge();}
  if (buttonXBOX.update()) {buttonStatus[POSXB] = buttonXBOX.fallingEdge();}
  if (button9.update()) {buttonStatus[POSB9] = button9.fallingEdge();}
  if (button10.update()) {buttonStatus[POSB10] = button10.fallingEdge();}
}

//ProcessInputs
void processInputs()
{
  if (leftStickJoy)
  {
    int leftStickX = buttonStatus[POSLT] * -30000 + buttonStatus[POSRT] * 30000;
    int leftStickY = buttonStatus[POSDN] * -30000 + buttonStatus[POSUP] * 30000;
    controller.stickUpdate(STICK_LEFT, leftStickX, leftStickY);    
    controller.dpadUpdate(0, 0, 0, 0);
  }
  else
  {
    //Update the DPAD
    controller.dpadUpdate(buttonStatus[POSUP], buttonStatus[POSDN], buttonStatus[POSLT], buttonStatus[POSRT]);
  }

  // If Xbox "Back" and joystick Up pressed simultaneously, map joystick to Xbox Left Stick
  // If Xbox "Back" and joystick Down pressed, map joystick to D-pad
  if (leftStickJoy && buttonStatus[POSDN] && buttonStatus[POSBK])
  {
    leftStickJoy = false;
  }
  else if (!leftStickJoy && buttonStatus[POSUP] && buttonStatus[POSBK])
  {
    leftStickJoy = true;
  }

  // If Xbox "Back" and joystick Left pressed simultaneously, use normal L1 & R1
  // If Xbox "Back" and joystick Right pressed, use analog L2 & R2
  if (!flipperL1R1 && buttonStatus[POSLT] && buttonStatus[POSBK])
  {
    flipperL1R1 = true;
  }
  else if (flipperL1R1 && buttonStatus[POSRT] && buttonStatus[POSBK])
  {
    flipperL1R1 = false;
  }
  
  //Buttons
  if (buttonStatus[POSB1]) {controller.buttonUpdate(BUTTON_A, 1);}
  else  {controller.buttonUpdate(BUTTON_A, 0);}
  if (buttonStatus[POSB2]) {controller.buttonUpdate(BUTTON_B, 1);}
  else {controller.buttonUpdate(BUTTON_B, 0);}
  if (buttonStatus[POSB3]) {controller.buttonUpdate(BUTTON_X, 1);}
  else {controller.buttonUpdate(BUTTON_X, 0);}
  if (buttonStatus[POSB4]) {controller.buttonUpdate(BUTTON_Y, 1);}
  else {controller.buttonUpdate(BUTTON_Y, 0);}
  if (buttonStatus[POSB9]) {controller.buttonUpdate(BUTTON_L3, 1);}
  else  {controller.buttonUpdate(BUTTON_L3, 0);}
  if (buttonStatus[POSB10]) {controller.buttonUpdate(BUTTON_R3, 1);}
  else {controller.buttonUpdate(BUTTON_R3, 0);}

  // If BACK and Left Flipper pressed simultaneously, set new plunger dead zone
  // Compensates for games where the in-game plunger doesn't begin pulling back until
  // the gamepad is pulled back ~half way. Just pull the plunger to the point just before
  // it begins to move in-game, and then press BACK & LB.
  if (buttonStatus[POSBK] && buttonStatus[POSL1])
  {
    deadZoneCompensation();
  }

  // detect double contact flipper switches tied to GPIO 13 & 14
  if (!doubleContactFlippers && !fourFlipperButtons)
  {
    if (buttonStatus[POSL2] && buttonStatus[POSL1])
    {
      flipperL1R1 = false;
      doubleContactFlippers = true;
    }
    if (buttonStatus[POSR2] && buttonStatus[POSR1])
    {
      flipperL1R1 = false;
      doubleContactFlippers = true;
    }
  }

  // detect four flipper buttons, second pair tied to GPIO 13 & 14
  // detection occurs if GPIO 13 is pressed WITHOUT FLIP_L being pressed
  // or GPIO 14 without FLIP_R being pressed (not possible with double contact switch)
  // 20190511 - Switch must be closed for ~250 ms in order to qualify mode change.
  if (!fourFlipperButtons)
  {
    if (buttonStatus[POSL2] && !buttonStatus[POSL1])
    {
      long currentTime = millis();
      if (fourButtonModeTriggeredLB == 0)
      {
        fourButtonModeTriggeredLB = currentTime;
      }
      else if (currentTime > fourButtonModeTriggeredLB + fourButtonModeThreshold)
      {
        flipperL1R1 = true;
        fourFlipperButtons = true;
        doubleContactFlippers = false;
      }
    }
    // reset check timer if necessary
    else if (fourButtonModeTriggeredLB > 0) fourButtonModeTriggeredLB = 0;

    if (buttonStatus[POSR2] && !buttonStatus[POSR1])
    {
      long currentTime = millis();
      if (fourButtonModeTriggeredRB == 0)
      {
        fourButtonModeTriggeredRB = currentTime;
      }
      else if (currentTime > fourButtonModeTriggeredRB + fourButtonModeThreshold)
      {
        flipperL1R1 = true;
        fourFlipperButtons = true;
        doubleContactFlippers = false;
      }
    }
    // reset check timer if necessary
    else if (fourButtonModeTriggeredRB > 0) fourButtonModeTriggeredRB = 0;
  }

  // Bumpers
  // Standard mode: FLIP_L and FLIP_R map to L1/R1, and optionally GPIO 13 & 14 map to L2/R2
  if (flipperL1R1)
  {
    uint8_t leftTrigger = 0;
    uint8_t rightTrigger = 0;
    if (buttonStatus[POSL1]) {controller.buttonUpdate(BUTTON_LB, 1);}
    else {controller.buttonUpdate(BUTTON_LB, 0);}
    if (buttonStatus[POSR1]) {controller.buttonUpdate(BUTTON_RB, 1);}
    else {controller.buttonUpdate(BUTTON_RB, 0);}
    if (buttonStatus[POSL2]) leftTrigger = 255;
    else leftTrigger = 0;
    if (buttonStatus[POSR2]) rightTrigger = 255;
    else rightTrigger = 0;
    controller.triggerUpdate(leftTrigger, rightTrigger);
  }

  // L2/R2 Flippers (standard mode swapped)
  else if (!flipperL1R1 && !doubleContactFlippers)
  {
    uint8_t leftTrigger = 0;
    uint8_t rightTrigger = 0;
    if (buttonStatus[POSL2]) {controller.buttonUpdate(BUTTON_LB, 1);}
    else {controller.buttonUpdate(BUTTON_LB, 0);}
    if (buttonStatus[POSR2]) {controller.buttonUpdate(BUTTON_RB, 1);}
    else {controller.buttonUpdate(BUTTON_RB, 0);}
    if (buttonStatus[POSL1]) {leftTrigger = 255;}
    else {leftTrigger = 0;}
    if (buttonStatus[POSR1]) {rightTrigger = 255;}
    else {rightTrigger = 0;}
    controller.triggerUpdate(leftTrigger, rightTrigger);
  }

  // Double Contact Flippers
  else if (!flipperL1R1 && doubleContactFlippers)
  {
    uint8_t leftTrigger = 0;
    uint8_t rightTrigger = 0;
    if (buttonStatus[POSL1] && buttonStatus[POSL2]) {leftTrigger = 255;}
    else if (buttonStatus[POSL1] && !buttonStatus[POSL2]) {leftTrigger = 25;}
    else if (!buttonStatus[POSL1] && !buttonStatus[POSL2]) {leftTrigger = 0;}
    if (buttonStatus[POSR1] && buttonStatus[POSR2]) {rightTrigger = 255;}
    else if (buttonStatus[POSR1] && !buttonStatus[POSR2]) {rightTrigger = 25;}
    else if (!buttonStatus[POSR1] && !buttonStatus[POSR2]) {rightTrigger = 0;}
    controller.triggerUpdate(leftTrigger, rightTrigger);
  }

  //Middle Buttons
  if (buttonStatus[POSST]&&buttonStatus[POSBK]){controller.buttonUpdate(BUTTON_LOGO, 1);}
  else if (buttonStatus[POSST]){controller.buttonUpdate(BUTTON_START, 1);}
  else if (buttonStatus[POSBK]){controller.buttonUpdate(BUTTON_BACK, 1);}
  else if (buttonStatus[POSXB]){controller.buttonUpdate(BUTTON_LOGO, 1);}
  else {controller.buttonUpdate(BUTTON_LOGO, 0); controller.buttonUpdate(BUTTON_START, 0); controller.buttonUpdate(BUTTON_BACK, 0);}

  //Experimental Analog Input
  //Analog flippers
  if (analogFlippers)
  {
    uint8_t leftTrigger = map(analogRead(pinLT), 0, 512, 0, 255);
    uint8_t rightTrigger = map(analogRead(pinRT), 0, 512, 0, 255);
    controller.triggerUpdate(leftTrigger, rightTrigger);
  }
  
  //Tilt
  if (accelerometerEnabled && !leftStickJoy)
  {
    /* Get a new sensor event */ 
    sensors_event_t event; 
    accel.getEvent(&event);

    // Zero accelerometer when START is first pressed (PinSim Yellow Start Button)
    if (buttonStatus[POSST] && !accelerometerCalibrated)
    {
      accelerometerCalibrated = true;
      zeroX = event.acceleration.x * nudgeMultiplier * -1;
      zeroY = event.acceleration.y * nudgeMultiplier * -1;
    }

    // Re-calibrate accelerometer if both BACK and RIGHT FLIPPER pressed
    if (buttonStatus[POSBK] && buttonStatus[POSR1])
    {
      accelerometerCalibrated = true;
      zeroX = event.acceleration.x * nudgeMultiplier * -1;
      zeroY = event.acceleration.y * nudgeMultiplier * -1;
    }

    int leftStickX = zeroX + (event.acceleration.x * nudgeMultiplier);
    int leftStickY = zeroY + (event.acceleration.y * nudgeMultiplier);
    if (millis() > tiltEnableTime)
    {
      controller.stickUpdate(STICK_LEFT, leftStickX, leftStickY);
    }
  }

  // Plunger
  // This is based on the Sharp GP2Y0A51SK0F Analog Distance Sensor 2-15cm
  if (plungerEnabled)
  {
    int reading = analogRead(pinPlunger);
  
    if ((reading - lastReading) > -10 && (reading - lastReading) < 10 || (reading - lastReading > 75) || (reading - lastReading < -75))
    {
      ave.push(reading);
    }
    lastReading = reading;
    int16_t averageReading = ave.mean();

    // it appears the distance sensor updates at about 60hz, no point in checking more often than that
    if (millis() > plungerReportTime)
    {
      // restore zero value after a plunge
      if (zeroValueBuffer)
      {
        zeroValue = zeroValueBuffer;
        zeroValueBuffer = 0;
      }
      plungerReportTime = millis() + plungerReportDelay;
      int16_t currentDistance = readingToDistance(averageReading);
      distanceBuffer = currentDistance;

      // if plunger is pulled
      if (currentDistance +50 < plungerMaxDistance && currentDistance > plungerMinDistance + 50)
      {
        currentlyPlunging = true;
        // Attempt to detect plunge
        int16_t adjustedPlungeTrigger = map(currentDistance, plungerMaxDistance, plungerMinDistance, plungeTrigger/2, plungeTrigger);
        if (currentDistance - lastDistance >= adjustedPlungeTrigger)
        {
            // we throw STICK_RIGHT to 0 to better simulate the physical behavior of a real analog stick
            controller.stickUpdate(STICK_RIGHT, 0, 0);
            // disable plunger momentarily to compensate for spring bounce
            plungerReportTime = millis() + 1000;
            distanceBuffer = plungerMaxDistance;
            lastDistance = plungerMaxDistance;
            if (zeroValue)
            {
              zeroValueBuffer = zeroValue;
              zeroValue = 0;
            }
            return;
        }
        lastDistance = currentDistance;

        // Disable accelerometer while plunging and for 1 second afterwards.
        if (currentDistance < plungerMaxDistance - 50) tiltEnableTime = millis() + 1000;
      }

      // cap max
      else if (currentDistance <= plungerMinDistance + 50)
      {
        currentlyPlunging = true;
        controller.stickUpdate(STICK_RIGHT, 0, -32760);
        distanceBuffer = plungerMinDistance;
        tiltEnableTime = millis() + 1000;
      }
      
      // cap min
      else if (currentDistance > plungerMaxDistance)
      {
        currentlyPlunging = false;
        distanceBuffer = plungerMaxDistance;
      }

      else if (currentlyPlunging) currentlyPlunging = false;
    }

    if (currentlyPlunging)
    {
      controller.stickUpdate(STICK_RIGHT, 0, map(distanceBuffer, plungerMaxDistance, plungerMinDistance, zeroValue, -32760));
    }
    else controller.stickUpdate(STICK_RIGHT, 0, map(distanceBuffer, plungerMaxDistance, plungerMinDistance, 0, -32760));
  }

  // Rumble
  analogWrite(rumbleSmall, controller.rumbleValues[1]);
  analogWrite(rumbleLarge, controller.rumbleValues[0]);

  // Duplicate rumble signals on both motors (causes unacceptable current draw)
//  if (controller.rumbleValues[0] > 0 && controller.rumbleValues[1] == 0x00)
//  {
//    analogWrite(rumbleSmall, controller.rumbleValues[0]);
//  }
//  if (controller.rumbleValues[1] > 0 && controller.rumbleValues[0] == 0x00)
//  {
//    analogWrite(rumbleLarge, controller.rumbleValues[1]);
//  }
 
}

uint16_t readingToDistance(int16_t reading)
{
    // The signal from the IR distance detector is curved. Let's linearize. Thanks for the help Twitter!
    float voltage = reading / 310.0f;
    float linearDistance = ((0.1621f * voltage) + 1.0f) / (0.1567f * voltage);
    return linearDistance * 100;
}

uint16_t getPlungerAverage()
{
  for (int i=0; i<numSamples; i++)
  {
    int reading = analogRead(pinPlunger);
  
    if ((reading - lastReading) > -10 && (reading - lastReading) < 10)
    {
      ave.push(reading);
    }
    lastReading = reading;
  }
  int averageReading = ave.mean();
  return averageReading;
}

void getPlungerMax()
{
  flashStartButton();
  plungerMin = getPlungerAverage();
  plungerMax = plungerMin + 1;
  int averageReading = ave.mean();
  while (averageReading < plungerMin + 100)
  {
    // wait for the plunger to be pulled
    int reading = analogRead(pinPlunger);
    if ((reading - lastReading) > -10 && (reading - lastReading) < 10)
    {
      ave.push(reading);
    }
    lastReading = reading;
    averageReading = ave.mean();
  }

  while (averageReading > plungerMin)
  {
    // start recording plungerMax
    int reading = analogRead(pinPlunger);
    if ((reading - lastReading) > -10 && (reading - lastReading) < 10)
    {
      ave.push(reading);
    }
    lastReading = reading;
    averageReading = ave.mean();
    if (averageReading > plungerMax) plungerMax = averageReading;
  }

  EEPROM.writeInt(0,plungerMax);
  EEPROM.writeInt(10,plungerMin);
  flashStartButton();
}

void deadZoneCompensation()
{
  zeroValue = map(distanceBuffer, plungerMaxDistance, plungerMinDistance, 0, -32768) + 10;
  if (zeroValue > 0) zeroValue = 0;
  flashStartButton();
  buttonUpdate();
  // ensure just one calibration per button press
  while (digitalRead(POSBK) == LOW)
  {
    // wait...
  }
}

void flashStartButton()
{
  for (int i=0; i<10; i++)
  {
    digitalWrite(pinLED1, HIGH);
    delay(50);
    digitalWrite(pinLED1, LOW);
    delay(50);
  }
}

//Setup
void setup() 
{
  setupPins();
  delay(500);
  
  // rumble test (hold Left Flipper on boot)
  if (digitalRead(pinLB) == LOW)
  {
    for (int str=0; str < 256; str++)
    {
      analogWrite(rumbleSmall, str);
      delay(10);
    }
    for (int str=255; str > 0; str--)
    {
      analogWrite(rumbleSmall, str);
      delay(10);
    }
  
    for (int str=0; str < 256; str++)
    {
      analogWrite(rumbleLarge, str);
      delay(10);
    }
    for (int str=255; str > 0; str--)
    {
      analogWrite(rumbleLarge, str);
      delay(10);
    }
  }

  // Hold Right Flipper on boot to disable accelerometer
  if (digitalRead(pinRB) == LOW)
  {
    accelerometerEnabled = false;
    leftStickJoy = true;
  }

  /* Initialise the sensor */
  if (accelerometerEnabled)
  {
    if(!accel.begin())
    {
      /* There was a problem detecting the ADXL345 ... check your connections */
      accelerometerEnabled = false;
      flashStartButton();
    }
  }
  
  if (accelerometerEnabled)
  {
    /* Set the range to whatever is appropriate for your project */
    // accel.setRange(ADXL345_RANGE_16_G);
    // accel.setRange(ADXL345_RANGE_8_G);
    // accel.setRange(ADXL345_RANGE_4_G);
    accel.setRange(ADXL345_RANGE_2_G);
    
    delay(2500); // time to lower the cabinet lid
    sensors_event_t event; 
    accel.getEvent(&event);
    zeroX = event.acceleration.x * nudgeMultiplier * -1;
    zeroY = event.acceleration.y * nudgeMultiplier * -1;
  }
  else delay(1000);

  // plunger setup
  plungerMin = getPlungerAverage();
  if (plungerEnabled) 
  {
    plungerMax = EEPROM.readInt(0);
    plungerMin = EEPROM.readInt(10);
  }
  else plungerMin = getPlungerAverage();

  // to calibrate, hold A or START when plugging in the Teensy LC
  if (digitalRead(pinB1) == LOW) getPlungerMax();
  else if (digitalRead(pinST) == LOW) getPlungerMax();

  // linear conversions
  if (plungerEnabled)
  {
    plungerMaxDistance = readingToDistance(plungerMin);
    plungerMinDistance = readingToDistance(plungerMax);
    lastDistance = plungerMaxDistance;
  }
}

void loop() 
{
  //Poll Buttons
  buttonUpdate();
  
  //Process all inputs and load up the usbData registers correctly
  processInputs();

  //Update the LED display
  controller.LEDUpdate();

  //Send data
  controller.sendXinput();

  //Receive data
  controller.receiveXinput();
}
