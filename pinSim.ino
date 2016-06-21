/*
    PinSim Controller v20160620
    Controller for PC Pinball games
    https://www.youtube.com/watch?v=18EcIxywXHg
    
    Based on the excellent MSF_FightStick XINPUT project by Zack "Reaper" Littell
    https://github.com/zlittell/MSF-XINPUT
    
    Uses the Teensy-LC
    
    IMPORTANT PLUNGER NOTE:
    You MUST calibrate the plunger range at least once by holding down START
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
boolean accelerometerEnabled = true;
boolean plungerEnabled = true;
int16_t nudgeMultiplier = 9000; // accelerometer multiplier (higher = more sensitive)
int16_t plungeTrigger = 60; // threshold to trigger a plunge (lower = more sensitive)

// probably leave these alone
int16_t zeroValue = 0; // xbox 360 value for neutral analog stick
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
#define pinB9 13  //Button 9 (Left Analog Press)
#define pinB10 14  //Button 10 (Right Analog Press)
#define pinPlunger 15 //IR distance for plunger
#define pinLED1 16  //Onboard LED 1
#define pinLED2 17  //Onboard LED 2
#define rumbleSmall 20 // Large Rumble Motor
#define rumbleLarge 22 // Large Rumble Motor
#define pinLT 21 //Left Analog Trigger
#define pinRT 23 //Right Analog Trigger

#define NUMBUTTONS 15  //Number of all buttons
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
#define POSLB 8
#define POSRB 9
#define POSB9 10
#define POSB10 11
#define POSST 12
#define POSBK 13
#define POSXB 14

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
Bounce button9 = Bounce(pinB9, MILLIDEBOUNCE);
Bounce button10 = Bounce(pinB10, MILLIDEBOUNCE);
Bounce buttonSTART = Bounce(pinST, MILLIDEBOUNCE);
Bounce buttonBACK = Bounce(pinBK, MILLIDEBOUNCE);
Bounce buttonXBOX = Bounce(pinXB, MILLIDEBOUNCE);

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
    pinMode(pinB9, INPUT_PULLUP);
    pinMode(pinB10, INPUT_PULLUP);
    pinMode(pinST, INPUT_PULLUP);
    pinMode(pinBK, INPUT_PULLUP);
    pinMode(pinLED1, OUTPUT);
    pinMode(pinLED2, OUTPUT);
    //Set the LED to low to make sure it is off
    digitalWrite(pinLED1, LOW);
    //Set the LED to high to turn it on
    digitalWrite(pinLED2, HIGH);
    //Rumble
    pinMode(rumbleSmall, OUTPUT);
    pinMode(rumbleLarge, OUTPUT);
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
  if (buttonLB.update()) {buttonStatus[POSLB] = buttonLB.fallingEdge();}
  if (buttonRB.update()) {buttonStatus[POSRB] = buttonRB.fallingEdge();}
  if (button9.update()) {buttonStatus[POSB9] = button9.fallingEdge();}
  if (button10.update()) {buttonStatus[POSB10] = button10.fallingEdge();}
  if (buttonSTART.update()) {buttonStatus[POSST] = buttonSTART.fallingEdge();}
  if (buttonBACK.update()) {buttonStatus[POSBK] = buttonBACK.fallingEdge();}
  if (buttonXBOX.update()) {buttonStatus[POSXB] = buttonXBOX.fallingEdge();}
}

//ProcessInputs
void processInputs()
{
  //Update the DPAD
  controller.dpadUpdate(buttonStatus[POSUP], buttonStatus[POSDN], buttonStatus[POSLT], buttonStatus[POSRT]);

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
  else {controller.buttonUpdate(BUTTON_L3, 0);}
  if (buttonStatus[POSB10]) {controller.buttonUpdate(BUTTON_R3, 1);}
  else {controller.buttonUpdate(BUTTON_R3, 0);}

  // If A and Left Flipper pressed simultaneously, set new plunger dead zone
  // Compensates for games where the in-game plunger doesn't begin pulling back until
  // the gamepad is pulled back ~half way. Just pull the plunger to the point just before
  // it begins to move in-game, and then press A & LB.
  if (buttonStatus[POSB1] && buttonStatus[POSLB])
  {
    deadZoneCompensation();
  }

  //Bumpers
  if (buttonStatus[POSLB]) {controller.buttonUpdate(BUTTON_LB, 1);}
  else {controller.buttonUpdate(BUTTON_LB, 0);}
  if (buttonStatus[POSRB]) {controller.buttonUpdate(BUTTON_RB, 1);}
  else {controller.buttonUpdate(BUTTON_RB, 0);}
  
  //Middle Buttons

  if (buttonStatus[POSST]&&buttonStatus[POSBK]){controller.buttonUpdate(BUTTON_LOGO, 1);}
  else if (buttonStatus[POSST]){controller.buttonUpdate(BUTTON_START, 1);}
  else if (buttonStatus[POSBK]){controller.buttonUpdate(BUTTON_BACK, 1);}
  else if (buttonStatus[POSXB]){controller.buttonUpdate(BUTTON_LOGO, 1);}
  else {controller.buttonUpdate(BUTTON_LOGO, 0); controller.buttonUpdate(BUTTON_START, 0); controller.buttonUpdate(BUTTON_BACK, 0);}

  //Triggers
//  uint8_t leftTrigger = map(analogRead(A7), 0, 1023, 0, 255);
//  uint8_t rightTrigger = map(analogRead(A9), 0, 1023, 0, 255);
//  controller.triggerUpdate(leftTrigger, rightTrigger);
  
  //Analog Input
  //Tilt

  if (accelerometerEnabled)
  {
    /* Get a new sensor event */ 
    sensors_event_t event; 
    accel.getEvent(&event);

    // zero accelerometer whenever A is pushed (PinSim Yellow Start Button)
    if (buttonStatus[POSB1])
    {
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
      plungerReportTime = millis() + plungerReportDelay;
      int16_t currentDistance = readingToDistance(averageReading);
      distanceBuffer = currentDistance;
      if (currentDistance < plungerMaxDistance && currentDistance > plungerMinDistance + 50)
      {
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
            return;
        }
        lastDistance = currentDistance;

        // Disable accelerometer while plunging and for 1 second afterwards.
        if (currentDistance < plungerMaxDistance - 50) tiltEnableTime = millis() + 1000;
      }

      // cap max
      else if (currentDistance <= plungerMinDistance + 50)
      {
        controller.stickUpdate(STICK_RIGHT, 0, -32768);
        distanceBuffer = plungerMinDistance;
        tiltEnableTime = millis() + 1000;
      }
      
      // cap min
      else if (currentDistance > plungerMaxDistance)
      {
        controller.stickUpdate(STICK_RIGHT, 0, 0);
        distanceBuffer = plungerMaxDistance;
      }
    }

    controller.stickUpdate(STICK_RIGHT, 0, map(distanceBuffer, plungerMaxDistance, plungerMinDistance, zeroValue, -32768));
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
  flashStartButton();
}

void deadZoneCompensation()
{
  zeroValue = map(distanceBuffer, plungerMaxDistance, plungerMinDistance, 0, -32768) - 10;
  if (zeroValue > 0) zeroValue = 0;
  flashStartButton();
  buttonUpdate();
  // ensure just one calibration per button press
  while (digitalRead(pinB1) == LOW)
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
    
    delay(1000);
    sensors_event_t event; 
    accel.getEvent(&event);
    zeroX = event.acceleration.x * nudgeMultiplier * -1;
    zeroY = event.acceleration.y * nudgeMultiplier * -1;
  }
  else delay(1000);

  // plunger setup
  plungerMin = getPlungerAverage();
  if (plungerEnabled) plungerMax = EEPROM.readInt(0);

  // to calibrate, hold A when plugging in the Teensy LC
  if (digitalRead(pinB1) == LOW) getPlungerMax();

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
