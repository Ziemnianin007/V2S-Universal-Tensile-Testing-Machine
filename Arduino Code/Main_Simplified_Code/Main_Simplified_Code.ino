/*
Universal Tensile Testing Machine Code By Xieshi aka CrazyBlackStone
VERSION 2.1.1 REV 3
Designed for use with custom PCB
Project published under CC-BY-NC-SA license https://creativecommons.org/licenses/by-nc-sa/4.0/
*/

//Need Wire library. It should come preinstalled with Arduino IDE.
#include <Wire.h>
 
//Need HX711 library https://github.com/bogde/HX711
#include <HX711.h>

//Need U8x8 library, which is included in the u8g2 library https://github.com/olikraus/u8g2
#include <U8x8lib.h>

#include <string.h>

#include <stdio.h>


//----------USER PARAMETERS----------

//PLX-DAQ - Excel data collection. Uncomment this if you have PLX-DAQ set up for data analysis.
#define plxdaq

//Calibration setup: if you know your load cell's calibration factor, uncomment and change the calibration factor below. If you don't want to calibrate the load cell, comment this out.
#define calibration

#define calibrationFactor -1.9305 * 2280 * 0.09806 / 0.484458317// this value is obtained by calibrating the scale with known weights, details in https://github.com/bogde/HX711

//set this to the basic steps per revolution of your stepper motor (not counting in gear ratio). It is 200 for 1.8 degree stepper motors and 400 for 0.9 degree stepper motors. Leave unchanged if you don't know - in most cases it is 200 steps per rev.
#define stepsPerRev 200
//set this to the microstepping setting. This does not control the microstepping of the driver - it simply sets the step rate. Set the microstepping mode in hardware through the MS pins.
#define microStep 16
//set this to the gear ratio of the stepper motor you're using
#define gearRatio 100
//set this to the lead screw lead you're using
#define leadScrewPitch 8
//don't change this unless you want to change the modulus start speed
#define modulusSpeedMultiplier 25
//don't change this unless you want to change the slow speed
#define slowSpeedMultiplier 50
//don't change this unless you want to change the fast speed
#define fastSpeedMultiplier 100
//modulus test threshold. Used to determine when to change speed to fast.
#define moveSpeedMultiplier 200 //Those seems to be mm/min
//modulus test threshold. Used to determine when to change speed to fast.
#define modulusThreshold 30000
//reading attempts to be taken on each reading; the average of the attempts will be displayed. Depends on your load cell amplifier's configuration. 10hz - 1 attempt, 80hz - 8 attempts.
#define readAttempts 1
 
//pin definitions
#define EStopPin 2
#define tarePin 3
#define moveUpPin 4
#define modePin 5
#define startPin 6
#define moveDownPin 7
#define ledPin 8
#define endStop1Pin 9
#define endStop2Pin 10
#define D11 11
#define D12 12
#define enablePin 13
#define DTPin A0
#define SCKPin A1
#define dCalDTPin A2
#define dCalSCKPin A3

//parameters
bool testStart = false;
bool moveStepper = false;
byte stepperDir = 0;
byte stepperDirParams = 0;
byte stepperStatus = 0; //0 = stopped, 1 = moving
int multiplier = 1;
#define modulusSpeed stepsPerRev * microStep * gearRatio / leadScrewPitch / 60 * modulusSpeedMultiplier
#define slowSpeed stepsPerRev * microStep * gearRatio / leadScrewPitch / 60 * slowSpeedMultiplier
#define fastSpeed stepsPerRev * microStep * gearRatio / leadScrewPitch / 60 * fastSpeedMultiplier
#define moveSpeed stepsPerRev * microStep * gearRatio / leadScrewPitch / 60 * moveSpeedMultiplier
#define constantForSpeedCalc stepsPerRev * microStep * gearRatio / leadScrewPitch / 60
int stepperSpeed = slowSpeed;
byte stepperSpeedHigh;
byte stepperSpeedLow;
#define slowMeasurementDelay 0.8 * 1000000
#define fastMeasurementDelay 0.1 * 1000000
float measurementDelay = slowMeasurementDelay;
float measurementMax = 0;
float measurement = 0;
int serialCmdMoveSpeed = constantForSpeedCalc * moveSpeedMultiplier;
int forceAbsoluteLimit = 1500; //Force limit in Newtons, stops when reached
int howManyTimesForceLimitExceeded = 0;
int forceSwitchLimit = -1; //Force limit in Newtons, stops when reached
int howManyTimesForceSwitchExceeded = 0;

unsigned long lastMeasurement = 0;
unsigned long testStartTime = 0;
unsigned long testTime = 0;
byte mode = 1; //1 - tensile slow, 2 - tensile fast, 3 - compression slow, 4 - compression fast, 5 - modulus
bool modeOldState = HIGH;
bool startOldState = HIGH;
bool tareOldState = HIGH;
bool moveUpOldState = HIGH;
bool moveDownOldState = HIGH;
bool confirmMode = false;
bool emergencyStop = false;
bool emergencyStopForce = false;
String stringMode;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   
HX711 scale;
 
void setup() {
  Serial.begin(9600);
  Wire.begin();
  #ifndef plxdaq
  Serial.println(F("INITIALIZING"));
  #endif

  Serial.println(F("Parameters command format is as follows:"));
  Serial.println(F("set <int-stepper dir> <int-move speed [mm/min]> <int-force absolute limit>"));
  Serial.println(F("movrev <int-stepper dir> <int-move speed [mm/min]> <int-force absolute limit> <int-switch-dir-force>"));
  Serial.println(F("Dir=0 is stretching, max speed is 200"));
  Serial.println(F("Then use 'start'"));

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_inr21_2x4_r);

  u8x8.setCursor(0,0);
  
  scale.begin(DTPin, SCKPin);
 
  #ifdef calibration
  scale.set_scale(calibrationFactor); 
  #endif
 
  scale.tare(20);

  pinMode(moveUpPin, INPUT);
  digitalWrite(moveUpPin, HIGH);

  pinMode(moveDownPin, INPUT);
  digitalWrite(moveDownPin, HIGH);

  pinMode(modePin, INPUT);
  digitalWrite(modePin, HIGH);

  pinMode(startPin, INPUT);
  digitalWrite(startPin, HIGH);
  
  pinMode(tarePin, INPUT);
  digitalWrite(tarePin, HIGH);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(D11, OUTPUT);
  digitalWrite(D11, LOW);

  pinMode(D12, OUTPUT);
  digitalWrite(D12, LOW);
  
  pinMode(EStopPin, INPUT);
  digitalWrite(EStopPin, HIGH);

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
  
  pinMode(endStop1Pin, INPUT);
  digitalWrite(endStop1Pin, HIGH);

  pinMode(endStop2Pin, INPUT);
  digitalWrite(endStop2Pin, HIGH);

  attachInterrupt(digitalPinToInterrupt(EStopPin), stopNow, FALLING);

  #ifndef plxdaq
  Serial.println(F("INITIALIZATION COMPLETE"));
  Serial.println(F("To "));
  #else
  Serial.println(F("CLEARDATA"));
  Serial.println(F("LABEL,Time,Test Time,Value"));
  #endif
}
 
void loop() {
  //stop motor after reset

  //EMERGENCY STOP
  if (!digitalRead(endStop1Pin) || !digitalRead(endStop2Pin) && !emergencyStop)
  {
    stopNow();
    Serial.println(F("Endstop triggered - emergency stop"));
  }
  
  //EMERGENCY STOP force exceeded
  if (measurement>forceAbsoluteLimit || measurement<-forceAbsoluteLimit)
  {
    if (howManyTimesForceLimitExceeded<10){ //sometimes some noises/error make this value huge, that will prevent stopping test then for single event
      howManyTimesForceLimitExceeded = howManyTimesForceLimitExceeded+1;
    }
    else
    {
    emergencyStopForce = true;
    stopNow();
    Serial.println(F("Force limit exceeded - emergency stop"));
    }
  }
  else
  {
    howManyTimesForceLimitExceeded = 0;
  }

  if (emergencyStop)
  { 
    stepperSpeed = 0;
    stepperDir = 0;
    stepperStatus = 0;
    sendCommand();
    
    Serial.println(F("\n\n\n-EMERGENCY STOP - RESET TO CLEAR-\n-EMERGENCY STOP - RESET TO CLEAR-\n-EMERGENCY STOP - RESET TO CLEAR-"));
    u8x8.clear();
    if (emergencyStopForce == false)
    {
      u8x8.print(F("E-STOP\nRESET"));
    }
    else
    {
      u8x8.print(F("F-STOP\nRESET"));
    }
    
    for (;;);
  }
  
  //reverse movement after reaching force treshold
  if (forceSwitchLimit > 0 && moveStepper == true)
  {
    if (measurement>forceSwitchLimit || measurement<-forceSwitchLimit)
    {
      
      if (howManyTimesForceSwitchExceeded<10){ //sometimes some noises/error make this value huge, that will prevent stopping test then for single event
        howManyTimesForceSwitchExceeded = howManyTimesForceSwitchExceeded+1;
      }
      else
      {
        stepperSpeed = serialCmdMoveSpeed;
        if (stepperDirParams==1)
        {
          stepperDir = 0;
          multiplier = 1;
        }
        else
        {
          stepperDir = 1;
          multiplier = -1; 
        }
        stepperStatus = 1;
        u8x8.clear();
        u8x8.println("Test\nReverse");
        //Serial.println("\n-Reverse-\n");
        sendCommand();

        forceSwitchLimit = -1;
      }
    }
    else
    {
      howManyTimesForceSwitchExceeded = 0;
    }
  }

  String inputString;
  bool serialAvailable = false;
  while (Serial.available())
  {
    serialAvailable = true;
    inputString = Serial.readString();
    inputString.toLowerCase();
  }
  if (serialAvailable)
  {
    if (inputString == "start")
    {
      checkMode();
      Serial.println("Current mode is: " + stringMode + ". Confirm? Y/N");
      confirmMode = true;
    }
    else if (inputString == "stop")
    {
      stopTest();
    }
    else if (inputString == "tare")
    {
      tareScale();
    }
    else if (inputString == "up")
    {
      moveUp();
    }
    else if (inputString == "down")
    {
      moveDown();
    }
    else if (strstr(inputString.c_str(), "set"))
    {
      set_params(inputString);
    }
    else if (strstr(inputString.c_str(), "movrev"))
    {
      set_params_rev(inputString);
    }
    else if (inputString == "mode")
    {
      changeMode();
    }
    else if (inputString == "y" && confirmMode)
    {
      startTest();
    }
    else if (inputString == "n" && confirmMode)
    {
      confirmMode = false;
      Serial.println(F("\n-TEST CANCELLED-\n"));
    }
    else
    {
      Serial.println(F("Unknown Command"));
    }
  }
  
  bool modeNewState = digitalRead(modePin);
  bool moveUpNewState = digitalRead(moveUpPin);
  bool moveDownNewState = digitalRead(moveDownPin);
  bool startNewState = digitalRead(startPin);
  bool tareNewState = digitalRead(tarePin);

  if (!digitalRead(moveUpPin) && moveUpNewState != moveUpOldState && moveUpNewState == LOW)
  {
    moveUp();
  }
  else if (!digitalRead(moveDownPin) && moveDownNewState != moveDownOldState && moveDownNewState == LOW)
  {
    moveDown();
  }
  else if (!digitalRead(modePin) && modeNewState != modeOldState && modeNewState == LOW && !testStart)
  {
    changeMode();
    delay(10);
  }
  else if (!digitalRead(startPin) && startNewState != startOldState && startNewState == LOW)
  {
   if (!testStart && !moveStepper)
   {
     startTest();
   }
   else
   {
     stopTest();
   }
   delay(10);
  }
  else if (!digitalRead(tarePin) && tareNewState != tareOldState && tareNewState == LOW)
  {
   tareScale();
   delay(10);
  }
 
  modeOldState = modeNewState;
  startOldState = startNewState;
  moveUpOldState = moveUpNewState;
  moveDownOldState = moveDownNewState;
  tareOldState = tareNewState;

  testTime = millis() - testStartTime;

  if ((micros() - lastMeasurement) >= measurementDelay)
  {
    measurement = scale.get_units(readAttempts) * multiplier;
    
    String measurementStr = String(measurement);
    measurementStr.replace(".",",");
    String testTimeStr = String(testTime/1000) + "," + String(testTime%1000);
    String millisStr = String(millis());
    //Serial.println("DATA,TIME," + millisStr + "," + testTimeStr + "," + measurementStr);
    Serial.println(testTimeStr + ";" + measurementStr);
    
    if (!testStart)
    {
      String modeStr = String(mode);
      u8x8.clear();
      u8x8.println(measurementStr + "\nMode: " + modeStr);
    }
    else
    {
      if (measurement >= measurementMax)
      {
        measurementMax = measurement;
      }
      // #ifdef plxdaq
      // String testTimeStr = String(testTime);
      // String millisStr = String(millis());
      // //Serial.println("DATA,TIME," + millisStr + "," + testTimeStr + "," + measurementStr);
      // Serial.println(testTimeStr + ";" + measurementStr);
      // #endif
    }
    lastMeasurement = micros();
  }

  if (mode == 5 && testTime >= modulusThreshold && stepperSpeed != fastSpeed)
  {
    Serial.println(F("THRESHOLD REACHED"));
    stepperSpeed = fastSpeed;
    stepperDir = 0;
    multiplier = 1;
    stepperStatus = 1;
    sendCommand();
  }
}
 
 
void startTest()
{
  confirmMode = false;
  moveStepper = true;
  //digitalWrite(enablePin, LOW);

  Serial.println(F("\n-TEST START-\n"));

  scale.tare(20);
  measurementDelay = fastMeasurementDelay;
  if (mode == 1)
  {
    stepperSpeed = slowSpeed;
    stepperDir = 0;
    multiplier = 1;
  }
  else if (mode == 2)
  {
    stepperSpeed = fastSpeed;
    stepperDir = 0;
    multiplier = 1;
  }
  else if (mode == 3)
  {
    stepperSpeed = slowSpeed;
    stepperDir = 1;
    multiplier = -1;
  }
  else if (mode == 4)
  {
    stepperSpeed = fastSpeed;
    stepperDir = 1;
    multiplier = -1;
  }
  else if (mode == 5)
  {
    stepperSpeed = modulusSpeed;
    stepperDir = 0;
    multiplier = 1;
  }
  else if (mode == 6)
  {
    stepperSpeed = serialCmdMoveSpeed;
    stepperDir = stepperDirParams;
    if (stepperDirParams==0)
    {
      multiplier = 1;
    }
    else
    {
     multiplier = -1; 
    }
  }
  else if (mode == 7)
  {
    stepperSpeed = serialCmdMoveSpeed;
    stepperDir = stepperDirParams;
    if (stepperDirParams==0)
    {
      multiplier = 1;
    }
    else
    {
     multiplier = -1; 
    }
  }

  stepperStatus = 1;
  digitalWrite(ledPin, HIGH);
  digitalWrite(D11, HIGH);
  sendCommand();
  
  testStart = true;
  measurementMax = 0;
  testStartTime = millis();

  u8x8.clear();
  u8x8.println("Test\nStarted");
}
 
void stopTest()
{
  Serial.println("\n-STOP-\n");
  measurementDelay = slowMeasurementDelay;

  stepperSpeed = 0;
  stepperDir = 0;
  stepperStatus = 0;
  sendCommand();
  digitalWrite(ledPin, LOW);
  digitalWrite(D11, LOW);
  
  moveStepper = false;
  //digitalWrite(enablePin, HIGH);
  
  if (testStart)
  {
    u8x8.clear();
    String measMaxStr = String(measurementMax);
    u8x8.println("Max: \n" + measMaxStr);
    #ifndef plxdaq
    String testTimeStr = String(testTime);
    Serial.println("Time: " + testTimeStr + "\nMaximum value: " + measMaxStr); 
    #endif
    testStart = false;
    delay(10000);
  }
}

void set_params(String cmdString)
{
  //command format as follows: set <int-stepper dir> <int-move speed [mm/min]> <int-force absolute limit>
  mode=6;
  stepperDirParams = 0;
  int stepperDirParamsInt;
  int parametricSpeedMultiplier = 0;
  sscanf(cmdString.c_str(), "set %i %i %i", &stepperDirParamsInt, &parametricSpeedMultiplier, &forceAbsoluteLimit);
  serialCmdMoveSpeed = int(float(stepsPerRev) * float(microStep) * float(gearRatio) / float(leadScrewPitch) / float(60) * float(parametricSpeedMultiplier));
  if (stepperDirParamsInt != 0)
  {
    stepperDirParams = 1;
  }
  stepperSpeed = serialCmdMoveSpeed;
  stepperStatus = 1;
  if (stepperDirParams==0) //is stretching
  {
    Serial.println("Parameters set to:\ndir: stretching " + String(stepperDirParams) + "\nSpeed: " + String(parametricSpeedMultiplier) + "[mm/min]\nForce limit: " + String(forceAbsoluteLimit)+"N"); 
  }
  else
  {
    
    Serial.println("Parameters set to:\ndir: compressing " + String(stepperDirParams) + "\nSpeed: " + String(parametricSpeedMultiplier) + "[mm/min]\nForce limit: " + String(forceAbsoluteLimit)+"N"); 
  }
}

void set_params_rev(String cmdString)
{
  //command format as follows: set <int-stepper dir> <int-move speed [mm/min]> <int-force absolute limit>
  mode=7;
  stepperDirParams = 0;
  int stepperDirParamsInt;
  int parametricSpeedMultiplier = 0;
  sscanf(cmdString.c_str(), "movrev %i %i %i %i", &stepperDirParamsInt, &parametricSpeedMultiplier, &forceAbsoluteLimit, &forceSwitchLimit);
  serialCmdMoveSpeed = int(float(stepsPerRev) * float(microStep) * float(gearRatio) / float(leadScrewPitch) / float(60) * float(parametricSpeedMultiplier));
  if (stepperDirParamsInt != 0)
  {
    stepperDirParams = 1;
  }
  stepperSpeed = serialCmdMoveSpeed;
  stepperStatus = 1;
  if (stepperDirParams==0) //is stretching
  {
    Serial.println("Parameters set to:\ndir: stretching " + String(stepperDirParams) + "\nSpeed: " + String(parametricSpeedMultiplier) + "[mm/min]\nForce limit: " + String(forceAbsoluteLimit)+"N" + "\nForce switch: " + String(forceSwitchLimit)+"N\n" + String(serialCmdMoveSpeed)); 
  }
  else
  {
    
    Serial.println("Parameters set to:\ndir: compressing " + String(stepperDirParams) + "\nSpeed: " + String(parametricSpeedMultiplier) + "[mm/min]\nForce limit: " + String(forceAbsoluteLimit)+"N" + "\nForce switch: " + String(forceSwitchLimit)+"N\n" + String(serialCmdMoveSpeed)); 
  }
}

void tareScale()
{
  Serial.println(F("\n-SCALE TARE-\n"));   
  scale.tare(20);
}

void moveUp()
{
  moveStepper = true;
  //digitalWrite(enablePin, LOW);
  Serial.println(F("\n-MOVING UP-\n"));

  stepperSpeed = moveSpeed;
  stepperDir = 0;
  stepperStatus = 1;
  sendCommand();
}

void moveDown()
{
  moveStepper = true;
  //digitalWrite(enablePin, LOW);
  Serial.println(F("\n-MOVING DOWN-\n"));

  stepperSpeed = moveSpeed;
  stepperDir = 1;
  stepperStatus = 1;
  sendCommand();
}

void changeMode()
{
  if (mode < 5)
  {
    mode++;
  }
  else
  {
    mode = 1;
  }
  checkMode();
  Serial.println("CURRENT MODE: " + stringMode);
}

void checkMode()
{
  if (mode == 1)
  {
    stringMode = F("SLOW Tensile Test");
  }
  else if (mode == 2)
  {
    stringMode = F("FAST Tensile Test");
  }
  else if (mode == 3)
  {
    stringMode = F("SLOW Compression Test");
  }
  else if (mode == 4)
  {
    stringMode = F("FAST Compression Test");
  }
  else if (mode == 5)
  {
    stringMode = F("Tensile Modulus Test");
  }
  else if (mode == 6)
  {
    stringMode = F("Serial cmd parametric Test");
  }
  else if (mode == 7)
  {
    stringMode = F("Serial cmd parametric Test with reverse");
  }
}

void stopNow()
{
  digitalWrite(enablePin, HIGH);
  //pinMode(EStopPin, OUTPUT);
  //digitalWrite(EStopPin, LOW);
  
  moveStepper = false;
  emergencyStop = true;
}

void sendCommand()
{
  stepperSpeedHigh = *((uint8_t*)&(stepperSpeed)+1);
  stepperSpeedLow = *((uint8_t*)&(stepperSpeed)+0);
  byte sendInfo[4] = {stepperSpeedHigh, stepperSpeedLow, stepperDir, stepperStatus};
  Wire.beginTransmission(9);
  for (int i=0; i<4; i++)
  {
    Wire.write(sendInfo[i]);  //data bytes are queued in local buffer
  }
  byte sendStatus = Wire.endTransmission();
  digitalWrite(enablePin, LOW); //fixes rotating motor after reset
  if (sendStatus != 0)
  {
    String sendStatusStr = String(sendStatus);
    Serial.println("Send failed (" + sendStatusStr + ")- emergency stop");
    stopNow();
  }
}
