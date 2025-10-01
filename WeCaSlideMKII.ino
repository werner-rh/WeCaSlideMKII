/***
 * Project: WeCaSlide MKII
 * File   : WeCaSlideMKII.ino
 * Author : Werner Riemann 
 * Created: 11.06.2025
 * Board: Arduino Nano
 * 
 * Description: 2 Axis Camera slider with object tracking (slide and rotate). Both axis are driven by an Nema17 stepper motor.
 *              The slider uses the smallest Nema17 with 23mm hight. These types are strong and fast enough
 *              to move a heavy DSLR or DSR camera. 
 * 
 * Pins:
 * A0     - OUT Enable Stepper Motor rotation
 * A1     - OUT Enable Stepper Motor rail
 * A2     - IN analog battery voltage
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * A6     - 
 * A7     - not used, A6+A7 only as analog pins available on Nano, Pro Mini and Mini's
 * 
 * D1     - Digital IN push button 0 - dirRightButtonPin      - Direction Right, Start, Stop
 * D2     - Digital IN push button 1 - dirLeftButtonPin       - Direction Left, Start, Stop
 * D3     - Digital IN endswitch left side
 * D4     - Digital IN endswitch right side
 *
 * D5     - Rotary Encoder Switch
 * D6     - Rotary Encoder DT_pin
 * D7     - Rotary Encoder CLK_pin
 * D8     - Endswitch, reference point Cam rotate
 * D9     - OUT Direction Nema Stepper Motor Rail
 * D10    - OUT Pulse for Nema Stepper Motor Rail
 * D11    - OUT Direction Nema Stepper Motor Cam rotate
 * D12    - OUT Pulse for Nema Stepper Motor Cam rotate
 * D13    - remote Trigger Camera,    Digital out LEDPIN 13 (intern LED)
 * 
 */

#include <Arduino.h>

#include "WeCaSlideMKII.h"
#include "WRKeyStateDef.h"
#include "NemaStepper.h"
#include "WeCaDisplay.h"
#include "SliderData.h"



//-- gloabal accessable data ---------------------
uint8_t B100HzToggle = 0;  // 100 Hertz Signal
uint8_t ui10MilliSekCount = 0;

int encoder_value = 500;
int last_encoder_value  = 500;
uint8_t DT_pinstate;
uint8_t last_DT_pinstate;

//-- vars to store button states  ----
uint8_t encoderBUTTON_State=0;
uint8_t dirLeft_BUTTON_State=0;
uint8_t dirRight_BUTTON_State=0;

uint8_t left_Endswitch_State=0;
uint8_t right_Endswitch_State=0;
uint8_t cam_Endswitch_State=0;


SliderData slData(300, 40);   // all global needed data is hold in the SliderData class

WeCaDisplay Display(APP_VERSION);

// GT2 pulley circumference = number of teeth * division
// using 20 teeth pulle: 20 * 2mm = 40mm
NemaStepper railNema(9, 10, A0, 6400, 40, 1);
NemaStepper camNema(11, 12, A1, 6400, 40, 4);

// Interrupt is called once a millisecond, 
SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long currentMillis = millis();
  ui10MilliSekCount ++;

  if(ui10MilliSekCount >= 10 ) {
    ui10MilliSekCount = 0;
    B100HzToggle ^= 1;
  }

  ReadEncoder();  
}

void ReadEncoder() {
    DT_pinstate = digitalRead(ENCODER_DT_PIN);
    if (DT_pinstate != last_DT_pinstate) { //did DT changed state?
      if (digitalRead(ENCODER_CLK_PIN) == DT_pinstate) { // if DT changed state, check CLK
        encoder_value--; // rotation is counter-clockwise, decrement the value
      }else{
        encoder_value++; // rotation is clockwise, increment the value
      }
    last_DT_pinstate = DT_pinstate; //save the last state of DT
    }
}


/***
 * EncoderValueChange - increments or decrements the value of given var depending
 * from the direction the rotary encoder is moved. The Value of var is kept in range
 * of rangeMin and rangeMax.
 * 
 * param int * valToModify : pointer of var to modify
 * param int rangeMin: minimum value 
 * param int rangeMax: maximum value
*/
void EncoderValueChange(int * valToModify, int rangeMin, int rangeMax) {
    
    int aktValue = * valToModify;
    if(last_encoder_value != encoder_value)
    {

      if(encoder_value > last_encoder_value +1)
      {
        last_encoder_value = encoder_value;
        if(aktValue < rangeMax)
          aktValue ++;
      }
      
      if(encoder_value +1  < last_encoder_value )
      {
        last_encoder_value = encoder_value;
        if(aktValue > rangeMin)
        aktValue --;
      }
    }

    * valToModify = aktValue;
}


void setup() {
  
  pinMode(DIRLEFT_BUTTONPin, INPUT);          // input buttons direction left
  digitalWrite(DIRLEFT_BUTTONPin, HIGH);      // Pullup resistor on
  pinMode(DIRRIGHT_BUTTONPin, INPUT);         // input buttons direction right
  digitalWrite(DIRRIGHT_BUTTONPin, HIGH);     // Pullup resistor on
  pinMode(ENDSWITCH_LEFT_PIN, INPUT);         // left rail endswitch
  digitalWrite(ENDSWITCH_LEFT_PIN, HIGH);
  pinMode(ENDSWITCH_RIGHT_PIN, INPUT);        // right rail endswitch
  digitalWrite(ENDSWITCH_RIGHT_PIN, HIGH);
  pinMode(ENDSWITCH_CAM_REF_PIN, INPUT);      // entswitch camera zero/ref point
  digitalWrite(ENDSWITCH_CAM_REF_PIN, HIGH);

  pinMode(BATTERY_VOLTAGE_PIN, INPUT);        // analog input battery voltage

  pinMode(REMOTE_TRIGGER_PIN, OUTPUT);        // camera remote trigger relais
  digitalWrite(REMOTE_TRIGGER_PIN, LOW);


  // rotary encoder
  pinMode (ENCODER_SWITCH_PIN, INPUT);
  pinMode (ENCODER_DT_PIN, INPUT);
  pinMode (ENCODER_CLK_PIN, INPUT);
  digitalWrite(ENCODER_SWITCH_PIN, HIGH);  // enable intern pull up resistor
  digitalWrite(ENCODER_DT_PIN, HIGH);
  digitalWrite(ENCODER_CLK_PIN, HIGH);
  // Reads the initial state of DT
  last_DT_pinstate = digitalRead(ENCODER_DT_PIN);

  Display.Setup();

  Display.WelcomeScreen();

  // settings Nema Stepper
  railNema.setRailSizeSteps(DEFAULT_100CM_RAILSIZE);
  railNema.enableMotor(HIGH);             // enable the stepper driver output
  slData.railDirection = SLIDE_TORIGHT;
  railNema.setDirection(SLIDE_TORIGHT);
  railNema.setSpeedMMPS(slData.railSpeedMMPS);

  camNema.enableMotor(HIGH);
  slData.camDirection = STEPDIR_CLOCKWISE;
  camNema.setDirection(STEPDIR_CLOCKWISE);
  camNema.setSpeedRPM(slData.rotateSpeedTRPM);

  // Timer setup --------------------------
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);  
}


void CheckRemoteTrigger() {
  if(slData.remoteTrigger ==1) {  
    digitalWrite(REMOTE_TRIGGER_PIN, HIGH); // remote relais on
    slData.remoteRelaisActiv = 1; // this will be noticed in the main loop, to deactivate the relais after a delay  
  }
}


/***
 * Checks the state of the left and right push button and performs the action
 * for the current used slider mode.
 */
void CheckDirectionButtons() {
  uint8_t tmpDirRotation;

  if(dirLeft_BUTTON_State ==1) // move Slider to left
  {
    switch (slData.actModeNo) {
    case APP_MODE_PAN:
      if(railNema.getRunState() == 1 && slData.railDirection == SLIDE_TOLEFT) { 
        railNema.stopMotor(1);
        CheckRemoteTrigger(); //TODO: add remote trigger for stop record
        } 
      else { 
        if(railNema.getRunState() == 0)  // it's start from the stoped Sliderbase
          CheckRemoteTrigger();   
        SetRailMoveToSide(SLIDE_TOLEFT, slData.railSpeedMMPS);  
        }   
    break;

    case APP_MODE_ROTATE:
      if(camNema.getRunState() == 1 && slData.camDirection == CAM_COUNTERCLOCKWISE)
        { camNema.stopMotor(1); CheckRemoteTrigger();}
      else {
        if(camNema.getRunState() == 0)
          CheckRemoteTrigger();
        SetCamRotate(CAM_COUNTERCLOCKWISE, slData.rotateSpeedTRPM); }
    break;

    case APP_MODE_PANROTATE:
      if(railNema.getRunState() == 1 && slData.railDirection == SLIDE_TOLEFT) { 
        railNema.stopMotor(1); camNema.stopMotor(1);
        CheckRemoteTrigger();
        }
      else  { 
        if(railNema.getRunState() == 0)  // it's start from the stoped Sliderbase
            CheckRemoteTrigger(); 
        SetRailMoveToSide(SLIDE_TOLEFT, slData.railSpeedMMPS);
        tmpDirRotation = CAM_COUNTERCLOCKWISE;
        tmpDirRotation ^= slData.invertRotation;
        SetCamRotate(tmpDirRotation, slData.rotateSpeedTRPM); 
        }  
    break;

    case APP_MODE_PANTRACK:
      if(railNema.getRunState() == 1 && slData.railDirection == SLIDE_TOLEFT) { 
        railNema.stopMotor(1); camNema.stopMotor(1); CheckRemoteTrigger();}
      else  { 
        if(railNema.getRunState() == 0)  // it's start from the stoped Sliderbase
            CheckRemoteTrigger();    
        SetRailMoveToSide(SLIDE_TOLEFT, slData.railSpeedMMPS);
        SetCamRotate(CAM_CLOCKWISE, slData.trackRotateSpeedTPRM);        
        }  // to track we need the inverted direction to the rail
    break;
    }  

  }

  if(dirRight_BUTTON_State ==1) // move Slider to right 
  {
    switch (slData.actModeNo) {
    case APP_MODE_PAN:
      if(railNema.getRunState() == 1 && slData.railDirection == SLIDE_TORIGHT)
        { railNema.stopMotor(1); CheckRemoteTrigger();}
      else 
        { 
        if(railNema.getRunState() == 0)  // it's start from the stoped Sliderbase
          CheckRemoteTrigger();       
        SetRailMoveToSide(SLIDE_TORIGHT, slData.railSpeedMMPS);    
      }
    break;

    case APP_MODE_ROTATE:
      if(camNema.getRunState() == 1 && slData.camDirection == CAM_CLOCKWISE)
        { camNema.stopMotor(1); CheckRemoteTrigger();}
      else 
        { 
        if(camNema.getRunState() == 0)
          CheckRemoteTrigger();
        SetCamRotate(CAM_CLOCKWISE, slData.rotateSpeedTRPM); }    
    break;

    case APP_MODE_PANROTATE:
      if(railNema.getRunState() == 1 && slData.railDirection == SLIDE_TORIGHT)
        { railNema.stopMotor(1); camNema.stopMotor(1); CheckRemoteTrigger();}
      else { 
        if(railNema.getRunState() == 0)  // it's start from the stoped Sliderbase
            CheckRemoteTrigger();   
        SetRailMoveToSide(SLIDE_TORIGHT, slData.railSpeedMMPS);
        tmpDirRotation = CAM_CLOCKWISE;
        tmpDirRotation ^= slData.invertRotation;    // invert the rotaion rlated to the rail, if set
        SetCamRotate(tmpDirRotation, slData.rotateSpeedTRPM);     
        }
    break;

    case APP_MODE_PANTRACK:
      if(railNema.getRunState() == 1 && slData.railDirection == SLIDE_TORIGHT)
        { railNema.stopMotor(1); camNema.stopMotor(1); CheckRemoteTrigger();}
      else { 
        if(railNema.getRunState() == 0)  // it's start from the stoped Sliderbase
            CheckRemoteTrigger();   
        SetRailMoveToSide(SLIDE_TORIGHT, slData.railSpeedMMPS);
        SetCamRotate(CAM_COUNTERCLOCKWISE, slData.trackRotateSpeedTPRM);  // invert the rotaion rlated to the rail, if set
        }
    break;

    }
      
  }        
}

/***
 checks if the slider base reached left or right endswitch and reverts the movement if auto return is activ.
*/
void CheckToRevertSlide() {
  if(slData.railDirection == SLIDE_TOLEFT && left_Endswitch_State ==2 || slData.railDirection == SLIDE_TORIGHT && right_Endswitch_State ==2) {
    if(slData.railAutoReturn == 1) {
      slData.railDirection ^=1;
      slData.camDirection ^=1;
      railNema.setDirection(slData.railDirection);
      camNema.setDirection(slData.camDirection);

    } else {
      railNema.stopMotor(0);
      camNema.stopMotor(0);
    }

  }    

}



void CheckSlideBoundaries() {

  switch (slData.actModeNo) {
    case APP_MODE_PAN:
      if(slData.railDirection == SLIDE_TOLEFT && left_Endswitch_State ==2 || slData.railDirection == SLIDE_TORIGHT && right_Endswitch_State ==2) {
        if(slData.railAutoReturn == 1) {
          slData.railDirection ^=1;
          railNema.setDirection(slData.railDirection);
        } else {
          railNema.stopMotor(0); // endswitch touched, stop emidiatly
        }
      }

    break;

    case APP_MODE_ROTATE: //currently not used or implemented
 
    break;
    
    case APP_MODE_PANROTATE:
      CheckToRevertSlide();
    break;

    case APP_MODE_PANTRACK:
      CheckToRevertSlide();
    break;
  }  

}

void CalculateTrackSpeed() {

  // assume rail length 100cm. Usable length 89cm. Calculating triangle from center to outer right position
  // For calculation we use mm instead cm and have to multiply cm by 10
  // for testing, using a fixed value of 445mm. Later we will get the length from calibration routine.
  // We are calculationg with the triangle from the middle of the slider, to the object and the right
  // side of the slider rail.
  float distanceMM = (float)slData.distanceTrackCM * 10;
  float travelAngleDeg = 90.0 - slData.CalculateAngleDeg(distanceMM, 445);
  float travelTime = (float)445 / (float)slData.railSpeedMMPS;
  
  slData.trackRotateSpeedTPRM = slData.CalculateTrackRotSpeedTRPM(travelAngleDeg, travelTime, camNema.getGearReductionFactor());

}

void SetRailMoveToSide(uint8_t direction, uint8_t speedMMS) {
  railNema.stopMotor(1);
  slData.railDirection = direction;             // store last set direction
  railNema.setDirection(slData.railDirection);
  railNema.setSpeedMMPS(speedMMS);
  railNema.runMotor();
}

void SetCamRotate(uint8_t direction, int speedTRPMM) {
  camNema.stopMotor(1);
  slData.camDirection = direction;
  camNema.setDirection(slData.camDirection);
  camNema.setSpeedRPM(speedTRPMM);
  camNema.runMotor();
}

void SetCamHome() {
  camNema.stopMotor(1);
  slData.camDirection = STEPDIR_CLOCKWISE;
  camNema.setDirection(slData.camDirection);
  camNema.setSpeedRPM(300);
  camNema.runMotor();
}

void SetPanCamAngle(int destAngle) {
  if(camNema.getCurrentAngle() > destAngle) {
        SetCamRotate(CAM_COUNTERCLOCKWISE, 100);
      } else {
        SetCamRotate(CAM_CLOCKWISE, 100);
      }

  camNema.runMotor();
  while(camNema.getCurrentAngle() != destAngle) {
    camNema.motorStep();
  }

  camNema.stopMotor(1);
}

/***
 Performs the selected action from "Move to" and returns the related application state for the main state machine.
*/
uint8_t SetMoveToAction(uint8_t itemNo) {
  uint8_t newAppState = AST_ED_MOVETO;  // default jump back

  switch (itemNo) {
    // move to left
    case MSELECT_ITEM_PARAM_NO1: SetRailMoveToSide(SLIDE_TOLEFT, HOMING_RAIL_SPEED); newAppState = AST_MOVE_LEFT;
    break;
    // move to center
    case MSELECT_ITEM_PARAM_NO2:                                      
      if(railNema.getPositionSteps() > railNema.getRailSizeSteps()/2)
        SetRailMoveToSide(SLIDE_TOLEFT, HOMING_RAIL_SPEED);
      else
        SetRailMoveToSide(SLIDE_TORIGHT, HOMING_RAIL_SPEED);
      newAppState = AST_MOVE_CENTER;
    break;
    
    // move to right
    case MSELECT_ITEM_PARAM_NO3:  SetRailMoveToSide(SLIDE_TORIGHT, 80); newAppState = AST_MOVE_RIGHT;
    break;
    // move cam to Start angle
    case MSELECT_ITEM_PARAM_NO4:
      if(camNema.getCurrentAngle() > slData.startAngle) {
        SetCamRotate(CAM_COUNTERCLOCKWISE, 100);
      } else {
        SetCamRotate(CAM_CLOCKWISE, 100);
      }
      newAppState = AST_MOVE_STARTANGLE;
    break;
    // move rail to Start X position
    case MSELECT_ITEM_PARAM_NO5:
      slData.destStepPosition = slData.CmToSteps(slData.startXPos, 89,railNema.getRailSizeSteps());
      if(railNema.getPositionSteps() < slData.destStepPosition) {
        SetRailMoveToSide(SLIDE_TORIGHT, HOMING_RAIL_SPEED);
      } else {
        SetRailMoveToSide(SLIDE_TOLEFT, HOMING_RAIL_SPEED);
      }
      newAppState = AST_MOVE_STARTXPOS;
    break;
    // move cam to home position angle (90degree)
    case MSELECT_ITEM_PARAM_NO6:
      if(camNema.getCurrentAngle() > 90) {
        SetCamRotate(CAM_COUNTERCLOCKWISE, 100);
      } else {
        SetCamRotate(CAM_CLOCKWISE, 100);
      }
      newAppState = AST_MOVE_HOMEANGLE;
    break;
  }

  return newAppState;
}

void loop() {
  static uint8_t AppState=0;
  static uint8_t StateTrigger = 0;
  uint8_t aktStateTrigger;
  static uint8_t delaySecCounter=20;
  static uint8_t remoteRelaisDelay = 0;
  static uint8_t screenUpdateDelay = 0;

  int tmpVal = 0;

  aktStateTrigger = B100HzToggle;

  // >--- the code within the aktStateTrigger will be executed only 100 times per second.
  //      So you don't need further debounce for input buttons and switches
  if(aktStateTrigger != StateTrigger) {
    StateTrigger = aktStateTrigger;

    // check the state of all used buttons   
    CheckKeyState(&encoderBUTTON_State, ENCODER_SWITCH_PIN);
    CheckKeyState(&dirLeft_BUTTON_State, DIRLEFT_BUTTONPin);
    CheckKeyState(&dirRight_BUTTON_State, DIRRIGHT_BUTTONPin);

    CheckKeyState(&left_Endswitch_State, ENDSWITCH_LEFT_PIN);
    CheckKeyState(&right_Endswitch_State, ENDSWITCH_RIGHT_PIN);
    CheckKeyState(&cam_Endswitch_State, ENDSWITCH_CAM_REF_PIN);

    if(delaySecCounter >= 20) {
      // battery voltage, has to been read in a lower cycle
      slData.filterBatteryVoltage(analogRead(BATTERY_VOLTAGE_PIN));
      delaySecCounter=0; // reset the counter to wait for 2 seconds
    } else {
      delaySecCounter++;
    }
    
    
    // process application state machine --------------
    switch(AppState) {

      case AST_STARTUP:
        ReadSettings();
        if(encoderBUTTON_State == 1 || dirLeft_BUTTON_State ==1 || dirRight_BUTTON_State ==1) {
          Display.ScreenOut(1, 1);
          AppState = AST_MODE_SELECT;
        }
      break;

      case AST_MODE_SELECT: // choose the slider mode and set the related state
        if(encoderBUTTON_State == 1) {
          if(slData.actModeNo == APP_MODE_PAN) { // Mode Pan
            slData.actPanItem = MSELECT_ITEM_MODESELECT;
            AppState = AST_MODE_PAN;
          }

          if(slData.actModeNo == APP_MODE_ROTATE) { // Mode Rotate
            slData.actRotateItem = MSELECT_ITEM_MODESELECT;
            AppState = AST_MODE_ROTATE;
          }

          if(slData.actModeNo == APP_MODE_PANROTATE) { // Mode PanRotate
            slData.actPanRotateItem = MSELECT_ITEM_MODESELECT;
            AppState = AST_MODE_PANROTATE;
          }

          if(slData.actModeNo == APP_MODE_PANTRACK) { // Mode PanTrack
            slData.actPanTrackItem = MSELECT_ITEM_MODESELECT;
            AppState = AST_MODE_PANTRACK;
            CalculateTrackSpeed();
          }

          if(slData.actModeNo == APP_MODE_MOVETO) { // Mode Move to
            slData.actMoveToItem = MSELECT_ITEM_MODESELECT;
            AppState = AST_MODE_MOVETO;
          }

          if(slData.actModeNo == APP_MODE_HOMING) { // Mode Homing
            slData.actHomingItem = MSELECT_ITEM_MODESELECT;
            AppState = AST_HOMING;
          }

          if(slData.actModeNo == APP_MODE_CALIBRATE) { // Mode Calibrate
            slData.actCalibrateItem = MSELECT_ITEM_MODESELECT;
            AppState = AST_CALIBRATE;
          }

          if(slData.actModeNo == APP_MODE_SAVE) { // Mode Calibrate
            slData.actCalibrateItem = MSELECT_ITEM_MODESELECT;
            AppState = AST_SAVE;
          }

          Display.ScreenOut(1, 0);
        }

        tmpVal = slData.actModeNo;
        EncoderValueChange(&tmpVal, APP_MODE_PAN, APP_MODE_INFO);
        if(tmpVal != slData.actModeNo) {
          slData.actModeNo = tmpVal;
          Display.ScreenOut(1, 1);
        } else {
          if(screenUpdateDelay >= 200) {
            Display.ScreenOut(1, 1);
            screenUpdateDelay = 0;
          } else {
          screenUpdateDelay++;
          } 
        }
      break;

      case AST_MODE_PAN:  // adjust the values for pan mode
        if(encoderBUTTON_State == 1) {
          if(slData.actPanItem == MSELECT_ITEM_MODESELECT) { // go back to Mode select
            Display.ScreenOut(1, 1);
            AppState = AST_MODE_SELECT;
          }

          if(slData.actPanItem >= MSELECT_ITEM_PARAM_NO1 && slData.actPanItem <= MSELECT_ITEM_PARAM_NO4) { // go to adjust pan values
            Display.ScreenOut(slData.actPanItem, 1);
            AppState = AST_ED_PANSPEED;
          }
        }      

        tmpVal = slData.actPanItem;
        EncoderValueChange(&tmpVal, MSELECT_ITEM_MODESELECT, MSELECT_ITEM_PARAM_NO4);
        if(tmpVal != slData.actPanItem) {
          slData.actPanItem = tmpVal;
          Display.ScreenOut(slData.actPanItem, 0);
        }

        CheckDirectionButtons(); // check the left and right push button and perform action
      break;

      case AST_ED_PANSPEED:
        if(encoderBUTTON_State == 1) {  // go back to pan mode
          Display.ScreenOut(slData.actPanItem, 0);
          AppState = AST_MODE_PAN;
        }

        if(slData.actPanItem == MSELECT_ITEM_PARAM_NO1) { // adjust rail speed 
          tmpVal = slData.railSpeedMMPS; 
          EncoderValueChange(&tmpVal, 1, 150);
          if(tmpVal != slData.railSpeedMMPS) {
            slData.railSpeedMMPS = tmpVal;
            railNema.setSpeedMMPS(slData.railSpeedMMPS);
            Display.ScreenOut(slData.actPanItem, 1);
          }
        }

        if(slData.actPanItem == MSELECT_ITEM_PARAM_NO2) { // select cam angle for pan mode
          tmpVal = slData.camAngle; 
          EncoderValueChange(&tmpVal, 0, 359);
          if(tmpVal != slData.camAngle) {
            slData.camAngle = tmpVal;
            //TODO: add action to rotate the cam while changing the angle
            SetPanCamAngle(slData.camAngle);
            Display.ScreenOut(slData.actPanItem, 1);
          }
        }

        if(slData.actPanItem == MSELECT_ITEM_PARAM_NO3) { // toogle auto return mode
          tmpVal = slData.railAutoReturn; 
          EncoderValueChange(&tmpVal, 0, 1);
          if(tmpVal != slData.railAutoReturn) {
            slData.railAutoReturn = tmpVal;
            Display.ScreenOut(slData.actPanItem, 1);
          }
        }

          if(slData.actPanItem == MSELECT_ITEM_PARAM_NO4) { // toogle auto remote trigger
          tmpVal = slData.remoteTrigger; 
          EncoderValueChange(&tmpVal, 0, 1);
          if(tmpVal != slData.remoteTrigger) {
            slData.remoteTrigger = tmpVal;
            Display.ScreenOut(slData.actPanItem, 1);
            //digitalWrite(REMOTE_TRIGGER_PIN, tmpVal);
          }
        }

      break;

      case AST_MODE_ROTATE:
        if(encoderBUTTON_State == 1) {
          if(slData.actRotateItem == MSELECT_ITEM_MODESELECT) { // go back to select mode
            Display.ScreenOut(1, 1);
            AppState = AST_MODE_SELECT;
          }

          if(slData.actRotateItem >= MSELECT_ITEM_PARAM_NO1 && slData.actRotateItem <= MSELECT_ITEM_PARAM_NO2) { // go to adjust rotate values
            Display.ScreenOut(slData.actRotateItem, 1);
            AppState = AST_ED_ROTATE;
          }
        }

        tmpVal = slData.actRotateItem;
        EncoderValueChange(&tmpVal, MSELECT_ITEM_MODESELECT, MSELECT_ITEM_PARAM_NO2);
        if(tmpVal != slData.actRotateItem) {
          slData.actRotateItem = tmpVal;
          Display.ScreenOut(slData.actRotateItem, 0);
        }

        CheckDirectionButtons();
      break;

      case AST_ED_ROTATE:
        if(encoderBUTTON_State == 1) {
          Display.ScreenOut(slData.actRotateItem, 0);
          AppState = AST_MODE_ROTATE;
        }

        if(slData.actRotateItem == MSELECT_ITEM_PARAM_NO1) {
          tmpVal = slData.startAngle; 
          EncoderValueChange(&tmpVal, 0, 360);
          if(tmpVal != slData.startAngle) {
            slData.startAngle = tmpVal;
            Display.ScreenOut(slData.actRotateItem, 1);
          }
        }


        if(slData.actRotateItem == MSELECT_ITEM_PARAM_NO2) {
          tmpVal = slData.rotateSpeedTRPM; 
          EncoderValueChange(&tmpVal, 0, 360);
          if(tmpVal != slData.rotateSpeedTRPM) {
            slData.rotateSpeedTRPM = tmpVal;
            camNema.setSpeedRPM(slData.rotateSpeedTRPM);
            Display.ScreenOut(slData.actRotateItem, 1);
          }
        }

      break;

      case AST_MODE_PANROTATE:
         if(encoderBUTTON_State == 1) {
          if(slData.actPanRotateItem == MSELECT_ITEM_MODESELECT) { // go back to mode select
            Display.ScreenOut(1, 1);
            AppState = AST_MODE_SELECT;
          }

          if(slData.actPanRotateItem >= MSELECT_ITEM_PARAM_NO1 && slData.actPanRotateItem <= MSELECT_ITEM_PARAM_NO3) { 
            Display.ScreenOut(slData.actPanRotateItem, 1);
            AppState = AST_ED_PANROTATE;
          }
        }

        tmpVal = slData.actPanRotateItem;
        EncoderValueChange(&tmpVal, MSELECT_ITEM_MODESELECT, MSELECT_ITEM_PARAM_NO3);
        if(tmpVal != slData.actPanRotateItem) {
          slData.actPanRotateItem = tmpVal;
          Display.ScreenOut(slData.actPanRotateItem, 0);
        }

        CheckDirectionButtons();
      break;

      case AST_ED_PANROTATE:
        if(encoderBUTTON_State == 1) {
          Display.ScreenOut(slData.actPanRotateItem, 0);
          AppState = AST_MODE_PANROTATE;
        }

        if(slData.actPanRotateItem == MSELECT_ITEM_PARAM_NO1) {
          tmpVal = slData.startXPos; 
          EncoderValueChange(&tmpVal, 0, 100);
          if(tmpVal != slData.startXPos) {
            slData.startXPos = tmpVal;
            slData.destStepPosition = slData.CmToSteps(slData.startXPos, 89,railNema.getRailSizeSteps());
            Display.ScreenOut(slData.actPanRotateItem, 1);
          }
        }

        if(slData.actPanRotateItem == MSELECT_ITEM_PARAM_NO2) {
          tmpVal = slData.startAngle; 
          EncoderValueChange(&tmpVal, 0, 360);
          if(tmpVal != slData.startAngle) {
            slData.startAngle = tmpVal;
            Display.ScreenOut(slData.actPanRotateItem, 1);
          }
        }

        if(slData.actPanRotateItem == MSELECT_ITEM_PARAM_NO3) {
          tmpVal = slData.invertRotation; 
          EncoderValueChange(&tmpVal, 0, 1);
          if(tmpVal != slData.invertRotation) {
            slData.invertRotation = tmpVal;
            Display.ScreenOut(slData.actPanRotateItem, 1);
          }
        }
      break;

      case AST_MODE_PANTRACK:
        if(encoderBUTTON_State == 1) {
          if(slData.actPanTrackItem == MSELECT_ITEM_MODESELECT) { // goto Mode Mode select
            Display.ScreenOut(1, 1);
            AppState = AST_MODE_SELECT;
          }

          if(slData.actPanTrackItem >= MSELECT_ITEM_PARAM_NO1 && slData.actPanTrackItem <= MSELECT_ITEM_PARAM_NO2) { // goto Mode Mode select
            Display.ScreenOut(slData.actPanTrackItem, 1);
            AppState = AST_ED_PANTRACK;
          }
        }

        tmpVal = slData.actPanTrackItem;
        EncoderValueChange(&tmpVal, MSELECT_ITEM_MODESELECT, MSELECT_ITEM_PARAM_NO2);
        if(tmpVal != slData.actPanTrackItem) {
          slData.actPanTrackItem = tmpVal;
          Display.ScreenOut(slData.actPanTrackItem, 0);
        }

        CheckDirectionButtons();
      break;

      case AST_ED_PANTRACK:
        if(encoderBUTTON_State == 1) {
          Display.ScreenOut(slData.actPanTrackItem, 0);
          CalculateTrackSpeed();
          AppState = AST_MODE_PANTRACK;
        }

        if(slData.actPanTrackItem == MSELECT_ITEM_PARAM_NO1) {
          tmpVal = slData.railSpeedMMPS; 
          EncoderValueChange(&tmpVal, 0, 360);
          if(tmpVal != slData.railSpeedMMPS) {
            slData.railSpeedMMPS = tmpVal;
            Display.ScreenOut(slData.actPanTrackItem, 1);
          }
        }

        if(slData.actPanTrackItem == MSELECT_ITEM_PARAM_NO2) {
          tmpVal = slData.distanceTrackCM; 
          EncoderValueChange(&tmpVal, 0, 360);
          if(tmpVal != slData.distanceTrackCM) {
            slData.distanceTrackCM = tmpVal;
            Display.ScreenOut(slData.actPanTrackItem, 1);
          }
        }
      break;

      case AST_MODE_MOVETO:
        if(encoderBUTTON_State == 1) {
          if(slData.actMoveToItem == MSELECT_ITEM_MODESELECT) { // goto Mode Mode select
            Display.ScreenOut(1, 1);
            AppState = AST_MODE_SELECT;
          }

          // move selected and rotary button pressed -> perform action 
          if(slData.actMoveToItem >= MSELECT_ITEM_PARAM_NO1 && slData.actMoveToItem <= MSELECT_ITEM_PARAM_NO6 && slData.railHomeDone == 1) { // goto Mode Mode select
            Display.ScreenOut(slData.actMoveToItem, 1);
            AppState = SetMoveToAction(slData.actMoveToItem);

          }
        }

        tmpVal = slData.actMoveToItem;
        EncoderValueChange(&tmpVal, MSELECT_ITEM_MODESELECT, MSELECT_ITEM_PARAM_NO6);
        if(tmpVal != slData.actMoveToItem) {
          slData.actMoveToItem = tmpVal;
          Display.ScreenOut(slData.actMoveToItem, 0);
        }
      break;

      case AST_ED_MOVETO:
          
          // wait until rail stepper stoped running
          if(railNema.getRunState() == 0) {
            railNema.setSpeedMMPS(slData.railSpeedMMPS); //restore the adjusted values for speed and rotation
            camNema.setSpeedRPM(slData.rotateSpeedTRPM);
            Display.ScreenOut(slData.actMoveToItem, 0);
            AppState = AST_MODE_MOVETO;
          }
          
      break;

      case AST_HOMING:
        slData.homingStarted = 1;
        slData.railHomeDone = 0;
        Display.ScreenOut(slData.actHomingItem, 0);
        SetRailMoveToSide(SLIDE_TORIGHT, HOMING_RAIL_SPEED);
        AppState = AST_HOMING_RAIL;
      break;

      case AST_HOMING_RAIL:
        if(right_Endswitch_State ==2) { // home position at right side reached
          railNema.stopMotor(0);
          railNema.setPositionSteps(railNema.getRailSizeSteps()); // x position starts on left side. So right position must be full rail size
          slData.railHomeDone = 1;
          Display.ScreenOut(slData.actHomingItem, 0);
          railNema.setSpeedMMPS(slData.railSpeedMMPS); // restore current set rail speed

          SetCamHome();
          AppState = AST_HOMING_CAM;
        }
      break;

      case AST_HOMING_CAM:
        if(cam_Endswitch_State ==2) { // home position 90 degree reached
          camNema.stopMotor(0);
          slData.camHomeDone = 1;
          camNema.setCurrentAngle(90);
          Display.ScreenOut(slData.actHomingItem, 0);
          camNema.setSpeedRPM(slData.rotateSpeedTRPM); // restore user set rotation speed

          Display.ScreenOut(1, 1);
          slData.homingStarted = 0;
          slData.AllHomingDone = 1;
          AppState = AST_MODE_SELECT;
        }
      break;

      // all waiting states for "move to" positions or angle
      case AST_MOVE_LEFT:
       if(left_Endswitch_State == 2) {
          railNema.stopMotor(0);
          AppState = AST_ED_MOVETO;
        }
      break;

      case AST_MOVE_CENTER:
        if(slData.railDirection == SLIDE_TORIGHT) {
          if(railNema.getPositionSteps() >= (railNema.getRailSizeSteps() /2 - RAMP_ADDSTEPS)) {
            railNema.stopMotor(1);
            AppState = AST_ED_MOVETO;
          }
        }

        if(slData.railDirection == SLIDE_TOLEFT) {
          if(railNema.getPositionSteps() <= (railNema.getRailSizeSteps() /2 + RAMP_ADDSTEPS)) {
            railNema.stopMotor(1);
            AppState = AST_ED_MOVETO;
          }
        }
      break;

      case AST_MOVE_RIGHT:
         if(right_Endswitch_State == 2) {
          railNema.stopMotor(0);
          AppState = AST_ED_MOVETO;
        }
      break;

      case AST_MOVE_STARTANGLE:
        if(camNema.getCurrentAngle() == slData.startAngle) {
          camNema.stopMotor(0);
          AppState = AST_ED_MOVETO;
        }
      break;

      case AST_MOVE_STARTXPOS:
        if(slData.railDirection == SLIDE_TORIGHT) {
            if(railNema.getPositionSteps() >= slData.destStepPosition - RAMP_ADDSTEPS) {
            railNema.stopMotor(1);
            AppState = AST_ED_MOVETO;
          }
        }

        if(slData.railDirection == SLIDE_TOLEFT) {
            if(railNema.getPositionSteps() <= slData.destStepPosition + RAMP_ADDSTEPS) {
            railNema.stopMotor(1);
            AppState = AST_ED_MOVETO;
          }
        }
        
      break;

      case AST_MOVE_HOMEANGLE:
        if(camNema.getCurrentAngle() == 90) {
          camNema.stopMotor(0);
          AppState = AST_ED_MOVETO;
        }
      break;

      // states for rail calibration. During calibration the total amount of steps between left and right
      // slider endswitches are counted and stored. So we have the exact useable lenght in steps of the rail.
      case AST_CALIBRATE:
        slData.calibrateDone = 0;
        slData.calibrateStarted = 1;
        Display.ScreenOut(slData.actCalibrateItem, 0);
        
        SetRailMoveToSide(SLIDE_TOLEFT, HOMING_RAIL_SPEED);
        AppState = AST_CALIBRATE_GOLEFT;
      break;

      case AST_CALIBRATE_GOLEFT:
        if(left_Endswitch_State == 2) {
          railNema.stopMotor(0);
          railNema.resetCurrentStepCount();
          SetRailMoveToSide(SLIDE_TORIGHT, HOMING_RAIL_SPEED);
          AppState = AST_CALIBRATE_HOME;
        }
      break;

      case AST_CALIBRATE_HOME:
        if(right_Endswitch_State == 2) {
          railNema.stopMotor(0);
          railNema.setRailSizeSteps(railNema.getCurrentStepCount());
          slData.infoRailSizeSteps = railNema.getRailSizeSteps();
          SaveSettings();
          slData.calibrateDone = 1;
          Display.ScreenOut(1, 1);
          slData.calibrateStarted = 0;
          AppState = AST_MODE_SELECT;
          
        }
      break;

      case AST_SAVE:
        
        slData.dataSaveDone = 1;
        Display.ScreenOut(1, 1);
        SaveSettings();
        slData.dataSaveDone = 0;
        AppState = AST_MODE_SELECT;
      break;

    }    
    //turn off the relais for remote trigger
    if(slData.remoteRelaisActiv == 1) {
      remoteRelaisDelay++;
      if(remoteRelaisDelay >= 50) {
        slData.remoteRelaisActiv = 0;
        remoteRelaisDelay = 0;
        digitalWrite(REMOTE_TRIGGER_PIN, LOW);
      }
    }

  } // <--- State trigger end


  // --- Code below will be executed outside the state machine with no delays ---
  // do steps for Nema Steppermotor. Check endswitches before and only call if they are open


  CheckSlideBoundaries();
  railNema.motorStep();
  camNema.motorStep();
  
  
}


// --- Read and Save Settings and Parameter

void SaveSettings() {
  EEPROM.write(0, EEP_MARKER);      // mark the eeprom, that something useful is stored

  slData.EEPROMWriteLong(1, railNema.getRailSizeSteps());   // store the rail length in steps to address 1 - 4
  EEPROM.write(5, slData.railSpeedMMPS);                    // rail speed at adr 5
  slData.EEPROMWriteInt(6, slData.camAngle);                // camAngle at adr 6
  slData.EEPROMWriteInt(8, slData.startAngle);              // startAngle at adr 8
  slData.EEPROMWriteInt(10, slData.endAngle);               // endAngle at adr 10
  slData.EEPROMWriteInt(12, slData.rotateSpeedTRPM);        // rotateSpeedTRPM at adr 12
  EEPROM.write(14, slData.startXPos);
  EEPROM.write(15, slData.endXPos);  
  EEPROM.write(16, slData.distanceTrackCM); 
  EEPROM.write(17, slData.railAutoReturn); 
  EEPROM.write(18, slData.remoteTrigger);
  EEPROM.write(19, slData.invertRotation);

}

void ReadSettings() {
  uint8_t epmarker = EEPROM.read(0);

  if(epmarker == EEP_MARKER) {
    railNema.setRailSizeSteps(slData.EEPROMReadLong(1));  // read the rail size in steps at address 1 - 4
    slData.railSpeedMMPS = EEPROM.read(5);                // read rail speed at adr 5
    slData.camAngle = slData.EEPROMReadInt(6);            // camAngle at adr 6
    slData.startAngle = slData.EEPROMReadInt(8);          // startAngle at adr 8
    slData.endAngle = slData.EEPROMReadInt(10);           // endAngle at adr 8
    slData.rotateSpeedTRPM = slData.EEPROMReadInt(12);
    slData.startXPos = EEPROM.read(14); 
    slData.endXPos = EEPROM.read(15);
    slData.distanceTrackCM = EEPROM.read(16);
    slData.railAutoReturn = EEPROM.read(17);
    slData.remoteTrigger = EEPROM.read(18);
    slData.invertRotation = EEPROM.read(19);

    // set some parameter for info after reading
    slData.infoRailSizeSteps = railNema.getRailSizeSteps();

  }

}
