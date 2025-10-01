#include <stdint.h>
/***
 * Project: WeCaSlide MKII
 * File   : WeCaSlideMKII.h
 * Author : Werner Riemann 
 * Created: 11.06.2025
 * Board: Arduino Nano
 * 
 * Description: Prototypes and defines
 * 
 * 
 * 
 * 
 */

// ensure this library description is only included once
#ifndef WeCaSlideMKII_h
#define WeCaSlideMKII_h

#define APP_VERSION "1.01"

#define DIRRIGHT_BUTTONPin 1
#define DIRLEFT_BUTTONPin 2
#define ENDSWITCH_LEFT_PIN 3
#define ENDSWITCH_RIGHT_PIN 4
#define ENDSWITCH_CAM_REF_PIN 8


#define ENCODER_SWITCH_PIN 5
#define ENCODER_DT_PIN     6
#define ENCODER_CLK_PIN    7

#define REMOTE_TRIGGER_PIN 13
#define BATTERY_VOLTAGE_PIN A2

#define SLIDE_TORIGHT 1         // motor clockwise, Slider travels to right (note: motor axle viewed from top)
#define SLIDE_TOLEFT  0         // motor counterClockwise, Slider travels to left. Note: control unit with display is on right side
#define CAM_CLOCKWISE 1
#define CAM_COUNTERCLOCKWISE 0 

 // Defines Statemachine ----------------------------------------------
#define AST_STARTUP        0
#define AST_DEVTEST        1

#define AST_MODE_SELECT   10
#define AST_MODE_PAN      11
#define AST_ED_PANSPEED   12
#define AST_MODE_ROTATE   13
#define AST_ED_ROTATE     14
#define AST_MODE_PANROTATE 15
#define AST_ED_PANROTATE   16
#define AST_MODE_PANTRACK  17
#define AST_ED_PANTRACK    18

#define AST_MODE_MOVETO  20
#define AST_ED_MOVETO    21

#define AST_HOMING         30
#define AST_HOMING_RAIL    31
#define AST_HOMING_CAM     32

#define AST_CALIBRATE      35
#define AST_CALIBRATE_GOLEFT  36
#define AST_CALIBRATE_HOME    37

#define AST_SAVE           38

#define AST_MOVE_LEFT      41
#define AST_MOVE_CENTER    42
#define AST_MOVE_RIGHT     43
#define AST_MOVE_STARTANGLE  44     // 
#define AST_MOVE_STARTXPOS   45
#define AST_MOVE_HOMEANGLE   46

// prototypes ---------------------------------------------------------
void ReadEncoder();
void EncoderValueChange(int * valToModify, int rangeMin, int rangeMax);
void CheckRemoteTrigger();
void CheckDirectionButtons();
void CheckToRevertSlide();
void CheckSlideBoundaries();
void SetRailMoveToSide(uint8_t direction, uint8_t speedMMS);
void SetCamRotate(uint8_t direction, int speedTRPMM);
void SetCamHome();
uint8_t SetMoveToAction(uint8_t itemNo);
void SetPanCamAngle(int destAngle);
void CalculateTrackSpeed();

void SaveSettings();
void ReadSettings();

#endif