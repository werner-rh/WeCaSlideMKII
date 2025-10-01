#include <stdint.h>
/***
 * Project: Camera Slider motorized
 * File   : AppData.h
 * Author : Werner Riemann 
 * Created: 30.06.2025
 * Board: Arduino Nano
 * 
 * Description: global accessable data class for application data and vars
 * 
 * 
 * 
 * 
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "WeCaSlideMKII.h"

//--- Defines ------------------------------
#define MSELECT_ITEM_MODESELECT 1
#define MSELECT_ITEM_PARAM_NO1  2
#define MSELECT_ITEM_PARAM_NO2  3
#define MSELECT_ITEM_PARAM_NO3  4
#define MSELECT_ITEM_PARAM_NO4  5
#define MSELECT_ITEM_PARAM_NO5  6
#define MSELECT_ITEM_PARAM_NO6  7

#define APP_MODE_PAN        1
#define APP_MODE_ROTATE     2
#define APP_MODE_PANROTATE  3
#define APP_MODE_PANTRACK   4
#define APP_MODE_MOVETO     5
#define APP_MODE_HOMING     6
#define APP_MODE_CALIBRATE  7
#define APP_MODE_SAVE       8
#define APP_MODE_INFO       9

#define DEFAULT_100CM_RAILSIZE  142400    // default rail size in steps. 100cm VProfil has a useable lenght of about 89cm
#define DEFAULZ_80CM_RAILSIZE   110400    // default rail size in steps for an 80cm VProfil. Useable lenght about 69cm
#define HOMING_RAIL_SPEED       100

#define EEP_MARKER 13

class SliderData
{

private:

// data
public:
int railSpeedRPM;
uint8_t railSpeedMMPS;
int camAngle;                     // current cam angle for pan mode
int startAngle = 30;              // cam start angle Pan&Rotate
int endAngle;
int rotateSpeedTRPM;
uint8_t startXPos;
uint8_t endXPos;
uint8_t distanceTrackCM = 50;
uint8_t railAutoReturn;
uint8_t remoteTrigger;
uint8_t remoteRelaisActiv = 0;
uint8_t invertRotation = 0;

int trackRotateSpeedTPRM;

uint8_t railDirection = SLIDE_TORIGHT;   // Motor dreht counterclockwise, Slider fährt nach rechts
uint8_t camDirection = CAM_COUNTERCLOCKWISE;   // Motor dreht counterclockwise, Slider fährt nach rechts

uint8_t homingStarted = 0;
uint8_t railHomeDone = 0;
uint8_t camHomeDone = 0;
uint8_t AllHomingDone = 0;
uint8_t calibrateStarted = 0;
uint8_t calibrateDone = 0;
uint8_t dataSaveDone = 0;

int batteryVoltage;
// --- vars for ui and state machine ---
uint8_t actModeNo;
uint8_t actPanItem;
uint8_t actRotateItem;
uint8_t actPanRotateItem;
uint8_t actPanTrackItem;
uint8_t actMoveToItem;
uint8_t actHomingItem;
uint8_t actCalibrateItem;

unsigned long destStepPosition;
unsigned long infoRailSizeSteps;

// prototypes
public:
SliderData(int initRailRPM, int initCamRPM);
~SliderData();

void filterBatteryVoltage(int actValue);
float CalculateAngleDeg(float againstCathete, float onCathete);
float CalculateTrackRotSpeedTRPM(float angleToTravelDeg, float travelTimeSeconds, uint8_t gearReduction);
unsigned long CmToSteps(uint8_t cm, uint8_t railSizeDm, unsigned long railSizeSteps);

void EEPROMWriteInt(int p_address, int p_value);
unsigned int EEPROMReadInt(int p_address);
void EEPROMWriteLong(int address, long value);
long EEPROMReadLong(int address);

};
