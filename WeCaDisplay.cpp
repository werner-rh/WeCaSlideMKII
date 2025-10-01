/***
 * Project: Camera Slider motorized
 * File   : CSDisplay.h
 * Author : Werner Riemann 
 * Created: 15.12.2022
 * Board: Arduino Nano
 * 
 * Description: Modul for Display outputs
 * 
 * Pins:
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * 
 */

// --- see u8g2 documentation --------------------------
// https://github.com/olikraus/u8g2/wiki
// https://github.com/olikraus/u8g2/wiki/fntlistall
// Fontübersicht 
// https://github.com/olikraus/u8g2/wiki/fntgrpadobex11

#include "WeCaDisplay.h"
#include "SliderData.h"

#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


extern SliderData slData;

char szModNames[9][12] = {
  "Pan",
  "Rotate",
  "Pan &Rotate",
  "Pan &Track",
  "Move To",
  "Homing",
  "Calibrate",
  "Save",
  "Info"
};


char szParamNames[11][7] = {
  "Speed",
  "Angle",
  "SAngle",
  "EAngle",
  "RSpeed",
  "StartX",
  "EndX",
  "XPos",
  "InvRot",
  "ARet.",
  "ATrig."
};

char szMoveNames[4][7] = {
  "Left",
  "Center",
  "Right",
  "HAngle"
};

char szDone[6] = "Done!";
char szWait[6] = "Wait!";
char szWeCaSlide[15] = "WeCaSlide MKII";
char szVersion[9] = "Version:";
char szRail[5] = "Rail";
char szSize[5] = "Size";

char szDbPoint[2] = ":";
char szUnitDegree[4] = "deg";
char szUnitMMS[5] = "mm/s";
char szUnitRPM[4] = "rpm";
char szUnitTRPM[5] = "trpm";    // rotation speed in tenth rpm to enable slower speeds
char szUnitMM[3] = "mm";
char szUnitCM[3] = "cm";


U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // Global variables use 660 bytes (32%) of dynamic memory, leaving 1388 bytes for local variables. Maximum is 2048 bytes.


WeCaDisplay::WeCaDisplay(const char * version)
{
  strcpy(this->appVersion, version);
}

void WeCaDisplay::Setup()
{
  u8g2.begin();
}


// Fonts i am using
// u8g2_font_helvR10_tr , u8g2_font_helvB10_tr , u8g2_font_helvR08_tr

void WeCaDisplay::WelcomeScreen()
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB10_tr);
    u8g2.drawStr(6,30,szWeCaSlide);

    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.drawStr(20,50,szVersion);
    u8g2.drawStr(80,50,this->appVersion);

    u8g2.drawFrame(0,0,u8g2.getDisplayWidth(),u8g2.getDisplayHeight() );

  } while ( u8g2.nextPage() );
}


void WeCaDisplay::SetCursor(uint8_t x, uint8_t y, uint8_t cactive) {

  if(cactive == 1) {
    u8g2.drawTriangle(x, y-1, x, y+9, x+5, y+5);
  } else {
    u8g2.drawLine( x,  y,  x+4,  y+4);
    u8g2.drawLine( x+4,  y+4,  x,  y+8);
  }
  
}

void WeCaDisplay::DrawCheckbox(uint8_t x, uint8_t y, uint8_t checked) {

  u8g2.drawFrame(x, y-8, 8, 8);
  if(checked == 1) 
    u8g2.drawBox(x+2, y-6, 4, 4);
    
}

void WeCaDisplay::ItemCursor(uint8_t selectedItem, uint8_t itemActive) {
  if(selectedItem == MSELECT_ITEM_PARAM_NO1)
    SetCursor(0, 15, itemActive);
  if(selectedItem == MSELECT_ITEM_PARAM_NO2)
    SetCursor(0, 27, itemActive);  
  if(selectedItem == MSELECT_ITEM_PARAM_NO3)
    SetCursor(0, 39, itemActive);  

  if(selectedItem == MSELECT_ITEM_PARAM_NO4)
    SetCursor(58, 15, itemActive);  
  if(selectedItem == MSELECT_ITEM_PARAM_NO5)
    SetCursor(58, 27, itemActive);  
  if(selectedItem == MSELECT_ITEM_PARAM_NO6)
    SetCursor(58, 39, itemActive);  
}

void WeCaDisplay::PanScreen(uint8_t selectedItem, uint8_t itemActive) {
  char buf[4]="";

  itoa(slData.railSpeedMMPS, buf, 10);
  u8g2.drawStr(6,24,szParamNames[0]); // rail speed in mm/s
  u8g2.drawStr(36,24,szDbPoint);
  u8g2.drawStr(42,24,buf);
  u8g2.drawStr(64,24,szUnitMMS);
  u8g2.drawStr(6,36,szParamNames[1]); // cam angle
  u8g2.drawStr(36,36,szDbPoint);
  itoa(slData.camAngle, buf, 10);
  u8g2.drawStr(42,36,buf);
  u8g2.drawStr(64,36,szUnitDegree);

  u8g2.drawStr(6,48,szParamNames[9]); // rail Auto Return
  DrawCheckbox(42, 48, slData.railAutoReturn);
  u8g2.drawStr(64,48,szParamNames[10]); // cam remote trigger
  DrawCheckbox(92, 48, slData.remoteTrigger);

  if(selectedItem == MSELECT_ITEM_PARAM_NO4)
    SetCursor(58, 39, itemActive);  
  else
   ItemCursor(selectedItem, itemActive);
}


void WeCaDisplay::RotateScreen(uint8_t selectedItem, uint8_t itemActive) {
  char buf[4]="";
  uint8_t xs[4] = {6,44,50,72};

  u8g2.drawStr(xs[0],24,szParamNames[2]); // start angle
  u8g2.drawStr(xs[1],24,szDbPoint);
  itoa(slData.startAngle, buf, 10);
  u8g2.drawStr(xs[2],24,buf);
  u8g2.drawStr(xs[3],24,szUnitDegree);

  u8g2.drawStr(xs[0],36,szParamNames[4]); // rotate speed
  u8g2.drawStr(xs[1],36,szDbPoint);
  itoa(slData.rotateSpeedTRPM, buf, 10);
  u8g2.drawStr(xs[2],36,buf);
  u8g2.drawStr(xs[3],36,szUnitTRPM);

  ItemCursor(selectedItem, itemActive);
}

void WeCaDisplay::PanRotateScreen(uint8_t selectedItem, uint8_t itemActive) {
  char buf[4]="";
  
  u8g2.drawStr(6,24,szParamNames[5]); // start x in cm
  u8g2.drawStr(38,24,szDbPoint);
  itoa(slData.startXPos, buf, 10);
  u8g2.drawStr(44,24,buf);
  u8g2.drawStr(66,24,szUnitCM);

  u8g2.drawStr(6,36,szParamNames[2]); // start angle
  u8g2.drawStr(38,36,szDbPoint);
  itoa(slData.startAngle, buf, 10);
  u8g2.drawStr(44,36,buf);
  u8g2.drawStr(66,36,szUnitDegree);

  u8g2.drawStr(6,48,szParamNames[8]); // invert Rotation
  DrawCheckbox(52, 48, slData.invertRotation);

  ItemCursor(selectedItem, itemActive);

}

void WeCaDisplay::PanTrackScreen(uint8_t selectedItem, uint8_t itemActive) {
  char buf[4]="";

  itoa(slData.railSpeedMMPS, buf, 10);
  u8g2.drawStr(6,24,szParamNames[0]); // rail speed in mm/s
  u8g2.drawStr(46,24,szDbPoint);
  u8g2.drawStr(52,24,buf);
  u8g2.drawStr(70,24,szUnitMMS);

  u8g2.drawStr(6,36,"Distance"); // end x in cm
  u8g2.drawStr(46,36,szDbPoint);
  itoa(slData.distanceTrackCM, buf, 10);
  u8g2.drawStr(52,36,buf);
  u8g2.drawStr(70,36,szUnitCM);

  ItemCursor(selectedItem, itemActive);
}
//szMoveNames
void WeCaDisplay::MoveToScreen(uint8_t selectedItem, uint8_t itemActive) {
  u8g2.drawStr(6,24,szMoveNames[0]); // Left
  u8g2.drawStr(6,36,szMoveNames[1]); // Center
  u8g2.drawStr(6,48,szMoveNames[2]); // Right

  u8g2.drawStr(64,24,szParamNames[2]); // Start Angle
  u8g2.drawStr(64,36,szParamNames[5]); // StartX
  u8g2.drawStr(64,48,szMoveNames[3]);  // Home Angle

  ItemCursor(selectedItem, itemActive);
}

void WeCaDisplay::HomingScreen(uint8_t selectedItem, uint8_t itemActive) {

  if(slData.homingStarted == 1) {
    u8g2.drawStr(10,30,szModNames[slData.actModeNo -1]); 
    u8g2.drawStr(50,30,"Rail");
    if(!slData.railHomeDone)
      u8g2.drawStr(80,30,szWait);

    if(slData.railHomeDone) {
      u8g2.drawStr(80,30,szDone);
      u8g2.drawStr(10,44,szModNames[slData.actModeNo -1]);
      u8g2.drawStr(50,44,"Cam");
      if(slData.camHomeDone == 1)
        u8g2.drawStr(80,44,szDone);
      else
       u8g2.drawStr(80,44,szWait);
    }
  }
  
}

void WeCaDisplay::CalibrateScreen(uint8_t selectedItem, uint8_t itemActive) {

  if(slData.calibrateStarted == 1) {
    u8g2.drawStr(20,30,szModNames[slData.actModeNo -1]);
    if(slData.calibrateDone) {
      u8g2.drawStr(70,30,szDone);
    } else {
      u8g2.drawStr(70,30,"started");
      u8g2.drawStr(36,44,"Please wait!");
    }
    
  } 

}

void WeCaDisplay::SaveScreen(uint8_t selectedItem, uint8_t itemActive) {
  if(slData.dataSaveDone == 1)
    u8g2.drawStr(44,38,szDone);
}

void WeCaDisplay::InfoScreen(uint8_t selectedItem, uint8_t itemActive) {
  char szBuf[10] = "";
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawStr(6,28,szWeCaSlide); //30

  u8g2.setFont(u8g2_font_helvR08_tr);
  u8g2.drawStr(20,40,szVersion);    //46
  u8g2.drawStr(70,40,this->appVersion);
  u8g2.drawStr(20,52,"Railsize:");    //56
  ultoa(slData.infoRailSizeSteps, szBuf, 10);
  u8g2.drawStr(70,52,szBuf);
}


void WeCaDisplay::ScreenOut(uint8_t selectedItem, uint8_t itemActive) {

  u8g2.firstPage();   // print all items in this loop
  do {
    // --- mainline ---------------------
    u8g2.drawHLine(0, 12, 128); // line to seperate the head line
    
    if(selectedItem == 1) { // draw the cursor for item
        SetCursor(38, 2, itemActive);
    }
    
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(0,10,"Mode:");
    u8g2.drawStr(50,10,szModNames[slData.actModeNo -1]);

    // call app mode specific screens
    u8g2.setFont(u8g2_font_helvR08_tr);
    switch (slData.actModeNo) {
      case APP_MODE_PAN: PanScreen(selectedItem, itemActive);
      break;

      case APP_MODE_ROTATE: RotateScreen(selectedItem, itemActive);
      break;

      case APP_MODE_PANROTATE: PanRotateScreen(selectedItem, itemActive);
      break;

      case APP_MODE_PANTRACK: PanTrackScreen(selectedItem, itemActive);
      break;

      case APP_MODE_MOVETO: MoveToScreen(selectedItem, itemActive);
      break;

      case APP_MODE_HOMING: HomingScreen(selectedItem, itemActive);
      break;

      case APP_MODE_CALIBRATE: CalibrateScreen(selectedItem, itemActive);
      break;

      case APP_MODE_SAVE: SaveScreen(selectedItem, itemActive);
      break;

      case APP_MODE_INFO: InfoScreen(1,1);
      break;
    }

    StatusLine();   // print status line at last
   } while ( u8g2.nextPage() );

}

/***
  Prints battery status information to screen.
  This method has to called inside a u8g2.nextPage() loop.
*/
void WeCaDisplay::StatusLine() {
  char szbuf[8] ="";
  char szval[4] ="";
  uint8_t percentVal = 0;
  uint8_t barWidth=1;
  float batteryVoltage = 5.00 / 1024 * slData.batteryVoltage;

  // battery capacity isn't a linear relation
  if(batteryVoltage < 3.6 )  percentVal = 0;
  if(batteryVoltage >= 3.6)  percentVal = 10;
  if(batteryVoltage >= 3.7)  percentVal = 20;
  if(batteryVoltage >= 3.75)  percentVal = 30;
  if(batteryVoltage >= 3.8)  percentVal = 40;
  if(batteryVoltage >= 3.85)  percentVal = 50;
  if(batteryVoltage >= 3.9)  percentVal = 60;
  if(batteryVoltage >= 3.92)  percentVal = 70;
  if(batteryVoltage >= 3.95)  percentVal = 80;
  if(batteryVoltage >= 4.00)  percentVal = 90;
  if(batteryVoltage >= 4.10)  percentVal = 100;

  u8g2.drawHLine(0, 54, 128); // line to seperate the status line
  u8g2.drawFrame(0,56,14,8 ); // battery symbol

  barWidth = 12 * (int)percentVal/100; // calculate battery bar width
  u8g2.drawBox(1, 57, barWidth, 6); // infill battery symbol
  u8g2.drawVLine(14, 58, 4);    // battery plus pole

  u8g2.setFont(u8g2_font_helvR08_tr); 

  // battery capacity
  itoa(percentVal, szval, 10);
  if(percentVal < 100) strcat(szbuf, " ");
  strcat(szbuf, szval); 
  strcat(szbuf, "%");
  u8g2.drawStr(18,64,szbuf);

  // battery voltage
  dtostrf(batteryVoltage, 3, 2, szbuf); 
  strcat(szbuf, "V");
  u8g2.drawStr(48,64,szbuf);

}
