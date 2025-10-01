#include <stdint.h>
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
 * 
 * 
 */

#include <Arduino.h>

class WeCaDisplay
{
private:
    /* data */
    char appVersion[5];


public:
    WeCaDisplay(const char * version);
    void Setup();
    void WelcomeScreen();
    void StatusLine();
    
    void SetCursor(uint8_t x, uint8_t y, uint8_t cactive);
    void ItemCursor(uint8_t selectedItem, uint8_t itemActive);
    void DrawCheckbox(uint8_t x, uint8_t y, uint8_t checked);

    void PanScreen(uint8_t selectedItem, uint8_t itemActive);
    void RotateScreen(uint8_t selectedItem, uint8_t itemActive);
    void PanRotateScreen(uint8_t selectedItem, uint8_t itemActive);
    void PanTrackScreen(uint8_t selectedItem, uint8_t itemActive);
    void MoveToScreen(uint8_t selectedItem, uint8_t itemActive);
    void HomingScreen(uint8_t selectedItem, uint8_t itemActive);
    void CalibrateScreen(uint8_t selectedItem, uint8_t itemActive);
    void SaveScreen(uint8_t selectedItem, uint8_t itemActive);
    void InfoScreen(uint8_t selectedItem, uint8_t itemActive);
    void ScreenOut(uint8_t selectedItem, uint8_t itemActive);
};


