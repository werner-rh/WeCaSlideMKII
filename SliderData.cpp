#include "EEPROM.h"
// Class for application data for global access


#include "SliderData.h"



SliderData::SliderData(int initRailRPM, int initCamRPM)
{
  this->railSpeedRPM = initRailRPM;
  this->rotateSpeedTRPM = initCamRPM;
  this->railSpeedMMPS = 50;
  this->camAngle = 90;
  this->railAutoReturn = 0;
  this->remoteTrigger = 0;

  this->actModeNo = APP_MODE_HOMING;
}

SliderData::~SliderData()
{

}


/***
  Because the Nano ADC isn't so accurate and stable, we use a filter to integrate new values.
*/
void SliderData::filterBatteryVoltage(int actValue) {
  int valY;
  int valX;

  valX = actValue * 0.1;
  valY = batteryVoltage * 0.9;
  batteryVoltage = valX + valY + 0.05;  // due to a little lost of voltage on the cable, i add 0.05V. 
                                        // This is the difference to the messured value directly at the battery. 
}

float SliderData::CalculateAngleDeg(float againstCathete, float onCathete) {
  float angleRadian;
  float angleDegree;

  angleRadian = atan(againstCathete / onCathete);
  angleDegree = angleRadian*180/3.14159265;

  return angleDegree;
}


float SliderData::CalculateTrackRotSpeedTRPM(float angleToTravelDeg, float travelTimeSeconds, uint8_t gearReduction) {
  return (angleToTravelDeg / (360 * travelTimeSeconds)) * 60.0 * 10.0 * gearReduction;  // result in tenth rotattions per minute
}

unsigned long SliderData::CmToSteps(uint8_t cm, uint8_t railSizeCm, unsigned long railSizeSteps) {
  return (railSizeSteps * (unsigned long)cm / (unsigned long)railSizeCm);
}


//This function will write a 2 byte integer to the eeprom at the specified address and address
void SliderData::EEPROMWriteInt(int p_address, int p_value)
	{
	byte lowByte = ((p_value >> 0) & 0xFF);
	byte highByte = ((p_value >> 8) & 0xFF);

	EEPROM.write(p_address, lowByte);
	EEPROM.write(p_address + 1, highByte);
	}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int SliderData::EEPROMReadInt(int p_address)
	{
	byte lowByte = EEPROM.read(p_address);
	byte highByte = EEPROM.read(p_address + 1);

	return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
	}

// Funktion zum Schreiben eines long-Werts in das EEPROM
void SliderData::EEPROMWriteLong(int address, long value) {
  // Zerlegen des long-Werts in 4 Bytes
  byte byte1 = (value >> 24) & 0xFF;
  byte byte2 = (value >> 16) & 0xFF;
  byte byte3 = (value >> 8) & 0xFF;
  byte byte4 = value & 0xFF;

  // Schreiben der Bytes in das EEPROM
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
  EEPROM.write(address + 2, byte3);
  EEPROM.write(address + 3, byte4);
}

// Funktion zum Auslesen eines long-Werts aus dem EEPROM
long SliderData::EEPROMReadLong(int address) {
  // Lesen der Bytes aus dem EEPROM
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  byte byte3 = EEPROM.read(address + 2);
  byte byte4 = EEPROM.read(address + 3);

  // Zusammenfügen der Bytes zu einem long-Wert
  long value = ((long)byte1 << 24) | ((long)byte2 << 16) | ((long)byte3 << 8) | byte4;
  return value;
}



