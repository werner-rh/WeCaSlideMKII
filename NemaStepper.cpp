#include "Arduino.h"
/***
 * Project: NemaStepper
 * File   : NemaStepper.cpp
 * Author : Werner Riemann 
 * Created: 28.05.2023
 * Board: Arduino Nano
 * 
 * Description: implements the class
 * 
 * Pins : DirectionPin, StepPin and SleepPin
 * 
 */

#include "NemaStepper.h"

NemaStepper::NemaStepper(uint8_t dirPin, uint8_t stepPin, uint8_t sleepPin, int stepsPerRevolution, uint8_t pulleyCircumference, uint8_t reductionFactor)
{
    motorSignal = 0;

    this->directionPin = dirPin;
    this->steppingPin = stepPin;
    this->sleepingPin = sleepPin;
    this->stepsPerRevolution = stepsPerRevolution;
    this->pulleyCircumference = pulleyCircumference;
    this->curDirection = STEPDIR_CLOCKWISE;
    this->last_wavepart_time = 0;
    this->gearReductionFactor = reductionFactor; // default transmission 1:1
    this->angleStepSize = (unsigned long) stepsPerRevolution * (unsigned long) reductionFactor;

    pinMode(this->steppingPin, OUTPUT);
    pinMode(this->directionPin, OUTPUT);
    pinMode(this->sleepingPin, OUTPUT);
    digitalWrite(this->directionPin, STEPDIR_CLOCKWISE);
    digitalWrite(this->sleepingPin, LOW);  // default motor off    
}

NemaStepper::~NemaStepper()
{
}


void NemaStepper::setSpeedRPM(long speedInRPM) {
  // time in micro seconds for each step depending of the result speed RAMP_ADDSTEPS
  // to enable slow speeds, the param speedInRPM is interpreted as tenth rotations per minute.
  // so we need to 10 tenth for one rotation per minute
  this->step_waveTime = 60L * 1000L * 1000L / this->stepsPerRevolution / ( speedInRPM / 10.0);
  this->step_halfWaveTime = this->step_waveTime / 2;
  //this->rampup_halfWaveTime = this->step_halfWaveTime * RAMP_FAKTOR;
  this->rampup_halfWaveTime = this->step_halfWaveTime + RAMP_ADDSTEPS;
}

void NemaStepper::setSpeedMMPS(uint8_t speedInMMPS) {
  float circumf = this->pulleyCircumference;
  long speedTenthRPM = speedInMMPS / circumf * 60 * 10;
  setSpeedRPM(speedTenthRPM);

}

// schaltet den Motor ab. Lässt sich dann frei drehen. Das ist am Slider nicht unbedingt gewollt.
// Die Kamera sollte gehalten werden. Die Leitung sollte separat geschaltet werden
// Turns off the motor. It can then rotate freely. This isn't necessarily desirable on a slider.
// The camera should be held. The cable should be switched separately.
// 1, HIGH or True enables the MOSFet output of the driver
void NemaStepper::enableMotor(uint8_t enableState) {
  digitalWrite(this->sleepingPin, enableState);
}

void NemaStepper::setDirection(uint8_t direction) {
  this->curDirection = direction;
  digitalWrite(this->directionPin, this->curDirection);
}

uint8_t NemaStepper::getRunState() {
  return this->motorRun;
}

unsigned long NemaStepper::getCurrentStepCount() {
  return this->currentStepCount;
}

void NemaStepper::resetCurrentStepCount() {
  this->currentStepCount = 0;
}

void NemaStepper::setPositionSteps(unsigned long steps) {
  this->curStepPosition = steps;
}

unsigned long NemaStepper::getPositionSteps() {
  return this->curStepPosition;
}

void NemaStepper::setRailSizeSteps(unsigned long totalSteps) {
  this->railSizeSteps = totalSteps;
}

unsigned long NemaStepper::getRailSizeSteps() {
  return this->railSizeSteps;
}

void NemaStepper::setCurrentAngle(int curAngleDegree) {
  this->currentAngle = curAngleDegree;
  this->angleStepCount = angleStepSize * (unsigned long)curAngleDegree / 360.0;
}


unsigned long NemaStepper::getangleStepCount() {
  return this->angleStepCount;
}


int NemaStepper::getCurrentAngle() {
  return this->currentAngle;
}

uint8_t NemaStepper::getGearReductionFactor() {
  return this->gearReductionFactor;
}


void NemaStepper::stopMotor(uint8_t withRamp)
{
  
  if(withRamp == 1) {
    this->ramp_State = STEPPER_RAMPDOWN;
    this->rampdown_halfWaveTime = this->step_halfWaveTime;
    digitalWrite(this->steppingPin, this->motorSignal);

  } else {
    this->motorRun = 0;
    this->motorSignal = 0;
  }

}


void NemaStepper::runMotor()
{
  this->motorRun = 1;
  this->motorSignal = 1;
  this->ramp_State = STEPPER_RAMPUP;
  this->rampup_halfWaveTime = this->step_halfWaveTime + RAMP_ADDSTEPS;
  
  digitalWrite(this->steppingPin, this->motorSignal);
}

void NemaStepper::motorStep()
{
  static uint8_t rdelay=0;
  uint8_t doStepper = 0;
  unsigned long now = micros();

  // check if we are performing ramp up or ramp down or normal run
  switch (this->ramp_State) {
    case STEPPER_RUN:
      if(now - this->last_wavepart_time >= this->step_halfWaveTime)
        doStepper = 1;
    break;

    case STEPPER_RAMPUP:
      if(now - this->last_wavepart_time >= this->rampup_halfWaveTime) {
        if(this->rampup_halfWaveTime  > this->step_halfWaveTime) {
          
          if(rdelay == 0 || this->rampup_halfWaveTime > 200) {
            this->rampup_halfWaveTime --; 
            rdelay = 4;
          }
            
          else
           rdelay --;
        } else {
          this->ramp_State = STEPPER_RUN;
          rdelay = 4;
        }
        doStepper = 1;
      }
    break;

    case STEPPER_RAMPDOWN:
      if(now - this->last_wavepart_time >= this->rampdown_halfWaveTime) {
        if(this->rampdown_halfWaveTime <  RAMP_ADDSTEPS) {
          if(this->rampdown_halfWaveTime < RAMP_ADDSTEPS/2) {
            if(rdelay == 0) {
              this->rampdown_halfWaveTime ++;
              rdelay = 4;
            } else {
            rdelay--;
            }
          } else {
          this->rampdown_halfWaveTime ++;
          }
           
        } else {  // ramp down done
          this->motorRun = 0;
          this->motorSignal = 0;
        }

        doStepper = 1;
      }
    break;
  }


  // toggle the motorSignal if half of the full wave is passed
  if(doStepper == 1 && this->motorRun == 1)
  {
    this->motorSignal ^= 1;
    digitalWrite(this->steppingPin, this->motorSignal);
    this->last_wavepart_time = now;

    if(this->motorSignal) {

      if(this->curDirection == STEPDIR_CLOCKWISE) {
        this->currentStepCount ++;
        this->curStepPosition ++;
        
        if(this->angleStepCount >= angleStepSize -1)
          this->angleStepCount = 0;
        else
          this->angleStepCount ++;
      } 
      else {
        this->currentStepCount --;
        this->curStepPosition --;

        if(this->angleStepCount <= 1)
          this->angleStepCount = angleStepSize -1;
        else
          this->angleStepCount --;
      }
      //calculate and set current angle
      this->currentAngle =   (this->angleStepCount * 360.0) / angleStepSize;
   
    }
      
  }


}

