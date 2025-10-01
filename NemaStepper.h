#include <stdint.h>
/***
 * Project: NemaStepper
 * File   : NemaStepper.h
 * Author : Werner Riemann 
 * Created: 28.05.2023
 * Board: Arduino Nano
 * 
 * Description: Class definition for simple controlling a Nema17 Steppermotor
 *              using a A4988 or DRV8825 driver board. The class is only to rotate the
 *              step motor in different directions and different speeds.
 * 
 * Pins : DirectionPin, StepPin and SleepPin
 * 
 */
 
 #include <Arduino.h>

#define STEPDIR_CLOCKWISE 1
#define STEPDIR_COUNTERCLOCKWISE 0 
#define STPST_IDLE 0
#define STPST_RAMPUP 2
#define STPST_RUN  10

#define STEPPER_RUN 1
#define STEPPER_RAMPUP 2
#define STEPPER_RAMPDOWN 3

#define RAMP_FAKTOR 12
#define RAMP_ADDSTEPS   440

class NemaStepper
{
private:
    /* data */
    uint8_t directionPin;
    uint8_t steppingPin;
    uint8_t sleepingPin;
    int stepsPerRevolution;   // total number of steps for one revolution
    uint8_t pulleyCircumference;   // the circumference of the installed motor pulley in mm
    unsigned long step_waveTime; // delay time between steps, in ms, based on speed. This is the time for a full wave
    unsigned long step_halfWaveTime; // time for a half wave. Time to change from High part to low part of the full wave
    unsigned long rampup_halfWaveTime; // time for a half wave. Time to change from High part to low part of the full wave
    unsigned long rampdown_halfWaveTime; // time for a half wave. Time to change from High part to low part of the full wave
    unsigned long last_wavepart_time; // time stamp in us of when the last step was taken
    uint8_t ramp_State = STEPPER_RUN;

    unsigned long currentStepCount = 0;
    unsigned long curStepPosition = 0;   // to track the position on a rail in steps
    unsigned long railSizeSteps;     // represents the rail size in motor steps
    unsigned long angleStepCount =0;
    unsigned long angleStepSize =0;   // amount of steps for one 360 degree rotation  of the outer gear depending of the transmission after the motor axle
    int currentAngle;       // current angle in degree (0-359, 360=0) of the motor axle. related to the positon defined at start up.
                            // Clockwise rotation increment the angle, counter clockwise decrement the angle.
    uint8_t gearReductionFactor = 1;
    uint8_t curDirection;   // Direction of the motor axle. Seen from top on the axle: High(1) = clockwise, Low(0) = counter clockwise.
                            // invert the order of the motor cables to ensure the direction is correct 
    uint8_t motorRun=0;       // Motor Run/Stop Flag (1 Run, 0 Stop)
    uint8_t motorSignal=0;

public:
    NemaStepper(uint8_t dirPin, uint8_t stepPin, uint8_t sleepPin, int stepsPerRevolution, uint8_t pulleyCircumference, uint8_t reductionFactor);
    ~NemaStepper();

    void enableMotor(uint8_t enableState);
    uint8_t getRunState();
    unsigned long getCurrentStepCount();
    void resetCurrentStepCount();
    void setPositionSteps(unsigned long steps);
    unsigned long getPositionSteps();
    void setRailSizeSteps(unsigned long totalSteps);
    unsigned long getRailSizeSteps();
    
    void setSpeedRPM(long speedInRPM);
    void setSpeedMMPS(uint8_t speedInMMPS);
    void setDirection(uint8_t direction);
    void setCurrentAngle(int curAngleDegree);
    int getCurrentAngle();
    unsigned long getangleStepCount();
    uint8_t getGearReductionFactor();
    //uint8_t getRampState();
    
    void motorStep();
    void stopMotor(uint8_t withRamp);
    void runMotor();
    
    
};



