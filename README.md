Motorized and Arduino controlled Camera Slider
===

## WeCaSlide Version 1.01

## Description
Software for an motorized and Arduino controlled DIY 3D-printed Camera Slider. I called the
project "WeCaSlide MKII". MKII because it is my second Camera Slider, which is a completely
new construction and totaly different from the first project.

The Slider is designed to carry also quite heavy SLR cameras or system cameras with
bigger lenses. For this the slider has to be very stable and contain strong motors.

The Slider is driven by two Nema17 Stepper motors. On for the rail and one for the camera
rotation. The slider is powered by a built in battery pack with two 18650 lion cells in parallel.
The battery pack includes a charging circuit and a little stepup converter to provide 9V for the
stepper motors. 5V for the Arduino is provided by a 5V linear regulator.

For operating a 1.3" OLED screen, a rotary encoder and two push buttons are built in.

##Features:
- Pan mode (only slide along the rail with fix camera angle)
- Rotate (only rotate the camera angle on fix rail position)
- Pan & Rotate (slide along the rail and rotate the cam at same time)
- Pan & Track (slide along the rail with easy object trocking)
- adjustable speed
- adjustable distance for object tracking

All slider actions are started and stoped with the left and right push buttons. 


##Calibration
After assembling, the calibrate function should be executed. For calibration the slider moves
the whole rail from end to end and measures the amount of steps for the whole rail distance.
The measured value is stored into the eeprom of the Arduino and will be reload after power up.
You only need to perform the calibration once.


##How to perform easy object tracking:
- do homing after power up
- move the slider base to center with the "Move To" functions
- measure the distance from the camera (middle of the base) to the object to be tracked (in cm)
- adjust the distance in the Pan & Track menu
- now you can move the base to any starting point you like and start the object tracking
- active the remote trigger option if you want to use it and connect the remote cable to jack in the base and your camera

## Requirements software
- Arduino IDE 1.8.2 or higher
- u8g2 lib for the OLED display 

## Requirements hardware
- Arduino Nano 
- 1.3" OLED display with SH1106 chip
- 2x Nema17 stepper motors 23mm thickness
- 2x DRV8825 stepper driver
- TP4056 / 4096   USB-C charger modul
- 2x Li ion 18650 battery (protected)
- XL6009E voltage booster
- LM7805
- BC548 or similar
- resistor 1.5k
- diode 1N4001
- capacitor 2x 47µF or 1x 100µF
- miniatur relais
- 2x GT2 pulley 20T, GT2 idler, GT2 belt 2m 
- pulley 80T with bearing insert, it's part of 3D print files
- Alu V-Profil 20x40, 100cm (or 80cm if you like it shorter)
- 4x pom wheel for V-slot
- rotary encoder
- 2x push button 
- 2.5 mm bracket for remote camera trigger 
- 2x micro switch
- 3D printing files and complete parts list: https://www.printables.com/@werner_rh_510264/models

- more details and futher parts as screws, bearings are listed with example links in partlist.txt








