/*
  pedestal.h - Pedestal struct definition
 */
#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MMC56x3.h>
#include "coord.h"
#include "defs.h"

double steps2deg(long steps);
long deg2steps(double azDeg);


// Struct that wraps around pedestal control devices (Servo, Stepper, & Compass)
struct Pedestal {
    Servo servo;
    AccelStepper stepper;
    Adafruit_MMC5603 compass;
    sensors_event_t compassEvent;
    float min_x, max_x, cal_x;
    float min_y, max_y, cal_y;
    float min_z, max_z, cal_z;

    void begin();
    void zero();
    void setTargetAz(double azDeg);
    void setElevation(double el);
    void runStepper();
    double pointToHeading(double heading);
    double getHeading();
    double getAverageHeading();
    double getHeadingError( double targetAz, double currentAz );

};
