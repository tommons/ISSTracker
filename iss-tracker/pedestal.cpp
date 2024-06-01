/*
  pedestal.cpp - Functions to control pedestal orientation and pointer elevation
 */
#include "pedestal.h"
#include <elapsedMillis.h>

// Convert # of steps in stepper motor to equivalent relative pedestal angle in degrees
double steps2deg(long steps) {
    return double(-steps) * 360.0 / STEPS_PER_REV;
}

// Convert pedestal angle in degrees to equivalent # of steps in stepper motor
long deg2steps(double azDeg) {
    return floor(-azDeg * STEPS_PER_REV / 360.0);
}

// Initialize Servo, Stepper, and Compass (if Active)
void Pedestal::begin() {
    // Initialize Servo
    servo.attach(SERVO_PIN);
    servo.write(90);

    // Initialize stepper
    stepper = AccelStepper(AccelStepper::FULL4WIRE, STEP1, STEP2, STEP3, STEP4);
    stepper.setMaxSpeed(STEPPER_SPEED);
    stepper.setAcceleration(STEPPER_ACCEL);
    
    // Initialise the compass
    compass = Adafruit_MMC5603(12345);
    if (!DO_BYPASS_COMPASS and !compass.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
        Serial.println("No compass detected");
        while (CHECK_COMPASS_CONNECTION) delay(10);
    }

    // Calibrate the compass.  Do a full 360 spin
    Serial.println("Calibrating Compass");
    
    compass.getEvent(&compassEvent);
    min_x = max_x = compassEvent.magnetic.x;
    min_y = max_y = compassEvent.magnetic.y;
    min_z = max_z = compassEvent.magnetic.z;

    elapsedMillis printTime = 0;  
    elapsedMillis sampleTime = 0;  

    cal_x = -27.18;
    cal_y = 6.53;
    cal_z = 30.03;

    if( true )
    {
        cal_x = 0;
        cal_y = 0;
        cal_z = 0;
      stepper.move(deg2steps(360*2));
      while (stepper.distanceToGo() != 0) 
      {
          if( sampleTime > 100 )
          {
              compass.getEvent(&compassEvent);
            
              min_x = min(min_x, compassEvent.magnetic.x);
              min_y = min(min_y, compassEvent.magnetic.y);
              min_z = min(min_z, compassEvent.magnetic.z);

              max_x = max(max_x, compassEvent.magnetic.x);
              max_y = max(max_y, compassEvent.magnetic.y);
              max_z = max(max_z, compassEvent.magnetic.z);

              cal_x = (max_x + min_x) / 2;
              cal_y = (max_y + min_y) / 2;
              cal_z = (max_z + min_z) / 2;

              if( printTime > 500 )
              {
                Serial.print(" Hard offset: (");
                Serial.print(cal_x); Serial.print(", ");
                Serial.print(cal_y); Serial.print(", ");
                Serial.print(cal_z); Serial.println(")");  
                double heading =  atan2(compassEvent.magnetic.y - cal_y,compassEvent.magnetic.x - cal_x) * RAD_TO_DEG;
                Serial.print("Heading: ");
                Serial.println(heading);
                printTime = 0;
              }
              sampleTime = 0;
          }
          stepper.run();
      }
    }
    
}

// Set both pedestal azimuth and pointer elevation to zero
void Pedestal::zero() {
    // Reset Az/El Position to zero
    servo.write(90);
    stepper.runToNewPosition(0);
}

// Set target pedestal azimuth
void Pedestal::setTargetAz(double targetAzDeg) {
    double currAz = steps2deg(stepper.currentPosition());
    //double currAz = getAverageHeading();

    currAz = fmod(currAz + 360,360.0);

    Serial.printf("currAz: %0.3f\n",currAz);

    // Compute which direction is closer to current az
    double relAz = targetAzDeg - currAz;
    if (relAz > 180.0) {
        relAz -= 360.0;
    } else if (relAz < -180.0) {
        relAz += 360.0;
    }

    Serial.printf("relAz: %0.3f\n",relAz);

    stepper.move(deg2steps(relAz));
}

// Set pointer elevation
void Pedestal::setElevation(double el) {
    // Convert double elevation to long hundredths of degrees to improve precision
    servo.writeMicroseconds(map(long(el*100),0,18000,SERVO_MAX_PWM,SERVO_MIN_PWM));
}

// Run stepper if needed
void Pedestal::runStepper() {
    stepper.run();
}

// Attempt to point pedestal northward based on average of multiple compass measurements
double Pedestal::pointToHeading(double heading) {
    double currAz, relAz;
    currAz = getAverageHeading();
    relAz =  getHeadingError( heading, currAz );

    Serial.printf("PointToHeading Target Heading (deg): %0.3f\n", heading); 
    Serial.printf("Current Heading (deg): %0.3f\n", currAz);
    Serial.printf("Az To Move (deg) %0.3f\n", relAz);

    stepper.move(deg2steps(relAz));
    elapsedMillis sampleTime = 0;  
    while (stepper.distanceToGo() != 0) 
    {
        // check where we are
        if( 0 && sampleTime > 300 )
        {
          stepper.stop();
          delay(1000);
          double az = getAverageHeading();
          double error =  getHeadingError( heading, az );
          Serial.printf("Error check: az %0.1f err %0.1f\n", az, error);
          sampleTime = 0;
        }
        stepper.run();
    }
    currAz = getAverageHeading();

    double error =  getHeadingError( heading, currAz );

    Serial.printf("Az After Move (deg): %0.3f Error: %0.3f\n", currAz, error);

    return error;
}

double Pedestal::getHeadingError( double targetAz, double currentAz )
{
    double error = targetAz - currentAz;
    if (error > 180.0) {
        error -= 360.0;
    } else if (error < -180.0) {
        error += 360.0;
    }

    return error;
}

// Get reported compass heading in degrees from a single measurement
double Pedestal::getHeading() {
    // Get compass heading
    compass.getEvent(&compassEvent);
    double heading = atan2(compassEvent.magnetic.y - cal_y,compassEvent.magnetic.x - cal_x) * RAD_TO_DEG;
    heading += 90; // offset for pointer direction vs compass direction
    heading = fmod(heading + 360.0, 360.0);

    //Serial.printf("  getHeading %0.3f\n", heading);
    return heading;
}


// Get average compass heading over multiple measurements
double Pedestal::getAverageHeading() {
    // Compute average heading over a number of samples
    const size_t nSamples=100;
    double heading = 0;
    for (size_t i = 0; i < nSamples; ++i) {
        double h = getHeading();
        heading += h;
    }
    heading /= nSamples;
    heading += TRUE_NORTH_OFFSET_DEG;
    heading = fmod(heading + 360,360);

    return heading;
}
