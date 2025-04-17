#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include "AccelStepper.h"

#define PI (3.14159265358979)
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1
#define speedCap (1000)

// SDA to A4
// SCL to A5
// bno055 setup

Adafruit_BNO055 bno = Adafruit_BNO055(55);

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialize the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  double kP = 10;
  double kI = 0;
  double kD = 1.6248;
  double inte = 0; //double integral assignment
  double old_error = 0; //old error assignment
  double setpoint = 0;
  double dt = .01;
  double integral_clipping = 3; // max integral value
  double error = 0;

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(50000);
}


void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t orientationData; 
  sensors_event_t angVelocityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  // rotational position in rad
  // angular velocity in rad/s

  double x = 0;
  double vx = 0;

  x = orientationData.orientation.x;
  x = x * (PI / 180.0);
  vx = angVelocityData.gyro.x;

  Serial.print("x:");
  Serial.print(x);
  Serial.print(",");
  Serial.print("vx:");
  Serial.print(vx);

  error = x - setpoint;
  inte += error * dt;
  u = kP * error + kI * inte + kD * ((error - old_error)/dt);

  /*double wrapped_x = fmod(x - setpoint, 2 * PI);
  if (wrapped_x > PI) {
    wrapped_x -= 2*PI;
  }
  else if (wrapped_x < -PI) {
    wrapped_x += 2*PI;
  }

  inte += wrapped_x * dt;*/

  // arb clipping
  
  inte = constrain(inte, -integral_clipping, integral_clipping);
  double out_vel = kP * wrapped_x + kI * inte + kD * vx;
  Serial.print(",");
  Serial.print("PID vel:");
  Serial.println(out_vel);
  
  

  delay(100);
}

void drive_motor(double vel)
{
  if ((vel <= speedCap) && (vel >= 0)) {
    speed += vel;

}