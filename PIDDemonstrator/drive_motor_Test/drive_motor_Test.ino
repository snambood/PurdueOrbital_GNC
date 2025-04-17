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

// Create and instance of the stepper motor
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

double inte = 0; // integral assignment
double old_error = 0; //old error assignment
double old_time

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
  double setpoint = 0;
  double old_time = millis();
  double integral_clipping = 3; // max integral value
  double error = 0;

  current_time = millis()

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(50000);
}


void loop(void) 
{
  // rotational position in rad
  // angular velocity in rad/s
  double x = get_ang_position();
  double vx = get_ang_velocity();

  print_ang_vel(x, vx);
  
  error = x - setpoint;
  double dt = millis() - old_time; // This is in milliseconds
  dt /= 1000;                      // Convert to seconds
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


  print_PID(out_vel)

  drive_motor(out_vel) 
  
  old_time = millis()
  delay(100);
}

/*
This function uses the AdaFriut library to get the orientation data
  Arguments:
    None
  
  Output:
    (double) angular velocity in radians / s
*/
double get_ang_velocity(void){
  sensors_event_t angVelocityData; 
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_EULER);

  return(angVelocityData.gyro.x)
}

/*
This function uses the AdaFriut library to get the orientation data
  Arguments:
    None
  
  Output:
    (double) angular orientation in radians
*/
double get_ang_position(void){
  sensors_event_t orientationData; 
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  x = orientationData.orientation.x;
  x = x * (PI / 180.0);

  return(x)
}

/*
This function prints position and velocity to the serial monitor
  Arguments:
    (double) Current Angular Position
    (double) Current Angular Velocity
  
  Output:
    None
*/
void print_ang_vel(double position, double velocity){
  Serial.print("Current Position:");
  Serial.print(position);
  Serial.print(" rad, Current Velocity: ");
  Serial.print(velocity);
  Serial.print(" rad/s")
}


/*
This function prints the PID output and returns to next line
  Arguments:
    (double) PID velocity
  
  Output:
    None
*/
void print_PID(double vel){
  Serial.print(" , PID vel: ");
  Serial.print(vel);
  Serial.println(" m/s")
}

/*
This function runs the motor at a given velocity
*/
void drive_motor(double vel)
{
  stepper.setSpeed(speed);
  stepper.runSpeed();
}

