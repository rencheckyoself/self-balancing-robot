#include <EEPROM.h>
#include <Wire.h>

#include <QuadratureEncoder.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <PID_v1.h>

#include "Setup.h"
#include "imu_utility.hpp"
#include "motor_driver_utility.hpp"

#define READ_ANGLE euler.y()


// initialize encoders
static Encoders leftEncoder(MOTOR_LEFT_ENC1, MOTOR_LEFT_ENC2);
static Encoders rightEncoder(MOTOR_RIGHT_ENC1, MOTOR_RIGHT_ENC2);

static const int counts_per_rev = 900; // revolutions of the wheel
static int cnt = 1;

static volatile int left_enc_cnt = 0, right_enc_cnt = 0;

static int dirMotors, posLMotor, posRMotor, selectMotors;
static int dirLMotor, dirRMotor;

static Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

static double Setpoint = HOME_ANGLE, Input, Output;
static double Kp=2, Ki=5, Kd=1;
static PID anglePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialize the sensor */
  if (!bno.begin())
  {
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
  }

  loadCalibaration(&bno);

  anglePID.SetSampleTime(10); // 100Hz, determined by max sensor read rate.
  anglePID.SetOutputLimits(0,255); //PWM Output
  
  //initialize DC Motors
  pinMode(MOTOR_RIGHT_PWM,OUTPUT);
  pinMode(MOTOR_RIGHT_1,OUTPUT);
  pinMode(MOTOR_RIGHT_2,OUTPUT);
  pinMode(MOTOR_STANDBY,OUTPUT);
  pinMode(MOTOR_LEFT_PWM,OUTPUT);
  pinMode(MOTOR_LEFT_1,OUTPUT);
  pinMode(MOTOR_LEFT_2,OUTPUT);

  // Disable motors
  digitalWrite(MOTOR_STANDBY,LOW);
  
  Serial.println("DC Motors Initialized");

  delay(1000);
  
  Serial.println("Ready!");
}


void loop() {

  int start = 0;
  int left_done = 0, right_done = 0;
  float error = 0;
  float command = 0;
  
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  while( Serial.available() == 0){ } // While no serial data being sent from computer

  if(Serial.available() > 0)
  {
    start = Serial.parseInt(); // Read in serial data
    Serial.read();
    Serial.print("Start = ");
    Serial.println(start);
 
    while(start == 1)
    {
    
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      Input = READ_ANGLE;

      anglePID.Compute();

            
      
      if(READ_ANGLE > HOME_ANGLE)
      {
        moveMotorFWD(Output);
      }

      else if(READ_ANGLE < HOME_ANGLE)
      {
        moveMotorBKD(Output);
      }
      
//      if(leftEncoder.getEncoderCount() >= counts_per_rev*cnt)
//      {
//        Serial.println("DoneL");
//        analogWrite(MOTOR_LEFT_PWM, 0);
//        digitalWrite(MOTOR_LOW_2, HIGH);
//        left_done = 1;
//      }
//      
//      if(rightEncoder.getEncoderCount() >= counts_per_rev*cnt)
//      {
//        Serial.println("DoneR");
//        analogWrite(MOTOR_RIGHT_PWM, 0);
//        digitalWrite(MOTOR_RIGHT_2, HIGH);
//        right_done = 1;
//      }
//
//      if(left_done == 1 && right_done == 1)
//      {
//        start = 0;
//        right_done = 0;
//        left_done = 0;
//        cnt++;
//        Serial.println(cnt);
//      }
    }

    
   digitalWrite(MOTOR_STANDBY,LOW); //Disable Motors 
   
  }
}
