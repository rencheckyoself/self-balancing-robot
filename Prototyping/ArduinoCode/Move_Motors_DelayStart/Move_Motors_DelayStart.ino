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

static double Setpoint, Input, Output;
static double Kp=40, Ki=0, Kd=90;
static PID anglePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);

static float body_ang=0, abs_ba = 0;
// Function Prototypes
void loadCalibaration(void);

void setup() {

  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialize the sensor */
  if (!bno.begin())
  {
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
  }

  loadCalibaration();
  Setpoint = HOME_ANGLE;
  anglePID.SetSampleTime(1); // 200Hz
  anglePID.SetOutputLimits(-255, 255); //PWM Output
  anglePID.SetMode(AUTOMATIC);
  
  //initialize DC Motors
  pinMode(MOTOR_RIGHT_PWM,OUTPUT);
  pinMode(MOTOR_RIGHT_1,OUTPUT);
  pinMode(MOTOR_RIGHT_2,OUTPUT);
  pinMode(MOTOR_STANDBY,OUTPUT);
  pinMode(MOTOR_LEFT_PWM,OUTPUT);
  pinMode(MOTOR_LEFT_1,OUTPUT);
  pinMode(MOTOR_LEFT_2,OUTPUT);

  // Enable motors
  digitalWrite(MOTOR_STANDBY,HIGH);
  
  Serial.println("DC Motors Initialized");

  delay(1000);
  
  Serial.println("Ready!");
}


void loop() {
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  body_ang = READ_ANGLE;
  Input = body_ang;
  
  anglePID.Compute();
  
  Serial.print(body_ang);
  Serial.print(" ");
  
  if(body_ang >= HOME_ANGLE)
  {
    moveMotorBKD(int(Output)*-1);
    Serial.println(int(Output));
  }
  
  else if(body_ang < HOME_ANGLE)
  {
    moveMotorFWD(int(Output));
    Serial.println(int(Output));
  }   
}


/// \brief Load in Calibration data
///
void loadCalibaration(void)
{
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);  
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
//      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      displaySensorOffsets(calibrationData);

//      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

//      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  delay(1000);

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);

//  sensors_event_t event;
//  bno.getEvent(&event);
//
//  if (foundCalib){
//        Serial.println("Move sensor slightly to calibrate magnetometers");
//        while (!bno.isFullyCalibrated())
//        {
//            bno.getEvent(&event);
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
//    }
//    else
//    {
//        Serial.println("Please Calibrate Sensor: ");
//        while (!bno.isFullyCalibrated())
//        {
//            bno.getEvent(&event);
//
//            Serial.print("X: ");
//            Serial.print(event.orientation.x, 4);
//            Serial.print("\tY: ");
//            Serial.print(event.orientation.y, 4);
//            Serial.print("\tZ: ");
//            Serial.print(event.orientation.z, 4);
//
//            /* Optional: Display calibration status */
//            displayCalStatus(&bno);
//
//            /* New line for the next sample */
//            Serial.println("");
//
//            /* Wait the specified delay before requesting new data */
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
//    }
//    
//    Serial.println("\nFully calibrated!");
//    Serial.println("--------------------------------");
//    Serial.println("Calibration Results: ");
//    adafruit_bno055_offsets_t newCalib;
//    bno.getSensorOffsets(newCalib);
//    displaySensorOffsets(newCalib);
//
//    Serial.println("\n\nStoring calibration data to EEPROM...");
//
//    eeAddress = 0;
//    bno.getSensor(&sensor);
//    bnoID = sensor.sensor_id;
//
//    EEPROM.put(eeAddress, bnoID);
//
//    eeAddress += sizeof(long);
//    EEPROM.put(eeAddress, newCalib);
//    Serial.println("Data stored to EEPROM.");
//
//    Serial.println("\n--------------------------------\n");
//    delay(500);
}
