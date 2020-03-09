#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "imu_utility.hpp"

/* Functions From Examples Files */
/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(Adafruit_BNO055 &bno)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}


/// \brief Load in Calibration data
///
void loadCalibaration(Adafruit_BNO055 &bno)
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
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      displaySensorOffsets(calibrationData);

      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  delay(1000);
  
  bno.setExtCrystalUse(true);
}
