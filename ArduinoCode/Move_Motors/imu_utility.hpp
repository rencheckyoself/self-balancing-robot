#ifndef IMU_UTILITY_HG
#define IMU_UTILITY_HG

void displaySensorStatus(Adafruit_BNO055 &bno);
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
void loadCalibaration(Adafruit_BNO055 &bno);

#endif
