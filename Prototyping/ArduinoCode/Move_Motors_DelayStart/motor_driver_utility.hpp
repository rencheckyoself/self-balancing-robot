#ifndef MOTOR_DRIVER_UTILITY_HG
#define MOTOR_DRIVER_UTILITY_HG

typedef enum dir {FWD, BKD} dir;

void moveMotorFWD(int command);
void moveMotorBKD(int command);
void commandMotor(int command);



#endif
