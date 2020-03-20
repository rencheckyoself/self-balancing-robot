
#include "Arduino.h"

#include "motor_driver_utility.hpp"
#include "Setup.h"

static dir cur_direction = FWD;

/// \brief Function to move the motors forward
/// \param command: value 0 to 1 representing the percent effort to apply
void moveMotorFWD(int command)
{
    cur_direction = FWD;
//    digitalWrite(MOTOR_STANDBY,LOW); //Disable Motors

    // Set Directions    
    digitalWrite(MOTOR_RIGHT_1, HIGH);
    digitalWrite(MOTOR_RIGHT_2, LOW);

    digitalWrite(MOTOR_LEFT_1, HIGH);
    digitalWrite(MOTOR_LEFT_2, LOW);

    commandMotor(command);
}

/// \brief Function to move the motors backward
/// \param command: value 0 to 1 representing the percent effort to apply
void moveMotorBKD(int command)
{
    cur_direction = BKD;
//    digitalWrite(MOTOR_STANDBY, LOW); //Disable Motors

    // Set Directions    
    digitalWrite(MOTOR_RIGHT_1, LOW);
    digitalWrite(MOTOR_RIGHT_2, HIGH);
    
    digitalWrite(MOTOR_LEFT_1, LOW);
    digitalWrite(MOTOR_LEFT_2, HIGH);

    commandMotor(command);
}


/// \brief Function to to turn the motors ON
/// \param command (float): value 0 to 1 representing the percent effort to apply
void commandMotor(int command)
{
   
    int pwm_cmd = command; //(command/STALL_TORQUE)*255;
    // Set Effort
    analogWrite(MOTOR_RIGHT_PWM, pwm_cmd);
    analogWrite(MOTOR_LEFT_PWM, pwm_cmd);
}
