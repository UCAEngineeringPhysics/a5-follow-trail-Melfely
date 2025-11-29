#include <stdio.h>
#include "pico/stdlib.h"
#include "PWM.h"
#include "Sensor.h"
#include "DriveTrain.h"



int main()
{
    stdio_init_all();

    PWM::LED Red(28);
    PWM::LED Green(27);
    PWM::LED Blue(26);

    Sensor::MotorEncoder RightMotorEncoder(16, 17);
    

    //Drivetrain::DualMotor Drive(12, 7, 9, 8, 15, 13, 14);
    PWM::MOTOR RightMotor(7, 9, 8);
    stdio_init_all();

    //Drive.SetState(true);
    RightMotor.Forward(1);
    
    while (true) {
        printf("Right Wheel Speed: %f \n", RightMotorEncoder.AngularVelocity());
        //RightMotorEncoder.ResetEncoderCount();
        //printf("Test: %d - %d \n", RightMotorEncoder.encoderCounts, RightMotorEncoder.previousCounts);
        //sleep_ms(10);
    }
}
