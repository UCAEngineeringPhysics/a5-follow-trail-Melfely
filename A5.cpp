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
    Drivetrain::MotorInit RightMotorInit(7,9,8,16,17);
    Drivetrain::MotorInit LeftMotorInit(15,13,14,18,19);
    

    Drivetrain::EncodedDualMotor Drive(12, LeftMotorInit, RightMotorInit);

    Drive.SetState(true);
   
    Drive.SetSpeed(0.2);
    //Drive.DriveAmount(0.2);
    Drive.RotateAmount(180.0f);

    while(true);
    
    // int runTime = 6000;
    // int sleepTime = 10;

    // sleep_ms(2000);

   
    // while(runTime > 0){
    //     if (runTime > 1000 && runTime < 5000) {
    //          Drive.Forward(0.2);
    //     } else if (runTime < 1000) {
    //         Drive.Stop();
    //     }

    //     runTime = runTime - sleepTime;
    //     printf("0.200 %.4f\n", Drive._LeftMotor()->LinearVelocity() );
    //     sleep_ms(sleepTime);
        
    // }
    // Drive.SetState(false);
}
