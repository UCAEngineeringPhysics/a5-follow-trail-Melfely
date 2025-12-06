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
    Drivetrain::MotorInit RightMotorInit(15,14,13,10,11); //Motor Bank A
    Drivetrain::MotorInit LeftMotorInit(16,17,18,20,19); //Motor Bank B
    

    Drivetrain::EncodedDualMotor Drive(12, LeftMotorInit, RightMotorInit);

    Drive.SetState(true);
   
    Drive.SetSpeed(0.2);

    sleep_ms(2000);

    #pragma region Main Drive

    Drive.DriveAmount(0.75); //From Checkpoint 0 to 1

    while(Drive.IsRotating()) {
        Green.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Green.SetState(false);
    sleep_ms(3000);

    Drive.RotateAmount(-90); //Spin to aim at Checkpoint 2

    while(Drive.IsRotating()) {
        Blue.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Blue.SetState(false);

    sleep_ms(1000); //Short delay to not over or underspin

    Drive.DriveAmount(0.25); //Drive to halfway point between check point 1 and 2

    while(Drive.IsRotating()) {
        Green.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Green.SetState(false);

    sleep_ms(100); //Stop for hundred milisecond

    Drive.DriveAmount(0.25); //Drive to checkpoint 2 from halfway point

    while(Drive.IsRotating()) {
        Green.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Green.SetState(false);

    sleep_ms(3000); //Stop for 3 seconds

    Drive.RotateAmount(270); //Rotate to checkpoint 3

    while(Drive.IsRotating()) {
        Blue.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Blue.SetState(false);

    sleep_ms(1000); //Short delay to prevent over/underspin

    Drive.DriveAmount(0.25); //Drive to halfway point between check point 2 and 3

    while(Drive.IsRotating()) {
        Green.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Green.SetState(false);

    sleep_ms(100); //Stop for hundred milisecond

    Drive.DriveAmount(0.23); //Drive to checkpoint 3 from halfway point

    while(Drive.IsRotating()) {
        Green.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Green.SetState(false);

    sleep_ms(3000); //Stop for 3 seconds

    Drive.RotateAmount(-65); //Rotate to checkpoint 4

    while(Drive.IsRotating()) {
        Blue.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Blue.SetState(false);

    sleep_ms(100); //Short delay to prevent over/underspin

    Drive.DriveAmount(0.28); //Drive to halfway point between check point 3 and 4

    while(Drive.IsRotating()) {
        Green.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Green.SetState(false);

    sleep_ms(1000); //Stop for one second

    Drive.DriveAmount(0.28); //Drive to checkpoint 4 from halfway point

    while(Drive.IsRotating()) {
        Green.ToggleEvery(0.5);
        sleep_ms(10);
    }
    Green.SetState(false);

    sleep_ms(3000); //Stop for 3 seconds

    #pragma endregion
    
    //Drive.RotateAmount(-360);

    while(true); //Prevent program from stopping


    
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
