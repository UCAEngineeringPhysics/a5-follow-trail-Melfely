#include "DriveTrain.h"
#include <cmath>

#pragma region DualMotor

/// @brief Inits a two motor drive, using two PWM::MOTORS, assumes a motor drive that utilizes a STBYpin
/// @param STBYPin The Pin the STBY system will run on, this allows this class to enable and disable motor output
/// @param LeftMotorInit the init struct that will store all the needed values for init the left motor. This does NOT use encoders
/// @param RightMotorInit the init struct that will store all the needed values for init on the right motor. This does NOT use encoders
Drivetrain::DualMotor::DualMotor(uint STBYPin, MotorInit LeftMotorInit, MotorInit RightMotorInit) 
:
StandbyPin(new GPIO::PIN(STBYPin, true)),
LeftMotor( new PWM::MOTOR(LeftMotorInit.MotorPWM, LeftMotorInit.Pin1, LeftMotorInit.Pin2)),
RightMotor(new PWM::MOTOR(RightMotorInit.MotorPWM, RightMotorInit.Pin1, RightMotorInit.Pin2))
{

}

Drivetrain::DualMotor::DualMotor(uint STBYPin) 
:
StandbyPin(new GPIO::PIN(STBYPin, true)),
LeftMotor(nullptr),
RightMotor(nullptr)
{

}

/// @brief Classic ol deconstructor. Cleans up any pointers;
Drivetrain::DualMotor::~DualMotor() {
    delete StandbyPin;
    delete LeftMotor;
    delete RightMotor;
}



/// @brief Stops the motors
void Drivetrain::DualMotor::Stop() {
    LeftMotor->Stop();
    RightMotor->Stop();
}

/// @brief sets both motors to the same forward duty speed
/// @param speed a number between 0 and 1 that is the wanted speed
void Drivetrain::DualMotor::Forward(float speed) {
    LeftMotor->Forward(speed);
    RightMotor->Forward(speed);
}

/// @brief sets both motors to the same forward duty speed
/// @param speed a number between 0 and 1 that is the wanted speed
void Drivetrain::DualMotor::Backward(float speed) {
    LeftMotor->Backward(speed);
    RightMotor->Backward(speed);
}

/// @brief Sets the right motor to spin forward at given speed, and left motor to spin backwards at given speed, will be a left spin
/// @param speed a number between 0 and 1 that is the wanted speed
void Drivetrain::DualMotor::SpinLeft(float speed) {
    LeftMotor->Backward(speed);
    RightMotor->Forward(speed);
}

/// @brief Sets the right motor to spin backward at given speed, and left motor to spin forward at given speed, will be a right spin
/// @param speed a number between 0 and 1 that is the wanted speed
void Drivetrain::DualMotor::SpinRight(float speed){ 
    LeftMotor->Forward(speed);
    RightMotor->Backward(speed);
}

/// @brief Sets the STBYpin handled by the drivetrain to given state
/// @param state state to STBYpin to
void Drivetrain::DualMotor::SetState(bool state) {
    StandbyPin->SetState(state);
}

/// @brief Will return the duty or speed of the left motor
/// @return returns a float from 0 to 1
float Drivetrain::DualMotor::GetLeftDuty() {
    return LeftMotor->GetDuty();
}

/// @brief Will return the duty or speed of the left motor
/// @return returns a float from 0 to 1
float Drivetrain::DualMotor::GetRightDuty() {
    return RightMotor->GetDuty();
}

#pragma endregion

#pragma region EncodedDualMotor

Drivetrain::EncodedDualMotor::EncodedDualMotor(uint STBYPin, MotorInit LeftMotorInit, MotorInit RightMotorInit)
:
DualMotor(STBYPin)
{
    LeftMotor = new PWM::EncodedMotor(LeftMotorInit);
    RightMotor = new PWM::EncodedMotor(RightMotorInit);

    _LeftMotor()->SetSpeedMode(true); //Set the motors to PID Controlled
    _RightMotor()->SetSpeedMode(true); //Set the motors to PID Controlled
}

void Drivetrain::EncodedDualMotor::RotateAmount(float degrees, float speed) {
    float targetRadians = degrees * (M_PI / 180.0f);
    this->radians = targetRadians;

    float leftCounts = (gearRatio * encoderCPR * targetRadians * WHEELBASE) / (2.0f * M_PI * wheelRadius);
    float rightCounts = -1 * leftCounts;

    _LeftMotor()->RotateCounts(leftCounts, speed);
    _RightMotor()->RotateCounts(rightCounts, speed);
 
}

void Drivetrain::EncodedDualMotor::RotateAmount(float degrees) {
    this->RotateAmount(degrees, this->speed);
}

void Drivetrain::EncodedDualMotor::DriveAmount(float meters, float speed) {
    this->meters = meters; 

    
    int counts = (gearRatio * encoderCPR * meters) / (2 * M_PI * wheelRadius);
    
    _LeftMotor()->RotateCounts(counts, speed);
   _RightMotor()->RotateCounts(counts, speed);
}

void Drivetrain::EncodedDualMotor::DriveAmount(float meters) {
    this->DriveAmount(meters, this->speed);
}

#pragma endregion


#pragma region MotorInit

Drivetrain::MotorInit::MotorInit(uint MotorPWM, uint Pin1, uint Pin2) {
    this->MotorPWM = MotorPWM;
    this->Pin1 = Pin1;
    this->Pin2 = Pin2;
}

Drivetrain::MotorInit::MotorInit(uint MotorPWM, uint Pin1, uint Pin2, uint encPin1, uint encPin2) {
    this->MotorPWM = MotorPWM;
    this->Pin1 = Pin1;
    this->Pin2 = Pin2;
    this->encPin1 = encPin1;
    this->encPin2 = encPin2;
}


#pragma endregion

