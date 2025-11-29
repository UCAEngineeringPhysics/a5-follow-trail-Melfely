#include "EncodedMotor.h"

#pragma region EncodedMotor

PWM::EncodedMotor::EncodedMotor(Drivetrain::MotorInit motorInit)
: 
MOTOR(motorInit.MotorPWM, motorInit.Pin1, motorInit.Pin2),
MotorEncoder(motorInit.encPin1,motorInit.encPin2)
{

}

void PWM::EncodedMotor::RotateCounts(int counts, float speed) {
    this->SetCounts(counts);
    this->SetSpeed(speed);
}

void PWM::EncodedMotor::Forward() {
    Pin1.SetState(false);
    Pin2.SetState(true);
}

void PWM::EncodedMotor::Backward() {
    Pin1.SetState(true);
    Pin2.SetState(false);
}

void PWM::EncodedMotor::HandleMotorTimer_Callback(struct repeating_timer *t) {
    if ((endCounts >= encoderCounts && countsToRotate > 0)||( endCounts <= encoderCounts && countsToRotate < 0)) {
        Stop();
        SetCounts(0);
    }
}

void PWM::EncodedMotor::Stop() {
    Pin1.SetState(false);
    Pin2.SetState(false);
}

/// @brief Sets the number of counts you want to rotate. Use a negative for reverse
/// @param counts The number of counts to rotate
void PWM::EncodedMotor::SetCounts(int counts){
    countsToRotate = counts;
    endCounts = encoderCounts + countsToRotate;
}

/// @brief Gets the number of counts rotated from the encoder
/// @return the number of counts rotated in total
int PWM::EncodedMotor::GetCounts() {
    return encoderCounts;
}

/// @brief The count value that would you are attempting to rotate to
/// @return the final count. Note that if you called "resetCounts" after setting the value, your values might not line up.
int PWM::EncodedMotor::GetEndCounts() {
    return endCounts;
}


#pragma endregion