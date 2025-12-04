#include "EncodedMotor.h"
#include "DriveTrain.h" //Include this here instead of .h so no circular dependency issue
#include <algorithm>
#include <cmath>

#pragma region EncodedMotor

PWM::EncodedMotor::EncodedMotor(Drivetrain::MotorInit& motorInit)
: 
MOTOR(motorInit.MotorPWM, motorInit.Pin1, motorInit.Pin2),
MotorEncoder(motorInit.encPin1,motorInit.encPin2)
{
    //Create a timer that will trigger every millisecond regardless.
    add_repeating_timer_us(-1 * (100000 / timerFrequency), HandleMotor_Callback, this, &timer);
}

void PWM::EncodedMotor::RotateCounts(int counts, float speed) {
    this->SetCounts(counts);
    if (IsPIDControlled) {
        int direction = (counts >= 0) ? 1 : -1;
        float outSpeed = std::abs(speed) * direction;
        this->SetSpeed(outSpeed);
        
    } else {
        this->SetSpeed(speed);
        if(counts > 0) {
            this->Forward();
        } else if (counts < 0 ){
            this->Backward();
        }
    }
    
}

/// @brief Will Spin the motor forward at a given speed either as a percetnage of max or as a raw meters / second value.
/// @param speed the rate to spin the motor at, as a range of 0 to 1 If in PID control mode at the given m/s if in PID control
void PWM::EncodedMotor::Forward(float speed) {
    if (IsPIDControlled) {
        this->SetSpeed(std::abs(speed));
    } else {
        PWM::MOTOR::Forward(speed);
    }
}

/// @brief Will Spin the motor backward at a given speed either as a percetnage of max or as a raw meters / second value.
/// @param speed the rate to spin the motor at, as a range of 0 to 1 If in PID control mode at the given m/s if in PID control
void PWM::EncodedMotor::Backward(float speed) {
    if (IsPIDControlled) {
        this->SetSpeed(-1.0 * std::abs(speed));
    } else {
        PWM::MOTOR::Backward(speed);
    }
}

/// @brief Sets the motor driver pins for going forward.
void PWM::EncodedMotor::Forward() {
    
    Pin1.SetState(false);
    Pin2.SetState(true);
}

/// @brief Sets the motor driver pins for going backwards. 
void PWM::EncodedMotor::Backward() {
    
    Pin1.SetState(true);
    Pin2.SetState(false);
}

bool PWM::EncodedMotor::HandleMotor_Callback(struct repeating_timer *t) {
    PWM::EncodedMotor* self = (PWM::EncodedMotor*)t->user_data;
    self->HandleMotor();
    return true;
}

// Save current optimization settings
#pragma GCC push_options
// Force optimization to level 0 (None) for this section
// #pragma GCC optimize ("O0")

void PWM::EncodedMotor::HandleMotor() {
    timerCounts++;
    if ((encoderCounts >= endCounts && countsToRotate > 0)||( encoderCounts <= endCounts && countsToRotate < 0)) {
        Stop();
        SetCounts(0);
    }
    
    if (timerCounts >= pidRate && IsPIDControlled) { //Every pidRate calls of this timer, we should do the PID loop.

        if (this->countsToRotate != 0) {
            int error = this->endCounts - this->encoderCounts;

            float closeRatio = (float)error / STOPPINGCOUNTS;
            closeRatio = std::clamp(closeRatio, -1.0f, 1.0f); 

            float targetVelocity = closeRatio * speedMag;
            float clampedTargetVelocity = std::clamp(std::abs(targetVelocity), MINSPEED, MAXSPEED);
            int direction = (closeRatio >= 0) ? 1 : -1;
            float adjustedTargetVelocity = clampedTargetVelocity * direction;
            this->pidTargetSpeedMax = adjustedTargetVelocity;
        }

        if (std::abs(pidTargetSpeedMax) <= std::abs(pidTargetSpeed)) {
            this->pidTargetSpeed = pidTargetSpeedMax;
        } else if (std::abs(pidTargetSpeed) - 1 <= std::abs(this->AngularVelocity())) {
            this->pidTargetSpeed += (pidTargetSpeedMax >= 0) ? 0.5 * dT: -0.5 * dT;
            this->pidTargetSpeed = std::clamp(pidTargetSpeed, -std::abs(pidTargetSpeedMax), std::abs(pidTargetSpeedMax));
        }
        

        this->timerCounts = 0;

        //Get us the speed error
        float error = pidTargetSpeed - this->AngularVelocity();
        if (prevError == 0 && error == 0 && pidTargetSpeed == 0 ) {
            integralSum = 0;
        }

        //Proprotional term
        float P = Kp * error;

        //Intgeral term
        this->integralSum += error * dT;


        //Anti-Windup system, clamps the integral so it doesn't get stupid high
        float integralMax = maxOutput / (Ki > 0 ? Ki : 1.0f);
        integralSum = std::clamp(integralSum, -integralMax, integralMax);

        float I = Ki * integralSum;

        //Derivtative Term
        float derivative = (error - prevError) / dT;
        float D = Kd * derivative;

        prevError = error;

        //Feedforward
        float F = Kf * pidTargetSpeed;

        float output = P + I + D + F;

        output = std::clamp(output, -maxOutput, maxOutput);

        //Set Direction and Speed

        this->SetDuty(std::abs(output));

        (output >= 0) ? Forward() : Backward(); 

    }
}

//Renable standard optimizations
#pragma GCC pop_options

void PWM::EncodedMotor::SetSpeed(float speed) {
    if (IsPIDControlled) {
        float targetRadS = speed / wheelRadius;

        targetRadS = std::clamp(targetRadS, -MAXSPEED, MAXSPEED);

        this->speedMag = std::abs(targetRadS);
        this->pidTargetSpeedMax = targetRadS;

    } else {
        this->speedMag = speed; this->SetDuty(speed);
    }
    
}

void PWM::EncodedMotor::Stop() {
    Pin1.SetState(false);
    Pin2.SetState(false);
    this->pidTargetSpeedMax = 0;

    prevError = 0.0f; 
    integralSum = 0.0f;

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