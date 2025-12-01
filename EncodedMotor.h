#ifndef ENCODEDMOTOR_H
#define ENCODEDMOTOR_H

#include "PWM.h"
#include "Sensor.h"

//Declare this exists somewhere
namespace Drivetrain {
    struct MotorInit;
}

namespace PWM 
{
    
    class EncodedMotor : public PWM::MOTOR, public Sensor::MotorEncoder {
        public: 
            EncodedMotor(Drivetrain::MotorInit&);

            void RotateCounts(int counts, float speed);

            void RotateCounts(int counts) {this->RotateCounts(counts, this->speedMag);};

            void SetCounts(int counts);

            int GetCounts();

            int GetEndCounts();

            virtual ~EncodedMotor() {};

            void Forward(float speed) override;

            void Backward(float speed) override;

            void SetSpeed(float speed);
            
            void SetSpeedMode(bool mode) {IsPIDControlled = mode;};
            
            virtual void Stop() override;
          
        protected:
            int countsToRotate = 0; //The number of raw counts to rotate
            int endCounts = 0; //The count value to end on
            
            struct repeating_timer timer; //The timer for the PID and count system
            float speedMag = 0; //The magnitude of the wanted current speed. Should NOT have a sign
            
            void Forward();

            void Backward();

            static bool HandleMotor_Callback(struct repeating_timer *t);
            virtual void HandleMotor();

            int timerCounts = 0; //The number of cycles since last PID check

            static constexpr int pidRate = 100; //Number of timer cycles between each PID check

            bool IsPIDControlled = true; //Is this motor in PID speed control mode. Defaults to true.
            float pidTargetSpeed = 0.0f; //The pid target speed, is signed.

            float prevError = 0.0f; //The previous error value for the PID 
            float integralSum = 0.0f; //The integralCounter

            float maxOutput = 1.0f; //The max value of the PidController

            static constexpr int timerFrequency = 10000; //The number of times the timer is called per second. 
            static constexpr float dT = pidRate / (timerFrequency * 1.0f); //The rawTime between each pidCheck
            static constexpr float Kf = 0.045; //Feedforward Constant
            static constexpr float Kp = 0.10; //Proportional Constant 
            static constexpr float Ki = 0.05; //Integral Constant 
            static constexpr float Kd = 0.0; //Derivative Constant

            static constexpr float MAXSPEED = 19.5f; //Max Speed Magnitude in Rad/s 
            static constexpr float MINSPEED = 2.0f; //Min Speed Magnitude in Rad/s for prevent stalls when getting close to stuff

            static constexpr float STOPPINGCOUNTS = 1000.f; //In Encoder Counts
            
    };



}


#endif