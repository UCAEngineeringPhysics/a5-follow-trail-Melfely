#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "PWM.h"
#include "GPIO.h"
#include "Sensor.h"
#include "EncodedMotor.h"

namespace Drivetrain
{
    
    struct MotorInit {
        public: 
            MotorInit(uint MotorPWM, uint Pin1, uint Pin2, uint encPin1, uint encPin2);
            MotorInit(uint MotorPWM, uint Pin1, uint Pin2);

            uint MotorPWM;
            uint Pin1;
            uint Pin2;
            uint encPin1;
            uint encPin2;
    };

    class DualMotor {
        public:
            DualMotor(uint STBYPin, MotorInit LeftMotorInit, MotorInit RightMotorInit);

            virtual ~DualMotor();

            virtual void Forward(float speed);
            virtual void Backward(float speed);

            virtual void SpinLeft(float speed);
            virtual void SpinRight(float speed);

            virtual void SetState(bool state);

            virtual void Stop();

            virtual float GetLeftDuty();
            virtual float GetRightDuty();

        protected:
            PWM::MOTOR* LeftMotor;
            PWM::MOTOR* RightMotor;
            GPIO::PIN* StandbyPin;

            DualMotor(uint STBYPin);

        private:
             DualMotor() = delete;

    };

    #pragma region EncodedDualMotor
    class EncodedDualMotor : DualMotor {
        public:
            EncodedDualMotor(uint STBYPin, MotorInit LeftMotorInit, MotorInit RightMotorInit);

            /// @brief Sets the drivetrain speed to this value for all movement.
            /// @param speed the speed from 0 to 1 that the drive motors will spin at.
            void SetSpeed(float speed) {this->speed = speed;}

            void DriveAmount(float meters, float speed);
            void DriveAmount(float meters);

            void RotateAmount(float radians, float speed);
            void RotateAmount(float radians);

            using Drivetrain::DualMotor::Forward;
            using Drivetrain::DualMotor::Backward;
            using Drivetrain::DualMotor::SpinLeft;
            using Drivetrain::DualMotor::SpinRight;

            using Drivetrain::DualMotor::SetState;
            using Drivetrain::DualMotor::Stop;

            using Drivetrain::DualMotor::GetLeftDuty;
            using Drivetrain::DualMotor::GetRightDuty;


        protected:

            #pragma region Variables

            float speed = 0.5;

            float radians;
            float meters;

            #pragma endregion
            #pragma region Statics and Constants

            static constexpr float WHEELLENGTH = 1;
            static constexpr float wheelRadius = 0.025;
            static constexpr float gearRatio = 98.5;
            static constexpr float encoderCPR = 28; //Pulse Counts per revolution

            #pragma endregion

        private:
            EncodedDualMotor() = delete;
    };
    #pragma endregion
} // namespace DualMotor

#endif