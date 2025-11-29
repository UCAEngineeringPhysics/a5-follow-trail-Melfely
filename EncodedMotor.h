#ifndef ENCODEDMOTOR_H
#define ENCODEDMOTOR_H

#include "PWM.h"
#include "Sensor.h"
#include "DriveTrain.h"

namespace PWM 
{
    
    class EncodedMotor : public PWM::MOTOR, public Sensor::MotorEncoder {
        public: 
            EncodedMotor(Drivetrain::MotorInit motorInit);

            void RotateCounts(int counts, float speed);

            void SetCounts(int counts);

            int GetCounts();

            int GetEndCounts();

            virtual ~EncodedMotor() {};

            void Forward();

            void Backward();

            void SetSpeed(float speed){this->speed = speed;};
            
            void SetSpeedMode(bool mode) {IsExactSpeed = mode;};
            
            virtual void Stop() override;
          
        protected:
            int countsToRotate;
            int endCounts;
            struct repeating_timer timer;
            float speed;
            bool IsExactSpeed = true;

            virtual void HandleMotorTimer_Callback(struct repeating_timer *t);


    };



}


#endif