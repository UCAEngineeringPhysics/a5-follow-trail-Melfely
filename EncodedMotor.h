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

        protected:
            int countsToRotate;
            int endCounts;

    };



}


#endif