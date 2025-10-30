#ifndef SENSOR_H
#define SENSOR_H


#include "GPIO.h"
#include "PWM.h"
#include <unordered_map>
#include <optional>

namespace Sensor {

    class Distance {
        public:
            Distance(uint TriggerPin, uint EchoPin);
            float GetDistance();

        protected:
            /// @brief The PWM signal generator to allow the distance sensor to function.
            PWM::PIN TriggerPin;
            /// @brief The Echo or reciving pin to recieve the return signal
            GPIO::PIN EchoPin;

            Distance() = delete;

            void echoHandler(uint32_t events);
            static void echoHandler_callback(uint pin, uint32_t events);

            std::optional<float> distance; //Distance to wall in meters
            uint64_t startTime;

            static std::unordered_map<uint, Distance*> instanceMap; 
            //Thanks AI! The suggestion of using a map was great


    };
    #pragma region MotorEncoder
    class MotorEncoder {
        public:
            MotorEncoder(uint pinA, uint pinB);
            #pragma region Public Methods
            float LinearVelocity() {return wheelLinVelocity;}
            float AngularVelocity(){ return wheelAngVelocity;}
            void ResetEncoderCount() {this->encoderCounts = 0;}

            #pragma endregion
        protected:
            MotorEncoder() = delete;
            #pragma region Constants & Statics
                static constexpr float wheelRadius = 0.025;
                static constexpr float gearRatio = 98.5;
                static constexpr float encoderCPR = 28; //Pulse Counts per revolution
                static constexpr float timerFrequency = 100;

                static std::unordered_map<uint, MotorEncoder*> instanceMap;
            #pragma endregion
            #pragma region Fields
                GPIO::PIN EncodPinA;
                GPIO::PIN EncodPinB;

                bool pinAVal;
                bool pinBVal;

                int encoderCounts;
                int previousCounts;

                /// @brief Angular Velocity of the wheel
                float wheelAngVelocity;
                /// @brief Linear Velocity
                float wheelLinVelocity;

                struct repeating_timer timer;
            #pragma endregion
            #pragma region Protected Methods

            void PinAHandler(uint32_t events);
            void PinBHandler(uint32_t events);

            static void PinAHandler_Callback(uint pin, uint32_t events);
            static void PinBHandler_Callback(uint pin, uint32_t events);

            void MeasureVelocity();

            static bool MeasureVelocity_Callback(__unused struct repeating_timer *t);

            #pragma endregion
            

    };
    #pragma endregion
}


#endif