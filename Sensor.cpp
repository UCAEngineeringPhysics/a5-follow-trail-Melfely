#include "Sensor.h"
#include <cmath>

#pragma region Distance

/// @brief A map that will contains all pins that are instanced for the distance sensor. This way we can use a callback to call the correct instance easily.
std::unordered_map<uint, Sensor::Distance*> Sensor::Distance::instanceMap;

/// @brief Constructor for a distance Sensor
/// @param TriggerPin This is the pin that will be used for starting the cycle. Is PWM
/// @param EchoPin This is the pin that will be used to return the time it took. Is GPIO
Sensor::Distance::Distance(uint TriggerPin, uint EchoPin)
:
TriggerPin(TriggerPin, 12, 49999), EchoPin(EchoPin, false)
{
    this->TriggerPin.SetDuty((uint)(6));
    this->EchoPin.SetPulls(false, true);
    this->instanceMap[this->EchoPin.GetPin()] = this;
    this->EchoPin.SetIRQ( GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, &echoHandler_callback);

    this->distance = std::nullopt;
    this->startTime = 0;
}

/// @brief Method used by the echoHandler_Callback method to handle the measuring calculating the distance
/// @param events Internal Stuff
void Sensor::Distance::echoHandler(uint32_t events) {
    if (this->EchoPin.GetState() ) {
        this->startTime = time_us_64();
        //printf("StartTime: %llu -> ", startTime); //Debug Print
    } else {
            uint64_t dT = time_us_64() - this->startTime;
            //printf(" dT: %llu \n", dT); //Debug Print
            if (dT < 100) {
                this->distance = 0;
            } else if (dT > 100 && dT < 38000) {
                this->distance = dT / 58.0f / 100.0f;
            } else {
                this->distance = std::nullopt;
            }
            
    }
}

/// @brief A callback method that will be assigned to the IRQ for the Distance Sensor, this will the read what pin calls it, check the instanceMap to figure out which instance uses that pin, then call the actual handler
/// @param pin the pin that is calling the callback 
/// @param events Internal Stuff
void Sensor::Distance::echoHandler_callback(uint pin, uint32_t events) {
    if (instanceMap.contains(pin)) {
        instanceMap[pin]->echoHandler(events);
    }
}

/// @brief Returns the non-thread Safe Distance read by the sensor
/// @return this will return the distance in meters as a float, if a -1.0 then that is out of range
float Sensor::Distance::GetDistance() {
    if (this->distance != std::nullopt) {
            return *this->distance; 
    } else {
            return -1.0f;
    }

}

#pragma endregion
#pragma region MotorEncoder

std::unordered_map<uint, Sensor::MotorEncoder*> Sensor::MotorEncoder::instanceMap;

Sensor::MotorEncoder::MotorEncoder(uint pinA, uint pinB)
: EncodPinA(pinA, false), EncodPinB(pinB, false)
{
    //Assign Either pin to to be capable of finding this instance
    this->instanceMap[pinA] = this;
    this->instanceMap[pinB] = this;

    this->pinAVal = EncodPinA.GetState();
    this->pinBVal = EncodPinB.GetState();

    this->encoderCounts = 0;
    this->previousCounts = 0;

    this->wheelAngVelocity = 0;
    this->wheelLinVelocity = 0;

    this->EncodPinA.SetIRQ(GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, *PinAHandler_Callback);
    this->EncodPinB.SetIRQ(GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, *PinBHandler_Callback);
    
    this->timer.user_data = &pinA;

    //Make the timer negative, so the time is ALWAYS accurate regardless of callback execution time
    add_repeating_timer_ms(-1 * (1000 / timerFrequency), MeasureVelocity_Callback, NULL, &timer);

}

void Sensor::MotorEncoder::PinAHandler(uint32_t events) {
    this->pinAVal = EncodPinA.GetState();
    //This will subtract when pinA and pinB are equal, otherwise will add
    /*
    a = 0, b = 0 subtract
    a = 1, b = 0 add
    a = 0, b = 1 add
    a = 1, b = 1 subtract    
    */
    encoderCounts += 1 * (pinAVal != pinBVal) - 1 * (pinAVal == pinBVal);
}
void Sensor::MotorEncoder::PinBHandler(uint32_t events){
    this->pinBVal = EncodPinB.GetState();
    //This will add when pinA and pinB are equal, otherwise will subtract
    /*
    a = 0, b = 0 add
    a = 1, b = 0 subtract
    a = 0, b = 1 subtract
    a = 1, b = 1 add
    */
    encoderCounts -= 1 * (pinAVal != pinBVal) - 1 * (pinAVal == pinBVal);
}

void Sensor::MotorEncoder::PinAHandler_Callback(uint pin, uint32_t events){
    instanceMap[pin]->PinAHandler(events);
}
void Sensor::MotorEncoder::PinBHandler_Callback(uint pin, uint32_t events){
    instanceMap[pin]->PinBHandler(events);
}

void Sensor::MotorEncoder::MeasureVelocity(){
    int deltaCounts = this->encoderCounts - this->previousCounts;
    this->previousCounts = this->encoderCounts; //Update previous counts

    float countsPerSecond = deltaCounts * timerFrequency;
    float motorRPS = countsPerSecond / encoderCPR;
    float motorAngVelocity = motorRPS * 2 * M_PI;

    this->wheelAngVelocity = motorAngVelocity * gearRatio;
    this->wheelLinVelocity = this->wheelAngVelocity * wheelRadius;

}

bool Sensor::MotorEncoder::MeasureVelocity_Callback(__unused struct repeating_timer *t){
    //Take the void pointer, cast it to an uint pointer, then dereference it, getting us a uint number
    instanceMap[*(uint*)t->user_data]->MeasureVelocity();
    return true;

}

#pragma endregion