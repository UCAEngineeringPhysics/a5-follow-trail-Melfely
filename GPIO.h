//Helper methods to simplfy creating of many projects
#ifndef GPIO_H
#define GPIO_H


#include <stdio.h>
#include "pico/stdlib.h" 
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <cassert>
#include <atomic>
namespace GPIO
{
    class PIN {
        public:
            PIN(uint pin, bool output);

            virtual void Toggle();

            virtual void SetState(bool state);

            virtual bool GetState();

            virtual uint GetPin();

            virtual void ToggleEvery(float seconds);

            virtual void SetPulls(bool PullUp, bool PullDown);

            virtual void SetIRQ(uint32_t eventMask, gpio_irq_callback_t callback);

        protected:
            PIN(uint pin);
            PIN() = delete; //Remove default constructor
            const uint pinID;
            uint64_t timeAtLastCall_us= 0;
            float timePassed = 0;

    };

    class LED : PIN {

        public:
            LED(uint pin);

            using PIN::Toggle;
            using PIN::SetState;
            using PIN::GetState;
            using PIN::GetPin;
            using PIN::ToggleEvery;
            

        private:
            

    };

    class BUTTON : PIN {
        public:
            BUTTON(uint pin, bool IsPullUp);

            bool IsPullUp();

            bool IsPressed();

            using PIN::GetState;
            using PIN::GetPin;
        private:

    };
} //Namespace GPIO
#endif