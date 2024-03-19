#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>
#include "MovingAverageFilter.hpp"

namespace mtrn3100
{

    class shortLidar
    {
    public:
        shortLidar(uint8_t address, uint8_t enable) : shortLidar(address, enable, 0) {}
        shortLidar(uint8_t address, uint8_t enable, float bias) : address{address}, enable_pin{enable}, bias{bias}
        {
            pinMode(enable_pin, OUTPUT);
        }

        void setup()
        {
            Serial.println("Start Short Range LiDar");
            digitalWrite(enable_pin, LOW);
            delay(100);
            digitalWrite(enable_pin, HIGH);
            delay(100);

            sensor.init();
            sensor.configureDefault();
            sensor.setAddress(address);
            Serial.print("Sensor I2C address: 0x");
            Serial.println(sensor.readReg(0x212), HEX);

            sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
            sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
            sensor.setTimeout(500);
            sensor.stopContinuous();
            delay(100);

            // enable interrupt output on GPIO1
            sensor.writeReg(VL6180X::SYSTEM__MODE_GPIO1, 0x10);
            // clear any existing interrupts
            sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x03);

            sensor.startRangeContinuous(50);
            delay(100);
        }

        void sample()
        {
            if (rangeAvailable())
            {
                float range = static_cast<float>(readRangeNonBlockingMillimeters() - bias);
                range = max(range, 0);
                filter.sample(range);
            }
        }

        float value()
        {
            return filter.average();
        }

    private:
        VL6180X sensor;

        bool rangeAvailable()
        {
            return ((sensor.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO) & 0x04) != 0);
        }

        uint8_t readRangeNonBlockingMillimeters()
        {
            uint8_t range = sensor.readReg(VL6180X::RESULT__RANGE_VAL);
            sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);

            return range;
        }

        MovingAverageFilter<float, 3> filter;

        const uint8_t address;
        const uint8_t enable_pin;
        const float bias;
    };

} // namespace mtrn3100
