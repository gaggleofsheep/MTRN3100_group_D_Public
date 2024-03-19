#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "MovingAverageFilter.hpp"

namespace mtrn3100
{

    class longLidar
    {
    public:
        longLidar(uint8_t address, uint8_t enable) : longLidar(address, enable, 0) {}
        longLidar(uint8_t address, uint8_t enable, float bias) : address{address}, enable_pin{enable}, bias{bias}
        {
            pinMode(enable_pin, OUTPUT);
        }

        void setup()
        {
            digitalWrite(enable_pin, LOW);
            delay(1000);
            digitalWrite(enable_pin, HIGH);
            delay(50);

            sensor.init();
            sensor.setAddress(address);
            Serial.print("Front Sensor I2C address: 0x");
            Serial.println(sensor.readReg(0x212), HEX);
            delay(100);

            sensor.setTimeout(500);
            if (!sensor.init())
            {
                Serial.println("Failed to detect and initialize sensor!");
                while (1)
                    ;
            }

            sensor.startContinuous(50);
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
        VL53L0X sensor;

        bool rangeAvailable()
        {
            return ((sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0);
        }

        uint16_t readRangeNonBlockingMillimeters()
        {
            uint16_t range = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
            sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
            return range;
        }

        MovingAverageFilter<float, 3> filter;

        const uint8_t address;
        const uint8_t enable_pin;
        const float bias;
    };

} // namespace mtrn3100
