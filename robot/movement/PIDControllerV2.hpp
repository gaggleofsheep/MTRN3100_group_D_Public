#pragma once

#include <math.h>
#include "MovingAverageFilter.hpp"

namespace mtrn3100
{

    class PIDController
    {
    public:
        PIDController(float kp = 1, float ki = 1, float kd = 1, float wind_up = 1)
            : kp(kp), ki(ki), kd(kd), wind_up(wind_up) {}

        float compute(float input)
        {
            const uint32_t curr_time = micros();
            const float dt = static_cast<float>(curr_time - prev_time) / 1e6;
            prev_time = curr_time;

            error = setpoint - (input - encoderOffset);

            integral = prev_integral + dt * error;

            // Implement Integral windup protection
            if (integral > wind_up)
            {
                integral = wind_up;
            }
            else if (integral < -wind_up)
            {
                integral = -wind_up;
            }

            derivative = (error - prev_error) / dt;
            signal = kp * error + ki * integral + kd * derivative;



            // Serial.print(" Signal = ");
            // Serial.print(signal);
            // Serial.print("; Integral = ");
            // Serial.print(ki * integral);
            // Serial.print("; derivative = ");
            // Serial.print(kd * derivative);
            // Serial.print("");
            // Serial.print("\n");

            prev_integral = integral;
            prev_error = error;

            return signal;
        }

        void tune(float p, float i, float d)
        {
            kp = p;
            ki = i;
            kd = d;
        }

        // void setDeadband(float limit) { deadband = limit; }

        void zeroEncoderAndSetTarget(float currentEncoderCount, float setpointInput)
        {
            encoderOffset = currentEncoderCount;
            setpoint = setpointInput;
        }

    private:
        float kp;
        float ki;
        float kd;
        // float deadband;
        float wind_up;
        float error;
        float derivative;
        float integral;
        float signal;
        float prev_integral = 0;
        float prev_error = 0;
        float setpoint = 0;
        float encoderOffset = 0;
        uint32_t prev_time = micros();
    };

} // namespace mtrn3100