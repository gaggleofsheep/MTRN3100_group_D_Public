#pragma once

#include <math.h>
#include "MovingAverageFilter.hpp"

namespace mtrn3100
{

    class IMUPIDController
    {
    public:
        IMUPIDController(float kp = 1, float ki = 1, float kd = 1, float wind_up = 1)
            : kp(kp), ki(ki), kd(kd), wind_up(wind_up) {}

        float compute(float input)
        {
            const uint32_t curr_time = micros();
            const float dt = static_cast<float>(curr_time - prev_time) / 1e6;
            prev_time = curr_time;

            error = setpoint - (input - IMUOffset);

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

        void zeroIMUAndSetTarget(float current_yaw, float setpointInput)
        {
            IMUOffset = current_yaw;
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
        float IMUOffset = 0;
        uint32_t prev_time = micros();
    };

} // namespace mtrn3100