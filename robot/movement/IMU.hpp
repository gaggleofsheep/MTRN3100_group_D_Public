#pragma once

#include <ICM_20948.h>
#include <Wire.h>

#include "utilities.hpp"

namespace mtrn3100
{

    class IMU
    {
    public:
        IMU(bool adrValue = 0) : adrValue(adrValue) {}

        void begin()
        {
            bool initialised = false;
            while (!initialised)
            {
                icm.begin(Wire, adrValue);
                if (icm.status != ICM_20948_Stat_Ok)
                {
                    delay(500);
                    prevTime = micros();
                }
                else
                {
                    initialised = true;
                }
            }

            Serial.println(F("Device connected!"));
            bool success = true;

            // Enable sensors
            success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);
            success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
            success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);

            success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
            success &= (icm.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);

            // Enable and clear buffers
            success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
            success &= (icm.enableDMP() == ICM_20948_Stat_Ok);
            success &= (icm.resetDMP() == ICM_20948_Stat_Ok);
            success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);

            // Check success
            if (success)
            {
                Serial.println(F("DMP enabled!"));
            }
            else
            {
                Serial.println(F("Enable DMP failed!"));
                Serial.println(
                    F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
                while (1)
                    ;
            }
        }

        void calibrateAcceleration()
        {
            int samples = 20;
            float caliAx[samples];
            float caliAy[samples];
            float caliAz[samples];

            for (int i = 0; i < samples; i++)
            {
                icm.getAGMT();
                caliAx[i] = icm.accX();
                caliAy[i] = icm.accY();
                caliAz[i] = icm.accZ();
                delay(20);
            }

            offsetX = util::average(caliAx, samples);
            offsetY = util::average(caliAy, samples);
            offsetZ = util::average(caliAz, samples);
        }

        bool dataReady() { return icm.dataReady(); }

        void reset()
        {
            vx = 0, vy = 0, vz = 0;
            r = 0, p = 0, y = 0;
            px = 0, py = 0, pz = 0;
        }

        void read()
        {
            // Compute change in time since last measurement.
            const uint32_t currTime = micros();
            const float dt = static_cast<float>(currTime - prevTime) / 1e6;
            prevTime = currTime;

            // Get latest IMU readings.
            icm.getAGMT();

            // Convert acceleration from mg (milli-gravity) to mm/s^2.
            auto mg2mps2 = [](float const val)
            { return val / 9.81; };
            ax = mg2mps2(icm.accX() - offsetX);
            ay = mg2mps2(icm.accY() - offsetY);
            az = mg2mps2(icm.accZ() - offsetZ);

            icm.readDMPdataFromFIFO(&data);

            // Compute roll-pitch-yaw.
            if ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail))
            {
                if ((data.header & DMP_header_bitmap_Quat6) > 0)
                {
                    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                    double q2sqr = q2 * q2;

                    // Roll (x-axis rotation).
                    double t0 = +2.0 * (q0 * q1 + q2 * q3);
                    double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
                    r = atan2(t0, t1) * 180.0 / PI;

                    // Pitch (y-axis rotation).
                    double t2 = +2.0 * (q0 * q2 - q3 * q1);
                    t2 = t2 > 1.0 ? 1.0 : t2;
                    t2 = t2 < -1.0 ? -1.0 : t2;
                    p = asin(t2) * 180.0 / PI;

                    // Yaw (z-axis rotation).
                    double t3 = +2.0 * (q0 * q3 + q1 * q2);
                    double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
                    y = atan2(t3, t4) * 180.0 / PI;
                }
            }

            // Deadband acceleration values.
            const float radThreshold = 2;
            ax = util::deadband(ax, radThreshold);
            ay = util::deadband(ay, radThreshold);
            az = util::deadband(az, radThreshold);

            // COMPLETE THIS BLOCK. - DONE
            // Compute linear velocities in m/s.
            vx = vx + ax * dt;
            vy = vy + ay * dt;
            vz = vz + az * dt;

            // Compute global pose in m.
            if (!isnan(y))
            {
                // COMPLETE THIS BLOCK. - DONE
                px = px + vx * cos(y);
                py = py + vx * sin(y);
            }
        }

        // Linear accelerations in local frame.
        float accX() { return ax; } // Forward.
        float accY() { return ay; } // Sideways.
        float accZ() { return az; } // Up-down.

        // Linear velocities in local frame.
        float velX() { return vx; }
        float velY() { return vy; }
        float velZ() { return vz; }

        // Angles
        float roll() { return r; }
        float pitch() { return p; }
        float yaw() { return y; }

        // Pose in global frame.
        float posX() { return px; }
        float posY() { return py; }
        float posH() { return pz; }

    private:
        ICM_20948_I2C icm; // Instantiate IMU object from Sparkfun.
        icm_20948_DMP_data_t data;

        const bool adrValue;

        uint32_t prevTime = 0;

        // Linear.
        float ax, ay, az;
        float vx, vy, vz;

        // Calibration.
        float offsetX, offsetY, offsetZ, offsetYaw;

        // Angular.
        float r = 0, p = 0, y = 0;

        // Global Pose.
        float px = 0, py = 0, pz = 0;
    };

} // namespace mtrn3100
