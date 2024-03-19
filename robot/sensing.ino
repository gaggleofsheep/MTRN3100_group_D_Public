#include "robot.hpp"

void setUpSensors()
{

    // Setup IMU
    Wire.begin();
    delay(100);
    while (imu.wakeup() == false)
    {
        Serial.print(millis());
        Serial.println("\tCould not connect to GY521");
        delay(1000);
    }

    imu.setAccelSensitivity(2); // 8g
    imu.setGyroSensitivity(1);  // 500 degrees/s

    imu.setThrottle();
    Serial.println("start...");
    imu.setNormalize(0);

    // set calibration values from calibration sketch.
    imu.axe = 0;
    imu.aye = 0;
    imu.aze = 0;
    imu.gxe = 0;
    imu.gye = 0;
    imu.gze = -1.642;

    // Give IMU some time to initialise
    delay(500);

    // Setup LiDars
    leftSensor.setup();
    rightSensor.setup();
    frontSensor.setup();
}

// Function get reads sensor values and updates wall positions
// calls printWall function
void detectWalls()
{
    frontSensor.sample();
    leftSensor.sample();
    rightSensor.sample();
    float front_dist = frontSensor.value();
    float left_dist = leftSensor.value();
    float right_dist = rightSensor.value();

    currentWallPosition[0] = (front_dist  < 150) ? 1: 0;
    currentWallPosition[1] = (left_dist < 150) ? 1: 0;
    currentWallPosition[2] = (right_dist < 150) ? 1: 0;
}

// Prints Walls, [F: 0, L: 0, R: 0], 0 = no wall, 1 = wall
void printWalls()
{
    Serial.println("Current Wall Positions: ");
    Serial.print("F: ");
    Serial.print(currentWallPosition[0]);
    Serial.print(", ");
    Serial.print("L: ");
    Serial.print(currentWallPosition[1]);
    Serial.print(", ");
    Serial.print("R: ");
    Serial.println(currentWallPosition[2]);
}


