#include "robot.hpp"

// Drive the robot forwrd n cells.
void moveForward(int cell_count)
{
    //  Set position to zero
    l_encoder.position = 0;
    r_encoder.position = 0;

    // Set the target position.
    float l_target = cell_count * radians_per_cell;
    float r_target = cell_count * radians_per_cell;

    // Zero encoders and set target
    l_forward_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    r_forward_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);
    side_lidar_pid.zeroEncoderAndSetTarget(0, 0);
    l_front_lidar_pid.zeroEncoderAndSetTarget(0, 0);
    r_front_lidar_pid.zeroEncoderAndSetTarget(0, 0);
    imu.read();
    float yaw = imu.getYaw();
    float imu_setpoint = 0;
    imu_forward_pid.zeroIMUAndSetTarget(yaw, imu_setpoint);

    lidar_driving = false; // Set lidar driving to false for each new movement
    // Drive the motors until target is reached
    while (driveMotors(0) == 0) {}; // type = 0 for straight

    // Stop motors
    l_motor.setPWM(0);
    r_motor.setPWM(0);

    delay(500);
    changeCoordinates();
}

// Turn the robot left by the radians of rotation of the wheel
void turnLeft(int cell_count)
{
    //  Set position to zero
    l_encoder.position = 0;
    r_encoder.position = 0;

    // Set the target position.
    float l_target = l_encoder.position - cell_count * radians_per_90turn;
    float r_target = r_encoder.position + cell_count * radians_per_90turn;

    // Zero encoders and set target
    l_turn_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    r_turn_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);
    imu.read();
    float yaw = imu.getYaw();
    float imu_setpoint = 90;
    imu_forward_pid.zeroIMUAndSetTarget(yaw, imu_setpoint);

    // Drive the motors until target is reached
    while (driveMotors(1) == 0) {}; // type = 1 for right turn

    // Centre robot if there is a wall in front
    if (frontSensor.value() < 200) 
    {
        moveForward(1); 
    }

    // Stop motors
    l_motor.setPWM(0);
    r_motor.setPWM(0);

    delay(500);
    changeHeadingLeft();
}

// Turn the robot right by the radians of rotation of the wheel
void turnRight(int cell_count)
{
    //  Set position to zero
    l_encoder.position = 0;
    r_encoder.position = 0;

    // Set the target position.
    float l_target = l_encoder.position + cell_count * radians_per_90turn;
    float r_target = r_encoder.position - cell_count * radians_per_90turn;

    // Zero encoders and set target
    l_turn_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    r_turn_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);
    imu.read();
    float yaw = imu.getYaw();
    float imu_setpoint = yaw - 90;
    imu_forward_pid.zeroIMUAndSetTarget(yaw, imu_setpoint);

    // Drive the motors until target is reached
    while (driveMotors(1) == 0) {};     // type = 1 for turns

    // Centre robot if there is a wall in front using front LiDar
    if (frontSensor.value() < 200) 
    {
        moveForward(1); 
    }

    // Stop motors
    l_motor.setPWM(0);
    r_motor.setPWM(0);

    delay(500);
    changeHeadingRight();
}


float driveMotors(int type)

{
    float side_lidar_error = computeSideLidarError();
    float front_lidar_error = computeFrontLidarError();
    float target_heading = 0;
    float imu_error = computeIMUError(target_heading);

    if (abs(front_lidar_error) > 0) { lidar_driving = true; } // Switch to front lidar driving

    // Compute drive signals
    if (type == 0) // Move Forward
    {
        if (lidar_driving) // Front LiDar control to cell center
        {
            l_front_lidar_signal = l_front_lidar_pid.compute(front_lidar_error);
            r_front_lidar_signal = r_front_lidar_pid.compute(front_lidar_error);
            imu_signal = imu_forward_pid.compute(imu_error);
            if (leftSensor.value() < 60 || rightSensor.value() < 60) 
            { 
              imu_signal = 0.5 * imu_signal; 
            }

            l_drive_signal = l_front_lidar_signal - imu_signal + side_lidar_signal;
            r_drive_signal = r_front_lidar_signal + imu_signal - side_lidar_signal;
            if (l_drive_signal > CONTROL_SPEED) { l_drive_signal = CONTROL_SPEED; }
            if (r_drive_signal > CONTROL_SPEED) { r_drive_signal = CONTROL_SPEED; }
        }
        else // Normal drive to cell center
        {
            l_encoder_signal = l_forward_pid.compute(l_encoder.position);
            r_encoder_signal = r_forward_pid.compute(r_encoder.position);
            side_lidar_signal = side_lidar_pid.compute(side_lidar_error);
            imu_signal = imu_forward_pid.compute(imu_error);
            if (leftSensor.value() < 60 || rightSensor.value() < 60) 
            { 
              imu_signal = 0; 
            }

            l_drive_signal = l_encoder_signal - imu_signal + side_lidar_signal;
            r_drive_signal = r_encoder_signal + imu_signal - side_lidar_signal;
        }
    }
    else if (type == 1) // Turn left or right
    {
        l_encoder_signal = l_turn_pid.compute(l_encoder.position);
        r_encoder_signal = r_turn_pid.compute(r_encoder.position);

        l_drive_signal = l_encoder_signal;
        r_drive_signal = r_encoder_signal;
    } 

    CALIBRATE();                 // Uncomment for calibration
    return 0;                    // Prints sensors and stops motors

    // Set PWM
    l_motor.setPWM(l_drive_signal);
    r_motor.setPWM(r_drive_signal);

    // Update moving average filter
    l_move_filter.sample(l_drive_signal);
    r_move_filter.sample(r_drive_signal);

    // Target reached when change in encoder position is less than 0.2
    if (abs(l_move_filter.difference()) < 0.5 && 
        abs(r_move_filter.difference()) < 0.5 && 
        abs(l_drive_signal) != MAX_SPEED &&
        abs(l_drive_signal) != CONTROL_SPEED) //
    {
        l_move_filter.clear();
        r_move_filter.clear();
        return 1; // Target reached
    }
    else
    {
        return 0; // Target not yet reached
    }
}



float computeSideLidarError()
{
    // Get LiDar values
    leftSensor.sample();
    rightSensor.sample();
    float left_val = leftSensor.value();
    float right_val = rightSensor.value();

    float side_lidar_error = 0;
    int sensor_to_wall = 71;
    int no_wall = 110;
    if (left_val < no_wall) // Left wall
    {
        side_lidar_error = left_val - sensor_to_wall;
    }
    else if (right_val < no_wall) // Right wall
    {
        side_lidar_error = -(right_val - sensor_to_wall);
    }
    return side_lidar_error;
}

float computeFrontLidarError()
{
    // Get LiDar values
    float front_val = 0;
    while (front_val == 0)
    {
        frontSensor.sample();
        front_val = frontSensor.value();
    }

    float front_lidar_error = 0;
    int sensor_to_wall = 54;   // Robot centered in cell
    int wall_front = 200;
    if (front_val < wall_front) // Wall one cell in front
    {
        front_lidar_error = -(front_val - sensor_to_wall);
    }
    return front_lidar_error;
}

float computeIMUError(float target_heading)
{
    imu.read();
    float yaw = imu.getYaw();
    float imu_error = yaw;
    return imu_error;
}


void followMotionPlan(String motionPlan)
{
    String motion;
    if (isDigit(motionPlan.charAt(0)))
    {
        row = motionPlan.charAt(0) - '0';
        col = motionPlan.charAt(1) - '0';
        head = motionPlan.charAt(2);
        motion = motionPlan.substring(3);
    }
    else
    {
        motion = motionPlan.substring(0);
    }

    for (int i = 0; i < motion.length(); i++)
    {
        char m = motion[i];
        switch (m)
        {
        case 'F':
            moveForward(1);
            break;
        case 'L':
            turnLeft(1);
            break;
        case 'R':
            turnRight(1);
            break;
        }
    }
}

void debuggingPrints(float imu_error)
{
    //    frontSensor.sample();
    //    Serial.print("    Front Sensor val: ");
    //    Serial.print(frontSensor.value());
    // Serial.print("Left Sensor val:  ");
    // Serial.print(leftSensor.value());
    // Serial.print(";   Right Sensor val:  ");
    // Serial.print(rightSensor.value());
    // Serial.print("   side_lidar_signal:  ");
    // Serial.print(side_lidar_signal);
    // Serial.print("\n");

    Serial.print(" l_drive_signal ");
    Serial.print(l_drive_signal);
    Serial.print("               r_drive_signal ");
    Serial.print(r_drive_signal);
    Serial.print("\n");

    Serial.print(" Imu_error ");
    Serial.print(imu_error);

    // Serial.print(" l_encoder.position ");
    // Serial.print(l_encoder.position);
    // Serial.print("           r_encoder.position ");
    // Serial.print(r_encoder.position);
    // Serial.print("\n");
}

void CALIBRATE()
{
    frontSensor.sample();
    Serial.print("    Front Sensor val: ");
    Serial.print(frontSensor.value());

    leftSensor.sample();
    Serial.print("    Left Sensor val: ");
    Serial.print(leftSensor.value());
    Serial.print("\n");

    rightSensor.sample();
    Serial.print("    Right Sensor val: ");
    Serial.print(rightSensor.value());
}
