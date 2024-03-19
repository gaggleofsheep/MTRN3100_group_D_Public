// #pragma once

#ifndef ROBOT_H
#define ROBOT_H

// Arduino Setup
#include <arduino.h>
#include <math.h>

// Movement files
#include "movement/Encoder.hpp"
#include "movement/Motor.hpp"
#include "movement/PIDControllerV2.hpp"
#include "movement/IMUPIDControllerV2.hpp"
#include "movement/shortLidar.hpp"
#include "movement/longLidar.hpp"
#include "movement/GY521.hpp"
#include "movement/GY521_registers.hpp"
#include "movement/GY521.cpp"

// Mapping files
#include "mapping/Graph.hpp"
#include "mapping/graph2ascii.hpp"
#include "mapping/mazeSolving.hpp"
#include "mapping/ascii2graph.hpp"

// Sensor files
#include <VL53L0X.h> // download this library in the Arduino manager
#include <Wire.h>
#include <VL6180X.h>

// -------------- Sensor setup variables and declarations --------------

// Address definitions
#define NUM_SENSORS 3
#define LEFT_ADDRESS 0x20
#define RIGHT_ADDRESS 0x22
#define FRONT_ADDRESS 0x24
#define IMU_ADDRESS 0x68

// Initialise LiDar sensors
float frontDistance;
float leftDistance;
float rightDistance;
float prevFrontDistance;
float prevLeftDistance;
float prevRightDistance;
float headingError = 0;

// Set pin variables
int enablePin0 = 11;
int enablePin1 = 10;
int XSHUT = 12;

// Initialise LiDar sensors
mtrn3100::shortLidar leftSensor(LEFT_ADDRESS, enablePin0);
mtrn3100::shortLidar rightSensor(RIGHT_ADDRESS, enablePin1);
mtrn3100::longLidar frontSensor(FRONT_ADDRESS, XSHUT);

// Initialise IMU
GY521 imu(IMU_ADDRESS);

// Motor Setup
mtrn3100::Motor l_motor(2, 4, 3);
mtrn3100::Motor r_motor(7, 5, 6);

// Setup Encoder pins.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }



// -------------- PID's --------------

// PIDs for motors (moving Forward)
mtrn3100::PIDController l_forward_pid(29, 0, 0, 0);    // Best one for 200 PWM
mtrn3100::PIDController r_forward_pid(29, 0, 0, 0);    // Best one for 200 PWM

// PIDs for motors (turning)
mtrn3100::PIDController l_turn_pid(29.875, 0, 0, 0);    // Best one for 200 PWM
mtrn3100::PIDController r_turn_pid(29.875, 0, 0, 0);    // Best one for 200 PWM

// PID for side LiDar course correction
mtrn3100::PIDController side_lidar_pid(0.6, 0, 0.3, 0);     // Best one

// PID for front LiDar course correction
mtrn3100::PIDController l_front_lidar_pid(0.37, 60, 0, 0.5);     // Best one
mtrn3100::PIDController r_front_lidar_pid(0.41, 60, 0, 0.5);     // Best one

// PID for IMU course correction
mtrn3100::IMUPIDController imu_forward_pid(15, 0, 3, 0);     // Best one     


// Moving Average Filter for PID error
mtrn3100::MovingAverageFilter<float, 30> l_move_filter;
mtrn3100::MovingAverageFilter<float, 30> r_move_filter;


// -------------- Function Declarations --------------

// Function declarations for Movement
void moveForward(int cell_count);
void turnLeft(int turn_count);
void turnRight(int turn_count);
int driveMotors(float l_target, float r_target);
void followMotionPlan(String motionPlan);

// Function declarations for Mapping
void changeHeadingLeft();
void changeHeadingRight();
void changeCoordinates();
String getPose();
void neighbourCell(char direction);
char getDirection(int direction);
void updateGraph();
void shortestPath();
String pathToMotionPlan();
String createMotionPlan();
void getGraph();
void getSrtAndDest();
void autoMapping();

// Function declarations for Sensing
void setUpSensors();
void detectWalls();
void printWalls();


// -------------- Variable Declaration and DataStore --------------

// Map specifications.
const float cellSize = 250; // mm

// Robot specifications.
const int CONTROL_SPEED = 75;
const int MAX_SPEED = 200;      // Max PWM
const float wheel_radius = 23.59; // mm.
const float axle_length = 128.86; // mm.
const float radians_per_cell = cellSize / wheel_radius;
const float bias = 0.25;
const float radians_per_90turn = M_PI * axle_length / (4 * wheel_radius) + bias;
const float radians_per_degreeturn = 1/90 * (M_PI * axle_length / (4 * wheel_radius) + bias);

// PID signal variables.
float l_encoder_signal = 1;
float r_encoder_signal = 1;
float side_lidar_signal = 0;
float l_front_lidar_signal = 0;
float r_front_lidar_signal = 0;
float l_drive_signal = 1;
float r_drive_signal = 1;
float imu_signal = 0;

// Motion control variables.
float l_actual_pos = 0;
float r_actual_pos = 0;
float speed_scalar = 1;
bool lidar_driving = false;
int moveCount = 0;

// Other variables.
uint8_t times_spun = 0;
int row = 0;
int col = 0;
char head = 'S';
int nRow = 0;
int nCol = 0;
int currentWallPosition[3] = {0, 0, 0}; // [front, left, right], 0 = no wall, 1 = wall

// Initialise maze with nodes.
const int numRows = 5;
const int numCols = 9;
mtrn3100::Graph<int, int> mazeGraph;
int destIdx = 0;
int path[numRows*numCols];


#endif
