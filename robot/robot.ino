#include <arduino.h>
#include "robot.hpp"

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  setUpSensors();
  getGraph(); // uncomment for CV
  //autoMapping(); // uncomment for autonomous, make sure robot is placed in pose 00S
  getSrtAndDest();
}

void loop() {
    String motionPlan = createMotionPlan();
    Serial3.println(motionPlan);
    // String motionPlan = "FFLFFRRFRFRFLFLFLLFRFRFLFLFRRFFRFFRR";
    // followMotionPlan(motionPlan);
    Serial.println("Done");
    while(1);
}
