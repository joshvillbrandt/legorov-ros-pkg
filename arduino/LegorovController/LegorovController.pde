/*
  LegorovController.ino - ROS interface for the "Lego-rov" robot.
  Created by Josh Villbrandt (http://javconcepts.com/), November 6, 2012.
  Released into the public domain.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <ros.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <AFMotor.h>

// Global Variables
#define WATCHDOG_PERIOD 100 // 10 Hz
#define LINEAR_X_MAX_VEL 0.4 // m/s
#define ANGULAR_Z_MAX_VEL 1.6 // rad/s
#define DEADZONE 0.01 // percentage
#define CMD_VEL_DEBUG false
typedef ros::NodeHandle_<ArduinoHardware, 25, 25, 200, 200> SlimNodeHandle;
SlimNodeHandle  nh;
unsigned long watchdog_timer = 0;
AF_DCMotor left_motor(1, MOTOR12_64KHZ);
AF_DCMotor right_motor(2, MOTOR12_64KHZ);

// cmd_vel Callback    
void vel_callback( const geometry_msgs::Twist& cmdvel) {
  //nh.loginfo("inside vel_callback!");
  
  // normalize params
  float linear_x = constrain(cmdvel.linear.x, -1.0 * LINEAR_X_MAX_VEL, LINEAR_X_MAX_VEL) / LINEAR_X_MAX_VEL;
  float angular_z = constrain(cmdvel.angular.z, -1.0 * ANGULAR_Z_MAX_VEL, ANGULAR_Z_MAX_VEL) / ANGULAR_Z_MAX_VEL;
    
  // convert to individual motors
  float left_cmd = linear_x - angular_z;
  float right_cmd = linear_x + angular_z;
  
  // adjust center
  float val = max(left_cmd, right_cmd);
  if(val > 1) {
    left_cmd = left_cmd - (val - 1.0);
    right_cmd = right_cmd - (val - 1.0);
  }
  val = min(left_cmd, right_cmd);
  if(val < -1) {
    left_cmd = left_cmd - (val + 1.0);
    right_cmd = right_cmd - (val + 1.0);
  }
  
  // debug
  if(CMD_VEL_DEBUG) {
    char log_msg[20];
    sprintf(log_msg, "L: %d", (int)(left_cmd*255));
    nh.loginfo(log_msg);
    sprintf(log_msg, "R: %d", (int)(right_cmd*255));
    nh.loginfo(log_msg);
  }
  
  // set motors
  if(fabs(left_cmd) < DEADZONE) {
    left_motor.setSpeed(0);
    left_motor.run(RELEASE);
  }
  else {
    if(left_cmd > 0) left_motor.run(FORWARD);
    else left_motor.run(BACKWARD);
    left_motor.setSpeed(fabs(left_cmd) * 255);
  }
  if(fabs(right_cmd) < DEADZONE) {
    right_motor.setSpeed(0);
    right_motor.run(RELEASE);
  }
  else {
    if(right_cmd > 0) right_motor.run(FORWARD);
    else right_motor.run(BACKWARD);
    right_motor.setSpeed(fabs(right_cmd) * 255);
  }
  
  // update watchdog (don't tase me bro!)
  watchdog_timer = millis();
}
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", &vel_callback);

void setup()
{
  // ROS Setup
  nh.initNode();
  nh.subscribe(vel_sub);
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("LegorovController startup complete");
  
  // Send default motor command
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
}

void loop()
{
  // auto stop motors if we haven't received a cmd_vel message in a while
  if((millis() - watchdog_timer) > WATCHDOG_PERIOD) {
    left_motor.run(RELEASE);
    right_motor.run(RELEASE);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    watchdog_timer = millis();
  }
  
  // send / receive ROS messages
  nh.spinOnce();
  delay(1);
}
