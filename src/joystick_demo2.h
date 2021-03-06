#ifndef JOYSTICK_DEMO2_H_
#define JOYSTICK_DEMO2_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "geometry_msgs/Twist.h"

#include <dbw_polaris_msgs/ThrottleCmd.h>
#include <dbw_polaris_msgs/BrakeCmd.h>
#include <dbw_polaris_msgs/SteeringCmd.h>
#include <dbw_polaris_msgs/GearCmd.h>

typedef struct {
  ros::Time stamp;
  float linear_x;
  float angular_z;
} JoystickDataStruct;

class JoystickDemo {
public:
  JoystickDemo(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
private:
  void twist_callback(const geometry_msgs::Twist::ConstPtr& msg);
  void cmdCallback(const ros::TimerEvent& event);

  // Topics
  ros::Subscriber sub_twist_;
  ros::Publisher pub_brake_;
  ros::Publisher pub_throttle_;
  ros::Publisher pub_steering_;
  ros::Publisher pub_gear_;
  ros::Publisher pub_enable_;

  // Parameters
  float brake_gain_; // Adjust brake value
  float throttle_gain_; // Adjust throttle value
  float steer_gain_;

  // Variables
  ros::Timer timer_;
  JoystickDataStruct data_;
  geometry_msgs::Twist twist_;
  uint8_t counter_;
  float last_steering_filt_output_;
  ros::Time gear_changed_;
  ros::Time brake_time_;
  int last_gear_;

};

#endif /* JOYSTICK_DEMO2_H_ */

