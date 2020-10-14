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
  void gearCheck(const geometry_msgs::Twist::ConstPtr& msg);

  // Topics
  ros::Subscriber sub_twist_;
  ros::Publisher pub_brake_;
  ros::Publisher pub_throttle_;
  ros::Publisher pub_steering_;
  ros::Publisher pub_gear_;
  ros::Publisher pub_steering_cal_;
  ros::Publisher pub_enable_;
  ros::Publisher pub_disable_;

  // Parameters
  bool brake_; // Send brake commands
  bool throttle_; // Send throttle commands
  bool steer_; // Send steering commands
  bool shift_; // Send shift commands

  // Parameters
  float brake_gain_; // Adjust brake value
  float throttle_gain_; // Adjust throttle value
  float steer_gain_;

  // Parameters
  //bool ignore_; // Ignore driver overrides
  //bool enable_; // Use enable and disable buttons
  //bool count_; // Increment counter to enable watchdog
  //bool strq_; // Steering torque command (otherwise angle)
  //float svel_; // Steering command speed

  // Variables
  ros::Timer timer_;
  JoystickDataStruct data_;
  geometry_msgs::Twist twist_;
  uint8_t counter_;
  //float last_steering_filt_output_;

};

#endif /* JOYSTICK_DEMO2_H_ */

