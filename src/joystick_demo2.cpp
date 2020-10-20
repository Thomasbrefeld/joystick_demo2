#include "joystick_demo2.h"

JoystickDemo::JoystickDemo(ros::NodeHandle &node, ros::NodeHandle &priv_nh) : counter_(0){
  brake_gain_ = 1.0;
  throttle_gain_ = 1.0;
  priv_nh.getParam("brake_gain", brake_gain_);
  priv_nh.getParam("throttle_gain", throttle_gain_);
  brake_gain_ = std::min(std::max(brake_gain_,    (float)0), (float)1);
  throttle_gain_ = std::min(std::max(throttle_gain_, (float)0), (float)1);

  sub_twist_ = node.subscribe("/joy_to_twist", 1, &JoystickDemo::twist_callback, this);

  data_.linear_x = 0;
  data_.angular_z = 0;

  gear_changed_ = ros::Time::now();
  brake_time_ = ros::Time::now();
  last_gear_ = 0;

  pub_brake_ = node.advertise<dbw_polaris_msgs::BrakeCmd>("brake_cmd", 1);
  pub_throttle_ = node.advertise<dbw_polaris_msgs::ThrottleCmd>("throttle_cmd", 1);
  pub_steering_ = node.advertise<dbw_polaris_msgs::SteeringCmd>("steering_cmd", 1);
  pub_gear_ = node.advertise<dbw_polaris_msgs::GearCmd>("gear_cmd", 1);
  pub_enable_ = node.advertise<std_msgs::Empty>("enable", 1);

  timer_ = node.createTimer(ros::Duration(0.02), &JoystickDemo::cmdCallback, this);

}

void JoystickDemo::cmdCallback(const ros::TimerEvent& event){
  // Gear
  if (data_.linear_x < 0 && last_gear_ != -1){
    dbw_polaris_msgs::GearCmd gear_msg;
    gear_msg.cmd.gear = dbw_polaris_msgs::Gear::REVERSE;
    pub_gear_.publish(gear_msg); //might need to check timing
    last_gear_ = -1;
    gear_changed_ = ros::Time::now();
    ROS_INFO_STREAM("Gear changed to: REVERSE");
  }
  else if (data_.linear_x > 0 && last_gear_ != 1){
    dbw_polaris_msgs::GearCmd gear_msg;
    gear_msg.cmd.gear = dbw_polaris_msgs::Gear::DRIVE;
    pub_gear_.publish(gear_msg); //might need to check timing
    last_gear_ = 1;
    gear_changed_ = ros::Time::now();
    ROS_INFO_STREAM("Gear changed to: FORWARD");
  }

  if (fabs(data_.linear_x) > 0.0 && fabs(data_.linear_x <= 1.0) && event.current_real - gear_changed_ > ros::Duration(0.5)){
    // Throttle 
    dbw_polaris_msgs::ThrottleCmd throttle_msg;
    throttle_msg.enable = true;
    throttle_msg.ignore = false;
    throttle_msg.count = counter_;
    throttle_msg.pedal_cmd_type = dbw_polaris_msgs::ThrottleCmd::CMD_PERCENT;
    throttle_msg.pedal_cmd = fabs(data_.linear_x) * throttle_gain_;
    pub_throttle_.publish(throttle_msg);
    brake_time_ = ros::Time::now();
  }
  else {
    // Brake
    dbw_polaris_msgs::BrakeCmd brake_msg;
    brake_msg.enable = true;
    brake_msg.ignore = false;
    brake_msg.count = counter_;
    brake_msg.pedal_cmd_type = dbw_polaris_msgs::BrakeCmd::CMD_PERCENT;
    if ((event.current_real - brake_time_).toSec() < 1.0)
      brake_msg.pedal_cmd = (event.current_real - brake_time_).toSec() * .33;
    else
      brake_msg.pedal_cmd = .3;
    pub_brake_.publish(brake_msg);
  }

  dbw_polaris_msgs::SteeringCmd steering_msg;
  steering_msg.enable = true;
  steering_msg.ignore = false;
  steering_msg.count = counter_;
  steering_msg.cmd_type = dbw_polaris_msgs::SteeringCmd::CMD_ANGLE;
  float tau = 0.1;
  float filtered_steering_cmd = 0.02 / tau * (dbw_polaris_msgs::SteeringCmd::ANGLE_MAX * data_.angular_z) + (1 - 0.02 / tau) * last_steering_filt_output_;
  last_steering_filt_output_ = filtered_steering_cmd;
  steering_msg.steering_wheel_angle_cmd = filtered_steering_cmd;
  pub_steering_.publish(steering_msg);
}

void JoystickDemo::twist_callback(const geometry_msgs::Twist::ConstPtr& msg){
  pub_enable_.publish(std_msgs::Empty());

  data_.linear_x = msg->linear.x;
  data_.angular_z = msg->angular.z;
  data_.stamp = ros::Time::now();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "joystick_demo2");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // Create JoystickDemo class
  JoystickDemo n(node, priv_nh);

  // Handle callbacks until shutdown
  ros::spin();

  return 0;
}
