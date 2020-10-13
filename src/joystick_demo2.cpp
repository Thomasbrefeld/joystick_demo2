#include "joystick_demo2.h"

JoystickDemo::JoystickDemo(ros::NodeHandle &node, ros::NodeHandle &priv_nh) : counter_(0)
{
  brake_gain_ = 1.0;
  throttle_gain_ = 1.0;
  priv_nh.getParam("brake_gain", brake_gain_);
  priv_nh.getParam("throttle_gain", throttle_gain_);
  brake_gain_    = std::min(std::max(brake_gain_,    (float)0), (float)1);
  throttle_gain_ = std::min(std::max(throttle_gain_, (float)0), (float)1);

  sub_twist_ = node.subscribe("/joy_to_twist", 1, &JoystickDemo::twist_callback, this);

  data_.linear_x = 0;
  data_.angular_z = 0;

  pub_brake_ = node.advertise<dbw_polaris_msgs::BrakeCmd>("brake_cmd", 1);
  pub_throttle_ = node.advertise<dbw_polaris_msgs::ThrottleCmd>("throttle_cmd", 1);
  pub_steering_ = node.advertise<dbw_polaris_msgs::SteeringCmd>("steering_cmd", 1);
  pub_steering_cal_ = node.advertise<std_msgs::Empty>("calibrate_steering", 1);
  pub_gear_ = node.advertise<dbw_polaris_msgs::GearCmd>("gear_cmd", 1);
  pub_enable_ = node.advertise<std_msgs::Empty>("enable", 1);
  pub_disable_ = node.advertise<std_msgs::Empty>("disable", 1);

  timer_ = node.createTimer(ros::Duration(0.02), &JoystickDemo::cmdCallback, this);
}

void JoystickDemo::cmdCallback(const ros::TimerEvent& event)
{
  /*
  // Detect joy timeouts and reset
  if (event.current_real - data_.stamp > ros::Duration(0.1)) {
    data_.joy_throttle_valid = false;
    data_.joy_brake_valid = false;
    last_steering_filt_output_ = 0.0;
    return;
  }

  //ROS_INFO_STREAM(event.current_real - data_.stamp);

  // Optional watchdog counter
  if (count_) {
    counter_++;
  }

  // Brake
  if (brake_) {
    dbw_polaris_msgs::BrakeCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    msg.count = counter_;
    msg.pedal_cmd_type = dbw_polaris_msgs::BrakeCmd::CMD_PERCENT;
    msg.pedal_cmd = data_.brake_joy * brake_gain_;
    pub_brake_.publish(msg);
  }

  // Throttle
  if (throttle_) {
    dbw_polaris_msgs::ThrottleCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    msg.count = counter_;
    msg.pedal_cmd_type = dbw_polaris_msgs::ThrottleCmd::CMD_PERCENT;
    msg.pedal_cmd = data_.throttle_joy * throttle_gain_;
    pub_throttle_.publish(msg);
  }

  // Steering
  if (steer_) {
    dbw_polaris_msgs::SteeringCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    msg.count = counter_;
    if (!strq_) {
      msg.cmd_type = dbw_polaris_msgs::SteeringCmd::CMD_ANGLE;

      float raw_steering_cmd;
      if (data_.steering_mult) {
        raw_steering_cmd = dbw_polaris_msgs::SteeringCmd::ANGLE_MAX * data_.steering_joy;
      } else {
        raw_steering_cmd = 0.5 * dbw_polaris_msgs::SteeringCmd::ANGLE_MAX * data_.steering_joy;
      }

      float tau = 0.1;
      float filtered_steering_cmd = 0.02 / tau * raw_steering_cmd + (1 - 0.02 / tau) * last_steering_filt_output_;
      last_steering_filt_output_ = filtered_steering_cmd;

      msg.steering_wheel_angle_velocity = svel_;
      msg.steering_wheel_angle_cmd = filtered_steering_cmd;
    } else {
      msg.cmd_type = dbw_polaris_msgs::SteeringCmd::CMD_TORQUE;
      msg.steering_wheel_torque_cmd = dbw_polaris_msgs::SteeringCmd::TORQUE_MAX * data_.steering_joy;
    }
    pub_steering_.publish(msg);
  }

  // Gear
  if (shift_) {
    if (data_.gear_cmd != dbw_polaris_msgs::Gear::NONE) {
      dbw_polaris_msgs::GearCmd msg;
      msg.cmd.gear = data_.gear_cmd;
      pub_gear_.publish(msg);
    }
  }

  // Steering calibration
  if (steer_) {
    // Only calibrate on 'rising edge' of button based steer cal request
    static bool steering_cal_last = false;
    if (data_.steering_cal && !steering_cal_last) {
      pub_steering_cal_.publish(std_msgs::Empty());
    }
  }
  */
}

void JoystickDemo::twist_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO_STREAM(msg->linear.x);
  data_.linear_x = msg->linear.x;


  data_.stamp = ros::Time::now();
  //joy_ = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_demo2");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // Create JoystickDemo class
  JoystickDemo n(node, priv_nh);

  // Handle callbacks until shutdown
  ros::spin();

  return 0;
}