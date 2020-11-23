#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


ros::Publisher cmd_vel;
ros::Publisher pub_enable_;
ros::Subscriber joy_sub;

sensor_msgs::Joy joy_input;
geometry_msgs::Twist vel_msg;
std_msgs::Bool enable_msg;

ros::Timer timer_;

void joy_call_back(const sensor_msgs::Joy::ConstPtr& msg){
    vel_msg.linear.x = msg->axes[1];
    vel_msg.angular.z = msg->axes[3];
    cmd_vel.publish(vel_msg);

    if (msg->buttons[4]){
        enable_msg.data = false;
        ROS_INFO_STREAM("DISABLED");
    }
    else {
        if (msg->buttons[5]){
            enable_msg.data = true;
            ROS_INFO_STREAM("ENABLED");
        }
    }
    pub_enable_.publish(enable_msg);
}

void timer_cb(const ros::TimerEvent& event){
    pub_enable_.publish(enable_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_to_twist");
    ros::NodeHandle nh;

    //sets subscriber and publisher
    cmd_vel = nh.advertise<geometry_msgs::Twist>("/vehicle/cmd_vel", 100);
    pub_enable_ = nh.advertise<std_msgs::Bool>("vehicle/dbw_enabled", 100);
    joy_sub = nh.subscribe("/joy", 10, joy_call_back);

    timer_ = nh.createTimer(ros::Duration(0.02), timer_cb);

    //populates the joy_input with zeros
    for(int x = 0; x < 6; ++x)
        joy_input.axes.push_back(0);

    //fills the unused variables with zeros
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    ROS_INFO_STREAM("joy_to_twist started!");
    ros::spin();
}
