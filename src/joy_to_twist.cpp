#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float32.h"

ros::Publisher cmd_vel;
ros::Subscriber joy_sub;

sensor_msgs::Joy joy_input;
geometry_msgs::Twist vel_msg;

void joy_call_back(const sensor_msgs::Joy::ConstPtr& msg){
    vel_msg.linear.x = msg->axes[1];
    vel_msg.angular.z = msg->axes[3];
    cmd_vel.publish(vel_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_to_twist");
    ros::NodeHandle nh;

    //sets subscriber and publisher
    cmd_vel = nh.advertise<geometry_msgs::Twist>("/joy_to_twist", 1000);
    joy_sub = nh.subscribe("/joy", 10, joy_call_back);

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
