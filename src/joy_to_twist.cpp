#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float32.h"

ros::Publisher cmd_vel;
ros::Subscriber joy_sub;

sensor_msgs::Joy joy_input;
geometry_msgs::Twist vel_msg;

void joy_call_back(const sensor_msgs::Joy::ConstPtr& msg){
    joy_input.axes[1] = msg->axes[1];
    joy_input.axes[2] = msg->axes[2];
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_to_twist");
    ros::NodeHandle nh;

    ros::Rate loop_rate(60);

    //populates the joy_input with zeros
    for(int x = 0; x < 6; ++x)
        joy_input.axes.push_back(0);
    
    //fills the unused variables with zeros
    vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

    //sets subscriber and publisher
    cmd_vel = nh.advertise<geometry_msgs::Twist>("/joy_to_twist", 1000);
    joy_sub = nh.subscribe("/joy", 10, joy_call_back);

    while(true){
        //takes the input from the left up and down joystick and assigns it to linear x 
        vel_msg.linear.x = joy_input.axes[1];
        //takes the input from the right right and left joy stick and assigns it to angular z after multiplying by .6
        vel_msg.angular.z = joy_input.axes[2];
        //publishes the message to cmd_vel
        cmd_vel.publish(vel_msg);
        //sleeps for 1/60 of a second
        loop_rate.sleep();
        //spins
        ros::spinOnce();
    }
}
