
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>

geometry_msgs::Twist twist;

bool publish=false;
double L_GAIN;
double A_GAIN;
ros::ServiceClient stopclient;
std_srvs::Empty srv;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){

    //LOGITECH + BACK
    if (msg->buttons[6] && msg->buttons[8]){
       //shutdown
       ROS_WARN("SHUTDOWN by joypad command");
       system("shutdown -P now"); 
    }

    //L1+A: clear estop
    if (msg->buttons[4] && msg->buttons[0]){
        ROS_WARN("[JOYPAD] estop CLEAR");
        system("rosservice call /clear_estop");
        return;
    }

    //L1: ESTOP
    if (msg->buttons[4]){
        ROS_WARN("[JOYPAD] ESTOP TRIGGERED ");
        stopclient.call(srv);
        return;
    }

    if (msg->buttons[7]){
        ROS_INFO("[JOYPAD] starting autonomous driving");
    }

    L_GAIN = 0.3;
	A_GAIN = 0.3;

	L_GAIN = L_GAIN + (1 - msg->axes[5])*0.35;

    twist.angular.z = A_GAIN*msg->axes[3];
    twist.linear.x = L_GAIN*msg->axes[4];
    publish=true;
    
    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "Joy4Sandwich");
    
    ros::NodeHandle nh;

    ros::Publisher pub  = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
    
    stopclient = nh.serviceClient<std_srvs::Empty>("stop_motors");

    ros::Rate loop_rate(50);

    while(ros::ok()){
        if (publish){
            pub.publish(twist);
            publish=false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}