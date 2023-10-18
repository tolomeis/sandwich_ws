
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>
#include <sandwich/LaunchNode.h>
#include <sandwich/StopNode.h>

geometry_msgs::Twist twist;

bool publish = false;
double L_GAIN;
double A_GAIN;
ros::ServiceClient stopclient;
ros::ServiceClient nodestart_client;
ros::ServiceClient nodestop_client;
std_srvs::Empty srv;

bool autonomous_active = false;

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
        if(autonomous_active){
             ROS_INFO("[JOYPAD] stopping autonomous driving");
            sandwich::StopNode srv;
            srv.request.node_name = "cheese_ad";
            nodestop_client.call(srv);
            srv.request.node_name = "segnet";
            nodestop_client.call(srv);
            autonomous_active = false;
        }else{
            ROS_INFO("[JOYPAD] starting autonomous driving");
            sandwich::LaunchNode srv;
            srv.request.package_name = "sandwich_autonomous";
            srv.request.file_name = "sandwich_auto.launch";
            nodestart_client.call(srv);
            autonomous_active = true;
        }
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

    nodestart_client = nh.serviceClient<sandwich::LaunchNode>("launch_node");
    nodestop_client = nh.serviceClient<sandwich::StopNode>("stop_node");

    
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