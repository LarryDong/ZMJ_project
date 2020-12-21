
#include <iostream>
#include <ros/ros.h>
#include  <std_msgs/String.h>

using namespace std;




void handler(const std_msgs::String s){
    ROS_INFO_STREAM("Input message: " << s);
    if(s.data == "s"){
        ROS_INFO("Save Points Cloud.");
    }
    else{
        ROS_WARN("Incorrect command.");
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "my_debug");
    ros::NodeHandle nh;
    ROS_INFO("Debug node begin......");

    ros::Subscriber subString = nh.subscribe<std_msgs::String>("/debug", 1, handler);

    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}