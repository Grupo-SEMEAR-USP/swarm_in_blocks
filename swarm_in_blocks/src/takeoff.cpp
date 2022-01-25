/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <iostream>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("clover0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("clover0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("clover0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("clover0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time first_request = ros::Time::now();

    double w;
    w = 2*3.1415/5;
    
    double x, y;

    while(ros::ok()){
        
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(ros::Time::now()- first_request < ros::Duration(30.0)){
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            local_pos_pub.publish(pose);
        }

        

        if(ros::Time::now()- first_request > ros::Duration(30.0)){

            double t = (ros::Time::now() - first_request).toSec();
            x =  5*std::cos(w*t);
            y = 5*std::sin(w*t);

            std::cout << "t: " << t << "x: " << x << ", y:" << y << "w: " << w << "\n";
            
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 2;
            local_pos_pub.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}