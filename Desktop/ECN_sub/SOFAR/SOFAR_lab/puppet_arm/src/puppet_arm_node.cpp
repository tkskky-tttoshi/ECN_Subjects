/**
 * \file
 * \brief 
 * \author 
 * \version 0.1
 * \date 
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"
// Include here the ".h" files corresponding to the topic types you use.
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>

// You may have a number of globals here.
//...
using namespace std;
ros::Publisher pub_command;

sensor_msgs::JointState last_state;

string joint_name;

// Callback functions...
bool stateAvailable=false;
void jonitStateCallback(sensor_msgs::JointState state_msg){
    //1st element is "header" in topics
    if( state_msg.name.size() > 2 ){
        //last_state has all current joints elements
        //[1] left_e0 [2] left_e1...
        last_state = state_msg;
        stateAvailable=true;
    }
}


int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "puppet_arm_node");

    // Define your node handles
    // ...
    ros::NodeHandle nh_("~");
    ros::NodeHandle nh_glob;

    // Read the node parameters if any
    // ...
    if( !nh_.getParam("joint_name",joint_name) )
    {
      ROS_INFO("Couldn't find parameter: joint_name\n");
    }
    else
    {
      ROS_INFO("Controlling joint: %s\n",joint_name.c_str()) ;
    }
	
    // Declare your node's subscriptions and service clients
    // ...
	
    ros::Subscriber baxter_joint_state = nh_.subscribe<sensor_msgs::JointState> ("/robot/joint_states"  , 1 , jonitStateCallback);

    // Declare you publishers and service servers
    // ...
    pub_command = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);

    ros::Rate rate(50);   // Or other rate.
    while (ros::ok()){
        ros::spinOnce();
        if(!stateAvailable) continue;
        // here you build your command msg and publish it.
        //right joints are 9th 11th 13th 15th one
        //sensor_msgms::JointState parameters have "name" and "position"
        //baxter_core_msgs::JointCommand parameters have "mode" "command" "name"
        //mode 1:position 2:velocity 3:torque
        baxter_core_msgs::JointCommand new_left;
        new_left.mode = new_left.POSITION_MODE ;

        for (int i=9;i<=15;i++){
            new_left.position.push_back(new_left.position[i]);
            if (i%2==1){
                new_left.position[i-9]=-new_left.position[i-9];
            }
             new_left.name.push_back(last_state.name[i-7]);
        }

        pub_command.publish(new_left);

        rate.sleep();
    }
    ROS_INFO("ROS-Node Terminated\n");
}





