#include<ros/ros.h>

#include<sstream>
#include<stdio.h>
#include<vector>
#include<iostream>
#include<stdlib.h>
#include<math.h>

//#include<sensor_msgs/JointState.h>
//#include<baxter_core_msgs/JointCommand.h>
#include<tf/transform_broadcaster.h>


using namespace std;

int main(int argc, char** argv){
    ros::init(argc,argv,"tf_broadcaster");
    ros::NodeHandle nh_;
	
	//transform is a transformation between 2 frames
	//setOrigin is translation, setRotation is rotation;
    tf::Transform transform;
    tf::TransformBroadcaster br;

    //"right_hand" is arleady defined
    //gr4_frame is like child of right_hand
    ros::Rate rate(50);
	while(ros::ok()){
            ros::spinOnce();
            transform.setOrigin(tf::Vector3(0.0,0.0,0.3));
            transform.setRotation(tf::Quaternion(0,1,0,0));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"right_hand","gr4_frame"));
            rate.sleep();
        }
        return 0;

};
