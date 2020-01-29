#include<ros/ros.h>

#include<sstream>
#include<stdio.h>
#include<vector>
#include<iostream>
#include<stdlib.h>
#include<math.h>

#include<sensor_msgs/JointState.h>
#include<baxter_core_msgs/JointCommand.h>
#include<tf/transform_listener.h>

int main(int argc, char **argv){
    ros::init(argc,argv,"puppet_hand_node");
    ros::NodeHandle nh_;

    tf::TransformListener ln;

	ros::Publisher pub_commnad=nh_.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command",1);
	ros::ServiceClient client=nh_.serviceClient<baxter_core_msgs/SolvePositionIK>("/ExternalTools/right/PositionKineamaticsNode/IKService")
	baxter_core_msgs::SolvePositionIK srv;

	string joint_names[7]={"left_s0","left_e0","left_e1","left_w0","left_w1","left_w2"};

    ros::Rate loop_rate(10);
    while(ros::ok()){
        tf::StampedTransform desired_transform;
        try{
            ln.lookupTransform("base","gr4_frame",ros::Time(0),desired_transform);

        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            //ros::Duration(1.0).sleep();
        }

        std::cout << desired_transform.getOrigin().getX() << std::endl ;

		srv.request.pose_stamp[0].header.stamp=ros::Time::now();
		srv.request.pose_stamp[0].header.frame_id="base";
		srv.request.pose_stamp[0].pose.position=desired_transform.getOrigin();
		srv.request.pose_stamp[0].pose.orientation=desired_transform.getRotatin();
		//Another way
		//geometry_msgs::PoseStamped r_info;
		//r_info.pose.position.x=transform.getOrigin().getX();
		

		if(!client.exists()){
			ROS_ERROR("Service does not exist");
			continue;
		}
		if(!client.isvalid()){
			ROS_ERROR("Service is not Valid");
		}

		if(!client.call(srv)){
			ROS_ERROR("Call to Service Failed");

		}else{
			ROS_INFO("Success to Call");
			if(!srv.request.isValid[0]){
				ROS_ERROR("No Solution");
			}else{
				baxter_core_msgs::JointCommand new_left;
				new_left.mode=new_left.POSITION_MODE;

				for(int i=0;i<7;i++){
					new_left.name.push_back(joint_names[i]);
					new_left.command.push_back(srv.response.joints[0].position);

				}
				pub_command.publish(new_left);

			}
		}


        loop_rate.sleep();
    }
	ROS_INFO("ROS_NODE TERMINATED \n");

}
