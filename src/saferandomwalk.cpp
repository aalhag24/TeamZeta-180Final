/**
Write a node called "saferandomwalk" that moves the robot randomly in the environment but stops the 		robot as soon as it gets closer than 0.2m to any obstacle. 

To detect obstacles, use the onboard laser range finder publishing its data on the /scan topic.

Important: your robot must move in the environment. 

If it stands still or just rotates in place, you will get no credit.

*/


#include <ros/ros.h>
#include <stdlib.h> 
#include <string>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <tf/transform_listener.h>
#include <algorithm>

#include <sensor_msgs/LaserScan.h>
#include <Eigen/Eigenvalues>

using namespace std;


void GeneratePose(geometry_msgs::Twist &Pose);
void Scanning(const sensor_msgs::LaserScan msg);

sensor_msgs::LaserScan scanner;
geometry_msgs::Twist position;

int Center = 360;
bool First = true;
bool Changed = false;
bool Set = false;
bool Switch = true;

int main(int argc,char **argv) {
    ros::init(argc,argv,"saferandomwalk");
    ros::NodeHandle nh;

	ros::Subscriber SConvert = nh.subscribe("/scan",1000, &Scanning);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);

	ros::Rate rate(1);

//Initial Positions
	position.linear.x = 1;
	position.angular.z = M_PI/(6);
	pub.publish(position);

	while(ros::ok()){
		//ROS_INFO("5");
		if(Set){	
			//ROS_INFO(scanner.ranges[Center]);
				ROS_INFO("Center: [%f]", scanner.ranges[Center]);
				ROS_INFO("Left: [%f]", scanner.ranges[300]);
				ROS_INFO("Right: [%f]", scanner.ranges[420]);
			
			/// U TURN
			if(scanner.ranges[Center] < 2 && scanner.ranges[300] < 2 && scanner.ranges[420] < 2){
				position.linear.x = -1;	///NOT DOING ANYTHING??
				position.angular.z = 4*M_PI;
				Changed = true;
			}
			else{ /// Proper sensing for center, right and left
				if(scanner.ranges[Center] < 2){
					position.linear.x = 0;
					if(scanner.ranges[300] > 2 && scanner.ranges[420] > 2){position.angular.z=M_PI/(2);}
					if(scanner.ranges[700] < 2)
						position.angular.z += M_PI/8;
					if(scanner.ranges[50] < 2)
						position.angular.z -= M_PI/8;
					Changed = true;
				}	
				if(scanner.ranges[300] < 2 && scanner.ranges[420] < 2){
					position.linear.x = -0.2;
					//position.angular.z = 4*M_PI;
					Changed = true;
				}
				else{
					if(scanner.ranges[300] < 2){
						position.angular.z = position.angular.z - M_PI/(4);
						Changed = true;
					}
					if(scanner.ranges[420] < 2){
						position.angular.z = position.angular.z + M_PI/(4);
						Changed = true;
					}
				}
			}

			pub.publish(position);

			if(Changed){
				ros::Duration(0.75).sleep();
				Changed = false;			
			}
			else{
				Changed = true;
				GeneratePose(position);
				pub.publish(position);
				ROS_INFO("Generating new Direction");
			}
			//rate.sleep();
			//GeneratePose(position);
			//ros::spinOnce();
			ROS_INFO_STREAM(position);
		}
		else{
			ros::spinOnce();
		}
		pub.publish(position);

		//THIS MAY BE AN ISSUE
		//I HAVE TRIED spin
		// CAUSES ISSUES
		ros::spinOnce();
	}
    return 0;    
}

void Scanning(const sensor_msgs::LaserScan msg){
	scanner = msg;
	Set = true;
	//ROS_INFO("3");
}

void GeneratePose(geometry_msgs::Twist &Pose){
	float Valuex, Valuez;

	Valuex = rand() % 10+1;
	Valuez = rand() % 20+1;
	Valuex = Valuex/5;

	Pose.linear.x = Valuex;
	Pose.angular.z = M_PI/(Valuez*3);
}








