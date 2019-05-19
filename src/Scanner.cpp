#include <ros/ros.h>
#include <stdlib.h> 
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <sensor_msgs/LaserScan.h>
#include <angles/angles.h>
#include "LocalScan.msg"

using namespace std;
/** Flags **/
	bool ScanSet = false, LMMRSet = false, AMCLSet = false;
/** Function Definition */
	void Scanning(const sensor_msgs::LaserScan msg);
	void localMapMessageReceived(const nav_msgs::OccupancyGrid&);

/** Main Function **/
int main(int argc,char **argv) {
    ros::init(argc,argv,"saferandomwalk");
    ros::NodeHandle nh;

	ros::Subscriber subLaser = nh.subscribe("/scan",1000, &Scanning);
	ros::Subscriber sublocal=nh.subscribe("/move_base/local_costmap/costmap",100, &localMapMessageReceived);
	ros::Subscriber subAMCL = nh.subscribe("/amcl_pose", 1000, &AMCLDATA);


	while(!(ScanSet && LMMRSet){ ros::spinOnce(); }

	while(ros::ok(){
		// Take the msg and find the dir attributes
		// Take the local Map and extract information
	}

}

void Scanning(const sensor_msgs::LaserScan msg){
	scanner = msg;
	ScanSet = true;
}

void AMCLDATA(const geometry_msgs::PoseWithCovarianceStamped& msg){
	AMCLz = msg.pose.pose.orientation.z;
	AMCLw = msg.pose.pose.orientation.w;

	//ROS_INFO("AMCL: [%f]", AMCLz);
	ROS_INFO("AMCL: [%f] [%f]", msg.pose.pose.position.x, msg.pose.pose.position.y);
	
	AMCLSet = true;
}

void localMapMessageReceived(const nav_msgs::OccupancyGrid&msg) {
	ROS_INFO_STREAM("Received Local Map");
	ROS_INFO_STREAM("Dimensions: " << msg.info.width << " x " << msg.info.height);
	ROS_INFO_STREAM("Resolution " << msg.info.resolution);

	int z= 0;
	for (int i = 0 ; i < msg.info.height ; i++) {
		for ( int j = 0 ; j < msg.info.width ; j++ )  
			localmap << (int)msg.data[z++] << " " ;
	}

	LMMRSet = true;
}








