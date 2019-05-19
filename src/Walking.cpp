#include <ros/ros.h>
#include <stdlib.h> 
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/FrameGraph.h> 

#include <algorithm>
#include <sensor_msgs/PointCloud.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <nav_core/base_global_planner.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/cost_values.h>

#include <rotate_recovery/rotate_recovery.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <carrot_planner/carrot_planner.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>

#include <navfn/navfn_ros.h>
#include <angles/angles.h>

using namespace std;


struct Adj{
public:
	int V;
	float X,Y;

	bool Visited;
	Adj(int v, float x, float y) : V(v), X(x), Y(y){}
};

/** Function Definition */
	void AMCLDATA(const geometry_msgs::PoseWithCovarianceStamped& msg);

	void serviceActivated();
	void serviceDone(const actionlib::SimpleClientGoalState&, const move_base_msgs::MoveBaseResultConstPtr&);
	void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr&);


/** Global Variables */
sensor_msgs::LaserScan scanner;
geometry_msgs::Twist position;
move_base_msgs::MoveBaseGoal goal;


/** Flags */
	bool AMCLSet = false;
	bool IMUSet = false;
	bool NDone = true;
	bool First = true, Changed = false, Set = false, Switch = true;

	bool Found_Current_Pose = false;

/** Estimation Values */
	int Center = 360, Right = 420, Left = 300, FarRight = 700, FarLeft = 50;

	float AMCLz, AMCLw;
	float IMUz, IMUw;
	float Degree = 0.174533;
	float Interval = 0.75;

	int **m;

	string GlobalMapName;

/** Main Function **/
int main(int argc,char **argv) {
    ros::init(argc,argv,"saferandomwalk");
    ros::NodeHandle nh;

	ros::ServiceClient client =nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
	client.waitForExistence();

	ros::Subscriber subAMCL = nh.subscribe("/amcl_pose", 1000, &AMCLDATA);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);


	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>ac("move_base",true);
	ROS_INFO_STREAM("Waiting for server to be available...");
		while (!ac.waitForServer()) {}
	ROS_INFO_STREAM("done!");


	ros::Rate rate(1);
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(13.0);

/// Locate where you are using amcl
/**
	while(ros::Time::now() - start_time < timeout){
		position.linear.x = 0;
		position.angular.z = M_PI/(3);
		pub.publish(position);
		ROS_INFO("SPINNING");
	}
**/

	tf::TransformListener tf(ros::Duration(10));
	costmap_2d::Costmap2DROS costmap("my_costmap", tf);
	costmap_2d::Costmap2DROS global_costmap("global_costmap", tf);
	costmap_2d::Costmap2DROS local_costmap("local_costmap", tf);

	base_local_planner::TrajectoryPlannerROS tp;
	carrot_planner::CarrotPlanner cp;
	rotate_recovery::RotateRecovery rr;
	clear_costmap_recovery::ClearCostmapRecovery ccr;

	navfn::NavfnROS navfn;
	navfn.initialize("my_navfn_planner", &costmap);

	costmap.updateMap();
	costmap.start();

	tp.initialize("my_trajectory_planner", &tf, &costmap);
	cp.initialize("my_carrot_planner", &costmap);
	rr.initialize("my_rotate_recovery", &tf, &global_costmap, &local_costmap);
	ccr.initialize("my_clear_costmap_recovery", &tf, &global_costmap, &local_costmap);

	rr.runBehavior();
	ccr.runBehavior();


	while(ros::ok()){
		ac.sendGoal(goal,&serviceDone,&serviceActivated,&serviceFeedback);
/*
		if(Set){	
				ROS_INFO("Center: [%f]", scanner.ranges[Center]);
				ROS_INFO("Left: [%f]", scanner.ranges[Left]);
				ROS_INFO("Right: [%f]", scanner.ranges[Right]);
				
			if(scanner.ranges[Center] < 2 && scanner.ranges[Left] < 2 && scanner.ranges[Right] < 2){
				position.linear.x = -1;
				position.angular.z = 4*M_PI;
				Changed = true;
			}
			else{ 
				if(scanner.ranges[Center] < 2){
					position.linear.x = 0;
					if(scanner.ranges[Left] > 2 && scanner.ranges[Right] > 2){position.angular.z=M_PI/(2);}
					if(scanner.ranges[FarRight] < 2)
						position.angular.z += M_PI/8;
					if(scanner.ranges[FarLeft] < 2)
						position.angular.z -= M_PI/8;
					Changed = true;
				}	
				if(scanner.ranges[Left] < 2 && scanner.ranges[Right] < 2){
					position.linear.x = -0.2;
					Changed = true;
				}
				else{
					if(scanner.ranges[Left] < 2){
						position.angular.z = position.angular.z - M_PI/(4);
						Changed = true;
					}
					if(scanner.ranges[Right] < 2){
						position.angular.z = position.angular.z + M_PI/(4);
						Changed = true;
					}
				}
			}

			pub.publish(position);

			if(Changed){
				ros::Duration(Interval).sleep();
				Changed = false;			
			}
			else{
				Changed = true;
				//GeneratePose(position);
				pub.publish(position);
				//ROS_INFO("Generating new Direction");
			}
			//ROS_INFO_STREAM(position);
			ros::spinOnce();
		}
		else{
			pub.publish(position);
			ros::spinOnce();
		}
	*/ 
		ros::spinOnce();
	}
    return 0;    
}

void Scanning(const sensor_msgs::LaserScan msg){
	scanner = msg;
	Set = true;
}

void GeneratePose(geometry_msgs::Twist &Pose){
	float Valuex, Valuez;

	Valuex = rand() % 10+1;
	Valuez = rand() % 20+1;
	Valuex = Valuex/10;

	Pose.linear.x = Valuex;
	Pose.angular.z = M_PI/(Valuez*3);
}
void IMUDATA(const sensor_msgs::Imu::ConstPtr& msg){
	IMUz = msg->orientation.z;
	IMUw = msg->orientation.w;

	//ROS_INFO("IMU: [%f]", IMUz);

	IMUSet = true;
}

void AMCLDATA(const geometry_msgs::PoseWithCovarianceStamped& msg){
	AMCLz = msg.pose.pose.orientation.z;
	AMCLw = msg.pose.pose.orientation.w;

	//ROS_INFO("AMCL: [%f]", AMCLz);
	ROS_INFO("AMCL: [%f] [%f]", msg.pose.pose.position.x, msg.pose.pose.position.y);
	
	AMCLSet = true;
}

void serviceActivated() {
	ROS_INFO_STREAM("Service received goal");
}

void serviceDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
	ROS_INFO_STREAM("Service completed");
	ROS_INFO_STREAM("Final state " << state.toString().c_str());
	//ros::shutdown();
}

void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
	ROS_INFO_STREAM("Service still running");
	ROS_INFO_STREAM("Current pose (x,y) " << fb->base_position.pose.position.x << "," << fb->base_position.pose.position.y);
}

void localMapMessageReceived(const nav_msgs::OccupancyGrid&msg) {
	ROS_INFO_STREAM("Received Local Map");
	ROS_INFO_STREAM("Dimensions: " << msg.info.width << " x " << msg.info.height);
	ROS_INFO_STREAM("Resolution " << msg.info.resolution);
	std::ofstream localmap("src/Final/maps/FinalLocalmap.txt"); /// .txt
	int z= 0;
	for (int i = 0 ; i < msg.info.height ; i++) {
		for ( int j = 0 ; j < msg.info.width ; j++ )  
			localmap << (int)msg.data[z++] << " " ;
		localmap << std::endl;
	}
	localmap.close();
}

void gloabalMapMessageReceived(const nav_msgs::OccupancyGrid&msg) {
	ROS_INFO_STREAM("Received Global Map");
	ROS_INFO_STREAM("Dimensions: " << msg.info.width << " x " << msg.info.height);
	ROS_INFO_STREAM("Resolution " << msg.info.resolution);
	ROS_INFO_STREAM("Saving costmap to a file");
	std::ofstream globalmap("src/Final/maps/FinalGlobalmap.txt"); /// .txt
	std::ofstream globalmapEdit("src/Final/maps/SGM.txt");
	int z= 0;
	int Value;	

	for (int i = 0 ; i < msg.info.height ; i++) {
		for ( int j = 0 ; j < msg.info.width; j++ ){ 
			globalmap << (int)msg.data[z++] << " " ;
		}
		globalmap << std::endl;
	}
	globalmap.close();
	GlobalMapName = msg.header.frame_id;
}

