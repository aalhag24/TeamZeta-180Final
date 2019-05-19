#include <ros/ros.h>
#include <vector>
#include <logical_camera_plugin/logicalImage.h>
#include <Final/Num.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

struct treasureImg {
	//Stores info on treasures
	std::string name;
	float posX;
	float posY;
	float posZ;
	float rotX;
	float rotY;
	float rotZ;
	float rotW;
};

std::vector<treasureImg> storeImg;
bool recorded = false;

float AMCLow = 0.0, AMCLox = 0.0, AMCLoy = 0.0, AMCLoz = 0.0, AMCLx = 0.0, AMCLy = 0.0, AMCLz = 0.0;

void AMCLDATA(const geometry_msgs::PoseWithCovarianceStamped& msg){
	AMCLow = msg.pose.pose.orientation.w;
	AMCLox = msg.pose.pose.orientation.x;
	AMCLoy = msg.pose.pose.orientation.y;
	AMCLoz = msg.pose.pose.orientation.z;
	AMCLx = msg.pose.pose.position.x;
	AMCLy = msg.pose.pose.position.y;
	AMCLz = msg.pose.pose.position.z;
}

void getLC(const logical_camera_plugin::logicalImage&img) {
	recorded = false;
	treasureImg newTreasure;

	//Checks if the treasure is already stored in the vector
	for (int i = 0; i < storeImg.size(); i++) {
		if (storeImg.at(i).name == img.modelName) {
			recorded = true;
			break;
		}
	}

	if (!recorded) {
		//New treasure so storing in new struct and saving to vector
		newTreasure.name = img.modelName;
		newTreasure.posX = img.pose_pos_x + AMCLx;
		newTreasure.posY = img.pose_pos_y + AMCLy;
		newTreasure.posZ = img.pose_pos_z + AMCLz;
		newTreasure.rotX = img.pose_rot_x + AMCLox;
		newTreasure.rotY = img.pose_rot_y + AMCLoy;
		newTreasure.rotZ = img.pose_rot_z + AMCLoz;
		newTreasure.rotW = img.pose_rot_w + AMCLow;
		
		storeImg.push_back(newTreasure);

		ROS_INFO("ALl other Found Treasures");
		//Prints all the treasures currently stored in the vector
		for (int i = 0; i < storeImg.size(); i++){
			ROS_INFO_STREAM("AT: " << i);
			ROS_INFO_STREAM("Name : " << storeImg.at(i).name);
			ROS_INFO_STREAM("Pos x : " << storeImg.at(i).posX);
			ROS_INFO_STREAM("Pos y : " << storeImg.at(i).posY);
			ROS_INFO_STREAM("Pos z : " << storeImg.at(i).posZ);
			ROS_INFO_STREAM("Ori x : " << storeImg.at(i).rotX);
			ROS_INFO_STREAM("Ori y : " << storeImg.at(i).rotY);
			ROS_INFO_STREAM("Ori z : " << storeImg.at(i).rotZ);
			ROS_INFO_STREAM("Ori w : " << storeImg.at(i).rotW);
		}
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "logicalCamera");
	ros::NodeHandle nh;
	std::cout << "Working" << std::endl;

	//Subscriber for logical camera
	ros::Subscriber logCamSub = nh.subscribe("/objectsDetected",1000,&getLC);
	ros::Subscriber subAMCL = nh.subscribe("/amcl_pose", 1000, &AMCLDATA);

	ros::spin();
}
