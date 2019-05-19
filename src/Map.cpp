#include <ros/ros.h>
#include <stdlib.h> 
#include <string>
#include <cmath>
#include <valarray>
#include <fstream>
#include <iostream>
#include <vector>
#include <limits>

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
#include <angles/angles.h>

using namespace std;

struct Node{
	int X,Y,V;
	float absX, absY;
	bool Visited, Blocked, Buffer, BVisited;
	vector<Node*>adj;

	const int d = 1; //To be used for the heuristic
	vector<Node*> roadmap; //To be used for the pathing algorithm

	Node(int x, int y, bool B): X(x), Y(y), Blocked(B) {}
};

// Function Definition
	void gloabalMapMessageReceived(const nav_msgs::OccupancyGrid&);
	void AMCLDATA(const geometry_msgs::PoseWithCovarianceStamped& msg);
	//void BreathFirstSearch(int, int);
	//void DepthFirstSearch(int, int);
	void SpanBlocked(Node*, int);
	//void rotateMatrix(int mat[100][100]);
	float JundgeHueristic(Node*,Node *);
	float GetAngleOfLineBetweenTwoPoints(Node*, Node*);
	bool RayTrace(Node*, Node*);

	void serviceActivated();
	void serviceDone(const actionlib::SimpleClientGoalState&, const move_base_msgs::MoveBaseResultConstPtr&);
	void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr&);

// Global Variables
	bool AMCLSet = false, GMMRSet = false, SerDone = true;
	float OriginX, OriginY, Res;
	float AMCLx, AMCLy, AMCLz, AMCLw;
	string GlobalMapName;
	int buffer = 0;

	int ArrayList[400][400];
	int bSlist[100][100];
	vector< vector<Node*> >NodeList;
	deque<Node*>GoalList;
	vector< vector<Node*> >majorNodeList; //For the major nodes we will be primarily visiting

	geometry_msgs::Twist position;
	move_base_msgs::MoveBaseGoal goal;

// Main Function
int main(int argc,char **argv) {
    ros::init(argc,argv,"Map");
    ros::NodeHandle nh;

	ros::Subscriber subAMCL = nh.subscribe("/amcl_pose", 1000, &AMCLDATA);
	ros::Subscriber subglobal=nh.subscribe("/move_base/global_costmap/costmap",100, &gloabalMapMessageReceived);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);


	ros::ServiceClient client =nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
	client.waitForExistence();

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>ac("move_base",true);
	ROS_INFO_STREAM("Waiting for server to be available...");
		while (!ac.waitForServer()) {}
	ROS_INFO_STREAM("done!");


	ros::Rate rate(1);
	ros::Time start_time = ros::Time::now();

		ROS_INFO("Starting");
	// Wait for AMCL and Global Map
		while(!(AMCLSet && GMMRSet)){ ros::spinOnce(); }


		ROS_INFO("Compressing Map");
	// Compression
		int i = 0,j = 0,Val,ii=0,jj=0;

		for(j = 0;j<=396;j+=4){
			jj = j/4;
			for(i = 0;i<=396;i+=4){
				ii = i/4;

				Val = ArrayList[i][j] + ArrayList[i][j+1] + ArrayList[i][j+2] + ArrayList[i][j+3] + 
					ArrayList[i+1][j] + ArrayList[i+1][j+1] + ArrayList[i+1][j+2] + ArrayList[i+1][j+3] +
					ArrayList[i+2][j] + ArrayList[i+2][j+1] + ArrayList[i+2][j+2] + ArrayList[i+2][j+3] +
					ArrayList[i+3][j] + ArrayList[i+3][j+1] + ArrayList[i+3][j+2] + ArrayList[i+3][j+3];
				//cout << Val << "\t\t";
				if(Val < 8)
					bSlist[ii][jj] = false;
				else{
					bSlist[ii][jj] = true;
				}
				//cout << "x: " << ii << " y: " << jj << " - " << bSlist[ii][jj] << "\t\t";
			}
		}


		NodeList.resize(100, vector<Node*>(100));

		ROS_INFO("Setting NULL");
		for(j = 0; j < 100; j++){
			for(i = 0; i < 100; i++){
				NodeList[i][j] = new Node(i,j,bSlist[i][j]);
			}
		}

		ROS_INFO("Set and Gnerating Adj List");
	// Generate Adj list
		for(j = 1; j < 99;j++){
			for(i = 1; i < 99; i++){
				NodeList[i][j]->adj.push_back(NodeList[i][j+1]);	// Check that its NOT the Bottom Node
				NodeList[i][j]->adj.push_back(NodeList[i+1][j]);	// Check that its NOT the Right Node
				NodeList[i][j]->adj.push_back(NodeList[i][j-1]);	// Check that its NOT the Top Node
				NodeList[i][j]->adj.push_back(NodeList[i-1][j]);	// Check that its NOT the Left Node
			}
		}
		for(j = 0; j < 99; j++){
			if(j!=99)
				NodeList[0][j]->adj.push_back(NodeList[0][j+1]);	// Check that its NOT the Bottom Node
			NodeList[0][j]->adj.push_back(NodeList[1][j]);			// Check that its NOT the Right Node
			if(j!=0)
				NodeList[0][j]->adj.push_back(NodeList[0][j-1]);	// Check that its NOT the Top Node
		}
		for(j = 0; j < 99; j++){
			if(j!=99)
				NodeList[99][j]->adj.push_back(NodeList[99][j+1]);	// Check that its NOT the Bottom Node
			NodeList[99][j]->adj.push_back(NodeList[98][j]);		// Check that its NOT the Left Node
			if(j!=0)
				NodeList[99][j]->adj.push_back(NodeList[99][j-1]);	// Check that its NOT the Top Node
		}
		for(i = 0; i < 99; i++){
			NodeList[i][0]->adj.push_back(NodeList[i][1]);			// Check that its NOT the Bottom Node
			if(i!=0)
				NodeList[i][0]->adj.push_back(NodeList[i-1][0]);	// Check that its NOT the Left Node
			if(i!=99)
				NodeList[i][0]->adj.push_back(NodeList[i+1][0]);	// Check that its NOT the Right Node
		}
		for(i = 0; i < 99; i++){
			if(i!=0)
				NodeList[i][99]->adj.push_back(NodeList[i-1][99]);	// Check that its NOT the Left Node
			if(i!=99)
				NodeList[i][99]->adj.push_back(NodeList[i+1][99]);	// Check that its NOT the Right Node
			NodeList[i][99]->adj.push_back(NodeList[i][98]);		// Check that its NOT the Top Node
		}

			

		ROS_INFO("Buffering Map");
	// Buffering the Map
		deque<Node*> queue;

		for(int j = 0; j < 100; j++){
			for(int i = 0; i < 100; i++){
				NodeList[i][j]->Visited = false;
				NodeList[i][j]->BVisited = false;
			}
		} 

		Node *Parent = NodeList[50][50];
		queue.push_back(Parent);


		while(!queue.empty()){
			Parent = queue.front();
			queue.pop_front();
			for(vector<Node*>::iterator i = Parent->adj.begin(); i != Parent->adj.end(); ++i){
				if(!((*i)->Visited)){
					(*i)->Visited = true;
					queue.push_back((*i));
				}
				if((*i)->Blocked){
					SpanBlocked((*i), 0);
					for(int j = 0; j < 100; j++)
						for(int i = 0; i < 100; i++)
							NodeList[i][j]->BVisited = false;
				}
			}
		}
		delete Parent;		


		for(int j = 0; j < 100; j++){
			for(int i = 0; i < 100; i++){
				cout << (NodeList[i][j]->Buffer || NodeList[i][j]->Blocked) << " ";
			}
			cout << endl;
		}

		ROS_INFO("Convert X Y to abs values");
		float xStart = -10.0;
		float yStart = 10.0;
		float xinc = 0.2;
		float yinc = 0.2;

	// Convert X Y to abs values
		for(int j = 0; j < 100; j++){
			for(int i = 0; i < 100; i++){
				NodeList[i][j]->absX = xStart + (float)i*xinc;
				NodeList[i][j]->absY = yStart - (float)j*yinc;
				//cout << NodeList[i][j]->absX << " " << NodeList[i][j]->absY << endl;
			}
		}


		ROS_INFO("Generating Key Nodes");
	// Generating Key Nodes
		stack<Node*> Stack;
		for(int j = 0; j < 100; j++){
			for(int i = 0; i < 100; i++){
				NodeList[i][j]->Visited = false;
			}
		} 

		Parent = NodeList[50][50];
		Node* Criteria = Parent;
		Stack.push(Parent);

		float Best = 0.0, current = 0.0;
		int Counter = 0;
		
		NodeList[0][0]->X = 0;
		NodeList[0][0]->Y = 0; 
		NodeList[0][1]->Visited = true; 
		NodeList[1][0]->Visited = true;
		NodeList[99][99]->Visited = true; 


		while(!Stack.empty()){
			Parent = Stack.top();
			Stack.pop();
			current = JundgeHueristic(Criteria, Parent);
			if(current > Best){ Best = current;}
			else{ Best = 0; GoalList.push_back(Parent); Criteria = Parent; }


//cout << "Here " << !(Stack.empty()) << " " << Stack.size()<< endl;
//cout << Parent->X << " " << Parent->Y;
			for(vector<Node*>::iterator i = Parent->adj.begin(); i != Parent->adj.end(); ++i){
//cout << "\t" << (*i)->X << " " << (*i)->Y <<endl;
				if(!((*i)->Visited)){
					(*i)->Visited = true;
					Stack.push((*i));
				}
			}
		}
		delete Parent;
		delete Criteria;																											
		
cout << endl << endl;

		//for(deque<Node*>::iterator i = GoalList.begin(); i != GoalList.end(); ++i)
			//cout << (*i)->absX << " " << (*i)->absY << endl;
/**
		bool *visited = new bool[80]; //If there's approximately 400 places, 400/5 = 80 major points of reference.
			for(int k = 0; k < 880; k++) {
				visited[k] = false;
			}
		vector<Node*> queue; //For BFS

		

		//Add roadmap adjacency
		for (int k = 3; ik <= 396; k += 5) {
			for (int j = 3; j <= 396; j += 5) {
				visited[
				//up
				for (int y = 1; y <= 5; y++) {
					if (val[k][j - y] > 4) break;
					if (y == 5) NodeList[i][j]->roadmap.push_back(NodeList[k][j - y]);
				}

				//down
				for (int y = 1; y <= 5; y++) {
					if (val[k][j + y] > 4) break;
					if (y == 5) NodeList[i][j]->roadmap.push_back(NodeList[k][j + y]);
				}

				//left
				for (int x = 1; x <= 5; x++) {
					if (val[k - x][j] > 4) break;
					if (x == 5) NodeList[i][j]->roadmap.push_back(NodeList[k - x][j]);
				}

				//right
				for (int x = 1; x <= 5; x++) {
					if (val[k + x][j] > 4) break;
					if (x == 5) NodeList[i][j]->roadmap.push_back(NodeList[k + x][j]);
				}

			}
		}
**/

		// RIIIIGHTTT HEREEEEEE
		//masterNodes is the master list for all the nodes you should go to.
		//Closest one to (0,0) would be masterNodes[masterNodes.size()/2]
		//Reach that one first
		//Then the adjacent nodes to that node can be found in masterNodes[masterNodes.size()/2]->roadmap
		//Obviously replace the insides of the brackets with some sort of variable, masterNodes.size()/2 only accounts for the first one
		//Queue all the nodes from the roadmaps within the roadmaps and then start visiting them all separately to reach every node


		ROS_INFO("Ready");
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	Node* temp;
	//temp->absX = 0.0;
	//temp->absY = 0.0;
	Node* Old;

	while(ros::ok()){
		if(SerDone){
			//Old = temp;
			temp = GoalList.front();
			GoalList.pop_front();

			goal.target_pose.pose.position.x = temp->absX;
			goal.target_pose.pose.position.y = temp->absY;
			goal.target_pose.pose.orientation.w = 1.0;
			SerDone = false;
			ac.sendGoal(goal,&serviceDone,&serviceActivated,&serviceFeedback);
		}
		ros::spinOnce();
	}


	while(ros::ok()){ ros::spinOnce(); }
}


void serviceActivated() {
	ROS_INFO_STREAM("Service received goal");		
}

void serviceDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
	ROS_INFO_STREAM("Service completed");
	ROS_INFO_STREAM("Final state " << state.toString().c_str());
	SerDone = true;
}

void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
	ROS_INFO_STREAM("Service still running");
	ROS_INFO_STREAM("Current pose (x,y) " << fb->base_position.pose.position.x << "," << fb->base_position.pose.position.y);
}

float GetAngleOfLineBetweenTwoPoints(Node* p1, Node* p2){
	float xDiff = p2->absX - p1->absX;
	float yDiff = p2->absY - p1->absY;
	return atan2(yDiff, xDiff) * (180 /M_PI);
}






/// AMCL positioning
void AMCLDATA(const geometry_msgs::PoseWithCovarianceStamped& msg){
	AMCLw = msg.pose.pose.orientation.w;
	OriginX = AMCLx = msg.pose.pose.position.x;
	OriginY = AMCLy = msg.pose.pose.position.y;
	AMCLz = msg.pose.pose.position.z;

	//ROS_INFO("AMCL: [%f]", AMCLz);
	ROS_INFO("AMCL: [%f] [%f]", msg.pose.pose.position.x, msg.pose.pose.position.y);
	
	AMCLSet = true;
}

void gloabalMapMessageReceived(const nav_msgs::OccupancyGrid &msg) {
	ROS_INFO_STREAM("Received Global Map");
	ROS_INFO_STREAM("Dimensions: " << msg.info.width << " x " << msg.info.height);
	ROS_INFO_STREAM("Resolution " << msg.info.resolution);

	int z= 800*200 - 1;
	z+=200;
	int i,j;
	int Value;	

// Generate List
	if(!GMMRSet){
		ROS_INFO_STREAM("Starting");
		for(j = 0 ; j < 400; j++) {
			for(i = 0; i < 400; i++ ){ 
				Value = (int)msg.data[z];
				if(Value>1){ Value = 1; }
				else{ Value = 0; }
				ArrayList[i][j] = Value;
				z++;
			}
			z+= 400;
		}
	}
	GlobalMapName = msg.header.frame_id;
	Res = msg.info.resolution;
	GMMRSet = true;
}

void SpanBlocked(Node* a, int i){
	if(!(a->BVisited)){ a->Buffer = true; a->BVisited = true;  }
	if(i<buffer){
		i++;
		//cout << a->X << " " << a->Y << " has children " << endl;
		//for(vector<Node*>::iterator it = a->adj.begin(); it != a->adj.end(); ++it)
			//cout << "\t" << (*it)->X << " " << (*it)->Y << endl;
		for(vector<Node*>::iterator it = a->adj.begin(); it != a->adj.end(); ++it){
			SpanBlocked((*it), i);
		}
	}
}


// Ray Tracing USING Bresenham's line algorithm
float JundgeHueristic(Node*C, Node*P){ 
	//Is there a straight, uninterupted line, from Criteria to Parent
	//If yes, return the distance from Criteria to Parent
	//If no, return 0
	float distance = sqrt( pow(C->absX - P->absX, 2) + pow(C->absY - P->absY, 2));
//cout << C->absX << " " << C->absY << " | " << P->absX << " " << P->absY << " " << distance << endl;
	if(RayTrace(C,P))
		return distance;
	else
		return 0.0;
}

bool RayTrace(Node*C, Node*P){
	float Dis = 0.0, BDis = FLT_MAX;
	Node *Best = P;
	Node *tmp;

	while(Best != C){
		for(vector<Node*>::iterator it = Best->adj.begin(); it != Best->adj.end(); ++it){
			Dis = sqrt( pow((*it)->absX - C->absX, 2) + pow((*it)->absY - C->absY, 2) );
			if(Dis < BDis){
				BDis = Dis;
				tmp = (*it);
			}
		}
		if(tmp->Blocked || tmp->Buffer){
			return false;
		}
		else{
			Best = tmp;
		}
	}
	return true;
}




