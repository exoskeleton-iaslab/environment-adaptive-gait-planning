#include "utility.h"

bool new_hip=false;
bool new_foot=false;
bool new_pivot=false;
bool new_obs=false;


void hip_height_cb(const std_msgs::Float32::ConstPtr& msg){
	std::cout<<"Hip received"<<std::endl;
	new_hip=true;
	
}

void pivot_cb(const std_msgs::Float32::ConstPtr& msg){
	std::cout<<"Pivot received"<<std::endl;
	new_pivot=true;
	
}
void foothold_cb(const std_msgs::Float32::ConstPtr& msg){
	std::cout<<"Foot received"<<std::endl;
	new_foot=true;
	
}

void obstacle_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	std::cout<<"Obs received"<<std::endl;
	new_obs=true;
	
}

int main (int argc, char** argv) {
 ros::init (argc, argv, "cfftg_starter");
 ros::NodeHandle nh;
 ros::Subscriber sub_hip_height = nh.subscribe ("/CoM_height", 1, hip_height_cb);
 ros::Subscriber sub_pivot = nh.subscribe ("/pivot", 1, pivot_cb);
 ros::Subscriber sub_foothold = nh.subscribe ("/foothold", 1, foothold_cb);
 ros::Subscriber sub_obstacle = nh.subscribe ("/obstacle_shape", 1, obstacle_cb);
 ros::Publisher cfftg_pub = nh.advertise<std_msgs::Bool> ("cfftg_start", 1);
 std_msgs::Bool bmsg;
 bmsg.data=false;
 std::string s;
 bool new_data=false;
 
 while(ros::ok()){
 	new_data = new_hip && new_pivot && new_foot && new_obs;
 	if(new_data) {
	 	std::cout<<"CFFTG ready to start. Press any key and Enter to continue"<< std::endl;
	 	std::cin>>s;
		bmsg.data=true;
	 	cfftg_pub.publish(bmsg);
	 	ros::Duration(0.05).sleep();
	 	bmsg.data=false; 	
	 	new_hip=false;
 		new_pivot=false;
 		new_foot=false;
 		new_obs=false;
	 	
 	}
 	cfftg_pub.publish(bmsg);
 	ros::spinOnce();
 }
}
