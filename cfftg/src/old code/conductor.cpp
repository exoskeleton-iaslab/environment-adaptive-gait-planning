#include "utility.h"


using namespace std;

bool new_leg=false;
bool next_swing_leg;
bool ct_done=false;
bool cfftg_done=false;




void sl_cb(const std_msgs::Bool::ConstPtr& msg){
	next_swing_leg = msg->data;
	new_leg=true;
}

void foothold_cb(const std_msgs::Float32::ConstPtr& msg){
	ct_done = true;
}



void cfftg_cb(const std_msgs::Bool::ConstPtr& msg){
	cfftg_done = msg->data;
}






int main (int argc, char** argv) {
 ros::init (argc, argv, "conductor");
 ros::NodeHandle nh;
 
 int swing_leg=-1;
 bool first_step = true;
 bool cfftg_start=false;
 //Subscribers
 ros::Publisher swing_pub = nh.advertise<std_msgs::Int8> ("swing_leg", 1);
 ros::Publisher cfftg_pub = nh.advertise<std_msgs::Bool> ("cfftg_start", 1);
 ros::Subscriber sub_swing = nh.subscribe ("next_swing_leg", 1, sl_cb);
 ros::Subscriber sub_foothold = nh.subscribe ("foothold", 1, foothold_cb);
 
 ros::Subscriber sub_cfftg = nh.subscribe ("cfftg_done", 1, cfftg_cb);
 
 std_msgs::Int8 imsg;
 std_msgs::Bool bmsg;
 
 
 while(ros::ok()) {
 	if(first_step) { //arbitrary first step
 		imsg.data = -1;
 		swing_pub.publish(imsg);
 		if(new_leg) { //receive next swing leg by robotic vision node (only first time)
 			swing_leg = next_swing_leg;
 			imsg.data=swing_leg;
 			swing_pub.publish(imsg);
 			first_step=false;
 		}
 	}
 	else{
 		if(ct_done){
 			cout<<"CFFTG ready to start. Press enter to continue";
 			std::string s;
 			cin>>s;
 			bmsg.data=true;
 			cfftg_pub.publish(bmsg);
 			cfftg_start=true;
 		}
 		if(cfftg_start && cfftg_done){
 			bmsg.data=false;
 			cfftg_pub.publish(bmsg);
 			cfftg_start=false;
 			if(swing_leg==0) swing_leg=1;
 			else if(swing_leg==1) swing_leg=0;
 			ct_done=false;
 			cfftg_done=false;
 		}
 	}

 	ros::spinOnce();
 	}
 
 }
