#include "utility.h"

bool cfftg_done, first_step;
bool fs_arrived;
int next_swing_leg;
bool change;



void cfftg_cb(const std_msgs::Bool::ConstPtr& msg){
		if(msg->data==true) change=true; 
}
/*
void swing_cb(const std_msgs::Bool::ConstPtr& msg){
	next_swing_leg = msg->data;
}
*/

void fs_cb(const std_msgs::Bool::ConstPtr& msg){
	first_step = msg->data;
	fs_arrived=true;
}

int main (int argc, char** argv) {
 ros::init (argc, argv, "slm");
 ros::NodeHandle nh;
 //Subscribers
 ros::Subscriber sub = nh.subscribe ("/cfftg_done", 1, cfftg_cb);
 ros::Subscriber fs_sub = nh.subscribe ("/first_step", 1, fs_cb);
 //ros::Subscriber sub2 = nh.subscribe ("/next_swing_leg", 1000, swing_cb);
 ros::Publisher pub = nh.advertise<std_msgs::Int8> ("swing_leg", 1);
 ros::Publisher pub_fs = nh.advertise<std_msgs::Bool> ("first_step_global", 1);
 
 
 int swing_leg=-1; //initialization
 change=false;
 fs_arrived=false;
 cfftg_done=false;
 next_swing_leg=1;
 std_msgs::Int8 msg;
 std_msgs::Bool bmsg;
 bmsg.data=true;
 std::string sl;
 
 while(ros::ok()){
 	msg.data=swing_leg;
 	pub.publish(msg);
 	pub_fs.publish(bmsg);
 	if(swing_leg==-1 && fs_arrived){
 		std::cout<<"Checking next swing leg..."<<std::endl;
 		sl = first_step==false?"left":"right";
 		std::cout<<"Swing leg chosen by RV node is "<< sl <<std::endl;
 		swing_leg = first_step;
 		//swing_leg= next_swing_leg;
 	}
 	if(change) {
 		bmsg.data=false;
 		std::cout<<"Changing leg"<<std::endl;
 		if(swing_leg==-1) swing_leg=next_swing_leg;
 		else if(swing_leg==0) swing_leg=1;
 		else swing_leg=0;
 		//cfftg_done=false;
 		change=false;
 	}
 	ros::spinOnce();
 }
}
