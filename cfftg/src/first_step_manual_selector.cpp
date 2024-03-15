#include "utility.h"



int main (int argc, char** argv) {
 ros::init (argc, argv, "first_step_manual_selector");
 ros::NodeHandle nh;

 ros::Publisher fs_pub = nh.advertise<std_msgs::Bool> ("first_step", 1);
 std_msgs::Bool bmsg;
 bmsg.data=false;
 std::string s;
 std::cout<<"Select first step: type \"0\" for left leg or \"1\" for right leg, then press Enter."<< std::endl;
 std::cin>>s;
 while(ros::ok()) {
 	if(s=="0") bmsg.data=false;
 	else bmsg.data=true;
 	fs_pub.publish(bmsg);
 	ros::spinOnce();
 }
}
