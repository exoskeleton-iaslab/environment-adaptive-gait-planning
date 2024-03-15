#include "utility.h"


using namespace std;

vector<float> measurements; // measurements[i]: {left_hip, left_knee, right_hip, right_knee} 
float foothold;
float pivot_tip;
float com_z;
float ref_tilt;
float tilt;
bool next_leg;
vector<float> obstacle_shape;
bool new_ang=false;
bool new_foot=false;
bool new_piv=false;
bool new_com=false;
bool new_obs= false;
bool new_leg = false;
bool new_next = false;

bool new_tilt=false;
int leg;
//vector<float> rh = ; //right hip measurements
//vector<float> lh; //left hip measurements
//vector<float> rk; //right knee measurements
//vector<float> lk; //left knee measurements



void angles_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	//vector come as {LH,LK,RH,RK}
	measurements = msg->data;	
	//vector<float> temp;
	//for(int i=0; i<4;i++) measurements[i].push_back(msg->data[i]);
	//measurements.push_back(temp);
	new_ang=true;
}


void foothold_cb(const std_msgs::Float32::ConstPtr& msg){
	foothold = msg->data;
	new_foot=true;
}
/* COMMENTATO PERCHE' NON STO USANDO IL CLUSTERING PER OTTENERE LA POSIZIONE DELLA PUNTA DEL PIEDE PIVOT
void pivot_cb(const std_msgs::Float32::ConstPtr& msg){
	pivot_tip = msg->data;
	new_piv=true;
}
*/
void com_cb(const std_msgs::Float32::ConstPtr& msg){
	com_z = msg->data;
	new_com=true;
}


void obstacle_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	obstacle_shape = msg->data;	
	/*vector<float> temp;
	for(int i=0; i<4;i++) measurements[i].push_back(msg->data[i]);
	measurements.push_back(temp);*/
	new_obs=true;
}

void leg_cb(const std_msgs::Int8::ConstPtr& msg){
	leg = msg->data;
	new_leg=true;
}


void next_leg_cb(const std_msgs::Bool::ConstPtr& msg){
	next_leg = msg->data;
	new_next=true;
}



void ref_cb(const std_msgs::Float32::ConstPtr& msg){
	ref_tilt = msg->data;
}

void tilt_cb(const std_msgs::Float32::ConstPtr& msg){
	tilt = msg->data;
	new_tilt=true;
}




int main (int argc, char** argv) {
 ros::init (argc, argv, "coord_translation");
 ros::NodeHandle nh;
 //Subscribers
 ros::Subscriber sub_encoders = nh.subscribe ("/angle", 1000, angles_cb);
 ros::Subscriber sub_foothold = nh.subscribe("/foothold_raw", 1000, foothold_cb);
 //ros::Subscriber sub3 = nh.subscribe("/pivot_tip_raw", 1000, pivot_cb);
 ros::Subscriber sub_com = nh.subscribe("/CoM_height_raw", 1000, com_cb);
 ros::Subscriber sub_obstacles = nh.subscribe("/obstacle_shape_raw", 1000, obstacle_cb);
 ros::Subscriber sub_swing_leg = nh.subscribe("/swing_leg", 1000, leg_cb);
 ros::Subscriber sub_next_swing_leg = nh.subscribe("/next_swing_leg", 1000, next_leg_cb);
 ros::Subscriber sub_ref_tilt = nh.subscribe("/reference_tilt", 1000, ref_cb);
 ros::Subscriber sub_tilt = nh.subscribe("/tilt_angle", 1000, tilt_cb);
 //Publishers
 ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray> ("obstacle_shape", 1);
 ros::Publisher pub2 =nh.advertise<std_msgs::Float32> ("foothold", 1);
 ros::Publisher pub3 = nh.advertise<std_msgs::Float32> ("CoM_height", 1);
 ros::Publisher pub4 = nh.advertise<std_msgs::Float32> ("pivot", 1);
 ros::Publisher pub5 = nh.advertise<std_msgs::Float32> ("pivot_camera_coord", 1);
 
 
 float camera_com_offs_y= 0.24; //horizontal offset between camera and com (assumption: com = hip joint)
 float camera_com_offs_z= 0.1; //vertical offset between camera and com (updated from 0.12 to 0.1 15/11/2023)
 float camera_com_offs = sqrt(pow(camera_com_offs_z,2) +pow(camera_com_offs_y,2)); //offset between com and camera
 float camera_com_angle = asin(camera_com_offs_z/camera_com_offs); //angle between camera and com (  assumption:  camera_com_offs_z =  camera_com_offs* sin(camera_com_angle)  )
 float com_y=0;
 float L1; //should be provided as ros param
 float L2; //should be provided as ros param
 ros::param::get("~thigh_length", L1);
 ros::param::get("~shin_length", L2);
 //float dsh = 0.085; //should be provided as ros param
 //float d_y = 0.25;
 //float d_z = 0.015;
 //float d = sqrt(pow(d_y,2)+pow(d_z,2));
 //float fixed_angle= asin(d_y / d);
 bool new_data=false;
 float knee_y, knee_y_p;
 float knee_z, knee_z_p;
 float heel_y, heel_y_p;
 float heel_z, heel_z_p;
 float trans_com;
 float trans_pivot;
 vector<float> trans_obs;
 float trans_foothold;
 string s;
 vector<float> reconstructed_angles;
 
 
 while(ros::ok()){
 	new_data = new_foot && new_ang && new_com && new_obs && new_tilt; //removed new leg
	if(new_data) { //condition should be new_data, put true to run continuously
		//OLD COM CALCULATION (BEST ONE!)
		com_y = -camera_com_offs_y;
		com_z -= camera_com_offs_z;
		/*NEW COM CALCULATION (15/09/2023)
		float temp_com_z = com_z - d * sin(fixed_angle - ref_tilt + tilt);
		float temp_com_y = - d * cos (fixed_angle - ref_tilt + tilt);
		com_z = temp_com_z - dsh * cos(ref_tilt - tilt);
		com_y = temp_com_y - dsh * sin(ref_tilt - tilt); 
		*/
		if(leg==-1) {
			leg = next_leg==true?0:1;
			s = next_leg==true?"left":"right";
		}
		
		std::cout<<"Swing leg: "<< s <<std::endl;
		
		if(leg==0){ //left leg is the swinging one
			std::cout<<"CoM z: " << com_z <<std::endl;
			knee_y = com_y + L1* sin(measurements[0] * M_PI/180); //left angle has negative hip angle
			std::cout<<"Left knee y pos: " << knee_y <<std::endl;
			knee_z = com_z - L1 * cos(measurements[0]* M_PI/180);
			std::cout<<"Left knee z pos: " << knee_z <<std::endl;
			heel_y = knee_y + L2 * sin((measurements[0] + measurements[1])* M_PI/180);
			std::cout<<"Left foot y pos: " << heel_y <<std::endl;
			heel_z = knee_z - L2 * cos((measurements[0] + measurements[1])* M_PI/180);
			std::cout<<"Left foot z pos: " << heel_z <<std::endl;
			knee_y_p = com_y + L1* sin(measurements[2]* M_PI/180); //left angle has negative hip angle
			std::cout<<"Right knee y pos: " << knee_y_p <<std::endl;
			knee_z_p = com_z - L1 * cos(measurements[2]* M_PI/180);
			std::cout<<"Right knee z pos: " << knee_z_p <<std::endl;
			heel_y_p = knee_y_p + L2 * sin((measurements[2] + measurements[3])* M_PI/180);
			std::cout<<"Right foot y pos (pivot): " << heel_y_p <<std::endl;
			heel_z_p = knee_z_p - L2 * cos((measurements[2] + measurements[3])* M_PI/180);
			std::cout<<"Right foot z pos: " << heel_z_p <<std::endl;
			reconstructed_angles = leg_inverse_kinematics(com_y,L1,L2,knee_y,heel_y);
			reconstructed_angles[0] = rad_to_deg(reconstructed_angles[0]);
			reconstructed_angles[1] = rad_to_deg(reconstructed_angles[1]);
			cout<<"Reconstructed angles error (swing): Hip = "<< abs(reconstructed_angles[0]-measurements[0]) << "° , Knee = " << abs(reconstructed_angles[1]-measurements[1])<<"°" <<endl;
			reconstructed_angles = leg_inverse_kinematics(com_y,L1,L2,knee_y_p,heel_y_p);
			reconstructed_angles[0] = rad_to_deg(reconstructed_angles[0]);
			reconstructed_angles[1] = rad_to_deg(reconstructed_angles[1]);
			cout<<"Reconstructed angles error (support): Hip = "<< abs(reconstructed_angles[0]-measurements[2]) << "° , Knee = " << abs(reconstructed_angles[1]-measurements[3])<<"°" <<endl;
			
			
		}
		else if(leg==1){ //right leg is the swinging one
			std::cout<<"CoM z: " << com_z <<std::endl;
			knee_y = com_y + L1* sin(measurements[2]* M_PI/180); //right angle has positive hip angle
			std::cout<<"Right knee y pos: " << knee_y <<std::endl;
			knee_z = com_z - L1 * cos(measurements[2]* M_PI/180);
			std::cout<<"Right knee z pos: " << knee_z <<std::endl;
			heel_y = knee_y + L2 * sin((measurements[2] + measurements[3])* M_PI/180); //translation value
			std::cout<<"Right foot y pos: " << heel_y <<std::endl;
			heel_z = knee_z - L2 * cos((measurements[2] + measurements[3])* M_PI/180);
			std::cout<<"Right foot z pos: " << heel_z <<std::endl;
			knee_y_p = com_y + L1* sin(measurements[0]* M_PI/180); //left angle has negative hip angle
			std::cout<<"Left knee y pos: " << knee_y_p <<std::endl;
			knee_z_p = com_z - L1 * cos(measurements[0]* M_PI/180);
			std::cout<<"Left knee z pos: " << knee_z_p <<std::endl;
			heel_y_p = knee_y_p + L2 * sin((measurements[0] + measurements[1])* M_PI/180);
			std::cout<<"Left foot y pos (pivot): " << heel_y_p <<std::endl;
			heel_z_p = knee_z_p - L2 * cos((measurements[0] + measurements[1])* M_PI/180);
			std::cout<<"Left foot z pos: " << heel_z_p <<std::endl;
			reconstructed_angles = leg_inverse_kinematics(com_y,L1,L2,knee_y_p,heel_y_p);
			reconstructed_angles[0] = rad_to_deg(reconstructed_angles[0]);
			reconstructed_angles[1] = rad_to_deg(reconstructed_angles[1]);
			cout<<"Reconstructed angles error (support): Hip = "<< abs(reconstructed_angles[0]-measurements[0]) << "° , Knee = " << abs(reconstructed_angles[1]-measurements[1])<<"°" <<endl;
			reconstructed_angles = leg_inverse_kinematics(com_y,L1,L2,knee_y,heel_y);
			reconstructed_angles[0] = rad_to_deg(reconstructed_angles[0]);
			reconstructed_angles[1] = rad_to_deg(reconstructed_angles[1]);
			cout<<"Reconstructed angles error (swing): Hip = "<< abs(reconstructed_angles[0]-measurements[2]) << "° , Knee = " << abs(reconstructed_angles[1]-measurements[3])<<"°" <<endl;
		
		}
		trans_com = com_y - heel_y; //SERVE?
		trans_pivot= heel_y_p - heel_y;
		trans_obs = vector<float>(obstacle_shape.size());
		for(int i=0; i< obstacle_shape.size();i++) { //only change y coords, which are the even indexes
			if(i%2==0) trans_obs[i] = obstacle_shape[i] - heel_y;
			else trans_obs[i] = obstacle_shape[i];
		}
		if(trans_obs.size()>0){
			std::cout<<"Obstacles start at y: "<< trans_obs[0] << " and ends at y: "<<trans_obs[trans_obs.size()-2]<<std::endl; 
		}
		trans_foothold= foothold - heel_y;		 
		std_msgs::Float32MultiArray outp;
  		outp.layout.dim.push_back(std_msgs::MultiArrayDimension());
  		outp.layout.dim[0].size = trans_obs.size();
  		outp.layout.dim[0].stride = 1;
  		outp.layout.dim[0].label = "y-z"; // or whatever name you typically use to index
  		outp.data.clear();
  		outp.data.insert(outp.data.end(), trans_obs.begin(), trans_obs.end());
  		pub.publish(outp);
  		std_msgs::Float32 msg;
  		msg.data= trans_foothold;
  		pub2.publish(msg);
  		msg.data = com_z;
  		pub3.publish(msg);
  		msg.data = trans_pivot;
  		pub4.publish(msg);
  		msg.data= heel_y_p;
  		pub5.publish(msg);
  		new_ang=false;
  		new_com=false;
  		new_obs=false;
  		new_foot=false;
  		new_leg=false;
  		new_tilt=false;
	
	}
	ros::spinOnce();
 }
 }
