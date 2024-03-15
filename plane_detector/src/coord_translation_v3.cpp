#include "utility.h"


using namespace std;

vector<float> measurements; // measurements[i]: {left_hip, left_knee, right_hip, right_knee} 
float foothold;
float pivot_tip;
float cam_z;
float ref_tilt;
float tilt;
bool next_leg;
vector<float> obstacle_shape;
bool new_ang=false;
bool new_foot=false;
bool new_piv=false;
bool new_cam=false;
bool new_obs= false;
bool new_leg = false;
bool new_next = false;
bool first_step=true;
float foot_tip_pos;

bool new_tilt=false;
int leg;

float tilt_buff[5];
int tilt_buff_idx=0;
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
void cam_cb(const std_msgs::Float32::ConstPtr& msg){
	cam_z = msg->data;
	new_cam=true;
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
	/*
	if(tilt_buff_idx<5) {
		tilt_buff[tilt_buff_idx] = msg->data;
		tilt_buff_idx++;
		tilt = msg->data;
	}
	else {
		float mean=0;
		for(int i=0; i<4;i++){
			if(i==4){
				tilt_buff[i]=msg->data;
			}
			else{
				tilt_buff[i]=tilt_buff[i+1];
				
			}
			mean+=tilt_buff[i];
		}
		mean/=5;
		tilt=mean;
	}
	*/
	tilt = msg->data;
	new_tilt=true;
}

void fs_cb(const std_msgs::Bool::ConstPtr& msg){
	first_step = msg->data;
}

void pos_cb(const std_msgs::Float32::ConstPtr& msg){
	foot_tip_pos = msg->data;
}



int main (int argc, char** argv) {
 ros::init (argc, argv, "coord_translation");
 ros::NodeHandle nh;
 //Subscribers
 ros::Subscriber sub_encoders = nh.subscribe ("/angle", 1000, angles_cb);
 ros::Subscriber sub_foothold = nh.subscribe("/foothold_raw", 1000, foothold_cb);
 //ros::Subscriber sub3 = nh.subscribe("/pivot_tip_raw", 1000, pivot_cb);
 ros::Subscriber sub_com = nh.subscribe("/CoM_height_raw", 1000, cam_cb);
 ros::Subscriber sub_obstacles = nh.subscribe("/obstacle_shape_raw", 1000, obstacle_cb);
 ros::Subscriber sub_swing_leg = nh.subscribe("/swing_leg", 1000, leg_cb);
 ros::Subscriber sub_next_swing_leg = nh.subscribe("/next_swing_leg", 1000, next_leg_cb);
 ros::Subscriber sub_ref_tilt = nh.subscribe("/reference_tilt", 1000, ref_cb);
 ros::Subscriber sub_tilt = nh.subscribe("/tilt_angle", 1000, tilt_cb);
 ros::Subscriber sub_fs = nh.subscribe("/first_step_global", 1000, fs_cb);
 ros::Subscriber sub_pos = nh.subscribe("/foot_tip_pos", 1000, pos_cb);
 //Publishers
 ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray> ("obstacle_shape", 1);
 ros::Publisher pub2 =nh.advertise<std_msgs::Float32> ("foothold", 1);
 ros::Publisher pub3 = nh.advertise<std_msgs::Float32> ("CoM_height", 1);
 ros::Publisher pub4 = nh.advertise<std_msgs::Float32> ("pivot", 1);
 ros::Publisher pub5 = nh.advertise<std_msgs::Float32> ("pivot_camera_coord", 1);
 
 
 float camera_com_offs_y= 0.21; //horizontal offset between camera and com (assumption: com = hip joint)
 float camera_com_offs_z= 0.07; //vertical offset between camera and com (updated from 0.12 to 0.1 15/11/2023)
 float camera_com_offs = sqrt(pow(camera_com_offs_z,2) +pow(camera_com_offs_y,2)); //offset between com and camera
 float camera_com_angle = asin(camera_com_offs_z/camera_com_offs); //angle between camera and com (  assumption:  camera_com_offs_z =  camera_com_offs* sin(camera_com_angle)  )
 float com_z=0;
 float com_y=0;
 float L1; //should be provided as ros param
 float L2; //should be provided as ros param
 float front_foot_length;
 float delta_ang, ang_th;
 ros::param::get("~thigh_length", L1);
 ros::param::get("~shin_length", L2);
 ros::param::get("~front_foot_length", front_foot_length);
 ros::param::get("~delta_angle_iterative_adjustment", delta_ang);
 ros::param::get("~angle_threshold_iterative_adjustment", ang_th);
 //float dsh = 0.06; //should be provided as ros param
 //float d_y = 0.24;
 //float d_z = 0.1;
 //float d = sqrt(pow(d_y,2)+pow(d_z,2));
 //float fixed_angle_ch= asin(d_z / d);
 bool new_data=false;
 float knee_y, knee_y_p;
 float knee_z, knee_z_p;
 float heel_y, heel_y_p;
 float heel_z, heel_z_p;
 float trans_com;
 float trans_pivot;
 vector<float> trans_obs;
 float trans_foothold;
 string s, opp_s;
 vector<float> reconstructed_angles;
 
 
 while(ros::ok()){
 	new_data = new_foot && new_ang && new_cam && new_obs && new_tilt; //removed new leg
	if(new_data) { //condition should be new_data, put true to run continuously
		//OLD COM CALCULATION (BEST ONE!)
		com_y = -camera_com_offs_y;
		com_z = cam_z - camera_com_offs_z;
		//NEW COM CALCULATION (15/09/2023)
		//com_z =cam_z - d * sin(fixed_angle_ch + ref_tilt-tilt);
		//com_y = - d * cos (fixed_angle_ch+ ref_tilt-tilt);
		
		if(leg==-1) {
			leg = next_leg==true?1:0;
			
		}
		s = leg==true?"right":"left";
		opp_s = leg==true?"left":"right";
		std::cout<<"Swing leg: "<< s <<std::endl;
		//std::cout<<"Perceived tilt: "<< ref_tilt-tilt<<std::endl;
		
		if(leg==0){ //left leg is the swinging one
			/*
			knee_y = com_y + L1* sin((measurements[0]+ ref_tilt-tilt) * M_PI/180); //left angle has negative hip angle
			knee_z = com_z - L1 * cos((measurements[0]+ ref_tilt-tilt)* M_PI/180);
			heel_y = knee_y + L2 * sin((measurements[0] + measurements[1]+ ref_tilt-tilt)* M_PI/180);
			heel_z = knee_z - L2 * cos((measurements[0] + measurements[1]+ ref_tilt-tilt)* M_PI/180);
			knee_y_p = com_y + L1* sin((measurements[2]+ ref_tilt-tilt)* M_PI/180); //left angle has negative hip angle
			knee_z_p = com_z - L1 * cos((measurements[2]+ ref_tilt-tilt)* M_PI/180);
			heel_y_p = knee_y_p + L2 * sin((measurements[2] + measurements[3]+ ref_tilt-tilt)* M_PI/180);
			heel_z_p = knee_z_p - L2 * cos((measurements[2] + measurements[3]+ ref_tilt-tilt)* M_PI/180);
			*/
			knee_y = com_y + L1* sin((measurements[0]) * M_PI/180); //left angle has negative hip angle
			knee_z = com_z - L1 * cos((measurements[0])* M_PI/180);
			heel_y = knee_y + L2 * sin((measurements[0] + measurements[1])* M_PI/180);
			heel_z = knee_z - L2 * cos((measurements[0] + measurements[1])* M_PI/180);
			knee_y_p = com_y + L1* sin((measurements[2])* M_PI/180); //left angle has negative hip angle
			knee_z_p = com_z - L1 * cos((measurements[2])* M_PI/180);
			heel_y_p = knee_y_p + L2 * sin((measurements[2] + measurements[3])* M_PI/180);
			heel_z_p = knee_z_p - L2 * cos((measurements[2] + measurements[3])* M_PI/180);
			
			
			
			reconstructed_angles = leg_inverse_kinematics(com_y,L1,L2,knee_y,heel_y);
			reconstructed_angles[0] = rad_to_deg(reconstructed_angles[0]);
			reconstructed_angles[1] = rad_to_deg(reconstructed_angles[1]);
			//cout<<"Reconstructed angles error (swing): Hip = "<< abs(reconstructed_angles[0]-measurements[0]) << "° , Knee = " << abs(reconstructed_angles[1]-measurements[1])<<"°" <<endl;
			reconstructed_angles = leg_inverse_kinematics(com_y,L1,L2,knee_y_p,heel_y_p);
			reconstructed_angles[0] = rad_to_deg(reconstructed_angles[0]);
			reconstructed_angles[1] = rad_to_deg(reconstructed_angles[1]);
			//cout<<"Reconstructed angles error (support): Hip = "<< abs(reconstructed_angles[0]-measurements[2]) << "° , Knee = " << abs(reconstructed_angles[1]-measurements[3])<<"°" <<endl;
			
			
		}
		else if(leg==1){ //right leg is the swinging one
			/*
			knee_y = com_y + L1* sin((measurements[2]+ ref_tilt-tilt)* M_PI/180); //right angle has positive hip angle
			knee_z = com_z - L1 * cos((measurements[2]+ ref_tilt-tilt)* M_PI/180);
			heel_y = knee_y + L2 * sin((measurements[2] + measurements[3]+ ref_tilt-tilt)* M_PI/180); //translation value
			heel_z = knee_z - L2 * cos((measurements[2] + measurements[3]+ ref_tilt-tilt)* M_PI/180);
			knee_y_p = com_y + L1* sin((measurements[0]+ ref_tilt-tilt)* M_PI/180); //left angle has negative hip angle
			knee_z_p = com_z - L1 * cos((measurements[0]+ ref_tilt-tilt)* M_PI/180);
			heel_y_p = knee_y_p + L2 * sin((measurements[0] + measurements[1]+ ref_tilt-tilt)* M_PI/180);
			heel_z_p = knee_z_p - L2 * cos((measurements[0] + measurements[1]+ ref_tilt-tilt)* M_PI/180);
			*/
			knee_y = com_y + L1* sin((measurements[2])* M_PI/180); //right angle has positive hip angle
			knee_z = com_z - L1 * cos((measurements[2])* M_PI/180);
			heel_y = knee_y + L2 * sin((measurements[2] + measurements[3])* M_PI/180); //translation value
			heel_z = knee_z - L2 * cos((measurements[2] + measurements[3])* M_PI/180);
			knee_y_p = com_y + L1* sin((measurements[0])* M_PI/180); //left angle has negative hip angle
			knee_z_p = com_z - L1 * cos((measurements[0])* M_PI/180);
			heel_y_p = knee_y_p + L2 * sin((measurements[0] + measurements[1])* M_PI/180);
			heel_z_p = knee_z_p - L2 * cos((measurements[0] + measurements[1])* M_PI/180);
			
			reconstructed_angles = leg_inverse_kinematics(com_y,L1,L2,knee_y_p,heel_y_p);
			reconstructed_angles[0] = rad_to_deg(reconstructed_angles[0]);
			reconstructed_angles[1] = rad_to_deg(reconstructed_angles[1]);
			//cout<<"Reconstructed angles error (support): Hip = "<< abs(reconstructed_angles[0]-measurements[0]) << "° , Knee = " << abs(reconstructed_angles[1]-measurements[1])<<"°" <<endl;
			reconstructed_angles = leg_inverse_kinematics(com_y,L1,L2,knee_y,heel_y);
			reconstructed_angles[0] = rad_to_deg(reconstructed_angles[0]);
			reconstructed_angles[1] = rad_to_deg(reconstructed_angles[1]);
			//cout<<"Reconstructed angles error (swing): Hip = "<< abs(reconstructed_angles[0]-measurements[2]) << "° , Knee = " << abs(reconstructed_angles[1]-measurements[3])<<"°" <<endl;
		
		}
		
		std::cout<<"BEFORE PROCESSING"<<std::endl;
		std::cout<<"CoM y: " << com_y <<std::endl;
		std::cout<<"CoM z: " << com_z <<std::endl;
		std::cout<< s<<" knee y pos: " << knee_y <<std::endl;
		std::cout<<s<<" knee z pos: " << knee_z <<std::endl;
		std::cout<<s<<" foot y pos: " << heel_y <<std::endl;
		std::cout<<s<<" foot z pos: " << heel_z <<std::endl;
		std::cout<<opp_s<<" knee y pos (pivot): " << knee_y_p <<std::endl;
		std::cout<<opp_s<<" knee z pos (pivot): " << knee_z_p <<std::endl;
		std::cout<<opp_s<<" foot y pos (pivot): " << heel_y_p <<std::endl;
		std::cout<<opp_s<<" foot z pos (pivot): " << heel_z_p <<std::endl;
		
		if(obstacle_shape.size()>0){
			std::cout<<"Obstacles start at y: "<< obstacle_shape[0] << " and ends at y: "<<obstacle_shape[obstacle_shape.size()-2]<<std::endl; 
		}
		
		//PROTOTYPE: ROTATION AROUND CAMERA!
		
		int it=1; //just to check for errors
		float hyp,ang;
		bool switch_rotation=false;
		float tot_rotation=0;
		std::cout<<"Starting iterative adjustment..."<<std::endl;
		float init_diff = abs(heel_z - heel_z_p);
		// OLD CONDITION while(!(abs(heel_z) < ang_th && abs(heel_z_p)< ang_th) && it<=50) {
		while(!(abs(heel_z - heel_z_p)<ang_th) && it<=50) {	
			
			//COM
			//OLD CONDITION if(it>10 && !switch_rotation && heel_z> 0.1 || heel_z_p>0.1){
			if(it>3 && !switch_rotation && abs(heel_z - heel_z_p)>init_diff){
				delta_ang = -delta_ang;
				switch_rotation=true;
				//SWITCH ROTATION
				std::cout<<"Switching rotation"<<std::endl; 
			}
			hyp = euclidean_dist(0,cam_z, com_y, com_z); //hypotenuse
			//std::cout<<"Difference from known distance CAM-CoM and measured distance is"<<abs(d - hyp)<<std::endl;
			ang=calc_angle_from_side_sin(-com_y, hyp);
			com_y = - hyp * sin(ang + deg_to_rad(delta_ang));
			com_z =cam_z -hyp * cos(ang + deg_to_rad(delta_ang));
			//KNEE SWING
			hyp = euclidean_dist(0,cam_z, knee_y, knee_z); //hypotenuse
			ang=calc_angle_from_side_sin(-knee_y, hyp);
			knee_y = - hyp * sin(ang + deg_to_rad(delta_ang));
			knee_z =cam_z -hyp * cos(ang + deg_to_rad(delta_ang));
			//FOOT SWING
			hyp = euclidean_dist(0,cam_z, heel_y, heel_z); //hypotenuse
			ang=calc_angle_from_side_sin(-heel_y, hyp);
			heel_y = - hyp * sin(ang + deg_to_rad(delta_ang));
			heel_z =cam_z -hyp * cos(ang + deg_to_rad(delta_ang));
			//KNEE PIVOT
			hyp = euclidean_dist(0,cam_z, knee_y_p, knee_z_p); //hypotenuse
			ang=calc_angle_from_side_sin(-knee_y_p, hyp);
			knee_y_p = - hyp * sin(ang + deg_to_rad(delta_ang));
			knee_z_p =cam_z -hyp * cos(ang + deg_to_rad(delta_ang));
			//FOOT PIVOT
			hyp = euclidean_dist(0,cam_z, heel_y_p, heel_z_p); //hypotenuse
			ang=calc_angle_from_side_sin(-heel_y_p, hyp);
			heel_y_p = - hyp * sin(ang + deg_to_rad(delta_ang));
			heel_z_p =cam_z -hyp * cos(ang + deg_to_rad(delta_ang));
			
			
			tot_rotation+=delta_ang;
			
			
			std::cout<<"Current swing foot z-coordinate: "<< heel_z <<std::endl;
			std::cout<<"Current pivot foot z-coordinate: "<< heel_z_p <<std::endl;
			std::cout<<"Current Angle: "<< tot_rotation <<std::endl;
			it++;
		
		}
		if(it>50) std::cout<<"Iterative adjustment failed!"<<std::endl;
		switch_rotation=false;
		delta_ang = abs(delta_ang);
		
		
		//TRANSLATION Z-AXIS
		
		com_z-=heel_z;
		knee_z-=heel_z;
		knee_z_p-=heel_z;
		heel_z_p-=heel_z;
		heel_z-=heel_z;
		
		
		
		
		
		//TRANSLATION TO ALIGN PIVOT FOOT
		/*
		float measured_p = foot_tip_pos - front_foot_length;
		float diff = measured_p - heel_y_p;
		std::cout<<"Perceived distance between measured and calculated pivot is: "<<diff<<std::endl;
		if(abs(diff)>0.05) {
			
			com_y+=diff;
			knee_y+=diff;
			knee_y_p+=diff;
			heel_y+=diff;
			heel_y_p+=diff;
		
		}
		*/
		
		
		
		//FINAL TRANSLATION TO ALIGN SWING FOOT WITH ORIGIN
		
		float max_h=0;
		
		trans_obs = vector<float>(obstacle_shape.size());
		for(int i=0; i< obstacle_shape.size();i++) { //only change y coords, which are the even indexes
			if(i%2==0) trans_obs[i] = obstacle_shape[i] - heel_y;
			else {
				trans_obs[i] = obstacle_shape[i];
				if(obstacle_shape[i]>max_h) max_h = obstacle_shape[i];
			}
		}
			
		
		com_y-=heel_y;
		heel_y_p-=heel_y;
		knee_y-= heel_y;
		knee_y_p-=heel_y;
		trans_pivot= heel_y_p;
		trans_foothold= foothold - heel_y;
		heel_y-=heel_y;

		//END OF V2
		
		
		//PRINT
		std::cout<<"AFTER PROCESSING"<<std::endl;
		std::cout<<"CoM y: " << com_y <<std::endl;
		std::cout<<"CoM z: " << com_z <<std::endl;
		std::cout<< s<<" knee y pos: " << knee_y <<std::endl;
		std::cout<<s<<" knee z pos: " << knee_z <<std::endl;
		std::cout<<s<<" foot y pos: " << heel_y <<std::endl;
		std::cout<<s<<" foot z pos: " << heel_z <<std::endl;
		std::cout<<opp_s<<" knee y pos (pivot): " << knee_y_p <<std::endl;
		std::cout<<opp_s<<" knee z pos (pivot): " << knee_z_p <<std::endl;
		std::cout<<opp_s<<" foot y pos (pivot): " << heel_y_p <<std::endl;
		std::cout<<opp_s<<" foot z pos (pivot): " << heel_z_p <<std::endl;
		
		
		if(trans_obs.size()>0){
			std::cout<<"Obstacles start at y: "<< trans_obs[0] << " and ends at y: "<<trans_obs[trans_obs.size()-2]<<std::endl; 
			std::cout<<"Obstacles Max Height: "<< max_h<<std::endl;
		}
			 
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
  		new_cam=false;
  		new_obs=false;
  		new_foot=false;
  		new_leg=false;
  		new_tilt=false;
	
	}
	ros::spinOnce();
 }
 }
