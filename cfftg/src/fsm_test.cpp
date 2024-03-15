#include "cfftg_algorithms.h" //will include also utility.h



using namespace std;

//GLOBAL VARS (UPDATED BY TOPICS)
bool cfftg_start;
float hip_height;
float pivot;
float endpoint;
int swing_leg;
vector<Point> obstacle_shape;
bool flag_start, flag_hip, flag_piv, flag_end, flag_obs, flag_swing, flag;


void start_cb(const std_msgs::Bool::ConstPtr& msg){
	cfftg_start=msg->data;
	flag_start=true;
}

void hip_height_cb(const std_msgs::Float32::ConstPtr& msg){
	hip_height=msg->data;
	flag_hip=true;
}

void pivot_cb(const std_msgs::Float32::ConstPtr& msg){
	pivot=msg->data;
	flag_piv=true;
}

void foothold_cb(const std_msgs::Float32::ConstPtr& msg){
	endpoint=msg->data;
	flag_end=true;
}

void obstacle_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	obstacle_shape.clear();
	if(msg->layout.dim[0].size > 0) {
		for(int i=0;i<msg->data.size()-1;i+=2){
			obstacle_shape.push_back(Point(msg->data[i],msg->data[i+1]));
		}
	}
	flag_obs=true;
}

void swing_cb(const std_msgs::Int8::ConstPtr& msg){
	swing_leg=msg->data;
	flag_swing=true;
}

int main (int argc, char** argv) {
	ros::init (argc, argv, "control");
	ros::NodeHandle nh;
	// PARAM AND TOPIC ACQUISITION (COMMENT THIS SECTION DURING TESTING)----------------------------------------------------------------------------------------------------------------------
	//TOPIC RETRIEVAL
	ros::Subscriber sub_start = nh.subscribe ("/cfftg_start", 1, start_cb);
	ros::Subscriber sub_hip_height = nh.subscribe ("/CoM_height", 1, hip_height_cb);
	ros::Subscriber sub_pivot = nh.subscribe ("/pivot", 1, pivot_cb);
	ros::Subscriber sub_foothold = nh.subscribe ("/foothold", 1, foothold_cb);
	ros::Subscriber sub_obstacle = nh.subscribe ("/obstacle_shape", 1, obstacle_cb);
	ros::Subscriber sub_swing = nh.subscribe ("/swing_leg", 1, swing_cb);
	ros::Publisher pub = nh.advertise<std_msgs::Bool> ("cfftg_done", 1);
	ros::Publisher pub_wp = nh.advertise<std_msgs::Float32MultiArray> ("joint_trajectory", 1);
	
	std::cout<<"Publisher and Subscribers set"<<std::endl;
	
	std_msgs::Bool msg;
	//PARAM RETRIEVAL
	float thigh_length,shin_length,front_foot_length, rear_foot_length,step_time, time_unit, knee_ang, hip_max_ext_time_coeff, foot_peak_time_coeff,foot_peak_time_coeff_2; //kinematic params
	//CFFTG params
	int max_it, restart_it;
	float default_sh, default_sv, min_sigma_h, min_sigma_v, decay_coeff_h, decay_coeff_v, min_dist_th, wp_time_dist, wp_derivative_th, joint_vel; 
	bool produce_txt_file;
	std::string txt_file_name,cfftg_type,obs_file_name, cart_traj_file_name, ang_traj_file_name,wp_selector_type, wp_strategy_type;
	//get params from parameter server
	ros::param::get("~front_foot_length", front_foot_length);
	ros::param::get("~rear_foot_length", rear_foot_length);
	ros::param::get("~thigh_length", thigh_length);
	ros::param::get("~shin_length", shin_length);
	ros::param::get("~step_time", step_time);
	ros::param::get("~time_unit", time_unit);
	ros::param::get("~knee_ang", knee_ang);
	ros::param::get("~cfftg_type", cfftg_type);
	ros::param::get("~min_dist_threshold", min_dist_th);
	ros::param::get("~max_it", max_it);
	ros::param::get("~restart_it", restart_it);
	ros::param::get("~default_sigma_h", default_sh);
	ros::param::get("~default_sigma_v", default_sv);
	ros::param::get("~min_sigma_h", min_sigma_h);
	ros::param::get("~min_sigma_v", min_sigma_v);
	ros::param::get("~decay_coeff_h", decay_coeff_h);
	ros::param::get("~decay_coeff_v", decay_coeff_v);
	ros::param::get("~produce_matrix_file", produce_txt_file);
	ros::param::get("~obstacle_filename", obs_file_name);
	ros::param::get("~cartesian_traj_filename", cart_traj_file_name);
	ros::param::get("~angular_traj_filename", ang_traj_file_name);
	
	ros::param::get("~wp_min_time_dist", wp_time_dist);
	ros::param::get("~wp_derivative_threshold", wp_derivative_th);
	ros::param::get("~wp_selector_type", wp_selector_type);
	ros::param::get("~wp_strategy_type", wp_strategy_type);
	ros::param::get("~joint_velocity", joint_vel);
	
	if(produce_txt_file) ros::param::get("~matrix_filename", txt_file_name);
	if(cfftg_type=="cubic_hip_linear_foot" || cfftg_type=="cubic")  ros::param::get("~hip_max_extension_time_coeff", hip_max_ext_time_coeff);
	if(cfftg_type=="cubic" || cfftg_type=="cubic_double")  ros::param::get("~foot_peak_time_coeff", foot_peak_time_coeff);
	if(cfftg_type=="cubic_double")  ros::param::get("~foot_peak_time_coeff_2", foot_peak_time_coeff_2);
	cfftg_start=false;
	std::cout<<"Params retrieved"<<std::endl;
		
	/*HARDCODED VALUES (COMMENT THIS SECTION DURING REAL EXPERIMENTS)-------------------------------------------------------------------------------------------------------------------------
	float thigh_length=0.48; //from param
	float shin_length=0.46; //from param
	float front_foot_length=0.21; //from param
	float rear_foot_length=0.07; //from param
	float step_time= 4.0; //from param
	int knee_ang = 15; //from param
	float hip_height = (thigh_length + shin_length) * 0.999; //from topic
	float pivot = 0.05; //from topic
	vector<Point> obstacle_shape(100); //from topic
	for(int i=0; i<42; i++) obstacle_shape[i] = Point(0.01*i, 0.0); 
	for(int i=42; i<52; i++) obstacle_shape[i] = Point(0.01*i , 0.2);
	for(int i=48;i<100; i++) obstacle_shape[i] = Point(0.01*i, 0.0);
	float end = 0.8; //from topic
	//CFFTG EXECUTION--------------------------------------------------------------------------------------------------------------------------------------------------------------
	*/
	while(ros::ok()){
		flag = flag_start && flag_hip && flag_piv && flag_end && flag_obs && flag_swing;
		if(cfftg_start && flag){
			std::string s;
			std::cout<<"Hip height:"<<hip_height<<"\t\t Pivot pos: "<<pivot <<std::endl; 
			std::cout<<"Foothold:"<<endpoint <<"\t\t Swing Leg: "<<swing_leg <<std::endl;
			std::cout<<"Obstacles: [";
			for(int i=0; i< obstacle_shape.size();i++){
				std::cout<<"( " << obstacle_shape[i].getx() <<" , "<<obstacle_shape[i].gety()<<" ) ";
			
			} 
			std::cout<<"]"<<std::endl;
			std::cout<<"Movement ready to start. Press any key and Enter to continue, or write \"no\" to abort"<< std::endl;
	 		std::cin>>s;
	 		if(s!="no") {
				cfftg_start=false; //so that the cfftg only activates when required
				do{
					s="";
	  				std::cout<<"Movement message published"<< std::endl;
					std::cout<<"If movement was successful and it's over, press \"a\" and Enter to continue. If movement failed, press \"r\" and Enter to try again."<< std::endl;
	 				std::cin>>s;
	 			}
	 			while(s!="a");
				msg.data=true;
				pub.publish(msg);
			}
			else break;
			//RESET SUBSCRIBERS FLAGS
			flag_start= false;
			flag_hip = false;
			flag_piv=false;
			flag_end=false;
			flag_obs = false;
			flag_swing=false;
		}

		ros::spinOnce();
	}
}
