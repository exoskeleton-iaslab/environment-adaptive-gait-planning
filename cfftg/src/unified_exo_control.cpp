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
			std::cout<<"CFFTG started"<<std::endl;
			//save_point_vect(obstacle_shape, "obstacles", "/home/minipc_exo/catkin_ws/obstacles"); OLD TXT FORMAT
			save_vect_for_matlab(obstacle_shape, obs_file_name);
			vector<vector<Point>> bundle;
			if(cfftg_type=="linear"){
				bundle= cfftg_linear(thigh_length, shin_length,  rear_foot_length, front_foot_length, endpoint + rear_foot_length, hip_height, pivot, knee_ang, obstacle_shape, step_time,time_unit, max_it, restart_it, default_sh, default_sv, min_sigma_h, min_sigma_v, decay_coeff_h, decay_coeff_v, produce_txt_file, txt_file_name);
			}
			else if(cfftg_type=="cubic_hip_linear_foot"){
				bundle= cfftg_linear_foot_cubic_hip(thigh_length, shin_length,  rear_foot_length, front_foot_length, endpoint + rear_foot_length, hip_height, pivot, knee_ang, min_dist_th, hip_max_ext_time_coeff, obstacle_shape, step_time,time_unit, max_it, restart_it, default_sh, default_sv, min_sigma_h, min_sigma_v, decay_coeff_h, decay_coeff_v, produce_txt_file, txt_file_name);
			}
			else if(cfftg_type=="cubic")  {
				//choose cfftg_cubic for the classic validated version, or cfftg_cubic_v2 for the version with the new score 
				bundle= cfftg_cubic_v2(thigh_length, shin_length,  rear_foot_length, front_foot_length, endpoint + rear_foot_length, hip_height, pivot, knee_ang, min_dist_th, hip_max_ext_time_coeff, foot_peak_time_coeff, obstacle_shape, step_time,time_unit, max_it, restart_it, default_sh, default_sv, min_sigma_h, min_sigma_v, decay_coeff_h, decay_coeff_v, produce_txt_file, txt_file_name);
			
			}
			else if(cfftg_type=="cubic_double") {
			
				std::cout<<"Starting double CFFTG"<<std::endl;
			
				bundle= cfftg_cubic_double(thigh_length, shin_length,  rear_foot_length, front_foot_length, endpoint + rear_foot_length, hip_height, pivot, knee_ang, min_dist_th, hip_max_ext_time_coeff, foot_peak_time_coeff,foot_peak_time_coeff_2, obstacle_shape, step_time,time_unit, max_it, restart_it, default_sh, default_sv, min_sigma_h, min_sigma_v, decay_coeff_h, decay_coeff_v, produce_txt_file, txt_file_name);
			
			
			}
			else break; //here different type of cfftg can be added
			vector<vector<float>> angles;
			if(bundle[0].size()!=1){ //if this condition is met cfftg succeded
				vector<float> supp_hip, supp_knee, swing_hip, swing_knee;
				std::vector<float> temp_angles;
				for(int i=0;i<bundle[0].size();i++){
					 temp_angles = leg_inverse_kinematics(bundle[0][i].getx(), thigh_length, shin_length, bundle[1][i].getx(), pivot);
					 supp_hip.push_back(temp_angles[0]);
					 supp_knee.push_back(temp_angles[1]);
					 temp_angles = leg_inverse_kinematics(bundle[0][i].getx(), thigh_length, shin_length, bundle[2][i].getx(), bundle[3][i].getx());
					 swing_hip.push_back(temp_angles[0]);
					 swing_knee.push_back(temp_angles[1]);
				}
				angles.push_back(supp_hip);
				angles.push_back(supp_knee);
				angles.push_back(swing_hip);
				angles.push_back(swing_knee);
			}
			//printing(testing only)
			cfftg_cartesian_bundle_to_file(bundle, cart_traj_file_name);
			//cfftg_angular_bundle_to_file(angles, ang_traj_file_name);
			save_ang_traj_matlab(angles,time_unit,"/home/minipc_exo/catkin_ws/angular_trajectories");
			//EXTRACT NOTABLE WAYPOINTS
			std::vector<float> pos[4];
			std::vector<float> vel[4];
			std::vector<Point> temp_ang_traj;
			std::vector<Point> wps;
			std::vector<float> wps_y;
	 		std::vector<float> wps_x;
	 		std::vector<float> velocities;
	 		std::vector<float> timing;
	 		std::vector<float> final_vel;
	 		std::vector<float> temp_msg_vec;
			//SELECT WP POSITIONS FROM SWING KNEE TRAJ
			float precision= step_time/(angles[3].size()-1);
			if(wp_strategy_type=="variable") {
				for(int i=3;i>=0;i--){
					if(i==3){ //reference trajectory
						for(int j=0;j<angles[3].size();j++){
							temp_ang_traj.push_back(Point(precision*j,angles[3][j]));
						}
						wps = select_waypoints(temp_ang_traj,wp_time_dist,wp_derivative_th,wp_selector_type);
						for(int j=0;j<wps.size();j++){
							//std::cout<<wps[i].getx()<<std::endl;
							wps_y.push_back(rad_to_deg(wps[j].gety()));
							wps_x.push_back(wps[j].getx());
		 				}
		 				timing = derivative(wps_x);
					}
					else {
						wps_y.clear(); 
						for(int j=0;j<wps_x.size();j++) {
							int idx = wps_x[j]/precision;
							wps_y.push_back(rad_to_deg(angles[i][idx]));
						}
					
					}
					std::cout<<"Wps for traj "<< i <<": ["<<std::endl;
					for(int i=0;i<wps_y.size();i++){
						std::cout<< wps_y[i] << ", " << wps_x[i] << std::endl;;
					}
					std::cout<<"]"<<std::endl;
					
					velocities = derivative(wps_y);
		 			final_vel.clear();
		 			for(int j=0;j<velocities.size();j++){
						final_vel.push_back(velocities[j]/timing[j]);
		 			}
		 			wps_y.erase(wps_y.begin()); //initial position is not needed
		 			//POSIZIONARE CORRETTAMENTE RISPETTO A SWING LEG!
		 			if(swing_leg==0) {
		 				if(i==0) { //supp hip
		 					//pos[2].insert(pos[2].end(), wps_y.begin(), wps_y.end());
		 					//vel[2].insert(vel[2].end(), final_vel.begin(), final_vel.end());
		 					pos[2]=wps_y;
		 					vel[2]=final_vel;
		 				}
		 				else if(i==1){ //supp knee
		 					//pos[3].insert(pos[3].end(), wps_y.begin(), wps_y.end());
		 					//vel[3].insert(vel[3].end(), final_vel.begin(), final_vel.end());
		 					pos[3]=wps_y;
		 					vel[3]=final_vel;
		 				}
		 				else if(i==2){
		 					//pos[0].insert(pos[0].end(), wps_y.begin(), wps_y.end());
		 					//vel[0].insert(vel[0].end(), final_vel.begin(), final_vel.end());
		 					pos[0]=wps_y;
		 					vel[0]=final_vel;
		 				}
		 				else {
		 					//pos[1].insert(pos[1].end(), wps_y.begin(), wps_y.end());
		 					//vel[1].insert(vel[1].end(), final_vel.begin(), final_vel.end());
		 					pos[1]=wps_y;
		 					vel[1]=final_vel;
		 				}
		 			}
		 			else {
		 				if(i==0) { //supp hip
		 					//pos[0].insert(pos[0].end(), wps_y.begin(), wps_y.end());
		 					//vel[0].insert(vel[0].end(), final_vel.begin(), final_vel.end());
		 					pos[0]=wps_y;
		 					vel[0]=final_vel;
		 				}
		 				else if(i==1){ //supp knee
		 					//pos[1].insert(pos[1].end(), wps_y.begin(), wps_y.end());
		 					//vel[1].insert(vel[1].end(), final_vel.begin(), final_vel.end());
		 					pos[1]=wps_y;
		 					vel[1]=final_vel;
		 				}
		 				else if(i==2){
		 					//pos[2].insert(pos[2].end(), wps_y.begin(), wps_y.end());
		 					//vel[2].insert(vel[2].end(), final_vel.begin(), final_vel.end());
		 					pos[2]=wps_y;
		 					vel[2]=final_vel;
		 				}
		 				else {
		 					//pos[3].insert(pos[3].end(), wps_y.begin(), wps_y.end());
		 					//vel[3].insert(vel[3].end(), final_vel.begin(), final_vel.end());
		 					pos[3]=wps_y;
		 					vel[3]=final_vel;
		 				}
		 			
		 			}
				}
				save_vect_for_matlab(wps, "/home/minipc_exo/catkin_ws/wps");
				for(int i=0;i<pos[0].size();i++){
					for(int j=0;j<4;j++){
						temp_msg_vec.push_back(pos[j][i]);
					}
				}
				for(int i=0;i<vel[0].size();i++){
					for(int j=0;j<4;j++){
						temp_msg_vec.push_back(vel[j][i]);
						//temp_msg_vec.insert(temp_msg_vec.end(), vel[j].begin()+i, vel[j].begin()+i+1);
					}
				}
			
			
			}
			else if(wp_strategy_type=="fixed"){ //only two wps, the maximum absolute value of the angle (the rise) and the final angular position
				if(swing_leg==0) {//left leg is the swinging one
					for(int j=0;j<angles[2].size();j++){
							temp_ang_traj.push_back(Point(precision*j,angles[2][j]));
					}
					wps = select_waypoints(temp_ang_traj,wp_time_dist,wp_derivative_th,"peak");
					temp_msg_vec.push_back(rad_to_deg(wps[1].gety()));
					for(int k=0;k<3;k++) temp_msg_vec.push_back(0);
					temp_msg_vec.push_back(rad_to_deg(wps[2].gety()));
					for(int k=0;k<3;k++) temp_msg_vec.push_back(0);
					temp_ang_traj.clear();
					for(int j=0;j<angles[3].size();j++){
							temp_ang_traj.push_back(Point(precision*j,angles[3][j]));
					}
					wps = select_waypoints(temp_ang_traj,wp_time_dist,wp_derivative_th,"peak");
					temp_msg_vec[1] = rad_to_deg(wps[1].gety());
					temp_msg_vec[5] = rad_to_deg(wps[2].gety());
					temp_ang_traj.clear();
					for(int j=0;j<angles[0].size();j++){
							temp_ang_traj.push_back(Point(precision*j,angles[0][j]));
					}
					wps = select_waypoints(temp_ang_traj,wp_time_dist,wp_derivative_th,"peak");
					temp_msg_vec[2] = rad_to_deg(wps[1].gety());
					temp_msg_vec[6] = rad_to_deg(wps[2].gety());
					for(int k=0;k<8;k++) temp_msg_vec.push_back(joint_vel);
					//temp_msg_vec.push_back(0.0);
					//for(int k=0;k<3;k++) temp_msg_vec.push_back(40.0);
					//temp_msg_vec.push_back(0.0);
					
				}
				else if(swing_leg==1){
					for(int j=0;j<angles[0].size();j++){
							temp_ang_traj.push_back(Point(precision*j,angles[0][j]));
					}
					wps = select_waypoints(temp_ang_traj,wp_time_dist,wp_derivative_th,"peak");
					temp_msg_vec.push_back(rad_to_deg(wps[1].gety()));
					for(int k=0;k<3;k++) temp_msg_vec.push_back(0);
					temp_msg_vec.push_back(rad_to_deg(wps[2].gety()));
					for(int k=0;k<3;k++) temp_msg_vec.push_back(0);
					temp_ang_traj.clear();
					for(int j=0;j<angles[2].size();j++){
							temp_ang_traj.push_back(Point(precision*j,angles[2][j]));
					}
					wps = select_waypoints(temp_ang_traj,wp_time_dist,wp_derivative_th,"peak");
					temp_msg_vec[2] = rad_to_deg(wps[1].gety());
					temp_msg_vec[6] = rad_to_deg(wps[2].gety());
					temp_ang_traj.clear();
					for(int j=0;j<angles[3].size();j++){
							temp_ang_traj.push_back(Point(precision*j,angles[3][j]));
					}
					wps = select_waypoints(temp_ang_traj,wp_time_dist,wp_derivative_th,"peak");
					temp_msg_vec[3] = rad_to_deg(wps[1].gety());
					temp_msg_vec[7] = rad_to_deg(wps[2].gety());
					//temp_msg_vec.push_back(40.0);
					//temp_msg_vec.push_back(0.0);
					for(int k=0;k<8;k++) temp_msg_vec.push_back(joint_vel);
					//temp_msg_vec.push_back(0.0);
					//for(int k=0;k<2;k++) temp_msg_vec.push_back(40.0);
				}
			
			}
			
			
			std::cout<<"Positions and velocities vector: ["<<std::endl;
			for(int i=0;i<temp_msg_vec.size();i++){
				std::cout<< temp_msg_vec[i] << ", ";
			
			}
			std::cout<<"]"<<std::endl;
			std::string s;
			std::cout<<"Movement ready to start. Press any key and Enter to continue, or write \"no\" to abort"<< std::endl;
	 		std::cin>>s;
	 		if(s!="no") {
				std_msgs::Float32MultiArray pos_and_wps;
				pos_and_wps.layout.dim.push_back(std_msgs::MultiArrayDimension());
	  			pos_and_wps.layout.dim[0].size = temp_msg_vec.size();
	  			pos_and_wps.layout.dim[0].stride = 1;
	  			pos_and_wps.layout.dim[0].label = "pos_vel_wp"; // or whatever name you typically use to index
	  			pos_and_wps.data.clear();
	  			pos_and_wps.data.insert(pos_and_wps.data.end(), temp_msg_vec.begin(), temp_msg_vec.end());
				//EXECUTE MOVEMENT SERVICE (TO BE DONE)
				cfftg_start=false; //so that the cfftg only activates when required
				do{
					pub_wp.publish(pos_and_wps);
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
