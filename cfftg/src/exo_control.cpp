#include "utility.h"



using namespace std;

//GLOBAL VARS (UPDATED BY TOPICS)
bool cfftg_start;
float hip_height;
float pivot;
float endpoint;
int swing_leg;
vector<Point> obstacle_shape;


void start_cb(const std_msgs::Bool::ConstPtr& msg){
	cfftg_start=msg->data;
}

void hip_height_cb(const std_msgs::Float32::ConstPtr& msg){
	hip_height=msg->data;
}

void pivot_cb(const std_msgs::Float32::ConstPtr& msg){
	pivot=msg->data;
}

void foothold_cb(const std_msgs::Float32::ConstPtr& msg){
	endpoint=msg->data;
}

void obstacle_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	if(msg->layout.dim[0].size > 0) {
		obstacle_shape.clear();
		for(int i=0;i<msg->data.size()-1;i+=2){
			obstacle_shape.push_back(Point(msg->data[i],msg->data[i+1]));
		}
	}
}

void swing_cb(const std_msgs::Int8::ConstPtr& msg){
	swing_leg=msg->data;
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
	float thigh_length,shin_length,front_foot_length, rear_foot_length,step_time,knee_ang; //params
	
	ros::param::get("~front_foot_length", front_foot_length);
	ros::param::get("~rear_foot_length", rear_foot_length);
	ros::param::get("~thigh_length", thigh_length);
	ros::param::get("~shin_length", shin_length);
	ros::param::get("~step_time", step_time);
	ros::param::get("~knee_ang", knee_ang);
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
		if(cfftg_start){
			std::cout<<"CFFTG started"<<std::endl;
			save_point_vect(obstacle_shape, "obstacles", "/home/minipc_exo/catkin_ws/obstacles");
			vector<vector<Point>> bundle = cfftg_sagittal(thigh_length, shin_length,  rear_foot_length, front_foot_length, endpoint + rear_foot_length, hip_height, pivot, knee_ang, obstacle_shape, step_time,0.05);
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
			cfftg_cartesian_bundle_to_file(bundle, "/home/minipc_exo/catkin_ws/cartesian_trajectories");
			cfftg_angular_bundle_to_file(angles, "/home/minipc_exo/catkin_ws/angular_trajectories");
			
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
			//SELECT WP POSITIONS FROM SWING KNEE TRAJ
			float precision= step_time/(angles[3].size()-1);
			for(int i=3;i>=0;i--){
				if(i==3){ //reference trajectory
					for(int j=0;j<angles[3].size();j++){
						temp_ang_traj.push_back(Point(precision*j,angles[3][j]));
					}
					wps = select_waypoints(temp_ang_traj,0.4,0.005,"derivative");
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
			
			std::vector<float> temp_msg_vec;
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
			std::cout<<"Positions and velocities vector: ["<<std::endl;
			for(int i=0;i<temp_msg_vec.size();i++){
				std::cout<< temp_msg_vec[i] << ", ";
			
			}
			std::cout<<"]"<<std::endl;
			std::string s;
			std::cout<<"Movement ready to start. Press any key and Enter to continue"<< std::endl;
	 		std::cin>>s;
			std_msgs::Float32MultiArray pos_and_wps;
			pos_and_wps.layout.dim.push_back(std_msgs::MultiArrayDimension());
  			pos_and_wps.layout.dim[0].size = temp_msg_vec.size();
  			pos_and_wps.layout.dim[0].stride = 1;
  			pos_and_wps.layout.dim[0].label = "pos_vel_wp"; // or whatever name you typically use to index
  			pos_and_wps.data.clear();
  			pos_and_wps.data.insert(pos_and_wps.data.end(), temp_msg_vec.begin(), temp_msg_vec.end());
  			pub_wp.publish(pos_and_wps);


			//EXECUTE MOVEMENT SERVICE (TO BE DONE)
			cfftg_start=false; //so that the cfftg only activates when required
			msg.data=true;
			pub.publish(msg);
		}
		else{
			msg.data=false;
			pub.publish(msg);
		}
		ros::spinOnce();
	}
}
