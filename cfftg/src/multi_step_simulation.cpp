#include "utility.h"

using namespace std;



bool shift_obs_track(std::vector<Point>& obs, float shift) {
	bool no_obs=true;
	while(obs[0].getx()<shift) {
		obs.erase(obs.begin());
		obs.push_back(Point(0,0));
	}
	for(int i=0;i<obs.size();i++) {
		obs[i].set(i*0.01, obs[i].gety());
		if(no_obs && obs[i].gety()>0){
			no_obs=false;
		}
	
	
	}
	return no_obs;
}

void shift_and_stitch_bundle(std::vector<std::vector<std::vector<Point>>>& multi_step_bundle,std::vector<std::vector<Point>>& single_step_bundle, float cumulative_shift) {
	//shift all of the currently calculated 
	for(int i=0; i<single_step_bundle.size();i++) {
		for(int j=0; j < single_step_bundle[i].size();j++) {
			single_step_bundle[i][j].set(single_step_bundle[i][j].getx() + cumulative_shift, single_step_bundle[i][j].gety());
		}
	}
	
	multi_step_bundle.push_back(single_step_bundle);






}





int main (int argc, char** argv) {
	ros::init (argc, argv, "ms_sim");
	ros::NodeHandle nh;
	
	vector<Point> obs_left; //better to have the full track (4/5 meters of length)
	vector<Point> obs_right;  //better to have the full track (4/5 meters of length)
	//POPULATE LEFT OBSTACLE TRACK
	for(int i=0; i<100; i++) obs_left.push_back(Point(0.01*i, 0.0));
	for(int i=100;i<110;i++) obs_left.push_back(Point(0.01*i, 0.15));
	for(int i=110;i<160;i++) obs_left.push_back(Point(0.01*i, 0.0));
	for(int i=160;i<170;i++) obs_left.push_back(Point(0.01*i, 0.2));
	for(int i=170;i<250;i++) obs_left.push_back(Point(0.01*i, 0.0));
	for(int i=250;i<275;i++) obs_left.push_back(Point(0.01*i, 0.1));
	for(int i=275;i<300;i++) obs_left.push_back(Point(0.01*i, 0.0));
	//POPULATE RIGHT OBSTACLE TRACK
	for(int i=0; i<100; i++) obs_right.push_back(Point(0.01*i, 0.0));
	for(int i=100;i<110;i++) obs_right.push_back(Point(0.01*i, 0.15));
	for(int i=110;i<200;i++) obs_right.push_back(Point(0.01*i, 0.0));
	for(int i=200;i<215;i++) obs_right.push_back(Point(0.01*i, 0.07));
	for(int i=215;i<300;i++) obs_right.push_back(Point(0.01*i, 0.0));
	
	vector<Point> obs_window_left;
	vector<Point> obs_window_right;
	//std::cout<<"Started"<< std::endl;
	//std::vector<float> res = fs_simulator(obs_left, obs_right , -1, rear_foot_length+ front_foot_length);
	//std::cout<< "Selected foothold for leg "<< res[1] << " is: " << res[0]<<std::endl;
	
	
	
	//CONSTANT VARS
	float thigh_length = 0.5; 
	float shin_length = 0.5;
	float front_foot_length = 0.18;
	float rear_foot_length = 0.09;
	float knee_ang=15; 
	float step_time = 4.0;
	//DYNAMIC VARS
	bool no_obs_left=false;
	bool no_obs_right = false;
	int swing_leg= -1;
	std::vector<float> next_foothold;
	std::vector<std::vector<std::vector<Point>>> multi_step_bundle;
	std::vector<std::vector<Point>> single_step_bundle;
	float hip_z = 0.95* (thigh_length + shin_length); //initialization
	float hip_y=0.0;
	float lfoot_y=0.0;
	float rfoot_y=0.0;
	float cumulative_shift=0;
	int it=1;
	while(!(no_obs_left || no_obs_right)) { //there are still obstacles to be surpassed
	
	
		if(swing_leg==0 || swing_leg==-1)
			std::cout<<"Left obstacle track:" << std::endl;
			for(int i=0;i<obs_left.size();i++){
			if(obs_left[i].gety()>0) std::cout<< obs_left[i].getx() << ", "<< obs_left[i].gety() << ", ";
		}
	
		if(swing_leg==1 || swing_leg==-1)
			std::cout<<"Right obstacle track:" << std::endl;
			for(int i=0;i<obs_right.size();i++){
			if( obs_right[i].gety()>0) std::cout<< obs_right[i].getx() << ", "<< obs_right[i].gety() << ", ";
		}
		
		
		
		float pivot;
		if(swing_leg==0) pivot = rfoot_y;
		else if(swing_leg==1) pivot = lfoot_y;
		else pivot =0; //both legs aligned
		next_foothold=fs_simulator(obs_left, obs_right , swing_leg, rear_foot_length+ front_foot_length,pivot);
		if(swing_leg==-1) swing_leg = next_foothold[1];
		if(swing_leg ==0) {
			single_step_bundle = cfftg_sagittal(thigh_length,  shin_length,  rear_foot_length, front_foot_length, next_foothold[0], hip_z, rfoot_y, knee_ang, obs_left ,step_time);
			if(single_step_bundle[0][0].getx()==-1) {
				std::cout<<"CFFTG failed, stopping simulation."<<std::endl;
				break;
			}
			else{
				//update swing foot and hip positions
				lfoot_y+=next_foothold[0];
				hip_y= single_step_bundle[0][single_step_bundle[0].size()-1].getx();
				hip_z= single_step_bundle[0][single_step_bundle[0].size()-1].gety();
				if(rfoot_y>0) {
					float shift = rfoot_y;
					std::cout<<"Current displacement: " << shift<< std::endl;
					lfoot_y-=shift;
					rfoot_y-=shift; //could be direclty set as 0, written in this way for better understanding
					hip_y-=shift;
					cumulative_shift+=shift;
					no_obs_left= shift_obs_track(obs_left, shift);
					no_obs_right= shift_obs_track(obs_right, shift);
					shift_and_stitch_bundle(multi_step_bundle,single_step_bundle, cumulative_shift); //should fuse the trajectories
					 
				}
				swing_leg=1; //change leg
			}
		
		}
		else if(swing_leg==1) {
			single_step_bundle = cfftg_sagittal(thigh_length,  shin_length,  rear_foot_length, front_foot_length, next_foothold[0], hip_z, lfoot_y, knee_ang, obs_right ,step_time);
			if(single_step_bundle[0][0].getx()==-1) {
				std::cout<<"CFFTG failed, stopping simulation."<<std::endl;
				break;
			}
			else{
				//update swing foot and hip positions
				rfoot_y+=next_foothold[0];
				hip_y= single_step_bundle[0][single_step_bundle[0].size()-1].getx();
				hip_z= single_step_bundle[0][single_step_bundle[0].size()-1].gety();
				if(lfoot_y>0) {
					float shift = lfoot_y;
					std::cout<<"Current displacement: " << shift<< std::endl;
					lfoot_y-=shift;
					rfoot_y-=shift; //could be direclty set as 0, written in this way for better understanding
					hip_y-=shift;
					cumulative_shift+=shift;
					no_obs_left= shift_obs_track(obs_left, shift);
					no_obs_right= shift_obs_track(obs_right, shift);
					shift_and_stitch_bundle(multi_step_bundle,single_step_bundle, cumulative_shift); //should fuse the trajectories
					 
				}
				swing_leg=0; //change leg
			}
		
		
		
		}
		
		std::cout<<"Step number "<< it <<" completed successfully"<<std::endl; 
		it++;
		
	
	
	
	}
	
	
	
	//while(lfoot_y && rfoot_y < 3.0){ MAIN LOOP
	
	//}
	
	
	
 
 
}
