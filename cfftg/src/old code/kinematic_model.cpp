#include "utility.h"

//VARIABLES

float thigh_length=0.5;
float shin_length=0.5;
float foot_length=0.25;
float ds_pelvis_height = (0.95)*(shin_length + thigh_length);


using namespace std;
//This function is meant to return hip trajectory vector and left/right knee trajectory vectors (Also foot tip trajectory optionally).
//NOTE: ds_pelvis_height stands for double support pelvis height
vector<vector<Point>> kinematic_model_evaluation(float ds_foot_pos,int knee_angle, vector<Point> foot_traj){

	float i_hip_pos= ds_foot_pos - ((ds_foot_pos - foot_traj[0].getx())/2);
	float f_hip_pos= (foot_traj[foot_traj.size()-1].getx()) - ((foot_traj[foot_traj.size()-1].getx() -ds_foot_pos)/2);
	//vector<float> hip_traj_coeffs(4);
	//vector<Point> hip_traj(foot_traj.size()); //MAYBE THE LENGTH OF VECTORS HAS TO BE CHANGED
	vector<Point> dsk_traj(foot_traj.size()); //double support leg knee trajectory
	vector<Point> sk_traj(foot_traj.size()); //swing leg knee trajectory
	vector<Point> foot_tip_traj(foot_traj.size()); //foot tip trajectory (used in visualization to understand if collision happens)


	//CRANCK-CONNECTING ROD CALCULATIONS
	float a= atan(sin(deg_to_rad(knee_angle))/ (cos(deg_to_rad(knee_angle)) + shin_length/thigh_length));
	float b= asin(shin_length/thigh_length * sin(a));
	float s= thigh_length + shin_length - (thigh_length*cos(b) + shin_length*cos(a));



	vector<float> hip_traj_coeffs=cubic_traj_gen(i_hip_pos, ds_pelvis_height, ds_foot_pos, thigh_length+shin_length-s, f_hip_pos, ds_pelvis_height); //GIUSTO
	//vector<float> hip_traj_coeffs=cubic_traj_gen(i_hip_pos, ds_pelvis_height, ds_foot_pos, 0.85, f_hip_pos, ds_pelvis_height); //TEST
	vector<float> x_h(foot_traj.size());
	x_h[0] = i_hip_pos;
	float h_scale = (f_hip_pos - i_hip_pos)/x_h.size();
	for(int i=1; i < foot_traj.size(); i++){
		x_h[i]= x_h[i-1] + h_scale;

	}
	vector<Point> hip_traj = compute_traj(x_h, hip_traj_coeffs);

	for(int i=0; i < hip_traj.size();i++){
		//float L3 = sqrt(pow(shin_length,2) + pow(foot_length,2));
		//float knee_tilt = asin(foot_length/L3); 
		float M = sqrt(pow(foot_traj[i].getx() - hip_traj[i].getx(),2)+ pow(foot_traj[i].gety() - hip_traj[i].gety(),2)); //dist hip to swing heel
		float M2 = sqrt(pow(ds_foot_pos - hip_traj[i].getx(),2) + pow(hip_traj[i].gety(),2)); //dist hip to support heel
		float alfa = acos((pow(thigh_length,2) + pow(M,2) - pow(shin_length,2))/(2*thigh_length*M));
		float alfa2 = acos((pow(thigh_length,2) + pow(M2,2) - pow(shin_length,2))/(2*thigh_length*M2));
		
		float tilt = asin((foot_traj[i].getx() - hip_traj[i].getx())/M);
		float tilt2 = asin((ds_foot_pos - hip_traj[i].getx())/M2);
		
		sk_traj[i] = Point( hip_traj[i].getx() + thigh_length * sin(alfa + tilt) , hip_traj[i].gety() - thigh_length * cos(alfa + tilt));
		dsk_traj[i] = Point( hip_traj[i].getx() + thigh_length * sin(alfa2 + tilt2) , hip_traj[i].gety() - thigh_length * cos(alfa2 + tilt2));
		
		float epsilon = asin(-( foot_traj[i].gety() - sk_traj[i].gety())/shin_length);
		foot_tip_traj[i] = Point(foot_traj[i].getx() + foot_length*cos(epsilon - M_PI/2), foot_traj[i].gety() + foot_length*sin(epsilon - M_PI/2));
	}
	vector<vector<Point>> bundle = {hip_traj, sk_traj, dsk_traj,foot_traj, foot_tip_traj};
	return bundle;

}
