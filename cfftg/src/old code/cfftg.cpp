#include "utility.h"
using namespace std::chrono;


//CONSTRUCTOR
Cfftg::Cfftg(){

}


//TRAJECTORY GENERATION FUNCTION
//start point of swing foot is the origin
//ARGS DESCRIPTION:
//	end: endpoint of swing foot (can be increased during computation)
//	pivot: position of support foot
//      step_time: duration of the swing motion
//	obstacle_shape: function describing the obstacle in the sagittal plane (y-z)
std::vector<std::vector<Point>> Cfftg::trajectory_generator(float thigh_length, float shin_length, float knee_angle, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float step_time, std::vector<Point> obstacle_shape){ 

	//CREATE TIME VECTOR-------------------------------------------------------------------------------------------------------------------------------------------
	float time_unit = 0.1; //[s] 100 ms
	std::vector<float> t(ceil(step_time/time_unit)+1);
	for(int i=0; i< t.size(); i++){
		t[i]= i*time_unit;
	}
	
	//FIND OBSTACLE MAX HEIGHT TO PROVIDE REASONABLE INITIAL GUESS-------------------------------------------------------------------------------------------------
	float obstacle_max_h=0.0;
	for(int i=0; i< obstacle_shape.size();i++){
		float temp= obstacle_shape[i].gety();
		if(temp > obstacle_max_h) obstacle_max_h=temp;
	}
	
	//CFFTG VARS INITIALIZATION------------------------------------------------------------------------------------------------------------------------------------
	
	float default_sigma_h= 0.2;
	float default_sigma_v = 0.05;
	float sigma_h = default_sigma_h;
	float sigma_v = default_sigma_v;
	float h = end/2; //initial guess for h
	float v;
	float default_v;
	float default_h = end/2;
	if( obstacle_max_h <= 0.35){
		default_v= obstacle_max_h + 0.05; //initial guess for v
	}
	else default_v=0.4; //max step height capped at 40 cm
	v=default_v;
	h = default_h;
	float best_v = v;
	float best_h = h;
	float best_score=-100;
	int best_dist=10;
	int it=0;
	bool stop=false;
	bool success = true;
	float ihpy; //initial hip position along y-axis
	float ihpz = hip_height; //initial hip position along z-axis
	float fhpy; //final hip position along y-axis
	//float fhpz = hip_height; //final hip position along z-axis
	float fhpz = final_hip_height(end, pivot, thigh_length + shin_length );
	if(fhpz==-1) {
		std::cout<< "Step Length is too large. Aborting."<<std::endl;
		success=false;
		stop=true;
	
	}
	float s_l = end; //step length
	//TRAJECTORIES
	std::vector<Point> foot_ty; //foot trajectory along the time/y-axis (vector of 2D points)
	std::vector<float> foot_tyc; //coefficients of the cubic polynomial representing the foot trajectory along the time/y-axis
	std::vector<Point> foot_tz; //foot trajectory along the y/z-axis (vector of 2D points)
	std::vector<float> foot_tzc; //coefficients of the cubic polynomial representing the foot trajectory along the y/z-axis
	std::vector<Point> hip_ty; //hip trajectory along the time/y-axis (vector of 2D points)
	std::vector<float> hip_tyc; //coefficients of the cubic polynomial representing the hip trajectory along the time/y-axis
	std::vector<Point> hip_tz; //hip trajectory along the y/z-axis (vector of 2D points)
	std::vector<float> hip_tzc; //coefficients of the cubic polynomial representing the hip trajectory along the y/z-axis
	std::vector<Point> knee_tz(t.size()-1); //knee tip trajectory (y/z-axis)
	std::vector<Point> tip_tz(t.size()-1); //foot tip trajectory (y/z-axis)
	std::vector<Point> heel_tz(t.size()-1); //foot heel trajectory (y/z-axis)
	std::vector<Point> suppk_tz(t.size()-1); //support knee trajectory (y/z-axis)
	//WAYPOINT STRUCTURE
	std::vector<Point> wp;
	
	//MAIN LOOP----------------------------------------------------------------------------------------------------------------------------------------------------
	//SET UP TIMER TO CHECK EXECUTION TIME
	auto i_timer = high_resolution_clock::now();
	
	while(!stop){
		//FOOT TRAJECTORY GENERATION---------------------------------------------------------------------------------------------------------------------------
		//std::vector<Point> wp = {Point(0.0,0.0) , Point(step_time*(h/s_l), h), Point(step_time, s_l)};
		std::vector<float> wp_vel = {0.0, 0.05, 0.0};
		//foot_ty = piecewise_cubic_poly(wp, t, wp_vel);
		foot_ty = compute_linear_traj_2D(t, 0.0, s_l);
		wp = {Point(0.0,0.0) , Point(h, v), Point(s_l,0.0)};

		std::vector<float> temp_t(t.size()-1);
		for(int i=0;i<foot_ty.size();i++) temp_t[i] = foot_ty[i].gety();

		foot_tz= piecewise_cubic_poly(wp, temp_t, wp_vel);
		//HIP TRAJECTORY GENERATION----------------------------------------------------------------------------------------------------------------------------
		ihpy = pivot/2;
		fhpy = (s_l - pivot)/2 + pivot;
		//wp = {Point(0.0,ihpy) , Point(step_time*(pivot/s_l), pivot), Point(step_time,fhpy)};
		//hip_ty= piecewise_cubic_poly(wp, t, wp_vel);
		hip_ty = compute_linear_traj_2D(t, ihpy, fhpy);
		//CRANK-CONNECTING ROD CALCULATIONS
		float a= atan(sin(deg_to_rad(knee_angle))/ (cos(deg_to_rad(knee_angle)) + shin_length/thigh_length));
		float b= asin(shin_length/thigh_length * sin(a));
		float s= thigh_length + shin_length - (thigh_length*cos(b) + shin_length*cos(a));

		for(int i=0;i<foot_ty.size();i++) temp_t[i] = hip_ty[i].gety();
		if(pivot > 0.05) wp = {Point(ihpy,ihpz) , Point(pivot, thigh_length+shin_length-s), Point(fhpy,fhpz)};
		else wp = {Point(ihpy,ihpz) , Point(fhpy,fhpz)};
		hip_tz = piecewise_cubic_poly(wp, temp_t, wp_vel);
		//FOOT TRAJECTORY EVALUATION---------------------------------------------------------------------------------------------------------------------------
		
		
		int score=0;
		float min_dist=10;
		//EVALUATE TRAJECTORY
		for(int i=0;i<foot_ty.size();i++){
			//THE SECTION BELOW SHOULD BE PUT INTO A UTILITY FUNCTION (FORWARD KINEMATICS LEG)
			//FORWARD KINEMATICS OF SWING LEG
			float M = sqrt(pow(foot_tz[i].getx() - hip_tz[i].getx(),2)+ pow(foot_tz[i].gety() - hip_tz[i].gety(),2));
			if(M>thigh_length+shin_length && i!= foot_ty.size()-1){
				 std::cout<<"Kinematic constraint for swing leg not satisfied at time "<< i << std::endl;
				 score=-10;
				 break;
			}
			float alfa = acos((pow(thigh_length,2) + pow(M,2) - pow(shin_length,2))/(2*thigh_length*M));
			float tilt = asin((foot_tz[i].getx() - hip_tz[i].getx())/M);
			knee_tz[i] = Point( hip_tz[i].getx() + thigh_length * sin(alfa + tilt) , hip_tz[i].gety() - thigh_length * cos(alfa + tilt));
			float epsilon = asin(( foot_tz[i].getx() - knee_tz[i].getx())/shin_length);
			tip_tz[i] = Point(foot_tz[i].getx() + front_foot_length*cos(epsilon), foot_tz[i].gety() + front_foot_length*sin(epsilon));
			heel_tz[i] = Point(foot_tz[i].getx() - rear_foot_length*cos(epsilon), foot_tz[i].gety() - rear_foot_length*sin(epsilon));
			//FORWARD KINEMATICS OF SUPPORT LEG
			float M2 = sqrt(pow(pivot - hip_tz[i].getx(),2)+ pow(hip_tz[i].gety(),2));
			if(M2>thigh_length+shin_length && i!= foot_ty.size()-1){
				std::cout<<"Kinematic constraint for support leg not satisfied at time "<< i << std::endl;
				//score=-10;
				//break;
			}
			float alfa2 = acos((pow(thigh_length,2) + pow(M2,2) - pow(shin_length,2))/(2*thigh_length*M2));
			float tilt2 = asin((pivot - hip_tz[i].getx())/M2);
			suppk_tz[i] = Point( hip_tz[i].getx() + thigh_length * sin(alfa2 + tilt2) , hip_tz[i].gety() - thigh_length * cos(alfa2 + tilt2));
			//END OF FORWARD KINEMATICS
			
			//KINEMATIC CONSTRAINTS CHECK
			if(sqrt(pow(suppk_tz[i].getx() - hip_tz[i].getx(),2) + pow(suppk_tz[i].gety() - hip_tz[i].gety(),2)) > abs(thigh_length+0.001)) {
				std::cout<<"Kinematic constraint (thigh) for support leg not satisfied at time "<< i << std::endl;
				//score=-10;
				//break;
			}
			if(sqrt(pow(suppk_tz[i].getx() - pivot,2) + pow(suppk_tz[i].gety(),2)) > abs(shin_length+0.001)) {
				std::cout<<"Kinematic constraint (shin) for support leg not satisfied at time "<< i << std::endl;
				//score=-10;
				//break;
			}
			if(sqrt(pow(knee_tz[i].getx() - hip_tz[i].getx(),2) + pow(knee_tz[i].gety() - hip_tz[i].gety(),2)) > abs(thigh_length+0.001)) {
				std::cout<<"Kinematic constraint (thigh) for swing leg not satisfied at time "<< i << std::endl;
				//score=-10;
				//break;
			}
			if(sqrt(pow(knee_tz[i].getx() - foot_tz[i].getx(),2) + pow(knee_tz[i].gety() - foot_tz[i].gety(),2)) > abs(shin_length+0.001)) {
				std::cout<<"Kinematic constraint (shin) for swing leg not satisfied at time "<< i << std::endl;
				//score=-10;
				//break;
			}	
			
			//CALCULATE FOOT LINE COEFFICIENTS FOR COLLISION DETECTION (y=m*x+q) 
			float m = (tip_tz[i].gety()- foot_tz[i].gety()) / (tip_tz[i].getx()- foot_tz[i].getx());
			float q =  foot_tz[i].gety() - m*foot_tz[i].getx();
			float b1,b2; //bounds
			//std::cout << "Coeffs: M1= " << m1 << ", Q1= " << q1 << ", M2= " << m2 << ", Q2= " <<q2 << std::endl;
			
			if(tip_tz[i].getx()>heel_tz[i].getx()){
				b1= heel_tz[i].getx();
				b2= tip_tz[i].getx();
			}
			else{
				b2= heel_tz[i].getx();
				b1= tip_tz[i].getx();
			}
			int temp_score=0;
			for(int j=0;j<obstacle_shape.size();j++){
				if(obstacle_shape[j].gety()>0){
					if(temp_score==0) {
						float dot = (obstacle_shape[j].getx()-heel_tz[i].getx()) * (tip_tz[i].getx()-heel_tz[i].getx()) + (obstacle_shape[j].gety()-heel_tz[i].gety())*(tip_tz[i].gety()-heel_tz[i].gety()) / pow(front_foot_length + rear_foot_length,2);
						if(dot>1) dot =1;
						if(dot<0) dot=0;
						Point proj = Point(heel_tz[i].getx() + dot*(tip_tz[i].getx()-heel_tz[i].getx()), heel_tz[i].gety() + dot*(tip_tz[i].gety()-heel_tz[i].gety()));
						float dist = sqrt(pow(proj.getx()-obstacle_shape[j].getx(),2) + pow(proj.gety()-obstacle_shape[j].gety(),2));
						if(dist< min_dist) min_dist=dist;
					}
					if(b1<obstacle_shape[j].getx() && obstacle_shape[j].getx()<b2 && obstacle_shape[j].gety()> m*obstacle_shape[j].getx() + q) {
						temp_score-=1;
						//if(temp_score<-5) break;
					}
				
				}
			
			}
			if (temp_score<score) score=temp_score;
			
			
		}
		
		std::cout << "Score for trajectory n. " << it << " with h= " << h << ", v= " << v << ", s_l= " <<s_l <<" min dist= "<< min_dist <<" is: " << score << std::endl;
	  	if(score == 0 && min_dist>0.03) stop=true;
	  	if(!stop) {
		  	if(score > best_score && best_score!=-100 || score==0 && min_dist<best_dist){
		  		best_h=h;
		  		best_v=v;
		  		best_score=score;
		  		if(score==0) best_dist=min_dist;
		  		if(sigma_h > 0.05) sigma_h = sigma_h/2;
		  		if(sigma_v > 0.01) sigma_v = sigma_v/2;
		  		//it=0;	
		  	}
		  	do{
		  		//std::cout << "Generating new values for h " << std::endl;
		  		h=gen_gaussian(best_h,sigma_h);
		  	}
		  	while(h<ihpy || h>fhpy);
		  	do{
		  		//std::cout << "Generating new values for v " << std::endl;
		  		v=gen_gaussian(best_v,sigma_v);
		  	}
		  	while( v <= obstacle_max_h || v > 0.4);
		  	
		  	
		  	if(it%100==0){
		  		//v=default_v;
				//h = default_h;
		  		sigma_h = default_sigma_h;
		  		sigma_v = default_sigma_v;
		  	
		  	}
		  	if(it>1000) {
		  		stop=true;
		  		success = false;
		  		std::cout<< "Solution not found" <<std::endl;
		  	}
		it++;
		}
	}
	//CALCULATE EXECUTION TIME
	auto f_timer = high_resolution_clock::now();
	auto exec_time = duration_cast<microseconds>(f_timer - i_timer);
  	std::cout<<"Total time: " << exec_time.count()/1000 << " milliseconds" << std::endl;
  	std::vector<std::vector<Point>> bundle;
  	if(!success){
  		Point fail_indicator(-1,-1);
  		bundle.push_back(std::vector<Point> {fail_indicator});
  	}
	else bundle = {hip_tz,suppk_tz,knee_tz,foot_tz,tip_tz,heel_tz};
	return bundle;
}
