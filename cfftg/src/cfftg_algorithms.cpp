#include "cfftg_algorithms.h"
using namespace std::chrono;

//LINEAR CFFTG---------------------------------------------------------------------------------------------------------------


std::vector<std::vector<Point>> cfftg_linear(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it, int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file, std::string txt_file_name){

	//CFFTG VARS INITIALIZATION------------------------------------------------------------------------------------------------------------------------------------
	float h,v; //horizontal (y-axis) and vertical (z-axis) position of the foot trajectory peak
	float default_h = end/2;
	float default_v;
	//FIND OBSTACLE MAX HEIGHT TO PROVIDE REASONABLE INITIAL GUESS FOR V -------------------------------------------------------------------------------------------------
	float obstacle_max_h=0.0;
	for(int i=0; i< obstacle_shape.size();i++){
		float temp= obstacle_shape[i].gety();
		if(temp > obstacle_max_h && obstacle_shape[i].getx()<end) obstacle_max_h=temp;
	}
	default_v= obstacle_max_h + 0.1; //initial guess for v
	v=default_v;
	h = default_h;
	float default_sigma_h= default_sh;
	float default_sigma_v = default_sv;
	float sigma_h = default_sigma_h;
	float sigma_v = default_sigma_v;
	float best_v = v;
	float best_h = h;
	float score;
	float best_score=-100;
	float min_dist;
	int best_dist=10;
	int it=0;
	bool stop=false;
	bool success = true;
	//TXT FILE OPENING
	std::ofstream outfile;
	if(produce_txt_file){
		outfile.open(txt_file_name + ".txt");
	}
	//TRAJECTORIES
	std::vector<float> t = create_time_vector(time_unit, step_time); //time vector
	std::vector<Point> foot_traj; //foot trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> hip_traj; //hip trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> knee_traj(t.size()); //knee tip trajectory (y/z-axis)
	std::vector<Point> tip_traj(t.size()); //foot tip trajectory (y/z-axis)
	std::vector<Point> heel_traj(t.size()); //foot heel trajectory (y/z-axis)
	std::vector<Point> supp_knee_traj(t.size()); //support knee trajectory (y/z-axis)
	//WAYPOINT STRUCTURE
	std::vector<Point> wp;
	std::vector<float> wp_vel_z;
	//HIP TRAJECTORY GENERATION---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	float ihpy = pivot/2; //initial hip position along y-axis. Can be change according to the results of the odometry module
	float ihpz = hip_height; //initial hip position along z-axis
	float fhpy=(end - pivot)/2 + pivot;  //final hip position along y-axis
	float fhpz = final_hip_height(end, pivot, thigh_length + shin_length); //final hip position along z-axis
	if(fhpz==-1) {
		std::cout<< "Step Length is too large. Aborting."<<std::endl;
		success=false;
		stop=true;
	
	}
	//CRANK-CONNECTING ROD CALCULATIONS
	float a= atan(sin(deg_to_rad(knee_angle))/ (cos(deg_to_rad(knee_angle)) + shin_length/thigh_length));
	float b= asin(shin_length/thigh_length * sin(a));
	float s= thigh_length + shin_length - (thigh_length*cos(b) + shin_length*cos(a));
	if(pivot>0.05) {
		wp = {Point(ihpy,ihpz), Point(pivot,thigh_length+shin_length-s), Point(fhpy,fhpz)};
		wp_vel_z = {0,0.05,0};
	}
	else {
		wp = {Point(ihpy,ihpz), Point(fhpy,fhpz)};
		wp_vel_z = {0,0};
	}
	hip_traj= generate_sagittal_trajectory(t, wp, "linear", "cubic", wp_vel_z);
	//CFFTG CYCLE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	wp_vel_z={0,0.05,0}; //reset velocities (will be fixed inside the cycle)
	auto i_timer = high_resolution_clock::now();//SET UP TIMER TO CHECK EXECUTION TIME
	while(!stop) {
		wp={Point(0,0),Point(h,v),Point(end,0)};
		std::cout<<"Generate foot traj for h: "<< h<<",  v: "<< v <<std::endl;
		foot_traj=  generate_sagittal_trajectory(t, wp, "linear", "cubic", wp_vel_z);
		score=0;
		min_dist=10;
		for(int i=0;i<foot_traj.size();i++) {
			std::vector<Point> swing_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], foot_traj[i], false);
			if(swing_leg.size()==1){
				score=-50; //kinematic constraint not satisfied
				break;
			
			}
			else {
				knee_traj[i] = swing_leg[0];
				tip_traj[i] = swing_leg[1];
				heel_traj[i] =swing_leg[2];
				std::vector<Point> support_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], Point(pivot,0), true);
				if(support_leg[0].getx()!=-1) supp_knee_traj[i] = support_leg[0];
				else{
					break;
				}
				std::vector<float> res = intersection_score_and_min_dist(obstacle_shape, heel_traj[i], tip_traj[i]); //res[0] = score , res[1]= min_dist (not calculated if res[0]<0)
				if(res[0]<score) score=res[0];
				if(res[1]<min_dist) min_dist = res[1];			
			}
			//UPDATE TEXT FILE
			if(produce_txt_file && i<hip_traj.size()){ //Sometimes hip traj has 1 point less than the others
				outfile << it << " ";
				outfile << hip_traj[i].getx() << " " << hip_traj[i].gety() << " ";
				outfile << supp_knee_traj[i].getx() << " " << supp_knee_traj[i].gety() << " ";
				outfile << knee_traj[i].getx() << " " << knee_traj[i].gety() << " ";
				outfile << foot_traj[i].getx() << " " << foot_traj[i].gety() << " ";
				outfile << heel_traj[i].getx() << " " << heel_traj[i].gety() << " ";
				outfile << tip_traj[i].getx() << " " << tip_traj[i].gety() << " ";
				outfile << end << " " << pivot << " " << h << " " << v << std::endl;  
		
			}	
		}		
		std::cout << "Score for trajectory n. " << it << " with h= " << h << ", v= " << v << ", step length= " <<end <<" min dist= "<< min_dist <<" is: " << score << std::endl;
		//std::cout<<"Update score"<<std::endl;
	  	if(score == 0 && min_dist>0.03) stop=true;
	  	else{
	  		if(score < 0) {
	  			if(score>best_score) {
	  				if(best_score!=-100) {
	  					best_h=h;
		  				best_v=v;
		  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
		  				if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
	  				}
	  				best_score=score;
	  			}
	  		}
	  		else if (score==0) {
	  			if(min_dist<best_dist) {
	  				best_dist=min_dist;
	  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
		  			if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
	  			}
	  		}
	  		//std::cout<<"Update R.V.s"<<std::endl;
	  		do{
		  		h=gen_gaussian(best_h,sigma_h);
		  	}
		  	while(h<0 || h<ihpy || h>fhpy);
		  	do{
		  		v=gen_gaussian(best_v,sigma_v);
		  	}
		  	while( v <= obstacle_max_h || v > 0.4);
		  	if(it%restart_it==0){
		  		//v=default_v;
				//h = default_h;
				//std::cout<<"Reset R.V.s"<<std::endl;
		  		sigma_h = default_sigma_h;
		  		sigma_v = default_sigma_v;
		  	}
		  	if(it>max_it) {
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
  	//CLOSE TXT FILE
  	if(produce_txt_file) {
  		outfile.close();
  	}
  	//CHECK SUCCESS AND RETURN BUNDLE OF TRAJECTORIES
  	std::vector<std::vector<Point>> bundle;
  	if(!success){
  		std::cout<< "Solution not found" <<std::endl;
  		Point fail_indicator(-1,-1);
  		bundle.push_back(std::vector<Point> {fail_indicator});
  	}
	else bundle = {hip_traj,supp_knee_traj,knee_traj,foot_traj,tip_traj,heel_traj};
	return bundle;


}

//------------------------ CUBIC HIP / LINEAR FOOT CFFTG----------------------------------------------------------------------
std::vector<std::vector<Point>> cfftg_linear_foot_cubic_hip(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, float min_dist_th, float hip_max_extension_time_coeff, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it, int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file, std::string txt_file_name){

	//CFFTG VARS INITIALIZATION------------------------------------------------------------------------------------------------------------------------------------
	float h,v; //horizontal (y-axis) and vertical (z-axis) position of the foot trajectory peak
	float default_h = obstacle_shape[0].getx();
	float default_v;
	//FIND OBSTACLE MAX HEIGHT TO PROVIDE REASONABLE INITIAL GUESS FOR V -------------------------------------------------------------------------------------------------
	float obstacle_max_h=0.0;
	for(int i=0; i< obstacle_shape.size();i++){
		float temp= obstacle_shape[i].gety();
		if(temp > obstacle_max_h && obstacle_shape[i].getx()<end) obstacle_max_h=temp;
	}
	default_v= obstacle_max_h + 0.1; //initial guess for v
	v=default_v;
	h = default_h;
	float default_sigma_h= default_sh;
	float default_sigma_v = default_sv;
	float sigma_h = default_sigma_h;
	float sigma_v = default_sigma_v;
	float best_v = v;
	float best_h = h;
	float score;
	float best_score=-100;
	float min_dist;
	int best_dist=10;
	int it=0;
	bool stop=false;
	bool success = true;
	//TXT FILE OPENING
	std::ofstream outfile;
	if(produce_txt_file){
		outfile.open(txt_file_name + ".txt");
	}
	//TRAJECTORIES
	std::vector<float> t = create_time_vector(time_unit, step_time); //time vector
	std::vector<Point> foot_traj; //foot trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> hip_traj; //hip trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> knee_traj(t.size()); //knee tip trajectory (y/z-axis)
	std::vector<Point> tip_traj(t.size()); //foot tip trajectory (y/z-axis)
	std::vector<Point> heel_traj(t.size()); //foot heel trajectory (y/z-axis)
	std::vector<Point> supp_knee_traj(t.size()); //support knee trajectory (y/z-axis)
	//WAYPOINT STRUCTURE
	std::vector<Point> wp; //foot wp
	std::vector<Point> wp_hip_y;
	std::vector<float> wp_vel_y;
	std::vector<float> wp_vel_z;
	//HIP TRAJECTORY GENERATION---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	float ihpy = pivot/2; //initial hip position along y-axis. Can be change according to the results of the odometry module
	float ihpz = hip_height; //initial hip position along z-axis
	float fhpy=(end - pivot)/2 + pivot;  //final hip position along y-axis
	float fhpz = final_hip_height(end, pivot, thigh_length + shin_length); //final hip position along z-axis
	if(fhpz==-1) {
		std::cout<< "Step Length is too large. Aborting."<<std::endl;
		success=false;
		stop=true;
	
	}
	//CRANK-CONNECTING ROD CALCULATIONS
	float a= atan(sin(deg_to_rad(knee_angle))/ (cos(deg_to_rad(knee_angle)) + shin_length/thigh_length));
	float b= asin(shin_length/thigh_length * sin(a));
	float s= thigh_length + shin_length - (thigh_length*cos(b) + shin_length*cos(a));
	if(pivot>0.05) {
		wp = {Point(ihpy,ihpz), Point(pivot,thigh_length+shin_length-s), Point(fhpy,fhpz)};
		wp_vel_z = {0,0.05,0};
	}
	else {
		wp = {Point(ihpy,ihpz), Point(fhpy,fhpz)};
		wp_vel_z = {0,0};
	}
	wp_hip_y ={Point(t[0],ihpy), Point(t[static_cast<int>(t.size()*hip_max_extension_time_coeff)],pivot), Point(t[t.size()-1],fhpy)};
	wp_vel_y = {0,0,0};
	hip_traj= generate_sagittal_trajectory_v2(t, "cubic", "cubic",wp, wp_vel_z, wp_hip_y, wp_vel_y);
	while(hip_traj.size()<t.size()) hip_traj.push_back(hip_traj[hip_traj.size()-1]);
	//CFFTG CYCLE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	wp_vel_z={0,0.05,0}; //reset velocities (will be fixed inside the cycle)
	auto i_timer = high_resolution_clock::now();//SET UP TIMER TO CHECK EXECUTION TIME
	while(!stop) {
		wp={Point(0,0),Point(h,v),Point(end,0)};
		std::cout<<"Generate foot traj for h: "<< h<<",  v: "<< v <<std::endl;
		foot_traj=  generate_sagittal_trajectory(t, wp, "linear", "cubic", wp_vel_z);
		score=0;
		min_dist=10;
		for(int i=0;i<foot_traj.size();i++) {
			std::vector<Point> swing_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], foot_traj[i], false);
			if(swing_leg.size()==1){
				score=-50; //kinematic constraint not satisfied
				//break;
			
			}
			else {
				knee_traj[i] = swing_leg[0];
				tip_traj[i] = swing_leg[1];
				heel_traj[i] =swing_leg[2];
				std::vector<Point> support_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], Point(pivot,0), true);
				if(support_leg[0].getx()!=-1) supp_knee_traj[i] = support_leg[0];
				else{
					score=-50;
				}
				std::vector<float> res = intersection_score_and_min_dist_v2(obstacle_shape, heel_traj[i], tip_traj[i]); //res[0] = score , res[1]= min_dist (not calculated if res[0]<0)
				if(res[0]<score) score=res[0];
				if(res[1]<min_dist) min_dist = res[1];			
			}
			//UPDATE TEXT FILE
			if(produce_txt_file && i<hip_traj.size()){ //Sometimes hip traj has 1 point less than the others
				outfile << it << " ";
				outfile << hip_traj[i].getx() << " " << hip_traj[i].gety() << " ";
				outfile << supp_knee_traj[i].getx() << " " << supp_knee_traj[i].gety() << " ";
				outfile << knee_traj[i].getx() << " " << knee_traj[i].gety() << " ";
				outfile << foot_traj[i].getx() << " " << foot_traj[i].gety() << " ";
				outfile << heel_traj[i].getx() << " " << heel_traj[i].gety() << " ";
				outfile << tip_traj[i].getx() << " " << tip_traj[i].gety() << " ";
				outfile << end << " " << pivot << " " << h << " " << v << std::endl;  
		
			}	
		}		
		std::cout << "Score for trajectory n. " << it << " with h= " << h << ", v= " << v << ", step length= " <<end <<" min dist= "<< min_dist <<" is: " << score << std::endl;
		//std::cout<<"Update score"<<std::endl;
	  	if(score == 0 && min_dist>min_dist_th) stop=true;
	  	else{
	  		if(score < 0) {
	  			if(score>best_score) {
	  				if(best_score!=-100 && it>10) {
	  					best_h=h;
		  				best_v=v;
		  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
		  				if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
	  				}
	  				best_score=score;
	  			}
	  		}
	  		else if (score==0) {
	  			if(min_dist<best_dist) {
	  				best_dist=min_dist;
	  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
		  			if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
	  			}
	  		}
	  		//std::cout<<"Update R.V.s"<<std::endl;
	  		do{
		  		h=gen_gaussian(best_h,sigma_h);
		  	}
		  	while(h<0 || h<ihpy || h>end); 
		  	do{
		  		v=gen_gaussian(best_v,sigma_v);
		  	}
		  	while( v <= obstacle_max_h || v > 0.4);
		  	if(it%restart_it==0){
		  		//v=default_v;
				//h = default_h;
				best_score=-10;
				//std::cout<<"Reset R.V.s"<<std::endl;
		  		sigma_h = default_sigma_h;
		  		sigma_v = default_sigma_v;
		  	}
		  	if(it>max_it) {
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
  	//CLOSE TXT FILE
  	if(produce_txt_file) {
  		outfile.close();
  	}
  	//CHECK SUCCESS AND RETURN BUNDLE OF TRAJECTORIES
  	std::vector<std::vector<Point>> bundle;
  	if(!success){
  		std::cout<< "Solution not found" <<std::endl;
  		Point fail_indicator(-1,-1);
  		bundle.push_back(std::vector<Point> {fail_indicator});
  	}
	else bundle = {hip_traj,supp_knee_traj,knee_traj,foot_traj,tip_traj,heel_traj};
	return bundle;


}

//--------------------------------FULL CUBIC CFFTG--------------------------------------------------------------------------------

std::vector<std::vector<Point>> cfftg_cubic(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, float min_dist_th, float hip_max_extension_time_coeff, float foot_peak_time_coeff, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it, int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file, std::string txt_file_name){

	//CFFTG VARS INITIALIZATION------------------------------------------------------------------------------------------------------------------------------------
	float h,v; //horizontal (y-axis) and vertical (z-axis) position of the foot trajectory peak
	
	float default_h,default_v;
	if(obstacle_shape.size()>0 && obstacle_shape[0].getx()<end) default_h=obstacle_shape[0].getx();
	else default_h = end/2;

	//FIND OBSTACLE MAX HEIGHT TO PROVIDE REASONABLE INITIAL GUESS FOR V -------------------------------------------------------------------------------------------------
	float obstacle_max_h=0.0;
	for(int i=0; i< obstacle_shape.size();i++){
		float temp= obstacle_shape[i].gety();
		if(temp > obstacle_max_h && obstacle_shape[i].getx()<end) obstacle_max_h=temp;
	}
	default_v= obstacle_max_h + 0.1; //initial guess for v
	v=default_v;
	h = default_h;
	float default_sigma_h= default_sh;
	float default_sigma_v = default_sv;
	float sigma_h = default_sigma_h;
	float sigma_v = default_sigma_v;
	float best_v = v;
	float best_h = h;
	float score;
	float best_score=-100;
	float min_dist;
	int best_dist=10;
	int it=0;
	bool stop=false;
	bool success = true;
	//TXT FILE OPENING
	std::ofstream outfile;
	if(produce_txt_file){
		outfile.open(txt_file_name + ".txt");
	}
	//TRAJECTORIES
	std::vector<float> t = create_time_vector(time_unit, step_time); //time vector
	std::vector<Point> foot_traj; //foot trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> hip_traj; //hip trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> knee_traj(t.size()); //knee tip trajectory (y/z-axis)
	std::vector<Point> tip_traj(t.size()); //foot tip trajectory (y/z-axis)
	std::vector<Point> heel_traj(t.size()); //foot heel trajectory (y/z-axis)
	std::vector<Point> supp_knee_traj(t.size()); //support knee trajectory (y/z-axis)
	//WAYPOINT STRUCTURE
	std::vector<Point> wp_z; 
	std::vector<Point> wp_y;
	std::vector<float> wp_vel_y;
	std::vector<float> wp_vel_z;
	//HIP TRAJECTORY GENERATION---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	float ihpy = pivot/2; //initial hip position along y-axis. Can be change according to the results of the odometry module
	float ihpz = hip_height; //initial hip position along z-axis
	float fhpy=(end - pivot)/2 + pivot;  //final hip position along y-axis
	float fhpz = final_hip_height(end, pivot, thigh_length + shin_length); //final hip position along z-axis
	if(fhpz==-1) {
		std::cout<< "Step Length is too large. Aborting."<<std::endl;
		success=false;
		stop=true;
	
	}
	//CRANK-CONNECTING ROD CALCULATIONS
	float a= atan(sin(deg_to_rad(knee_angle))/ (cos(deg_to_rad(knee_angle)) + shin_length/thigh_length));
	float b= asin(shin_length/thigh_length * sin(a));
	float s= thigh_length + shin_length - (thigh_length*cos(b) + shin_length*cos(a));
	if(pivot>0.05) {
		wp_z = {Point(ihpy,ihpz), Point(pivot,thigh_length+shin_length-s), Point(fhpy,fhpz)};
		wp_vel_z = {0,0,0};
	}
	else {
		wp_z = {Point(ihpy,ihpz), Point(fhpy,fhpz)};
		wp_vel_z = {0,0};
	}
	wp_y ={Point(t[0],ihpy), Point(t[static_cast<int>(t.size()*hip_max_extension_time_coeff)],pivot), Point(t[t.size()-1],fhpy)};
	wp_vel_y = {0,0.05,0};
	hip_traj= generate_sagittal_trajectory_v2(t, "cubic", "cubic",wp_z, wp_vel_z, wp_y, wp_vel_y);
	while(hip_traj.size()<t.size()) hip_traj.push_back(hip_traj[hip_traj.size()-1]);
	//CFFTG CYCLE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	wp_vel_z={1,0,-1}; //reset velocities (will be fixed inside the cycle)
	auto i_timer = high_resolution_clock::now();//SET UP TIMER TO CHECK EXECUTION TIME
	while(!stop) {
		wp_y = {Point(t[0],0), Point(t[static_cast<int>(t.size()*foot_peak_time_coeff)],h), Point(t[t.size()-1],end)};
		wp_vel_y = {0,0.1,0};
		wp_z={Point(0,0),Point(h,v),Point(end,0)};
		std::cout<<"Generate foot traj for h: "<< h<<",  v: "<< v <<std::endl;
		foot_traj=  generate_sagittal_trajectory_v2(t, "cubic", "cubic",wp_z, wp_vel_z, wp_y, wp_vel_y);
		while(foot_traj.size()<t.size()) foot_traj.push_back(foot_traj[foot_traj.size()-1]);
		score=0;
		min_dist=10;
		for(int i=0;i<foot_traj.size();i++) {
			std::vector<Point> swing_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], foot_traj[i], false);
			if(swing_leg.size()==1){
				score=-50; //kinematic constraint not satisfied
				//break;
			
			}
			else {
				knee_traj[i] = swing_leg[0];
				tip_traj[i] = swing_leg[1];
				heel_traj[i] =swing_leg[2];
				std::vector<Point> support_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], Point(pivot,0), true);
				if(support_leg[0].getx()!=-1) supp_knee_traj[i] = support_leg[0];
				else{
					score=-50;
				}
				std::vector<float> res = intersection_score_and_min_dist_v2(obstacle_shape, heel_traj[i], tip_traj[i]); //res[0] = score , res[1]= min_dist (not calculated if res[0]<0)
				if(res[0]<score) score=res[0];
				if(res[1]<min_dist) min_dist = res[1];			
			}
			//UPDATE TEXT FILE
			if(produce_txt_file && i<hip_traj.size()){ //Sometimes hip traj has 1 point less than the others
				outfile << it << " ";
				outfile << hip_traj[i].getx() << " " << hip_traj[i].gety() << " ";
				outfile << supp_knee_traj[i].getx() << " " << supp_knee_traj[i].gety() << " ";
				outfile << knee_traj[i].getx() << " " << knee_traj[i].gety() << " ";
				outfile << foot_traj[i].getx() << " " << foot_traj[i].gety() << " ";
				outfile << heel_traj[i].getx() << " " << heel_traj[i].gety() << " ";
				outfile << tip_traj[i].getx() << " " << tip_traj[i].gety() << " ";
				outfile << end << " " << pivot << " " << h << " " << v << std::endl;  
		
			}	
		}		
		std::cout << "Score for trajectory n. " << it << " with h= " << h << ", v= " << v << ", step length= " <<end <<" min dist= "<< min_dist <<" is: " << score << std::endl;
		//std::cout<<"Update score"<<std::endl;
	  	if(score == 0 && min_dist>min_dist_th) stop=true;
	  	else{
	  		if(score < 0) {
	  			if(score>best_score) {
	  				if(best_score!=-100 && it>10) {
	  					best_h=h;
		  				best_v=v;
		  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
		  				if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
	  				}
	  				best_score=score;
	  			}
	  		}
	  		else if (score==0) {
	  			if(min_dist<best_dist) {
	  				best_dist=min_dist;
	  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
		  			if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
	  			}
	  		}
	  		//std::cout<<"Update R.V.s"<<std::endl;
	  		do{
		  		h=gen_gaussian(best_h,sigma_h);
		  	}
		  	while(h<0 || h<ihpy || h>end); 
		  	do{
		  		v=gen_gaussian(best_v,sigma_v);
		  	}
		  	while( v <= obstacle_max_h || v > 0.4);
		  	if(it%restart_it==0){
		  		//v=default_v;
				//h = default_h;
				best_score=-10;
				//std::cout<<"Reset R.V.s"<<std::endl;
		  		sigma_h = default_sigma_h;
		  		sigma_v = default_sigma_v;
		  	}
		  	if(it>max_it) {
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
  	//CLOSE TXT FILE
  	if(produce_txt_file) {
  		outfile.close();
  	}
  	//CHECK SUCCESS AND RETURN BUNDLE OF TRAJECTORIES
  	std::vector<std::vector<Point>> bundle;
  	if(!success){
  		std::cout<< "Solution not found" <<std::endl;
  		Point fail_indicator(-1,-1);
  		bundle.push_back(std::vector<Point> {fail_indicator});
  	}
	else bundle = {hip_traj,supp_knee_traj,knee_traj,foot_traj,tip_traj,heel_traj};
	return bundle;


}









//--------------------------------FULL CUBIC CFFTG WITH REFINED SCORE--------------------------------------------------------------------------------

std::vector<std::vector<Point>> cfftg_cubic_v2(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, float min_dist_th, float hip_max_extension_time_coeff, float foot_peak_time_coeff, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it, int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file, std::string txt_file_name){

	//CFFTG VARS INITIALIZATION------------------------------------------------------------------------------------------------------------------------------------
	float h,v; //horizontal (y-axis) and vertical (z-axis) position of the foot trajectory peak
	
	float default_h,default_v;
	if(obstacle_shape.size()>0 && obstacle_shape[0].getx()<end) default_h=obstacle_shape[0].getx();
	else default_h = end/2;

	//FIND OBSTACLE MAX HEIGHT TO PROVIDE REASONABLE INITIAL GUESS FOR V -------------------------------------------------------------------------------------------------
	float obstacle_max_h=0.0;
	for(int i=0; i< obstacle_shape.size();i++){
		float temp= obstacle_shape[i].gety();
		if(temp > obstacle_max_h && obstacle_shape[i].getx()<end) obstacle_max_h=temp;
	}
	default_v= obstacle_max_h + 0.15; //initial guess for v
	v=default_v;
	h = default_h;
	float default_sigma_h= default_sh;
	float default_sigma_v = default_sv;
	float sigma_h = default_sigma_h;
	float sigma_v = default_sigma_v;
	float best_v = v;
	float best_h = h;
	float score;
	float best_score=-1000;
	float min_dist;
	int best_dist=10;
	int it=0;
	bool stop=false;
	bool success = true;
	//TXT FILE OPENING
	std::ofstream outfile;
	if(produce_txt_file){
		outfile.open(txt_file_name + ".txt");
	}
	//TRAJECTORIES
	std::vector<float> t = create_time_vector(time_unit, step_time); //time vector
	std::vector<Point> foot_traj; //foot trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> hip_traj; //hip trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> knee_traj(t.size()); //knee tip trajectory (y/z-axis)
	std::vector<Point> tip_traj(t.size()); //foot tip trajectory (y/z-axis)
	std::vector<Point> heel_traj(t.size()); //foot heel trajectory (y/z-axis)
	std::vector<Point> supp_knee_traj(t.size()); //support knee trajectory (y/z-axis)
	//WAYPOINT STRUCTURE
	std::vector<Point> wp_z; 
	std::vector<Point> wp_y;
	std::vector<float> wp_vel_y;
	std::vector<float> wp_vel_z;
	//HIP TRAJECTORY GENERATION---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	float ihpy = pivot/2; //initial hip position along y-axis. Can be change according to the results of the odometry module
	float ihpz = hip_height; //initial hip position along z-axis
	float fhpy=(end - pivot)/2 + pivot;  //final hip position along y-axis
	float fhpz = final_hip_height(end, pivot, thigh_length + shin_length); //final hip position along z-axis
	if(fhpz==-1) {
		std::cout<< "Step Length is too large. Aborting."<<std::endl;
		success=false;
		stop=true;
	
	}
	//CRANK-CONNECTING ROD CALCULATIONS
	float a= atan(sin(deg_to_rad(knee_angle))/ (cos(deg_to_rad(knee_angle)) + shin_length/thigh_length));
	float b= asin(shin_length/thigh_length * sin(a));
	float s= thigh_length + shin_length - (thigh_length*cos(b) + shin_length*cos(a));
	if(pivot>0.05) {
		wp_z = {Point(ihpy,ihpz), Point(pivot,thigh_length+shin_length-s), Point(fhpy,fhpz)};
		wp_vel_z = {0,0,0};
	}
	else {
		wp_z = {Point(ihpy,ihpz), Point(fhpy,fhpz)};
		wp_vel_z = {0,0};
	}
	wp_y ={Point(t[0],ihpy), Point(t[static_cast<int>(t.size()*hip_max_extension_time_coeff)],pivot), Point(t[t.size()-1],fhpy)};
	wp_vel_y = {0,0.05,0};
	hip_traj= generate_sagittal_trajectory_v2(t, "cubic", "cubic",wp_z, wp_vel_z, wp_y, wp_vel_y);
	while(hip_traj.size()<t.size()) hip_traj.push_back(hip_traj[hip_traj.size()-1]);
	//CFFTG CYCLE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	wp_vel_z={1,0,-1}; //reset velocities (will be fixed inside the cycle)
	auto i_timer = high_resolution_clock::now();//SET UP TIMER TO CHECK EXECUTION TIME
	while(!stop) {
		wp_y = {Point(t[0],0), Point(t[static_cast<int>(t.size()*foot_peak_time_coeff)],h), Point(t[t.size()-1],end)};
		wp_vel_y = {0,0.1,0};
		wp_z={Point(0,0),Point(h,v),Point(end,0)};
		std::cout<<"Generate foot traj for h: "<< h<<",  v: "<< v <<std::endl;
		foot_traj=  generate_sagittal_trajectory_v2(t, "cubic", "cubic",wp_z, wp_vel_z, wp_y, wp_vel_y);
		while(foot_traj.size()<t.size()) foot_traj.push_back(foot_traj[foot_traj.size()-1]);
		score=0;
		min_dist=10;
		for(int i=0;i<foot_traj.size();i++) {
			std::vector<Point> swing_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], foot_traj[i], false);
			if(swing_leg.size()==1){
				score=-1000; //kinematic constraint not satisfied
				//break;
			
			}
			else {
				knee_traj[i] = swing_leg[0];
				tip_traj[i] = swing_leg[1];
				heel_traj[i] =swing_leg[2];
				std::vector<Point> support_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], Point(pivot,0), true);
				if(support_leg[0].getx()!=-1) supp_knee_traj[i] = support_leg[0];
				else{
					score=-1000;
					//break;
				}
				if(score!=-1000){
					std::vector<float> res = intersection_score_and_min_dist_v2(obstacle_shape, heel_traj[i], tip_traj[i]); //res[0] = score , res[1]= min_dist (not calculated if res[0]<0)
					if(res[0]<0) score-=min_dist_th;
					else{
					//if(res[1]<min_dist) min_dist = res[1];
						if(res[1]< min_dist_th) score-=min_dist_th - res[1];
					}
				}
			}
			//UPDATE TEXT FILE
			if(produce_txt_file && i<hip_traj.size()){ //Sometimes hip traj has 1 point less than the others
				outfile << it << " ";
				outfile << hip_traj[i].getx() << " " << hip_traj[i].gety() << " ";
				outfile << supp_knee_traj[i].getx() << " " << supp_knee_traj[i].gety() << " ";
				outfile << knee_traj[i].getx() << " " << knee_traj[i].gety() << " ";
				outfile << foot_traj[i].getx() << " " << foot_traj[i].gety() << " ";
				outfile << heel_traj[i].getx() << " " << heel_traj[i].gety() << " ";
				outfile << tip_traj[i].getx() << " " << tip_traj[i].gety() << " ";
				outfile << end << " " << pivot << " " << h << " " << v << std::endl;  
		
			}	
		}		
		/*
		if(score!=-1000){
			score = min_dist - min_dist_th;
		}
		*/
		std::cout << "Score for trajectory n. " << it << " with h= " << h << ", v= " << v << ", step length= " <<end <<" min dist= "<< min_dist <<" is: " << score << std::endl;
		//std::cout<<"Update score"<<std::endl;
	  	if(score>=0) stop=true;
	  	else{
	  		if(score>best_score) {
	  			if(best_score!=-1000) {
	  				best_h=h;
		  			best_v=v;
		  			if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
		  			if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
	  			}
	  			best_score=score;
	  		}
	  		do{
		  		h=gen_gaussian(best_h,sigma_h);
		  	}
		  	while(h<0 || h<ihpy || h>end); 
		  	do{
		  		v=gen_gaussian(best_v,sigma_v);
		  	}
		  	while( v <= obstacle_max_h || v > 0.4);
		  	if(it%restart_it==0){
		  		//v=default_v;
				//h = default_h;
				best_score=-10;
				//std::cout<<"Reset R.V.s"<<std::endl;
		  		sigma_h = default_sigma_h;
		  		sigma_v = default_sigma_v;
		  	}
		  	if(it>max_it) {
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
  	//CLOSE TXT FILE
  	if(produce_txt_file) {
  		outfile.close();
  	}
  	//CHECK SUCCESS AND RETURN BUNDLE OF TRAJECTORIES
  	std::vector<std::vector<Point>> bundle;
  	if(!success){
  		std::cout<< "Solution not found" <<std::endl;
  		Point fail_indicator(-1,-1);
  		bundle.push_back(std::vector<Point> {fail_indicator});
  	}
	else bundle = {hip_traj,supp_knee_traj,knee_traj,foot_traj,tip_traj,heel_traj};
	return bundle;


}

























































//-------------------------------------FULL CUBIC DOUBLE CFFTG---------------------------------------------------------------------


std::vector<std::vector<Point>> cfftg_cubic_double(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, float min_dist_th, float hip_max_extension_time_coeff, float foot_peak_time_coeff_1, float foot_peak_time_coeff_2, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it, int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file, std::string txt_file_name){

	//CFFTG VARS INITIALIZATION------------------------------------------------------------------------------------------------------------------------------------
	float h,v,fixed_h,fixed_v; //horizontal (y-axis) and vertical (z-axis) position of the foot trajectory peak
	
	float default_h;
	//if(obstacle_shape.size()>0) default_h=obstacle_shape[0].getx();
	//else default_h= end/2;
	default_h= end/2;
	
	float default_v;
	//FIND OBSTACLE MAX HEIGHT TO PROVIDE REASONABLE INITIAL GUESS FOR V -------------------------------------------------------------------------------------------------
	float obstacle_max_h=0.0;
	float obs_points_surpassed=0;
	for(int i=0; i< obstacle_shape.size();i++){
		float temp= obstacle_shape[i].gety();
		if(temp > obstacle_max_h && obstacle_shape[i].getx()<end) obstacle_max_h=temp;
		if(obstacle_shape[i].getx()<end) obs_points_surpassed++;
	}
	std::cout<<"Obstacle points that have to be surpassed: " << obs_points_surpassed<<std::endl;
	default_v= obstacle_max_h + 0.1; //initial guess for v
	v=default_v;
	h = default_h;
	float default_sigma_h= default_sh;
	float default_sigma_v = default_sv;
	float sigma_h = default_sigma_h;
	float sigma_v = default_sigma_v;
	float best_v = v;
	float best_h = h;
	float score;
	float best_score=-100;
	float min_dist,min_dist_half;
	int best_dist=10;
	int it=0;
	bool stop=false;
	bool success = true;
	//TXT FILE OPENING
	std::ofstream outfile;
	if(produce_txt_file){
		outfile.open(txt_file_name + ".txt");
	}
	//TRAJECTORIES
	std::vector<float> t = create_time_vector(time_unit, step_time); //time vector
	std::vector<Point> foot_traj; //foot trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> hip_traj; //hip trajectory along the y/z-axis (vector of 2D points)
	std::vector<Point> knee_traj(t.size()); //knee tip trajectory (y/z-axis)
	std::vector<Point> tip_traj(t.size()); //foot tip trajectory (y/z-axis)
	std::vector<Point> heel_traj(t.size()); //foot heel trajectory (y/z-axis)
	std::vector<Point> supp_knee_traj(t.size()); //support knee trajectory (y/z-axis)
	//WAYPOINT STRUCTURE
	std::vector<Point> wp_z; 
	std::vector<Point> wp_y;
	std::vector<float> wp_vel_y;
	std::vector<float> wp_vel_z;
	std::vector<float> res;
	//std::vector<float> traj_res;
	//HIP TRAJECTORY GENERATION---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	float ihpy = pivot/2; //initial hip position along y-axis. Can be change according to the results of the odometry module
	float ihpz = hip_height; //initial hip position along z-axis
	float fhpy=(end - pivot)/2 + pivot;  //final hip position along y-axis
	float fhpz = final_hip_height(end, pivot, thigh_length + shin_length); //final hip position along z-axis
	if(fhpz==-1) {
		std::cout<< "Step Length is too large. Aborting."<<std::endl;
		success=false;
		stop=true;
	
	}
	//CRANK-CONNECTING ROD CALCULATIONS
	float a= atan(sin(deg_to_rad(knee_angle))/ (cos(deg_to_rad(knee_angle)) + shin_length/thigh_length));
	float b= asin(shin_length/thigh_length * sin(a));
	float s= thigh_length + shin_length - (thigh_length*cos(b) + shin_length*cos(a));
	if(pivot>0.05) {
		wp_z = {Point(ihpy,ihpz), Point(pivot,thigh_length+shin_length-s), Point(fhpy,fhpz)};
		wp_vel_z = {0,0.05,0};
	}
	else {
		wp_z = {Point(ihpy,ihpz), Point(fhpy,fhpz)};
		wp_vel_z = {0,0};
	}
	wp_y ={Point(t[0],ihpy), Point(t[static_cast<int>(t.size()*hip_max_extension_time_coeff)],pivot), Point(t[t.size()-1],fhpy)};
	wp_vel_y = {0,0.05,0};
	hip_traj= generate_sagittal_trajectory_v2(t, "cubic", "cubic",wp_z, wp_vel_z, wp_y, wp_vel_y);
	while(hip_traj.size()<t.size()) hip_traj.push_back(hip_traj[hip_traj.size()-1]);
	//CFFTG CYCLE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	wp_vel_z={0,0,0}; //reset velocities (will be fixed inside the cycle)
	wp_vel_y = {0,0.05,0};
	auto i_timer = high_resolution_clock::now();//SET UP TIMER TO CHECK EXECUTION TIME
	while(!stop) {
		wp_y = {Point(t[0],0), Point(t[static_cast<int>(t.size()*foot_peak_time_coeff_1)],h), Point(t[t.size()-1],end)};
		wp_z={Point(0,0),Point(h,v),Point(end,0)};
		std::cout<<"Generate foot traj for h: "<< h<<",  v: "<< v <<std::endl;
		foot_traj=  generate_sagittal_trajectory_v2(t, "cubic", "cubic",wp_z, wp_vel_z, wp_y, wp_vel_y);
		while(foot_traj.size()<t.size()) foot_traj.push_back(foot_traj[foot_traj.size()-1]);
		score=0;
		min_dist=10;
		min_dist_half=10;
		//traj_res.clear();
		for(int i=0;i<foot_traj.size();i++) {
			std::vector<Point> swing_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], foot_traj[i], false);
			if(swing_leg.size()==1){
				score=-50; //kinematic constraint not satisfied
				//break;
			}
			else {
				knee_traj[i] = swing_leg[0];
				tip_traj[i] = swing_leg[1];
				heel_traj[i] =swing_leg[2];
				std::vector<Point> support_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], Point(pivot,0), true);
				if(support_leg[0].getx()!=-1) supp_knee_traj[i] = support_leg[0];
				else{
					score=-50;
				}
				res = intersection_score_and_min_dist_v3(obstacle_shape, heel_traj[i], tip_traj[i]); //res[0] = score , res[1]= min_dist (not calculated if res[0]<0)
				if(res[0]<score) score=res[0];
				if(res[1]<min_dist) min_dist = res[1];
				//ELEMENT TO VALIDATE FIRST HALF OF THE FOOT TRAJ
				if(foot_traj[i].getx()<=h && res[1]< min_dist_half) min_dist_half = res[1];	
				//if(foot_traj[i].getx()<=h && res.size()> traj_res.size()) traj_res =res;		
			}
			//UPDATE TEXT FILE
			if(produce_txt_file && i<hip_traj.size()){ //Sometimes hip traj has 1 point less than the others
				outfile << it << " ";
				outfile << hip_traj[i].getx() << " " << hip_traj[i].gety() << " ";
				outfile << supp_knee_traj[i].getx() << " " << supp_knee_traj[i].gety() << " ";
				outfile << knee_traj[i].getx() << " " << knee_traj[i].gety() << " ";
				outfile << foot_traj[i].getx() << " " << foot_traj[i].gety() << " ";
				outfile << heel_traj[i].getx() << " " << heel_traj[i].gety() << " ";
				outfile << tip_traj[i].getx() << " " << tip_traj[i].gety() << " ";
				outfile << end << " " << pivot << " " << h << " " << v << std::endl;  
		
			}	
		}		
		std::cout << "Score for trajectory n. " << it << " with h= " << h << ", v= " << v << ", step length= " <<end <<" min dist= "<< min_dist <<" is: " << score << std::endl;
		//std::cout<<"Update score"<<std::endl;
	  	if(score == 0 && min_dist>min_dist_th) stop=true;
	  	else{
	  		if(score < 0) {
	  			if(score>best_score) {
	  				if(best_score!=-100 && it>10) {
	  					best_h=h;
		  				best_v=v;
		  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
		  				if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
	  				}
	  				best_score=score;
	  			}
	  		}
	  		else if (score==0) {
	  			if(min_dist<best_dist) {
	  				best_dist=min_dist;
	  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
		  			if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
	  			}
	  		}
	  		if(score>=-50 && score>=-0.25*obs_points_surpassed && min_dist_half>=min_dist_th) {
	  			fixed_h=h;
	  			std::cout<<"Fixed h: "<<fixed_h<<std::endl;
	  			fixed_v = v;
	  			stop=true;
	  			
	  		}
	  		//std::cout<<"Update R.V.s"<<std::endl;
	  		do{
		  		h=gen_gaussian(best_h,sigma_h);
		  	}
		  	while(h<0 || h<ihpy || h>end); 
		  	do{
		  		v=gen_gaussian(best_v,sigma_v);
		  	}
		  	while( v <= obstacle_max_h || v > 0.4);
		  	if(it%restart_it==0){
		  		//v=default_v;
				//h = default_h;
				best_score=-10;
				//std::cout<<"Reset R.V.s"<<std::endl;
		  		sigma_h = default_sigma_h;
		  		sigma_v = default_sigma_v;
		  	}
		  	if(it>max_it) {
		  		stop=true;
		  		success = false;
		  		std::cout<< "Solution not found" <<std::endl;
		  	}
		it++;
	  	}
	}
	
	//SECOND PART OF CFFTG
	if(success){
	
		int it_first_part=it;
		
		//RESET IMPORTANT VARS
		stop=false;
		best_dist=10;
		best_score=-100;
		sigma_h = default_sigma_h/4; //SEARCH MUST BE MORE LIMITED
		sigma_v = default_sigma_v/4;
		h = fixed_h + ((end-fixed_h)/2);
		v = fixed_v/2; 
		int peak_index=-1;
		//find foot trajectory peak index
		for(int i=0;i<foot_traj.size();i++){
			std::cout<<"("<<foot_traj[i].getx()<<", "<<foot_traj[i].gety() <<") ";
			if(abs(foot_traj[i].getx()-fixed_h)<0.0001) {
				peak_index=i;
				break;
			}
		}
		std::vector<Point> trimmed_foot_traj(t.size()-peak_index);
		std::vector<float> trimmed_t(t.size()-peak_index);
		std::copy(t.begin()+peak_index, t.end(), trimmed_t.begin());
	
		
		std::cout<<"Second part of CFFTG is starting"<<std::endl;
		while(!stop) {
			wp_y = {Point(t[peak_index],fixed_h), Point(t[static_cast<int>(t.size()*foot_peak_time_coeff_2)],h), Point(t[t.size()-1],end)};
			wp_vel_y = {0.05,0.05,0};
			wp_z={Point(fixed_h,fixed_v),Point(h,v),Point(end,0)};
			wp_vel_z={0,0.1,0};
			std::cout<<"Generate second part of foot traj for h: "<< h<<",  v: "<< v <<std::endl;
			trimmed_foot_traj=  generate_sagittal_trajectory_v2(trimmed_t, "cubic", "cubic",wp_z, wp_vel_z, wp_y, wp_vel_y);
			
			
			
			std::copy(trimmed_foot_traj.begin(),trimmed_foot_traj.end(), foot_traj.begin()+peak_index);
			while(foot_traj.size()<t.size()) foot_traj.push_back(foot_traj[foot_traj.size()-1]);
			score=0;
			min_dist=10;
			for(int i=0;i<peak_index;i++){
				if(produce_txt_file && i<hip_traj.size()){ //Sometimes hip traj has 1 point less than the others
					outfile << it << " ";
					outfile << hip_traj[i].getx() << " " << hip_traj[i].gety() << " ";
					outfile << supp_knee_traj[i].getx() << " " << supp_knee_traj[i].gety() << " ";
					outfile << knee_traj[i].getx() << " " << knee_traj[i].gety() << " ";
					outfile << foot_traj[i].getx() << " " << foot_traj[i].gety() << " ";
					outfile << heel_traj[i].getx() << " " << heel_traj[i].gety() << " ";
					outfile << tip_traj[i].getx() << " " << tip_traj[i].gety() << " ";
					outfile << end << " " << pivot << " " << h << " " << v << std::endl;  
				}	
			}
			
			for(int i=peak_index;i<foot_traj.size();i++) {
				std::vector<Point> swing_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], foot_traj[i], false);
				if(swing_leg.size()==1){
					score=-50; //kinematic constraint not satisfied
					//break;
				}
				else {
					knee_traj[i] = swing_leg[0];
					tip_traj[i] = swing_leg[1];
					heel_traj[i] =swing_leg[2];
					std::vector<Point> support_leg = calculate_full_leg_state(thigh_length, shin_length, front_foot_length, rear_foot_length,hip_traj[i], Point(pivot,0), true);
					if(support_leg[0].getx()!=-1) supp_knee_traj[i] = support_leg[0];
					else{
						score=-50;
					}
					res = intersection_score_and_min_dist_v2(obstacle_shape, heel_traj[i], tip_traj[i]); //res[0] = score , res[1]= min_dist (not calculated if res[0]<0)
					if(res[0]<score) score=res[0];
					if(res[1]<min_dist) min_dist = res[1];
					//ELEMENT TO VALIDATE FIRST HALF OF THE FOOT TRAJ	
				}
				//UPDATE TEXT FILE
				if(produce_txt_file && i<hip_traj.size()){ //Sometimes hip traj has 1 point less than the others
					outfile << it << " ";
					outfile << hip_traj[i].getx() << " " << hip_traj[i].gety() << " ";
					outfile << supp_knee_traj[i].getx() << " " << supp_knee_traj[i].gety() << " ";
					outfile << knee_traj[i].getx() << " " << knee_traj[i].gety() << " ";
					outfile << foot_traj[i].getx() << " " << foot_traj[i].gety() << " ";
					outfile << heel_traj[i].getx() << " " << heel_traj[i].gety() << " ";
					outfile << tip_traj[i].getx() << " " << tip_traj[i].gety() << " ";
					outfile << end << " " << pivot << " " << h << " " << v << std::endl;  
			
				}	
			}		
			std::cout << "Score for trajectory n. " << it << " with h= " << h << ", v= " << v << ", step length= " <<end <<" min dist= "<< min_dist <<" is: " << score << std::endl;
			//std::cout<<"Update score"<<std::endl;
		  	if(score == 0 && min_dist>min_dist_th) stop=true;
		  	else{
		  		if(score < 0) {
		  			if(score>best_score) {
		  				if(best_score!=-100 && it>10) {
		  					best_h=h;
			  				best_v=v;
			  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
			  				if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
		  				}
		  				best_score=score;
		  			}
		  			
		  		}
		  		else if (score==0) {
		  			if(min_dist<best_dist) {
		  				best_dist=min_dist;
		  				if(sigma_h > min_sigma_h) sigma_h = sigma_h/decay_coeff_h; //old sigma th was 0.05
			  			if(sigma_v > min_sigma_v) sigma_v = sigma_v/decay_coeff_v;
		  			}
		  		}
		  		//std::cout<<"Update R.V.s"<<std::endl;
		  		do{
			  		h=gen_gaussian(best_h,sigma_h);
			  	}
			  	while(h<=fixed_h || h>end); 
			  	do{
			  		v=gen_gaussian(best_v,sigma_v);
			  	}
			  	while( v <= 0 || v > fixed_v);
			  	if(it%restart_it==0){
			  		//v=default_v;
					//h = default_h;
					best_score=-10;
					//std::cout<<"Reset R.V.s"<<std::endl;
			  		sigma_h = default_sigma_h;
			  		sigma_v = default_sigma_v;
			  	}
			  	if(it >max_it+it_first_part) {
			  		stop=true;
			  		success = false;
			  		std::cout<< "Solution not found" <<std::endl;
			  	}
			it++;
		  	}
		}
	
	}

	
	
	
	
	
	//CALCULATE EXECUTION TIME
	auto f_timer = high_resolution_clock::now();
	auto exec_time = duration_cast<microseconds>(f_timer - i_timer);
  	std::cout<<"Total time: " << exec_time.count()/1000 << " milliseconds" << std::endl;
  	//CLOSE TXT FILE
  	if(produce_txt_file) {
  		outfile.close();
  	}
  	//CHECK SUCCESS AND RETURN BUNDLE OF TRAJECTORIES
  	std::vector<std::vector<Point>> bundle;
  	if(!success){
  		std::cout<< "Solution not found" <<std::endl;
  		Point fail_indicator(-1,-1);
  		bundle.push_back(std::vector<Point> {fail_indicator});
  	}
	else bundle = {hip_traj,supp_knee_traj,knee_traj,foot_traj,tip_traj,heel_traj};
	return bundle;


}

