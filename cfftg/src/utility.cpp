#include "utility.h"
using namespace std::chrono;

//Point Class Method Definition--------------------------------------------

Point::Point(){
	set(0.0, 0.0);
}
Point::Point(float x0, float y0){
	set(x0, y0);
}
void Point::set(float x0, float y0){
	x = x0;
	y = y0;
}
float Point::getx(){;
	return x;
}
float Point::gety(){
	return y;
}

void Point::print(){
	std::cout << "(" << x << ", "
	<< y << ")";
}

//Utility Methods----------------------------------------------------------

float deg_to_rad(float degrees) {
    return degrees * (M_PI/180);
}


float norm(Point a, Point b) {
	return sqrt(pow(a.getx()-b.getx(),2) + pow(a.gety()-b.gety(),2));
}


std::vector<float> cubic_traj_gen(float y1, float z1, float y2, float z2, float y3, float z3){
	std::vector<float> traj_coeffs(4);
	Eigen::Matrix4f A;
	Eigen::Vector4f b;
	A << pow(y1,3),pow(y1,2), y1, 1, pow(y2,3),pow(y2,2), y2, 1, pow(y3,3),pow(y3,2), y3, 1,   3*pow(y2,2),2*y2, 1, 0;
	b << z1,z2,z3,0;
	Eigen::Vector4f x = A.colPivHouseholderQr().solve(b);
	for(int i=0;i<4;i++){
		traj_coeffs[i]= x[i];
	}
	return traj_coeffs;

}

std::vector<float> piecewise_cubic_traj_gen(float y_i, float z_i, float y_f, float z_f, float v_i, float v_f){
	std::vector<float> traj_coeffs(4);
	Eigen::Matrix4f A;
	Eigen::Vector4f b;
	A << pow(y_i,3),pow(y_i,2), y_i, 1, pow(y_f,3),pow(y_f,2), y_f, 1, 3*pow(y_i,2),2*y_i, 1, 0,   3*pow(y_f,2),2*y_f, 1, 0;
	b << z_i,z_f,v_i,v_f;
	Eigen::Vector4f x = A.colPivHouseholderQr().solve(b);
	for(int i=0;i<4;i++){
		traj_coeffs[i]= x[i];
	}
	return traj_coeffs;

}

float gen_gaussian(float mean, float stddev){
	std::random_device rd{};
	std::mt19937 gen{rd()};
	std::normal_distribution<float> d{mean,stddev};
	return d(gen);
}

//note: x is a vector containing the independent variable values that will be evaluated to produce f(x)
std::vector<Point> compute_traj(std::vector<float> x, std::vector<float> traj_coeffs){
	std::vector<Point> traj(x.size());
	float temp;
	float y;
	for(int i=0;i<x.size();i++){
		temp= pow(x[i],2);
		y= temp * x[i]* traj_coeffs[0] + temp * traj_coeffs[1] + x[i]* traj_coeffs[2] + traj_coeffs[3];
		traj[i] = Point(x[i],y);
		
	}
	return traj;
}

//indep_var is the independent variable points (sorted in ascending order) of the cubic equation, for example t in x=f(t)
std::vector<Point> piecewise_cubic_poly(std::vector<Point> waypoints,std::vector<float> indep_var, std::vector<float> wp_velocities){
	std::vector<Point> trajectory,temp_traj;
	std::vector<float> temp_var,coeffs;
	float wp_next;
	int index = 0;
	for(int i=0;i<waypoints.size()-1;i++){
		wp_next = waypoints[i+1].getx();
		while(index<indep_var.size()  && indep_var[index] <= wp_next) {
			temp_var.push_back(indep_var[index]);
			index++;
		}
		coeffs = piecewise_cubic_traj_gen(temp_var[0],waypoints[i].gety(),temp_var[temp_var.size()-1],waypoints[i+1].gety(), wp_velocities[i],wp_velocities[i+1]);
		temp_traj = compute_traj(temp_var,coeffs);
		trajectory.insert(trajectory.end(),temp_traj.begin(),temp_traj.end());
		temp_var.clear();
	}
	return trajectory;


}

std::vector<Point> compute_linear_traj_2D(std::vector<float> x, float y_i, float y_f){
	std::vector<Point> traj(x.size());
	float y;
	for(int i=0;i<x.size();i++){
		y= y_i + ((y_f - y_i)/ (x[x.size()-1] - x[0]) * x[i]) ;
		traj[i] = Point(x[i],y);
		
	}
	return traj;
}


std::vector<float> compute_linear_traj_1D(std::vector<float> x, float y_i, float y_f){
	std::vector<float> val(x.size());
	float y;
	for(int i=0;i<x.size();i++){
		val[i] = y_i + ((y_f - y_i)/ (x[x.size()-1] - x[0]) * x[i]) ;
		
	}
	return val;
}


float final_hip_height(float foothold, float pivot, float leg_length) {
	float min_height_constraint = 0.9; //we dont want hip height to fall under 0.9*leg_length 
					     //(this leaves for a 10% change of height w.r.t. maximum extension, only reached when hip height = leg length)
	float factor = 2* sqrt(1-pow(min_height_constraint,2));
	bool feasible_step = foothold <= (factor*leg_length) + pivot;
	float max_height, height;
	float hip_foot_y_dist_final = (foothold-pivot)/2;
	if(feasible_step) {
		max_height = sqrt(pow(leg_length,2) - pow(hip_foot_y_dist_final,2));
		if(max_height - 0.01 >= min_height_constraint*leg_length) height = max_height - 0.01;
		else height = min_height_constraint*leg_length;
		return height;
	}
	else return -1; 
}



std::vector<Point> leg_forward_kinematics(Point com, float thigh_length, float shin_length, float hip_ang, float knee_ang){ //angles in radians, coordinates in meters
	std::vector<Point> positions; 
	positions.push_back(Point(com.getx() + thigh_length * sin(hip_ang), com.gety() - thigh_length* cos(hip_ang)));
	positions.push_back(Point(positions[0].getx() + shin_length* sin(hip_ang+ knee_ang), positions[0].gety() - shin_length* cos(hip_ang+ knee_ang)));
	return positions; //positions[0] = Point(knee_y,knee_z)   positions[1] = Point(ankle_y,ankle_z)
}

std::vector<float> leg_inverse_kinematics(float com_y, float thigh_length, float shin_length, float knee_y, float ankle_y) { //coordinates in meters
	std::vector<float> angles;
	angles.push_back(asin((knee_y - com_y)/thigh_length)); //hip angle (rad)
	angles.push_back(asin((ankle_y - knee_y)/shin_length) - angles[0]); //knee angle (rad)
	return angles;
}

std::vector<float> create_time_vector(float time_unit, float duration) {
	std::vector<float> t(ceil(duration/time_unit)+1);
	for(int i=0; i< t.size(); i++){
		t[i]= i*time_unit;
	}
	return t;
}

std::vector<Point> calculate_full_leg_state(float thigh_length, float shin_length, float front_foot_length, float rear_foot_length,Point hip, Point ankle, bool knee_only) {
	std::vector<Point> positions;
	float M = sqrt(pow(ankle.getx() - hip.getx(),2)+ pow(ankle.gety() - hip.gety(),2));
	if(M-(thigh_length+shin_length)>0.03){
		std::cout<<"Kinematic constraint (leg length) not satisfied "<< std::endl;
		positions.push_back(Point(-1,-1));
		
	}
	else{
		float temp = (pow(thigh_length,2) + pow(M,2) - pow(shin_length,2))/(2*thigh_length*M);
		if(temp>1) temp=1;
		if(temp<-1) temp=-1;
		float alfa = acos(temp);
		
		temp = (ankle.getx() - hip.getx())/M;
		if(temp>1) temp=1;
		if(temp<-1) temp=-1;
		float tilt = asin(temp);
		Point knee = Point( hip.getx() + thigh_length * sin(alfa + tilt) , hip.gety() - thigh_length * cos(alfa + tilt));
		positions.push_back(knee);
		if(!knee_only) {
			temp = ( ankle.getx() - knee.getx())/shin_length;
			if(temp>1) temp=1;
			if(temp<-1) temp=-1;
			float epsilon = asin(temp);
			Point tip = Point(ankle.getx() + front_foot_length*cos(epsilon), ankle.gety() + front_foot_length*sin(epsilon));
			positions.push_back(tip);
			Point heel = Point(ankle.getx() - rear_foot_length*cos(epsilon), ankle.gety() - rear_foot_length*sin(epsilon));
			positions.push_back(heel);
		}
	}
	return positions;

}


float point_to_segment_dist(Point p, Point a, Point b) { //calculates distance between point p and segment defined by the two extremes a and b 
	float L = pow(a.getx()-b.getx(),2) + pow(a.gety()-b.gety(),2);
	float dot = (p.getx()- a.getx()) * (b.getx()-a.getx()) + (p.gety()-a.gety())*(b.gety()-a.gety()) / pow(L,2);
	if(dot>1) dot =1;
	if(dot<0) dot=0;
	Point proj = Point(a.getx() + dot*(b.getx()-a.getx()), a.gety() + dot*(b.gety()-a.gety()));
	float dist = norm(p,proj);
	return dist;
}

float point_to_segment_dist_v2(Point p, Point a, Point b) { //calculates distance between point p and segment defined by the two extremes a and b 
	float dist;
	Point ab = Point(b.getx()-a.getx(), b.gety()-a.gety());
	Point bp = Point(p.getx()-b.getx(), p.gety()-b.gety());
	Point ap = Point(p.getx()-a.getx(), p.gety()-a.gety());
	
	float ab_bp, ab_ap;
	ab_bp = ab.getx()*bp.getx() + ab.gety()*bp.gety();
	ab_ap = ab.getx()*ap.getx() + ab.gety()*ap.gety();
	if(ab_bp>0){
		dist = sqrt(bp.gety()*bp.gety() + bp.getx()*bp.getx());
	}
	else if(ab_ap<0){
		dist = sqrt(ap.gety()*ap.gety() + ap.getx()*ap.getx());
	}
	else{
		float mod = sqrt(ab.gety()*ab.gety() + ab.getx()*ab.getx());
		dist = abs(ab.getx() * ap.gety() - ab.gety()*ap.getx()) / mod;
	}
	
	
	return dist;
}



std::vector<float> intersection_score_and_min_dist(std::vector<Point> obstacle_shape, Point heel, Point tip){
	//CALCULATE FOOT LINE COEFFICIENTS FOR COLLISION DETECTION (y=m*x+q) 
	float m = (tip.gety()- heel.gety()) / (tip.getx()- heel.getx());
	float q = heel.gety() - m*heel.getx();
	float b1,b2; //bounds
	if(tip.getx()>heel.getx()){
		b1= heel.getx();
		b2= tip.getx();
	}
	else{
		b2= heel.getx();
		b1= tip.getx();
	}
	int temp_score=0;
	float min_dist=10;
	float dist;
	//std::cout<<"Calculating collision for heel: [" << heel.getx() << ", " << heel.gety() << "]      tip= [" << tip.getx() << ", " << tip.gety() << std::endl;
	for(int j=0;j<obstacle_shape.size();j++){
		if(obstacle_shape[j].gety()>0){
			if(temp_score==0) {
				dist = point_to_segment_dist(obstacle_shape[j], heel, tip);
				if(dist< min_dist) min_dist=dist;
			}
			if(b1<=obstacle_shape[j].getx() && obstacle_shape[j].getx()<=b2 && obstacle_shape[j].gety()>= m*obstacle_shape[j].getx() + q) {
				std::cout<<"Collision detected at y: " << obstacle_shape[j].getx()<< std::endl;
				temp_score-=1;
			}	
		}	
	}
	//if(temp_score!=0) min_dist=10; //to avoid misunderstandings
	std::vector<float> results;
	results.push_back(temp_score);
	results.push_back(min_dist);
	return results;

}

std::vector<float> intersection_score_and_min_dist_v2(std::vector<Point> obstacle_shape, Point heel, Point tip){
	//CALCULATE FOOT LINE COEFFICIENTS FOR COLLISION DETECTION (y=m*x+q) 
	float m = (tip.gety()- heel.gety()) / (tip.getx()- heel.getx());
	float q = heel.gety() - m*heel.getx();
	float b1,b2; //bounds
	if(tip.getx()>heel.getx()){
		b1= heel.getx();
		b2= tip.getx();
	}
	else{
		b2= heel.getx();
		b1= tip.getx();
	}
	int temp_score=0;
	float min_dist=10;
	float dist;
	//std::cout<<"Calculating collision for heel: [" << heel.getx() << ", " << heel.gety() << "]      tip= [" << tip.getx() << ", " << tip.gety() << std::endl;
	for(int j=0;j<obstacle_shape.size();j++){
		if(obstacle_shape[j].gety()>0){
			if(temp_score==0) {
				dist = point_to_segment_dist_v2(obstacle_shape[j], heel, tip);
				if(dist< min_dist) min_dist=dist;
			}
			if(b1<=obstacle_shape[j].getx() && obstacle_shape[j].getx()<=b2 && obstacle_shape[j].gety()>= m*obstacle_shape[j].getx() + q) {
				std::cout<<"Collision detected at y: " << obstacle_shape[j].getx()<< std::endl;
				temp_score-=1;
			}	
		}	
	}
	//if(temp_score!=0) min_dist=10; //to avoid misunderstandings
	std::vector<float> results;
	results.push_back(temp_score);
	results.push_back(min_dist);
	return results;

}

std::vector<float> intersection_score_and_min_dist_v3(std::vector<Point> obstacle_shape, Point heel, Point tip){
	std::vector<float> results; //OUTPUT VECT
	//CALCULATE FOOT LINE COEFFICIENTS FOR COLLISION DETECTION (y=m*x+q) 
	float m = (tip.gety()- heel.gety()) / (tip.getx()- heel.getx());
	float q = heel.gety() - m*heel.getx();
	float b1,b2; //bounds
	if(tip.getx()>heel.getx()){
		b1= heel.getx();
		b2= tip.getx();
	}
	else{
		b2= heel.getx();
		b1= tip.getx();
	}
	int temp_score=0;
	float min_dist=10;
	float dist;
	//std::cout<<"Calculating collision for heel: [" << heel.getx() << ", " << heel.gety() << "]      tip= [" << tip.getx() << ", " << tip.gety() << std::endl;
	for(int j=0;j<obstacle_shape.size();j++){
		if(obstacle_shape[j].gety()>0){
			if(temp_score==0) {
				dist = point_to_segment_dist_v2(obstacle_shape[j], heel, tip);
				if(dist< min_dist) min_dist=dist;
			}
			if(b1<=obstacle_shape[j].getx() && obstacle_shape[j].getx()<=b2 && obstacle_shape[j].gety()>= m*obstacle_shape[j].getx() + q) {
				std::cout<<"Collision detected at y: " << obstacle_shape[j].getx()<< std::endl;
				results.push_back(j);
				temp_score-=1;
			}	
		}	
	}
	//if(temp_score!=0) min_dist=10; //to avoid misunderstandings
	std::cout<<"Collision evaluation finished"<<std::endl;
	results.insert(results.begin(), min_dist);
	results.insert(results.begin(), temp_score);
	
	return results;

}


std::vector<Point> generate_sagittal_trajectory(std::vector<float> time, std::vector<Point> wp, std::string y_traj_type, std::string z_traj_type, std::vector<float> wp_vel_z) {
	std::vector<float> traj_y;
	std::vector<Point> traj;
	if(y_traj_type =="linear") {
		traj_y= compute_linear_traj_1D(time, wp[0].getx(), wp[wp.size()-1].getx());
	}
	if(z_traj_type=="cubic") {
		traj = piecewise_cubic_poly(wp, traj_y, wp_vel_z);
	}
	return traj;
}

//CFFTG VARS:
// Starting point of the foot trajectory assumed at (0;0) (sagittal plane)
// end: endpoint of the foot trajectory along the y-axis (sagittal plane). z-axis coordinate is assumed 0 (ground level). 
// hip_height: measured hip height (z-axis) at the start of the trajecotry (y_coord can be inferred through kinematics or by assuming that the position is between swing and support foot)
// pivot: position of the support on the y-axis
// knee_angle: desired knee angle when hip y-axis coord = pivot. This is needed to compute the high of the com in that moment (according to the crank connecting rod model)
// time_unit: sampling time of the trajectory
std::vector<std::vector<Point>> cfftg_sagittal(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, std::vector<Point> obstacle_shape, float step_time, float time_unit){

	//CFFTG VARS INITIALIZATION------------------------------------------------------------------------------------------------------------------------------------
	float h,v; //horizontal (y-axis) and vertical (z-axis) position of the foot trajectory peak
	float default_h = end/2;
	float default_v;
	//FIND OBSTACLE MAX HEIGHT TO PROVIDE REASONABLE INITIAL GUESS FOR V -------------------------------------------------------------------------------------------------
	float obstacle_max_h=0.0;
	for(int i=0; i< obstacle_shape.size();i++){
		float temp= obstacle_shape[i].gety();
		if(temp > obstacle_max_h) obstacle_max_h=temp;
	}
	default_v= obstacle_max_h + 0.1; //initial guess for v
	v=default_v;
	h = default_h;
	float default_sigma_h= 0.2;
	float default_sigma_v = 0.05;
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
		  				if(sigma_h > 0.01) sigma_h = sigma_h/2; //old sigma th was 0.05
		  				if(sigma_v > 0.01) sigma_v = sigma_v/2;
	  				}
	  				best_score=score;
	  			}
	  		}
	  		else if (score==0) {
	  			if(min_dist<best_dist) {
	  				best_dist=min_dist;
	  				if(sigma_h > 0.01) sigma_h = sigma_h/2; //old sigma th was 0.05
		  			if(sigma_v > 0.01) sigma_v = sigma_v/2;
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
		  	if(it%500==0){
		  		//v=default_v;
				//h = default_h;
				//std::cout<<"Reset R.V.s"<<std::endl;
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
	auto f_timer = high_resolution_clock::now();
	auto exec_time = duration_cast<microseconds>(f_timer - i_timer);
  	std::cout<<"Total time: " << exec_time.count()/1000 << " milliseconds" << std::endl;
  	std::vector<std::vector<Point>> bundle;
  	if(!success){
  		std::cout<< "Solution not found" <<std::endl;
  		Point fail_indicator(-1,-1);
  		bundle.push_back(std::vector<Point> {fail_indicator});
  	}
	else bundle = {hip_traj,supp_knee_traj,knee_traj,foot_traj,tip_traj,heel_traj};
	return bundle;


}

void cfftg_cartesian_bundle_to_file(std::vector<std::vector<Point>> bundle, std::string filename){ //this structure can be taken as input by matlab
	std::ofstream outfile;
	outfile.open(filename + ".txt");
	for(int i = 0; i < bundle.size(); i++){
		std::cout<<"Length of trajectory "<< i<< " : " << bundle[i].size()<< std::endl;
		if(i==0) outfile << "hip " << " = ";
		else if(i==1) outfile << "supp_knee " << " = ";
		else if(i==2) outfile << "knee " << " = ";
		else if (i==3) outfile << "foot " << " = ";
		else if(i==4) outfile << "tip " << " = ";
		else if(i==5)  outfile << "heel " << " = ";
		outfile << "[ ";
		for(int j=0; j < bundle[i].size();j++){
			outfile << bundle[i][j].getx() << " " << bundle[i][j].gety();
			if(j!=bundle[i].size()-1){
				outfile << " ; ";
			}
		}
		outfile << " ]; "<<std::endl;
	}
	outfile.close(); 
}

void cfftg_angular_bundle_to_file(std::vector<std::vector<float>> bundle, std::string filename){ //this scructure can be taken as input by matlab
	std::ofstream outfile;
	outfile.open(filename + ".txt");
	std::cout<<"Saving angular trajectories"<< std::endl;
	for(int i=0; i<bundle.size();i++){
		if(i==0) outfile << "hip_ang_supp " << " = ";
		else if(i==1) outfile << "knee_ang_supp " << " = ";
		else if(i==2) outfile << "hip_ang_swing " << " = ";
		else if (i==3) outfile << "knee_ang_swing " << " = ";
		outfile << "[ ";
		for(int j=0; j<bundle[i].size();j++) {
			outfile << bundle[i][j];
			if(i!=bundle[i].size()-1){
					outfile << " , ";
			}
		}
		outfile << " ]; "<<std::endl;
	}
	std::cout<<"Finished saving angular trajectories"<< std::endl;
	
	outfile.close(); 

}

std::vector<float> derivative(std::vector<float> data) { // UTILITY FUNCTION CURRENTLY NOT USED
	std::vector<float> derivative;
	for(int i=1; i<data.size();i++) {
		derivative.push_back(data[i]-data[i-1]);	
		
	}
	return derivative;

}


std::vector<Point> select_waypoints(std::vector<Point> traj, float wp_dist_threshold, float precision, std::string type) {
	std::vector<Point> waypoints;
	if(type=="derivative") {
		std::vector<float> derivative;
		for(int i=1; i<traj.size(); i++) {
			derivative.push_back(traj[i].gety()-traj[i-1].gety());
		}
		
		waypoints.push_back(traj[0]); //FIRST POINT IS ALWAYS INCLUDED
		for(int i=0; i<derivative.size(); i++) {
			if(abs(derivative[i])< precision && traj[i].getx() - waypoints[waypoints.size()-1].getx() > wp_dist_threshold &&  abs(traj[i].getx()- traj[traj.size()-1].getx())> wp_dist_threshold) {
				waypoints.push_back(traj[i]);
			}
		}
		waypoints.push_back(traj[traj.size()-1]);
	}
	else if(type=="uniform_spaced") {
		waypoints.push_back(traj[0]); //FIRST POINT IS ALWAYS INCLUDED
		for(int i=0;i<traj.size();i++){
			if(traj[i].getx()>=waypoints[waypoints.size()-1].getx()+ wp_dist_threshold) waypoints.push_back(traj[i]);
		}
		if(waypoints[waypoints.size()-1].getx() - traj[traj.size()-1].getx() < 0.05) waypoints.pop_back(); 
		
			waypoints.push_back(traj[traj.size()-1]);
	}
	else if(type=="peak") {
		// VERSION 1
		/*
		float init_value=traj[0].gety();
		waypoints.push_back(traj[0]);
		float max_diff=0;
		float diff,peak_value;
		
		for(int i=0;i<traj.size();i++){
			diff = abs(init_value - traj[i].gety());
			if(diff>max_diff){ 
				max_diff=diff;
				peak_value = i; //save index
			}
		}
		if(traj[peak_value].gety()==traj[traj.size()-1].gety()) { //peak_value is the final value
			peak_value = static_cast<int>(traj.size()/2);
		}
		waypoints.push_back(traj[peak_value]);
		waypoints.push_back(traj[traj.size()-1]);
		*/
		//VERSION 2
		float init_value;
		int j=0;
		while(traj[j].gety()!=traj[j].gety()){
			j++;
		}
		init_value=traj[j].gety();
		waypoints.push_back(traj[0]);
		float max_diff_up=0;
		float max_diff_down=0;
		float diff,peak_value_up,peak_value_down;
		
		for(int i=0;i<traj.size();i++){
			if(traj[i].gety()==traj[i].gety()) diff = traj[i].gety() - init_value;
			else diff=0;
			if(diff>0) { //angle is rising
				if(diff>max_diff_up){ 
					max_diff_up=diff;
					peak_value_up = i; //save index
				}
			
			}
			if(diff<0) { //angle is rising
				if(diff<max_diff_down){ 
					max_diff_down=diff;
					peak_value_down = i; //save index
				}
			
			}
		}
		if(max_diff_up==0) {
			if(traj[peak_value_down].gety()==traj[traj.size()-1].gety()) { //peak_value is the final value
				waypoints.push_back(traj[static_cast<int>(traj.size()/2)]);
			}
			else {
				waypoints.push_back(traj[peak_value_down]);
			
			}
		
		}
		else if(max_diff_down==0) {
			if(traj[peak_value_up].gety()==traj[traj.size()-1].gety()) { //peak_value is the final value
				waypoints.push_back(traj[static_cast<int>(traj.size()/2)]);
			}
			else {
				waypoints.push_back(traj[peak_value_up]);
			
			}
		
		}
		else {
			if(traj[0].gety() > traj[traj.size()-1].gety()) {
				waypoints.push_back(traj[peak_value_up]);
			
			
			}
			else waypoints.push_back(traj[peak_value_down]);
		
		
		
		
		}
		waypoints.push_back(traj[traj.size()-1]);
		
	}
	
	return waypoints;
	

}





float rad_to_deg(float rad){
	return rad*(180/M_PI);
}


void save_point_vect(std::vector<Point> vec, std::string vec_name, std::string filename) {
	std::ofstream outfile;
	outfile.open(filename + ".txt");
	outfile << vec_name << " = [";
	for(int j=0; j<vec.size();j++) {
			outfile << vec[j].getx() << " " << vec[j].gety();
			if(j!=vec.size()-1){
				outfile << " ; ";
			}
	}
	outfile << " ]; "<<std::endl;
	outfile.close(); 	
}


void save_vect_for_matlab(std::vector<Point> vec, std::string filename){
	std::ofstream outfile;
	outfile.open(filename + ".txt");
	for(int j=0; j<vec.size();j++) {
			outfile << vec[j].getx() << " " << vec[j].gety()<<std::endl;
	}
	outfile.close(); 


}



void save_ang_traj_matlab(std::vector<std::vector<float>> bundle, float time_unit,std::string filename){
	std::ofstream outfile;
	outfile.open(filename + ".txt");
	for(int i=0;i < bundle[0].size();i++){
		outfile<< i*time_unit;
		for(int j=0;j<bundle.size();j++){
			outfile<<" "<<bundle[j][i];
		}
		outfile<<std::endl;
	}
	outfile.close(); 

}




std::vector<Point> generate_sagittal_trajectory_v2(std::vector<float> time, std::string y_traj_type, std::string z_traj_type, std::vector<Point> wp_z,std::vector<float> wp_vel_z,std::vector<Point> wp_y, std::vector<float> wp_vel_y ) {
	std::vector<float> traj_y;
	std::vector<Point> traj;
	if(y_traj_type =="linear") {
		traj_y= compute_linear_traj_1D(time, wp_z[0].getx(), wp_z[wp_z.size()-1].getx());
	}
	else if(y_traj_type =="cubic") {
		std::vector<Point> temp_traj;
		temp_traj= piecewise_cubic_poly(wp_y, time, wp_vel_y);
		for(int i=0;i<temp_traj.size();i++) traj_y.push_back(temp_traj[i].gety());
	
	}
	if(z_traj_type=="cubic") {
		traj = piecewise_cubic_poly(wp_z, traj_y, wp_vel_z);
	}
	return traj;
}

//NEW VERSION OF CFFTG (WITH CUBIC TRAJECTORIES IN TIME)
std::vector<std::vector<Point>> cfftg_sagittal_v2(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, std::vector<Point> obstacle_shape, float step_time, float time_unit){

	//CFFTG VARS INITIALIZATION------------------------------------------------------------------------------------------------------------------------------------
	float h,v; //horizontal (y-axis) and vertical (z-axis) position of the foot trajectory peak
	float default_h;
	if(obstacle_shape.size()==0){
		default_h = end/2;
	}
	else default_h = obstacle_shape[0].getx();
	float default_v;
	//FIND OBSTACLE MAX HEIGHT TO PROVIDE REASONABLE INITIAL GUESS FOR V -------------------------------------------------------------------------------------------------
	float obstacle_max_h=0.0;
	for(int i=0; i< obstacle_shape.size();i++){
		float temp= obstacle_shape[i].gety();
		if(temp > obstacle_max_h) obstacle_max_h=temp;
	}
	default_v= obstacle_max_h + 0.1; //initial guess for v
	v=default_v;
	h = default_h;
	float default_sigma_h= 0.05;
	float default_sigma_v = 0.05;
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
	std::vector<float> wp_vel_z;
	std::vector<float> wp_vel_y;
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
	hip_traj= generate_sagittal_trajectory(t, wp_z, "linear", "cubic", wp_vel_z);
	//CFFTG CYCLE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	wp_vel_z={0,0.05,0}; //reset velocities (will be fixed inside the cycle)
	wp_vel_y={0,0.05,0};
	wp_y = {Point(t[0],0), Point(t[static_cast<int>(t.size()/2)]/2,h), Point(t[t.size()-1],end)};
	auto i_timer = high_resolution_clock::now();//SET UP TIMER TO CHECK EXECUTION TIME
	while(!stop) {
		
		wp_z={Point(0,0),Point(h,v),Point(end,0)};
		std::cout<<"Generate foot traj for h: "<< h<<",  v: "<< v <<std::endl;
		foot_traj=  generate_sagittal_trajectory_v2(t, "cubic", "cubic", wp_z, wp_vel_z,wp_y,wp_vel_y);
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
		  				if(sigma_h > 0.001) sigma_h = sigma_h/1.5; //old sigma th was 0.05
		  				if(sigma_v > 0.001) sigma_v = sigma_v/1.5;
	  				}
	  				best_score=score;
	  			}
	  		}
	  		else if (score==0) {
	  			if(min_dist<best_dist) {
	  				best_dist=min_dist;
	  				if(sigma_h > 0.001) sigma_h = sigma_h/1.5; //old sigma th was 0.05
		  			if(sigma_v > 0.001) sigma_v = sigma_v/1.5;
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
		  	if(it%500==0){
		  		//v=default_v;
				//h = default_h;
				//std::cout<<"Reset R.V.s"<<std::endl;
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
	auto f_timer = high_resolution_clock::now();
	auto exec_time = duration_cast<microseconds>(f_timer - i_timer);
  	std::cout<<"Total time: " << exec_time.count()/1000 << " milliseconds" << std::endl;
  	std::vector<std::vector<Point>> bundle;
  	if(!success){
  		std::cout<< "Solution not found" <<std::endl;
  		Point fail_indicator(-1,-1);
  		bundle.push_back(std::vector<Point> {fail_indicator});
  	}
	else bundle = {hip_traj,supp_knee_traj,knee_traj,foot_traj,tip_traj,heel_traj};
	return bundle;


}



