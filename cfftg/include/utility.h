#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <chrono>
#include <map>
#include <random>
#include <string>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>





//2D Point class--------------------------------------------------------------------------------

class Point{
	private:
	float x, y;
	public:
	Point();
	Point (float x0, float y0);
	void set(float x0, float y0);
	float getx();
	float gety();
	void print();
};


// CFFTG class------------------------------------------------------------------------------------------

class Cfftg {
	private:
	//USER VARS
	//float step_duration= 5.0; //better as parameter in the function
	//float foot_length=0.15; //HARDCODED (FOR NOW)
	//float thigh_length=0.5;
	//float shin_length = 0.5;
	//float safe_dist = (foot_length) + 0.01 ;

	//CONSTANTS
	//const float bb_v_coeff = sqrt(3)/2; //foot has a bounding box that accounts for 60° of downward rotation along the X axis passing through the centroid of the foot
	//const float default_sigma_v= 0.03;
	
	public:
	Cfftg();
	std::vector<std::vector<Point>> trajectory_generator(float thigh_length, float shin_length, float knee_angle, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float step_time, std::vector<Point> obstacle_shape);
	
	

};


//--------------------------------------------------------------------------------------------------
//KINEMATIC MODEL METHOD (NOT USED)
std::vector<std::vector<Point>> kinematic_model_evaluation(float ds_foot_pos,int knee_angle, std::vector<Point> foot_traj);

//UTILITY METHODS

float deg_to_rad(float degrees);
float norm(Point a, Point b);
std::vector<float> cubic_traj_gen(float y1, float z1, float y2, float z2, float y3, float z3);
std::vector<float> piecewise_cubic_traj_gen(float y_i, float z_i, float y_f, float z_f, float v_i, float v_f);
std::vector<Point> piecewise_cubic_poly(std::vector<Point> waypoints,std::vector<float> indep_var, std::vector<float> wp_velocities);
float gen_gaussian(float mean, float stddev);
std::vector<Point> compute_traj(std::vector<float> x, std::vector<float> traj_coeffs);
std::vector<Point> compute_linear_traj_2D(std::vector<float> x, float y_i, float y_f);
std::vector<float> compute_linear_traj_1D(std::vector<float> x, float y_i, float y_f);
float final_hip_height(float foothold, float pivot, float leg_length);
std::vector<Point> leg_forward_kinematics(Point com, float thigh_length, float shin_length, float hip_ang, float knee_ang);
std::vector<float> leg_inverse_kinematics(float com_y, float thigh_length, float shin_length, float knee_y, float ankle_y);
std::vector<float> create_time_vector(float time_unit, float duration);
std::vector<Point> calculate_full_leg_state(float thigh_length, float shin_length, float front_foot_length, float rear_foot_length,Point hip, Point ankle, bool knee_only);
float point_to_segment_dist(Point p, Point a, Point b);
float point_to_segment_dist_v2(Point p, Point a, Point b);
std::vector<float> intersection_score_and_min_dist(std::vector<Point> obstacle_shape, Point heel, Point tip);
std::vector<float> intersection_score_and_min_dist_v2(std::vector<Point> obstacle_shape, Point heel, Point tip); //this version adjusts the calculation of min dist (RECOMMENDED)
std::vector<float> intersection_score_and_min_dist_v3(std::vector<Point> obstacle_shape, Point heel, Point tip); //this version also output index of collided obstacle points
std::vector<Point> generate_sagittal_trajectory(std::vector<float> time, std::vector<Point> wp, std::string y_traj_type="linear", std::string z_traj_type="cubic", std::vector<float> wp_vel_z={});
std::vector<std::vector<Point>> cfftg_sagittal(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, std::vector<Point> obstacle_shape, float step_time, float time_unit=0.1);
void cfftg_cartesian_bundle_to_file(std::vector<std::vector<Point>> bundle, std::string filename);
void cfftg_angular_bundle_to_file(std::vector<std::vector<float>> bundle, std::string filename);
std::vector<float> derivative(std::vector<float> data);
std::vector<Point> select_waypoints(std::vector<Point> traj, float wp_dist_threshold, float precision,std::string type);
std::vector<float> fs_simulator(std::vector<Point> left_obs, std::vector<Point> right_obs, int swing_leg, float foot_length, float pivot);
float rad_to_deg(float rad);
void save_point_vect(std::vector<Point> vec, std::string vec_name, std::string filename);
std::vector<Point> generate_sagittal_trajectory_v2(std::vector<float> time, std::string y_traj_type, std::string z_traj_type, std::vector<Point> wp_z,std::vector<float> wp_vel_z,std::vector<Point> wp_y={}, std::vector<float> wp_vel_y={} );
std::vector<std::vector<Point>> cfftg_sagittal_v2(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, std::vector<Point> obstacle_shape, float step_time, float time_unit);
void save_vect_for_matlab(std::vector<Point> vec, std::string filename);
void save_ang_traj_matlab(std::vector<std::vector<float>> bundle, float time_unit,std::string filename);




