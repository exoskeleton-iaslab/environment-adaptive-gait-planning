#include "utility.h"


std::vector<std::vector<Point>> cfftg_linear(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it,int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file=false, std::string txt_file_name="");



std::vector<std::vector<Point>> cfftg_linear_foot_cubic_hip(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, float min_dist_th, float hip_max_extension_time_coeff, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it, int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file, std::string txt_file_name);




std::vector<std::vector<Point>> cfftg_cubic(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, float min_dist_th, float hip_max_extension_time_coeff, float foot_peak_time_coeff, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it, int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file, std::string txt_file_name);


std::vector<std::vector<Point>> cfftg_cubic_v2(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, float min_dist_th, float hip_max_extension_time_coeff, float foot_peak_time_coeff, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it, int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file, std::string txt_file_name);



std::vector<std::vector<Point>> cfftg_cubic_double(float thigh_length, float shin_length, float rear_foot_length, float front_foot_length, float end, float hip_height, float pivot, float knee_angle, float min_dist_th, float hip_max_extension_time_coeff, float foot_peak_time_coeff_1, float foot_peak_time_coeff_2, std::vector<Point> obstacle_shape, float step_time, float time_unit, int max_it, int restart_it, float default_sh, float default_sv, float min_sigma_h, float min_sigma_v, float decay_coeff_h, float decay_coeff_v, bool produce_txt_file, std::string txt_file_name);
