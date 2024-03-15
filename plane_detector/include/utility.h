#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <memory>
#include <stdlib.h>
#include <fstream>
#include <chrono>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
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
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Int8.h>

struct cmp_xyz {
	inline bool operator() (const pcl::PointXYZ& pt1, const pcl::PointXYZ& pt2){
		return (pt1.y < pt2.y);
	}

};

void printPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const char* filename) {
 std::ofstream outfile;
 outfile.open(filename);
 outfile << "[ ";
 
for(int i = 0; i < pc->points.size(); i++){
uint32_t rgb = *reinterpret_cast<uint32_t*>(&pc->points[i].rgb);
uint8_t r = (rgb>>16)&0x000000ff;
float rf = static_cast<float>(r) / 255; 
uint8_t g = (rgb>>8)&0x000000ff;
float rg = static_cast<float>(g) / 255; 
uint8_t b = (rgb)&0x000000ff;
float rb = static_cast<float>(b) / 255; 

	outfile << pc->points[i].x << " " << pc->points[i].y << " " << pc->points[i].z << " " << rb << " " << rg << " " << rb << " ; ";

}
outfile << " ] "<<std::endl;
outfile.close();
}

struct cmp_xyzrgb {
	inline bool operator() (const pcl::PointXYZRGB& pt1, const pcl::PointXYZRGB& pt2){
		return (pt1.y < pt2.y);
	}

};



float rotate_point_cloud_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, Eigen::Vector3f reference_plane_normal, Eigen::Vector3f point_cloud_plane_normal) {
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Zero();
  	float dot = reference_plane_normal.dot(point_cloud_plane_normal);
  	Eigen::Vector3f cross = dot<=0? point_cloud_plane_normal.cross(reference_plane_normal):reference_plane_normal.cross(point_cloud_plane_normal);
  	dot = -abs(dot);
	transform_2(0,1) = - cross[2];
	transform_2(1,0) =  cross[2];
	transform_2(0,2) =  cross[1];
	transform_2(2,0) =  -cross[1];
	transform_2(1,2) =  -cross[0];
	transform_2(2,1) =  cross[0];
  	transform_1 = transform_1 + transform_2 + (transform_2 * transform_2)*(1/(1+dot));
  	pcl::transformPointCloud (*pc, *pc, transform_1); // Applicazione parametri rotazione
  	return (acos(dot/point_cloud_plane_normal.norm())*180.0 / M_PI)-90;
}

float rotate_point_cloud_plane_v2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, Eigen::Vector3f point_cloud_plane_normal, Eigen::Vector3f reference_plane_normal) {
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Zero();
  	float dot = point_cloud_plane_normal.dot(reference_plane_normal);
  	Eigen::Vector3f cross =  point_cloud_plane_normal.cross(reference_plane_normal);
  	//dot = -abs(dot);
	transform_2(0,1) = - cross[2];
	transform_2(1,0) =  cross[2];
	transform_2(0,2) =  cross[1];
	transform_2(2,0) =  -cross[1];
	transform_2(1,2) =  -cross[0];
	transform_2(2,1) =  cross[0];
  	transform_1 = transform_1 + transform_2 + (transform_2 * transform_2)*(1/(1+dot));
  	pcl::transformPointCloud (*pc, *pc, transform_1); // Applicazione parametri rotazione
  	return (acos(dot/point_cloud_plane_normal.norm())*180.0 / M_PI)-90;
}

std::vector<float> leg_forward_kinematics(float com_y, float com_z, float L1, float L2, float hip_ang, float knee_ang){ //angles in radians, coordinates in meters
	std::vector<float> positions; // {knee pos y, knee pos z, ankle pos y, ankle pos z}
	positions.push_back(com_y + L1* sin(hip_ang));
	positions.push_back(com_z - L1* cos(hip_ang));
	positions.push_back(positions[0] + L2* sin(hip_ang+ knee_ang));
	positions.push_back(positions[1] - L2* cos(hip_ang+ knee_ang));
	return positions;
}

std::vector<float> leg_inverse_kinematics(float com_y, float thigh_length, float shin_length, float knee_y, float ankle_y) { //coordinates in meters
	std::vector<float> angles;
	angles.push_back(asin((knee_y - com_y)/thigh_length)); //hip angle (rad)
	angles.push_back(asin((ankle_y - knee_y)/shin_length) - angles[0]); //knee angle (rad)
	return angles;
}

float rad_to_deg(float rad){
	return rad*(180/M_PI);
}

float deg_to_rad(float degrees) {
    return degrees * (M_PI/180);
}

float euclidean_dist(float p1_x, float p1_y, float p2_x, float p2_y) {
	return sqrt(pow(p2_x - p1_x,2) + pow(p2_y - p1_y,2));
}

float calc_angle_from_side_sin(float side, float hypotenuse ){ 
return asin(side/hypotenuse);

} 



