#include "utility.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <iostream>
using namespace std::chrono;

//global vars
pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr mean_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_pivot_obs_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //to save obstacles of previous cycle
bool first_step=true;
bool new_data=false;
bool new_cloud=false;
bool new_leg=false;

bool origin_eliminated=false;
bool ref_acquired=false;
//variables below should be provided as ros params
float ema_coeff;
float max_step_length=1.0;
float step_length=0.0;
//variables below should be provided by topics
int swing_leg=-1; // "-1": both legs can be swing leg(first step)	"0": left swing leg	"1": right swing leg
float reference_tilt;
//float max_sl_const = 0.0; //constant used to infer maximum step length
float leaf_size;
int pc_counter =0; //used to create a point cloud buffer
pcl::PointCloud<pcl::PointXYZRGB> cloud_buffer[5]; //buffer of size 5

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob) {
    pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
    *temp_cloud = *cloud_blob;
    auto start= high_resolution_clock::now();
    //AVERAGING OF THE POINTCLOUD----------------------------------------
    if(!new_data)	{
        pcl::fromPCLPointCloud2 (*temp_cloud, *mean_cloud);
    }
    else{
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2 (*temp_cloud, *new_cloud);
        /*
        if(pc_counter<5) {
            //*cloud_buffer[pc_counter] =new pcl::PointCloud<pcl::PointXYZRGB>;
            std::cout<<"Fin qua ok"<<std::endl;
            //cloud_buffer[pc_counter] = new pcl::PointCloud<pcl::PointXYZRGB>;
            cloud_buffer[pc_counter] = * new_cloud;
            std::cout<<"Si rompe qui"<<std::endl;
            pc_counter++;
            *mean_cloud= *new_cloud;
        }
        else{
            for(int i=4;i>0;i--) {
                cloud_buffer[i-1] = cloud_buffer[i];
            }
            cloud_buffer[4] = *new_cloud;

            for(int j=0;j<mean_cloud->height;j++){
                for(int i=0;i<mean_cloud->width;i++){
                    //REMOVE ORIGIN POINT FROM CLOUD
                    //POINTCLOUD AVERAGING UPDATE
                    float means[3] = {0,0,0};
                    float medians[3][5];
                    for(int k=0;k<5;k++) {

                        //if(cloud_buffer[k].at(i,j).x!=NAN) means[0] += cloud_buffer[k].at(i,j).x;
                        //if(cloud_buffer[k].at(i,j).y!=NAN) means[1] += cloud_buffer[k].at(i,j).y;
                        //if(cloud_buffer[k].at(i,j).z!=NAN) means[2] += cloud_buffer[k].at(i,j).z;

                        medians[0][k] = cloud_buffer[k].at(i,j).x;
                        medians[1][k] = cloud_buffer[k].at(i,j).y;
                        medians[2][k] = cloud_buffer[k].at(i,j).z;
                    }
                    std::sort(medians[0], medians[0]+5);
                    std::sort(medians[1], medians[1]+5);
                    std::sort(medians[2], medians[2]+5);
                    means[0] = medians[0][2];
                    means[1] = medians[1][2];
                    means[2] = medians[2][2];

                    if(mean_cloud->at(i,j).x!=NAN) mean_cloud->at(i,j).x = means[0];
                    if(mean_cloud->at(i,j).y!=NAN) mean_cloud->at(i,j).y = means[1];
                    if(mean_cloud->at(i,j).z!=NAN) mean_cloud->at(i,j).z = means[2];

                    if(mean_cloud->at(i,j).x==0.0f && mean_cloud->at(i,j).y==0.0f) {
                        mean_cloud->at(i,j).x =NAN;
                        mean_cloud->at(i,j).y =NAN;
                        mean_cloud->at(i,j).z =NAN;
                    }

                }
            }
        }
        */
        *mean_cloud=*new_cloud;

    }
    //DOWNSAMPLING---------------------------------------------------
    pcl::PCLPointCloud2::Ptr in(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*mean_cloud, *in);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (in);
    //sor.setLeafSize (0.025f, 0.025f, 0.025f); ORIGINAL
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
    sor.filter (*cloud_filtered_blob);
    //Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *input_cloud);
    new_cloud=true;
    auto stop= high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout<<"Callback function time: " << duration.count()/1000 << " milliseconds" << std::endl;
}

void pivot_cb(const std_msgs::Float32::ConstPtr& msg){
    float pivot=msg->data; // pivot in CAMERA COORDS!!!!!!!
    //max_step_length = max_sl_const + pivot;//DA FINIRE
}

void leg_cb(const std_msgs::Int8::ConstPtr& msg){
    swing_leg=msg->data;
    new_leg=true;
}

void fs_cb(const std_msgs::Bool::ConstPtr& msg){
    first_step=msg->data; // pivot in CAMERA COORDS!!!!!!!
    //max_step_length = max_sl_const + pivot;//DA FINIRE
}

void step_length_cb(const std_msgs::Float32::ConstPtr& msg){
    step_length=msg->data;
}

void multiple_planes(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& planes,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_remaining){
    int min_inliers = 350;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setKSearch(50);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType(pcl::SAC_MSAC);
    seg.setNormalDistanceWeight(0.001);
    seg.setDistanceThreshold(0.01);
    seg.setMaxIterations(1000);
    seg.setEpsAngle(0.1f * M_PI / 180.0f);
    seg.setRadiusLimits(0.0, 0.05);
    seg.setProbability(0.99);



    while (cloud_remaining->points.size() > min_inliers) {
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setInputCloud(cloud_remaining);
        ne.compute(*cloud_normals);
        seg.setInputNormals(cloud_normals);
        seg.setInputCloud(cloud_remaining);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < min_inliers) {
            if (planes.size() == 0 and min_inliers > 200) {
                min_inliers -= 50;
                continue;
            }
            break;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (int index : inliers->indices) {
            plane->points.push_back(cloud_remaining->points[index]);
        }

        extract.setInputCloud(cloud_remaining);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*cloud_filtered);
        cloud_remaining.swap(cloud_filtered);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_clusters(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree_clusters->setInputCloud(cloud_remaining);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.04);
        ec.setMinClusterSize(30);
        ec.setMaxClusterSize(5000);
        ec.setSearchMethod(tree_clusters);
        ec.setInputCloud(cloud_remaining);
        ec.extract(cluster_indices);
        for (const auto& cluster : cluster_indices) {
            uint8_t r = 255, g = 0, b = 255;
            float min_y = 1000, max_y = -1000, min_x = 1000, max_x = -1000;
            for (const auto& index : cluster.indices) {
                float distance = coefficients->values[0] * cloud_remaining->points[index].x +
                                 coefficients->values[1] * cloud_remaining->points[index].y +
                                 coefficients->values[2] * cloud_remaining->points[index].z +
                                 coefficients->values[3];
                if (std::abs(distance) < 0.03) {
                    continue;
                }
                if (cloud_remaining->points[index].y < min_y) {
                    min_y = cloud_remaining->points[index].y;
                }
                if (cloud_remaining->points[index].y > max_y) {
                    max_y = cloud_remaining->points[index].y;
                }
                if (cloud_remaining->points[index].x < min_x) {
                    min_x = cloud_remaining->points[index].x;
                }
                if (cloud_remaining->points[index].x > max_x) {
                    max_x = cloud_remaining->points[index].x;
                }
                if (std::abs(max_y - min_y) > 0.2 || std::abs(max_x - min_x) > 0.2) {
                    continue;
                }
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                input_cloud->points.push_back(cloud_remaining->points[index]);
                input_cloud->points.back().rgb = *reinterpret_cast<float*>(&rgb);
            }
        }

        Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        normal.normalize();
        if (std::abs(normal.z()) > 0.7) {
            planes.push_back(plane);
        }

    }

    std::cout << "Totally found " << planes.size() << " planes." << std::endl;
}

float find_minimal_euclidian_distance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const Eigen::Vector3f& p){
    float min_distance = std::numeric_limits<float>::max();
    for (const auto point : cloud->points) {
        float distance = std::sqrt(std::pow(point.x - p.x(), 2) +
                                   std::pow(point.y - p.y(), 2));
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    return min_distance;
}

int define_ground_plane(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> planes, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ground_cloud, float& camera_offs_z, int& ground_points) {
    float min_distance = std::numeric_limits<float>::max();
    int min_index = 0;
    Eigen::Vector3f n_z(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f normal = compute_normal_pca(planes[0]);
    for (int idx = 0; idx < planes.size(); idx++) {
        if (planes[idx]->points.size() < 350) {
            continue;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        *plane = *planes[idx];
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(plane);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-0.2, 0.2);
        pass.filter(*plane);
        float tilt = rotate_point_cloud_plane(plane, n_z, normal.normalized());
        Eigen::Vector3f custom_point(0.0f, -4.0f, 0.0f);
        float distance = find_minimal_euclidian_distance(plane, custom_point);
        if (distance < min_distance) {
            min_distance = distance;
            min_index = idx;
        }
    }
    for (auto& point : planes[min_index]->points) {
        ground_cloud->points.push_back(point);
    }
    return min_index;
}

Eigen::Vector3f compute_normal_pca(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane) {
    Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*plane, centroid);
    pcl::computeCovarianceMatrixNormalized(*plane, centroid, covariance_matrix);
    Eigen::SelfAdjointEigenSolver <Eigen::Matrix3f> solver(covariance_matrix);
    return solver.eigenvectors().col(0);
}

void set_foothold_plane(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& planes,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ground_cloud,
                        int ground_plan_index, float feet_length, float camera_height, float mean_ground_z,
                        int& closest_plane_index, float& high_step, float& current_angle) {
    closest_plane_index = -1;
    high_step = 0.0;
    current_angle = 0.0;
    Eigen::Vector3f custom_point(0.0f, -5.0f, 0.0f);
    int default_closest_plane = -1;
    float min_distance = std::numeric_limits<float>::max();
    for (int idx = 0; idx < planes.size(); idx++) {
        float current_high = 0.0;
        for (auto& point : planes[idx]->points) {
            current_high += std::abs(point.z - mean_ground_z);
        }
        current_high /= planes[idx]->points.size();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        *plane = *planes[idx];
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(plane);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-0.2, 0.2);
        pass.filter(*plane);

        float min_plane_y = find_minimal_euclidian_distance(plane, custom_point);
        float plane_distance = std::abs(plane->points[0].y - plane->points[plane->points.size() - 1].y);
        std::cout << "plane_distance: " << plane_distance << std::endl;
        std::cout << "min_distance: " << min_distance << std::endl;
        std::cout << "feet_length: " << feet_length << std::endl;
        if (min_plane_y < min_distance){
            default_closest_plane = idx;
        }
        float eps = 0.05;
        if (min_plane_y < min_distance && (feet_length+eps) < plane_distance) {
            min_distance = min_plane_y;
            closest_plane_index = idx;
            high_step = current_high;
            Eigen::Vector3f normal_plane = compute_normal_pca(planes[idx]);
            current_angle = std::acos(normal_plane.dot(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
            current_angle = current_angle * 180 / M_PI;
            if (current_angle > 90.0) {
                current_angle = 180.0 - current_angle;
            }
        }

        if (closest_plane_index == -1){
            closest_plane_index = default_closest_plane;
        }
    }
    planes[closest_plane_index]->header = ground_cloud->header;
    planes[closest_plane_index]->height = 1;
    planes[closest_plane_index]->width = planes[closest_plane_index]->points.size();

    std::cout << "High step is " << high_step << std::endl;
    std::cout << "Angle is " << current_angle << std::endl;

//            if (std::abs(mean_ground_z - camera_height) > 0.05 && std::abs(current_angle) < 10.0) {
//                std::cout << "Ground plane is not detected properly height is " << std::abs(mean_ground_z) << std::endl;
//                float dist_y = std::abs(ground_cloud->points[0].y - ground_cloud->points[ground_points - 1].y);
//                float dist_z = std::abs(std::abs(mean_ground_z) - camera_height);
//                transpose_z(obs_cloud, dist_z, 0.0);
//                transpose_z(input_cloud, dist_z, 0.0);
//                transpose_z(input_cloud_original, dist_z, 0.0);
//                for(auto& plane : planes) {
//                    transpose_z(plane, dist_z, 0.0);
//                }
//                transpose_z(ground_cloud, 0.0, dist_y);
//                // planes.push_back(ground_cloud);
//            }

}

void align_point_cloud_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ground_cloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& obs_cloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud_original,
                         std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& planes,
                         bool& alignment_z, bool& ref_acquired, float& reference_tilt, float& tilt_ang) {
    if(!alignment_z) {
        Eigen::Vector3f n_z(0.0f, 0.0f, 1.0f);
        Eigen::Vector3f normal = compute_normal_pca(ground_cloud);
        float tilt;
        for (auto plane: planes) {
            tilt = rotate_point_cloud_plane(plane, n_z, normal.normalized());
        }
        tilt = rotate_point_cloud_plane(ground_cloud, n_z, normal.normalized());
        tilt = rotate_point_cloud_plane(input_cloud, n_z, normal.normalized());
        tilt = rotate_point_cloud_plane(obs_cloud, n_z, normal.normalized());
        tilt = rotate_point_cloud_plane(input_cloud_original, n_z, normal.normalized());
        if (!ref_acquired) {
            reference_tilt = tilt;
            ref_acquired = true;
        } else {
            tilt_ang = tilt;
            alignment_z = true;
        }
    }
}

void transpose_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& points, float z, float y){
    for (auto& point : points->points) {
        point.z += z;
        point.y -= y;
    }
}

void group_obstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& obs_cloud, ros::Publisher& obstacles_distance_pub) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obs_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    obs_cloud2->header = obs_cloud->header;
    obs_cloud2->height = 1;
    obs_cloud2->width = obs_cloud->size();
    *obs_cloud2 = *obs_cloud;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_clusters(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree_clusters->setInputCloud(obs_cloud2);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.04);
    ec.setMinClusterSize(30);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree_clusters);
    ec.setInputCloud(obs_cloud2);
    ec.extract(cluster_indices);

    obs_cloud->points.clear();
    std::vector<std::vector<pcl::PointXYZRGB>> current_clusters;

    for (const auto& cluster : cluster_indices) {
        std::vector<pcl::PointXYZRGB> single_cluster;
        for (const auto& index : cluster.indices) {
            single_cluster.push_back(obs_cloud2->points[index]);
        }
        current_clusters.push_back(single_cluster);
    }

    int i=-1;
    std_msgs::Float32MultiArray msg;
    for (const auto& cluster : current_clusters) {
        i++;
        uint8_t r = 255, g = 0, b = 255;

        float min_distance = std::numeric_limits<float>::max();
        for (const auto& point : cluster) {
            float distance = std::sqrt(std::pow(point.x, 2) +
                                       std::pow(point.y, 2) +
                                       std::pow(point.z, 2));
            if (distance < min_distance) {
                min_distance = distance;
            }
            pcl::PointXYZRGB colored_point = point;
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            colored_point.rgb = *reinterpret_cast<float*>(&rgb);
            obs_cloud->points.push_back(colored_point);
        }
//        if (min_distance < 1.5){
//            msg.data.push_back(min_distance);
//        }
        std::cout << "Obstacle at distance: " << min_distance << " meters " << "of cluster number " << i << std::endl;
    }
    obstacles_distance_pub.publish(msg);
}

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

std::vector<float> extractNearestClusterYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::vector<float> result;
    if(cloud->points.empty())
        return result;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.04);
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    if(cluster_indices.empty())
        return result;

    int best_idx = -1;
    float best_dist = 9999.0f;

    for(size_t i = 0; i < cluster_indices.size(); i++){
        float cx = 0, cy = 0, cz = 0;

        for(int idx : cluster_indices[i].indices){
            cx += cloud->points[idx].x;
            cy += cloud->points[idx].y;
            cz += cloud->points[idx].z;
        }

        cx /= cluster_indices[i].indices.size();
        cy /= cluster_indices[i].indices.size();
        cz /= cluster_indices[i].indices.size();

        float dist = std::sqrt(cx*cx + cy*cy + cz*cz);
        if(dist < best_dist){
            best_dist = dist;
            best_idx = i;
        }
    }

    for(int idx : cluster_indices[best_idx].indices){
        result.push_back(cloud->points[idx].y);
        result.push_back(cloud->points[idx].z);
    }

    return result;
}


void color_planes(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& planes, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& planes_cloud) {
    int jj=0, zz=0, rr=255;
    std::sort(planes.begin(), planes.end(), [](const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& a, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& b) {
        return a->points.size() > b->points.size();
    });
    for (int idx = 0; idx < planes.size(); idx++) {
        std::cout << "Plane " << idx << " has " << planes[idx]->points.size() << " points." << std::endl;
        uint8_t r = rr % 256, g = jj % 256, b = zz % 256;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
        jj += 130;
        zz += 75;
        rr -= 50;

        for (auto& point : planes[idx]->points) {
            point.rgb = *reinterpret_cast<float*>(&rgb);
            planes_cloud->points.push_back(point);
        }
    }
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "robotic_vision");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub_cloud = nh.subscribe ("camera/depth/color/points", 1, cloud_cb);
    ros::Subscriber sub_pivot = nh.subscribe ("pivot_camera_coord", 1, pivot_cb);
    ros::Subscriber sub_leg = nh.subscribe ("swing_leg", 1, leg_cb);
    ros::Subscriber sub_fs = nh.subscribe ("first_step_global", 1, fs_cb);
    ros::Subscriber sub_step_length = nh.subscribe ("step_length", 1, step_length_cb);
    //bool subscriber should be added in order to know whether the robotic vision module should be active
    ros::Publisher obstacles_distance_pub = nh.advertise<std_msgs::Float32MultiArray>("obstacles_distance", 1);
    ros::Publisher pub = nh.advertise<pcl::PCLPointCloud2> ("outcloud", 1);
    ros::Publisher pub2 = nh.advertise<pcl::PCLPointCloud2> ("obstacles", 1);
    ros::Publisher pub3 = nh.advertise<pcl::PCLPointCloud2> ("tracks", 1);
    ros::Publisher pub_planes = nh.advertise<pcl::PCLPointCloud2> ("planes", 1);
    ros::Publisher pub_ground = nh.advertise<pcl::PCLPointCloud2> ("ground", 1);
    ros::Publisher pub4 = nh.advertise<std_msgs::Float32MultiArray> ("obstacle_shape_raw", 1);
    ros::Publisher pub5 = nh.advertise<std_msgs::Float32MultiArray> ("obstacle_shape_raw_unseen", 1);
    //ros::Publisher pub5 = nh.advertise<std_msgs::Bool> ("next_swing_leg", 1);
    ros::Publisher pub6 = nh.advertise<std_msgs::Float32> ("CoM_height_raw", 1);
    ros::Publisher pub7 =nh.advertise<std_msgs::Float32> ("foothold_raw", 1);
    ros::Publisher pub8 = nh.advertise<std_msgs::Float32> ("reference_tilt", 1);
    ros::Publisher pub9 = nh.advertise<std_msgs::Float32> ("tilt_angle", 1);
    ros::Publisher pub10 = nh.advertise<std_msgs::Float32> ("foot_tip_pos", 1);
    ros::Publisher foothold_height = nh.advertise<std_msgs::Float32>("foothold_height", 1);
    ros::Publisher stair_edge = nh.advertise<std_msgs::Float32>("stair_edge_raw", 1);
    float dist_bt_feet;
    std::string temp;
    ros::param::get("~dist_origin_foot", dist_bt_feet);
    //std::cout<< temp << std::endl;
    //dist_bt_feet= std::stof(temp);
    std::cout<< dist_bt_feet << std::endl;
    float ff_l,rf_l;
    ros::param::get("~front_foot_length", ff_l);
    ros::param::get("~rear_foot_length", rf_l);
    float feet_length = ff_l + rf_l;
    float feet_width;
    ros::param::get("~foot_width", feet_width);
    float max_step_height;
    ros::param::get("~max_step_height", max_step_height);
    ros::param::get("~ema_coeff", ema_coeff);
    float thigh_length, shin_length;
    ros::param::get("~thigh_length", thigh_length);
    ros::param::get("~shin_length", thigh_length);
    float add_dist;
    ros::param::get("~add_dist", add_dist);
    ros::param::get("~leaf_size", leaf_size);
    int ransac_max_it;
    float ransac_th;
    ros::param::get("~ransac_max_it", ransac_max_it);
    ros::param::get("~ransac_th", ransac_th);
    bool x_alignment;
    ros::param::get("~x_alignment", x_alignment);
    float camera_height;
    ros::param::get("~camera_height", camera_height);

    //float min_height_constraint = 0.9; //max CoM height allowed: 0.9 * leg length (SHOULD BE A PARAM??)
    //max_sl_const = 2* sqrt(1-pow(min_height_constraint,2)) *  (thigh_length+shin_length);
    //max_step_length = max_sl_const;
    float final_step_height = -1.0;
    float slope_height = 1000000.0;
    std_msgs::Float32 msg_stair_edge, msg_step_height;
    msg_stair_edge.data = -1;
    std::cout<<"Max it: " << ransac_max_it << std::endl;
    std::cout<<"Threshold: " << ransac_th << std::endl;
    bool enter_flag = true;
    std::queue<float> step_height_queue;
    std::queue<float> stair_edge_queue;
    while(ros::ok()){
        new_data= new_leg && new_cloud;
        if(new_data){
            auto start= high_resolution_clock::now();
            //plane detection setup
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (ransac_max_it);
            seg.setDistanceThreshold (ransac_th);
            seg.setInputCloud (input_cloud);
            int i = 0, nr_points = (int) input_cloud->points.size();
            pcl::IndicesPtr remaining (new std::vector<int>);
            remaining->resize (nr_points);
            for (size_t i = 0; i < remaining->size (); i++) { (*remaining)[i] = static_cast<int>(i); }

            //variables used for z-axis transalation
            float camera_offs_z=0.0;
            int ground_points=0;
            bool alignment_z=false; //to check whether alignment on the z-axis has been performed
            float tilt_ang;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //pointcloud containing ground plane points
            ground_cloud->header = input_cloud->header; //needed to display in the right frame
            ground_cloud->height=1; //unordered cloud

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_original(new pcl::PointCloud<pcl::PointXYZRGB>);
            *input_cloud_original = *input_cloud;
            input_cloud_original->header = input_cloud->header;
            input_cloud_original->height = 1;
            input_cloud_original->width = input_cloud->size();
            *ground_cloud = *input_cloud;

            //track bounds and conditions
            float high_obs_bound_y=max_step_length; //nearest obstacle w. height >20 cm
            float bound_1 = (dist_bt_feet + feet_width + 0.02f);
            float bound_2 = (dist_bt_feet-0.02f);
            float bound_3 = max_step_length + 0.5f;
            bool condition_1, condition_2, condition_3;

            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> planes;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_remaining(new pcl::PointCloud<pcl::PointXYZRGB>);
            *cloud_remaining = *ground_cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr obs_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            obs_cloud->header = input_cloud->header;
            obs_cloud->height=1;
            multiple_planes(planes, obs_cloud, cloud_remaining);
            if(planes.size() == 0) {
                std::cout << "No planes found." << std::endl;
                planes.push_back(cloud_remaining);
            }

            ground_cloud->points.clear();
            ground_cloud->header = input_cloud->header;
            ground_cloud->height=1;
            int ground_plan_index = define_ground_plane(planes, ground_cloud, camera_offs_z, ground_points);

            align_point_cloud_z(ground_cloud, input_cloud, obs_cloud, input_cloud_original, planes, alignment_z, ref_acquired, reference_tilt, tilt_ang);
            float mean_ground_z = 0.0;
            for (auto& point : planes[ground_plan_index]->points) {
                mean_ground_z += point.z;
                ground_points++;
            }
            mean_ground_z /= ground_points;

            ground_cloud->width = ground_cloud->points.size();

            int closest_plane_index = -1;
            float high_step = 0.0, current_angle = 0.0;
            set_foothold_plane(planes, ground_cloud, ground_plan_index, feet_length, camera_height, mean_ground_z, closest_plane_index, high_step, current_angle);

            camera_offs_z = 0.0, ground_points = 0;
            for (auto& point : ground_cloud->points) {
                camera_offs_z += point.z;
                ground_points++;
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr planes_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            color_planes(planes, planes_cloud);
            planes_cloud->header = input_cloud->header;
            planes_cloud->width = planes_cloud->size();
            planes_cloud->height=1;

            for (size_t i = 0; i < planes.size(); i++)
            {
                if (i == closest_plane_index)
                    continue;
                *obs_cloud += *planes[i];
            }
            // *obs_cloud += *input_cloud;
            group_obstacles(obs_cloud, obstacles_distance_pub);
            obs_cloud->width = obs_cloud->size();

            //ALIGN THE POINTCLOUD WITH RVIZ FRAME(X-AXIS)------------------------------------------------------------

            if(x_alignment){
                float mi_x;
                float mi_y=10;
                float ma_x;
                float ma_y=10;
                for(int i=0;i<ground_cloud->points.size();i++){
                    if(ground_cloud->points[i].x>0) {
                        if(ground_cloud->points[i].y < ma_y){ //swapped the disequality from < to >
                            ma_x=ground_cloud->points[i].x;
                            ma_y=ground_cloud->points[i].y;
                        }
                    }
                    if(ground_cloud->points[i].x<0){
                        if(ground_cloud->points[i].y < mi_y){
                            mi_x=ground_cloud->points[i].x;
                            mi_y=ground_cloud->points[i].y;
                        }
                    }
                }
                Eigen::Vector3f n_x(1.0f,0.0f, 0.0f);
                Eigen::Vector3f n_align(ma_x-mi_x,ma_y-mi_y,0.0f);
                n_align.normalize();
                float tilt_ang_x;
                tilt_ang_x = rotate_point_cloud_plane_v2(input_cloud,n_align,n_x);
                std::cout<<"Rotation around x-axis is: "<<tilt_ang_x<<std::endl;
                rotate_point_cloud_plane_v2(ground_cloud,n_align,n_x);
                rotate_point_cloud_plane_v2(planes_cloud,n_align,n_x);
                rotate_point_cloud_plane_v2(obs_cloud,n_align,n_x);
                rotate_point_cloud_plane_v2(planes[closest_plane_index],n_align,n_x);
            }


            //POINTCLOUD TRANSLATION (Z-AXIS)--------------------------------------------------------------------------
            camera_offs_z/=ground_points;
            Eigen::Affine3f offset_transform = Eigen::Affine3f::Identity();
            //std::cout << "Camera height w.r.t. ground: " << -camera_offs_z << std::endl;
            offset_transform.translation() <<0.0,0.0,-camera_offs_z; //added translation on z-axis
            pcl::transformPointCloud (*input_cloud, *input_cloud, offset_transform); // Apply traslation
            pcl::transformPointCloud (*ground_cloud, *ground_cloud, offset_transform);
            pcl::transformPointCloud (*obs_cloud, *obs_cloud, offset_transform);
            pcl::transformPointCloud (*planes_cloud, *planes_cloud, offset_transform);
            pcl::transformPointCloud (*planes[closest_plane_index], *planes[closest_plane_index], offset_transform);
            //TRACKS PROCESSING-----------*CURRENTLY UNDER DEVELOPMENT*-------------------------------------------------------------------------------
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr ltrack_obs_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //obstacle points inside the left track
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr rtrack_obs_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //obstacle points inside the right track
            ltrack_obs_cloud->header = input_cloud->header; //needed to display in the right frame
            rtrack_obs_cloud->header = input_cloud->header; //needed to display in the right frame
            ltrack_obs_cloud->height=1;
            rtrack_obs_cloud->height=1;
            //Identify obstacle points near tracks
            for(int i=0;i<obs_cloud->points.size();i++){
                condition_1 = obs_cloud->points[i].x>-bound_1 && obs_cloud->points[i].x<-bound_2;
                condition_2 = obs_cloud->points[i].x<bound_1 && obs_cloud->points[i].x>bound_2;
                //condition_3 = obs_cloud->points[i].y < bound_3 && obs_cloud->points[i].y>=0; //neglect obstacles behind the camera so that feet are not considered
                condition_3 = obs_cloud->points[i].y < bound_3; //NEW
                if(condition_3){
                    if(condition_1) {
                        //if an obstacle is too high, cant step past that(applies to both legs)
                        if(obs_cloud->points[i].z>2.0 && obs_cloud->points[i].y<high_obs_bound_y){
                            high_obs_bound_y=obs_cloud->points[i].y;

                        }
                        ltrack_obs_cloud->points.push_back(obs_cloud->points[i]);

                    }
                    if(condition_2) {
                        //if an obstacle is too high, cant step past that(applies to both legs)
                        if(obs_cloud->points[i].z>2.0 && obs_cloud->points[i].y<high_obs_bound_y){
                            high_obs_bound_y=obs_cloud->points[i].y;

                        }
                        rtrack_obs_cloud->points.push_back(obs_cloud->points[i]);
                    }
                    if(((obs_cloud->points[i].x>-bound_1-0.03f && obs_cloud->points[i].x<-bound_2+0.03f) || (obs_cloud->points[i].x<bound_1+0.03f && obs_cloud->points[i].x>bound_2-0.03f)) && obs_cloud->points[i].y < bound_3 - 0.5f) {
                        for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it) {
                            uint8_t r = 0, g = 255, b = 0;
                            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                            if(input_cloud->at(*it).x==obs_cloud->points[i].x && input_cloud->at(*it).y==obs_cloud->points[i].y && input_cloud->at(*it).z==obs_cloud->points[i].z) {
                                input_cloud->at(*it).rgb = *reinterpret_cast<float*>(&rgb); //color the obstacle point (green)
                                break;
                            }

                        }
                    }
                }
            }


            ltrack_obs_cloud->width = ltrack_obs_cloud->points.size();
            rtrack_obs_cloud->width = rtrack_obs_cloud->points.size();

            //FOOT RECOGNITION (ADDED 18/01/2024)

            std::sort(ltrack_obs_cloud->points.begin(),ltrack_obs_cloud->points.end(), cmp_xyzrgb());
            std::sort(rtrack_obs_cloud->points.begin(),rtrack_obs_cloud->points.end(), cmp_xyzrgb());
            float consec_dist=0; //measure distance between consecutive obstacle points (useful to know where the foot cluster ends)
            float foot_tip_pos=-1;



            if(swing_leg==0){
                std::cout<<"Checking distance between left foot and obs"<<std::endl;
                for(int i=1; i< rtrack_obs_cloud->points.size(); i++) {
                    consec_dist = rtrack_obs_cloud->points[i].y - rtrack_obs_cloud->points[i-1].y;
                    if(consec_dist>0.05f ){ //threshold "a caso" che identifica la separazione tra il piede e gli altri ostacoli.
                        std::cout<<"Found distance for right foot"<<std::endl;
                        foot_tip_pos = rtrack_obs_cloud->points[i-1].y;
                        break;
                    }
                    if(i == rtrack_obs_cloud->points.size()-1) {
                        std::cout<<"Found distance for right foot. There are no obstacles on the track"<<std::endl;
                        foot_tip_pos = rtrack_obs_cloud->points[i].y;
                    }
                }
                // ELIMINA PIEDE DA LISTA OSTACOLI
                /*
                while (ltrack_obs_cloud->points.size()>0 && ltrack_obs_cloud->points[0].y<=foot_tip_pos+0.04) {
                    ltrack_obs_cloud->points.erase(ltrack_obs_cloud->points.begin());
                }
                */
            }
            else if(swing_leg==1){
                std::cout<<"Checking distance between right foot and obs"<<std::endl;
                for(int i=1; i< ltrack_obs_cloud->points.size(); i++) {
                    consec_dist = ltrack_obs_cloud->points[i].y - ltrack_obs_cloud->points[i-1].y;
                    if(consec_dist>0.05f){ //threshold "a caso" che identifica la separazione tra il piede e gli altri ostacoli
                        std::cout<<"Found distance for left foot"<<std::endl;
                        foot_tip_pos = ltrack_obs_cloud->points[i-1].y;
                        break;
                    }
                    if(i == ltrack_obs_cloud->points.size()-1) {
                        std::cout<<"Found distance for left foot. There are no obstacles on the track"<<std::endl;
                        foot_tip_pos = ltrack_obs_cloud->points[i].y;
                    }
                }
                // ELIMINA PIEDE DA LISTA OSTACOLI
                /*
                while (rtrack_obs_cloud->points.size()>0 && rtrack_obs_cloud->points[0].y<=foot_tip_pos+0.04) {
                    rtrack_obs_cloud->points.erase(rtrack_obs_cloud->points.begin());
                }
                */
            }
            if(foot_tip_pos==-1) consec_dist=0;
            std::cout<<"Distance between pivot foot and obs is: " << consec_dist<<std::endl;
            std::cout<<"Pivot tip position (camera coords) is: "<< foot_tip_pos<<std::endl;

            //END OF FOOT RECOGNITION


            //PIVOT OBSTACLES SAVING
            /*
            if(!first_step){ //first step has already been performed, so we substitute the swinging leg obs with the previously saved ones
                if(swing_leg==0){
                    ltrack_obs_cloud->points = old_pivot_obs_cloud->points;
                    ltrack_obs_cloud->width = 	old_pivot_obs_cloud->width;
                }
                else if(swing_leg==1){
                    rtrack_obs_cloud->points = old_pivot_obs_cloud->points;
                    rtrack_obs_cloud->width = 	old_pivot_obs_cloud->width;
                }
            }
            //save obstacles in front of pivot
            if(swing_leg==0){
                old_pivot_obs_cloud->points = rtrack_obs_cloud->points;
                old_pivot_obs_cloud->width = rtrack_obs_cloud->points.size();

            }
            else if(swing_leg==1){
                old_pivot_obs_cloud->points = ltrack_obs_cloud->points;
                old_pivot_obs_cloud->width = ltrack_obs_cloud->points.size();

            }

            if(first_step) {	//initialize saved obs header, and set flag to substitute them in the next cycle
                old_pivot_obs_cloud->header = input_cloud->header;
                old_pivot_obs_cloud->height=1;
                //first_step=true;
            }
            */
            //END OF PIVOT OBSTACLES SAVING

            //EVALUATE TRACKS----------------------------------------------------------------------------
            std::vector<pcl::PointXYZRGB> tracks[2]; //points that could be stepped on(represented as tracks)
            float stddev;
            if(first_step) stddev = 0.4f; //stddev of optimal step length
            else stddev = 0.55f; //CHANGED 13/03/2024
            float mean = step_length;
            float exponential_component;
            float min_obs_dist;
            float max_obs_dist;
            float min_score;
            float score;
            float dist;
            uint8_t r,g,b;
            uint32_t rgb;
            for(int i=0;i<planes[closest_plane_index]->points.size();i++){
                condition_1 = planes[closest_plane_index]->points[i].x>-bound_1 && planes[closest_plane_index]->points[i].x<-bound_2;
                condition_2 = planes[closest_plane_index]->points[i].x<bound_1 && planes[closest_plane_index]->points[i].x>bound_2;
                condition_3 = planes[closest_plane_index]->points[i].y < high_obs_bound_y;
                min_score=1.0f;
                score=1.0f;
                if(condition_3){
                    if(condition_1) { //left track
                        for(int j=0;j<ltrack_obs_cloud->points.size();j++){
                            if(ltrack_obs_cloud->points[j].y<high_obs_bound_y){//second condition added 18/01/2024
                                dist=sqrt(pow(planes[closest_plane_index]->points[i].x-ltrack_obs_cloud->points[j].x,2)+pow(planes[closest_plane_index]->points[i].y-ltrack_obs_cloud->points[j].y,2));
                                //min_obs_dist = ltrack_obs_cloud->points[j].z * (1+add_dist);
                                min_obs_dist = ltrack_obs_cloud->points[j].z + add_dist;
                                max_obs_dist = min_obs_dist + 0.05;
                                if(dist<min_obs_dist) {
                                    score =0.0f;
                                }
                                else {score =  dist>max_obs_dist? 1.0f: dist/max_obs_dist;}
                                if(score<min_score) min_score = score;
                            }
                        }
                        exponential_component = exp(-(pow((planes[closest_plane_index]->points[i].y-mean) / stddev,2.0)));
                        b = (uint8_t)255*min_score* exponential_component;
                        rgb = (uint32_t)b;
                        planes[closest_plane_index]->points[i].rgb = *reinterpret_cast<float*>(&rgb);
                        tracks[0].push_back(planes[closest_plane_index]->points[i]);
                    }
                    if(condition_2) { //right track
                        for(int j=0;j<rtrack_obs_cloud->points.size();j++){
                            if(rtrack_obs_cloud->points[j].y<high_obs_bound_y){ //second condition added 18/01/2024
                                dist=sqrt(pow(planes[closest_plane_index]->points[i].x-rtrack_obs_cloud->points[j].x,2)+pow(planes[closest_plane_index]->points[i].y-rtrack_obs_cloud->points[j].y,2));
                                //min_obs_dist = rtrack_obs_cloud->points[j].z* (1+add_dist);
                                min_obs_dist = rtrack_obs_cloud->points[j].z + add_dist;
                                max_obs_dist = min_obs_dist + 0.05;
                                if(dist<min_obs_dist) {
                                    score =0.0f;
                                }
                                else {score =  dist>max_obs_dist? 1.0f: dist/max_obs_dist;}
                                if(score<min_score) min_score = score;
                            }
                        }
                        exponential_component = exp(-(pow((planes[closest_plane_index]->points[i].y-mean) / stddev,2.0)));
                        r = (uint8_t)255*min_score* exponential_component;
                        rgb = (uint32_t)r<<16;
                        planes[closest_plane_index]->points[i].rgb = *reinterpret_cast<float*>(&rgb);
                        tracks[1].push_back(planes[closest_plane_index]->points[i]);
                    }
                    if(!(condition_1 || condition_2)) { //point not on tracks
                        rgb = (uint32_t)0;
                        planes[closest_plane_index]->points[i].rgb = *reinterpret_cast<float*>(&rgb);
                    }
                }
                else {
                    rgb = (uint32_t)0;
                    planes[closest_plane_index]->points[i].rgb = *reinterpret_cast<float*>(&rgb);
                }
            }

            /* PRINT SWING LEG OBSTACLE TRACK
            if(swing_leg==0){
                std::cout<<"Obstacles on the left track: [ "<<std::endl;
                for(int i=0;i<ltrack_obs_cloud->points.size();i++){
                    std::cout<<"("<< ltrack_obs_cloud->points[i].y<<"  "<<ltrack_obs_cloud->points[i].z<<")";
                    if(i<ltrack_obs_cloud->points.size()-1) std::cout<<" , ";
                }
                std::cout<<" ]"<<std::endl;
            }
            else if (swing_leg==1) {
                std::cout<<"Obstacles on the right track: [ "<<std::endl;
                for(int i=0;i<rtrack_obs_cloud->points.size();i++){
                    std::cout<<"("<< rtrack_obs_cloud->points[i].y<<"  "<<rtrack_obs_cloud->points[i].z<<")";
                    if(i<rtrack_obs_cloud->points.size()-1) std::cout<<" , ";
                }
                std::cout<<" ]"<<std::endl;


            }*/


            std::cout<<"Swing leg is: "<<swing_leg<<std::endl;


            //FIND BEST FOOTHOLD FROM TRACKS----------------------------------------------------------------------------
            std::vector<pcl::PointXYZRGB> best_window[2];
            bool valid_move[2] = {swing_leg!=1,swing_leg!=0}; //if swing_leg==-1 both legs can be the swinging one
            float window_y_position =0.0f;
            int l_i=0;
            int u_i=0;
            float max_score[2] ={0.0f,0.0f};
            float max_score_win_y_pos[2] ={0.0f, 0.0f}; //heel
            long score_sum=0;
            std::vector<pcl::PointXYZRGB> window;
            bool invalid=false;
            for(int i=0;i<2;i++) {
                l_i=0;
                u_i=0;
                if(valid_move[i] && tracks[i].size()>10) { //20 is a reasonable number of points
                    std::sort(tracks[i].begin(), tracks[i].end(), cmp_xyzrgb());
                    if(tracks[i].size()>0) window_y_position = tracks[i][0].y;
                    else window_y_position=0.0f; //shouldnt happen
                    while(window_y_position<=max_step_length-feet_length){
                        score_sum=0;
                        score=0.0; //reusing score variable declared at line 303
                        window.clear();
                        while(l_i < tracks[i].size() && tracks[i][l_i].y<window_y_position) l_i++;
                        if(l_i> u_i) u_i = l_i;
                        while(u_i < tracks[i].size() && tracks[i][u_i].y<= window_y_position+feet_length) u_i++;
                        //fill window and calculate its cumulative score
                        for(int k = l_i; k<=u_i; k++){
                            uint32_t value = *reinterpret_cast<uint32_t*>(&tracks[i][k].rgb);
                            if(value==0){
                                break; //invalid window (point with value 0 means obstacle is too close)
                            }
                            else{
                                if(i==1) value = value >>16;
                                score_sum += static_cast<long>(value);
                                window.push_back(tracks[i][k]);
                            }
                        }
                        //evaluate if window is compliant and update best window
                        if(window.size()>0 && abs((window[window.size()-1].y - window[0].y) - feet_length)<0.02 && window[window.size()-1].y< high_obs_bound_y){
                            score = score_sum / static_cast<float>(window.size()); //MEAN SCORE
                            if(score > 0 && score > max_score[i])  {
                                max_score[i] = score;
                                max_score_win_y_pos[i]= window_y_position;
                                best_window[i].clear();
                                best_window[i]=window;
                            }
                        }
                        window_y_position+=0.01;
                    }

                }
            }
            /*
            bool next_swing_leg;
            if(swing_leg = -1) {
                if(max_score[0] > max_score[1]) next_swing_leg = 0;
                else next_swing_leg = 1;
                std::cout<< "First step position: " << max_score_win_y_pos[next_swing_leg] << "for leg " << next_swing_leg<< std::endl;
            }
           else {
               std::cout << "Next step position: " << max_score_win_y_pos[swing_leg]<< std::endl;

           }

           */
            std::cout << "Next step position: " << max_score_win_y_pos[swing_leg]<< std::endl;

            //Color selected foothold points
            for(int i=0; i<2;i++) {
                for(int k=0; k<best_window[i].size();k++){
                    int colored_points=0;
                    for(int j=0; j< planes[closest_plane_index]->points.size();j++){
                        if(best_window[i][k].x ==planes[closest_plane_index]->points[j].x &&  best_window[i][k].y ==planes[closest_plane_index]->points[j].y && best_window[i][k].z ==planes[closest_plane_index]->points[j].z){
                            if(planes[closest_plane_index]->points[j].z < slope_height){
                                slope_height = planes[closest_plane_index]->points[j].z;
                            }
                            uint8_t r = 255, g = 255, b = 0;
                            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                            planes[closest_plane_index]->points[j].rgb = *reinterpret_cast<float*>(&rgb);
                            colored_points++;
                            if(colored_points==best_window[i].size()) break;
                        }
                    }
                }
            }

            //PUBLISH DATA ------------------------------------------------------------------------------------------
            pcl::PCLPointCloud2 outcloud;
            pcl::toPCLPointCloud2(*input_cloud, outcloud); //cloud with highlighted obstacles
            pub.publish (outcloud);
            pcl::PCLPointCloud2 outcloud2;
            pcl::toPCLPointCloud2(*obs_cloud, outcloud2); //obstacles only
            pub2.publish (outcloud2);
            pcl::PCLPointCloud2 outcloud3;
            pcl::toPCLPointCloud2(*planes[closest_plane_index], outcloud3); //ground only
            pub3.publish (outcloud3);
            pcl::PCLPointCloud2 outcloud4;
            pcl::toPCLPointCloud2(*planes_cloud, outcloud4); //planes only
            pub_planes.publish(outcloud4);
            pcl::PCLPointCloud2 outcloud5;
            pcl::toPCLPointCloud2(*ground_cloud, outcloud5); //ground only
            pub_ground.publish(outcloud5);

            //publish obstacle shape of swing leg in sagittal plane
            std::vector<float> o_p;
            std::vector<float> o_p_unseen;
            /*
            if(swing_leg==-1){
                if(next_swing_leg==0){
                    //std::sort(ltrack_obs_cloud->points.begin(), ltrack_obs_cloud->points.end(), cmp_xyzrgb());
                    for(int i=0; i<ltrack_obs_cloud->points.size(); i++) {
                        o_p.push_back(ltrack_obs_cloud->points[i].y);
                        o_p.push_back(ltrack_obs_cloud->points[i].z);
                    }
                }
                else{
                    //std::sort(rtrack_obs_cloud->points.begin(), rtrack_obs_cloud->points.end(), cmp_xyzrgb());
                    for(int i=0; i<rtrack_obs_cloud->points.size(); i++) {
                        o_p.push_back(rtrack_obs_cloud->points[i].y);
                        o_p.push_back(rtrack_obs_cloud->points[i].z);
                    }
                }
            }
            */

            if(swing_leg==0){
                o_p = extractNearestClusterYZ(ltrack_obs_cloud);
                o_p_unseen = extractNearestClusterYZ(rtrack_obs_cloud);
            }

            if(swing_leg==1){
                o_p = extractNearestClusterYZ(rtrack_obs_cloud);
                o_p_unseen = extractNearestClusterYZ(ltrack_obs_cloud);
            }


            std_msgs::Float32MultiArray outp; //obstacle array: even indexes-> y coordinate   odd indexes->z coordinate
            outp.layout.dim.push_back(std_msgs::MultiArrayDimension());
            outp.layout.dim[0].size = o_p.size();
            outp.layout.dim[0].stride = 1;
            outp.layout.dim[0].label = "y-z"; // or whatever name you typically use to index
            outp.data.clear();
            outp.data.insert(outp.data.end(), o_p.begin(), o_p.end());
            pub4.publish(outp);

            std_msgs::Float32MultiArray outp_unseen;
            outp_unseen.layout.dim.push_back(std_msgs::MultiArrayDimension());
            outp_unseen.layout.dim[0].size = o_p_unseen.size();
            outp_unseen.layout.dim[0].stride = 1;
            outp_unseen.layout.dim[0].label = "y-z";
            outp_unseen.data.clear();
            outp_unseen.data.insert(outp_unseen.data.end(), o_p_unseen.begin(), o_p_unseen.end());
            pub5.publish(outp_unseen);

            //publish foothold
            std_msgs::Float32 msg;
            /*
            std_msgs::Bool bmsg;
            bmsg.data= next_swing_leg==0?true:false; // true means left leg
            pub5.publish(bmsg);
            */
            msg.data = -camera_offs_z;
            pub6.publish(msg);
            /*
            if (swing_leg==-1) msg.data = max_score_win_y_pos[next_swing_leg];
            else msg.data = max_score_win_y_pos[swing_leg];
            */
            msg.data = max_score_win_y_pos[swing_leg];
            pub7.publish(msg);
            msg.data = reference_tilt;
            pub8.publish(msg);
            msg.data = tilt_ang;
            pub9.publish(msg);
            msg.data= foot_tip_pos;
            pub10.publish(msg);

            Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
            Eigen::Vector3f normal_plane = compute_normal_pca(planes[closest_plane_index]);
            float dot = normal_plane.dot(z_axis);
            Eigen::Vector3f cross = z_axis.cross(normal_plane);
            float sign = (cross.z() < 0) ? -1.0f : 1.0f;

            if (enter_flag && high_step != 0.0) {
                if(std::abs(current_angle) > 10.0){
                    if (sign > 0){
                        final_step_height = slope_height + rf_l;
                    }
                    else{
                        final_step_height = slope_height - rf_l;
                    }
                }
                else{
                    float check_z_mean = 0.0;
                    for (auto& point : planes[closest_plane_index]->points) {
                        check_z_mean += point.z;
                    }
                    check_z_mean /= planes[closest_plane_index]->points.size();
                    if (check_z_mean > 0.0){
                        final_step_height = high_step;
                    }
                    else{
                        final_step_height = -high_step;
                    }
                }

                pcl::PassThrough<pcl::PointXYZRGB> pass_1;
                pass_1.setInputCloud(planes[closest_plane_index]);
                pass_1.setFilterFieldName("x");
                pass_1.setFilterLimits(-0.3, 0.3);
                pass_1.filter(*planes[closest_plane_index]);

                if(step_height_queue.size() > 15 && stair_edge_queue.size() > 15){
                    enter_flag = false;
                }
                step_height_queue.push(final_step_height);
                stair_edge_queue.push(std::abs(planes[closest_plane_index]->points[0].y));

                final_step_height = mostFrequentElement(step_height_queue);
                msg_step_height.data= final_step_height;

                msg_stair_edge.data = mostFrequentElement(stair_edge_queue);
            }

            if (!enter_flag) {
                foothold_height.publish(msg_step_height);
            }
            if (!enter_flag){
                stair_edge.publish(msg_stair_edge);
            }

            //EXECUTION TIME OF MAIN FUNCTION----------------------------------------------------------
            auto stop= high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            std::cout<<"Main function time: " << duration.count()/1000 << " milliseconds" << std::endl;
            //--------------------------------------------------------------------------------------------
            new_leg=false;
            new_cloud=false;
        }
        ros::spinOnce();
    }

}
