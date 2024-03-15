#include "utility.h"
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
float max_step_length=0.7;
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

int main (int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "robotic_vision");
	ros::NodeHandle nh;
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub_cloud = nh.subscribe ("camera/depth/color/points", 1, cloud_cb);
	ros::Subscriber sub_pivot = nh.subscribe ("pivot_camera_coord", 1, pivot_cb);
	ros::Subscriber sub_leg = nh.subscribe ("swing_leg", 1, leg_cb);
	ros::Subscriber sub_fs = nh.subscribe ("first_step_global", 1, fs_cb);
	//bool subscriber should be added in order to know whether the robotic vision module should be active
	ros::Publisher pub = nh.advertise<pcl::PCLPointCloud2> ("outcloud", 1);
	ros::Publisher pub2 = nh.advertise<pcl::PCLPointCloud2> ("obstacles", 1);
	ros::Publisher pub3 = nh.advertise<pcl::PCLPointCloud2> ("tracks", 1);
	ros::Publisher pub4 = nh.advertise<std_msgs::Float32MultiArray> ("obstacle_shape_raw", 1);
	//ros::Publisher pub5 = nh.advertise<std_msgs::Bool> ("next_swing_leg", 1);
	ros::Publisher pub6 = nh.advertise<std_msgs::Float32> ("CoM_height_raw", 1);
	ros::Publisher pub7 =nh.advertise<std_msgs::Float32> ("foothold_raw", 1);
	ros::Publisher pub8 = nh.advertise<std_msgs::Float32> ("reference_tilt", 1);
	ros::Publisher pub9 = nh.advertise<std_msgs::Float32> ("tilt_angle", 1);
	ros::Publisher pub10 = nh.advertise<std_msgs::Float32> ("foot_tip_pos", 1);
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
	//float min_height_constraint = 0.9; //max CoM height allowed: 0.9 * leg length (SHOULD BE A PARAM??)
	//max_sl_const = 2* sqrt(1-pow(min_height_constraint,2)) *  (thigh_length+shin_length); 
	//max_step_length = max_sl_const; 
	
	
	std::cout<<"Max it: " << ransac_max_it << std::endl;
	std::cout<<"Threshold: " << ransac_th << std::endl;	
	
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
	  		// While 90% of the original cloud is still there 
	  		
	  		while (remaining->size () > 0.9* nr_points) { //so that we find only 1 plane. This mechanism should be upgraded if used outdoor
	  		
	    			// Segment the largest planar component from the remaining cloud
	    			seg.setIndices (remaining);
	    			seg.segment (*inliers, *coefficients);
	    			if (inliers->indices.size () == 0) break;
	    			
	    			//ALIGN THE POINTCLOUD WITH RVIZ FRAME(Z-AXIS)------------------------------------------------------------
	    			if(!alignment_z){
	    				Eigen::Vector3f n_z(0.0f, 0.0f, 1.0f);
	    				Eigen::Vector3f n_plane(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
	  					n_plane.normalize();
	  					if(!ref_acquired) {
	  						reference_tilt = rotate_point_cloud_plane(input_cloud, n_z, n_plane);
	  						ref_acquired=true;
	  					}
	    				else tilt_ang = rotate_point_cloud_plane(input_cloud, n_z, n_plane);
	    				alignment_z=true;
					}
	    			
	    			
	    			// Extract the ground plane inliers
	    			std::vector<int>::iterator it = remaining->begin();
	    			
				for (size_t i = 0; i < inliers->indices.size();i++) {
				      int curr = inliers->indices[i];
				      // Remove it from further consideration.
				      while (it != remaining->end() && *it < curr) { ++it; }
				      if (it == remaining->end()) break;
				      if (*it == curr) {
				      	//ESTIMATE MEAN DISTANCE BETWEEN CAMERA AND GROUND ON THE Z-AXIS
				      	camera_offs_z+= input_cloud->at(*it).z;
				      	ground_points++;
				      	ground_cloud->points.push_back(input_cloud->at(*it));
				 		it = remaining->erase(it); //remove ground points from outliers
				      }
				}
				i++;
			}
			ground_cloud->width = ground_cloud->points.size(); //update dimension of the cloud
			//PROCESS OUTLIERS (OBSTACLES)--------------------------------------------------------------------------
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr obs_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //ALL obstacle points
  			
  			obs_cloud->header = input_cloud->header; //needed to display in the right frame
  			
			obs_cloud->width = remaining->size();
			obs_cloud->height=1;
			
			//track bounds and conditions
			float high_obs_bound_y=max_step_length; //nearest obstacle w. height >20 cm 
			float bound_1 = (dist_bt_feet + feet_width + 0.02f);
	    	float bound_2 = (dist_bt_feet-0.02f);
	    	float bound_3 = max_step_length + 0.5f;
	    	bool condition_1, condition_2, condition_3;
	    	
			for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it) {
	    			uint8_t r = 0, g = 255, b = 0;
	    			uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	    			input_cloud->at(*it).rgb = *reinterpret_cast<float*>(&rgb); //color the obstacle point (green)
	    			obs_cloud->points.push_back(input_cloud->at(*it)); //save point into obstacles pointcloud
	  		}
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
		  		rotate_point_cloud_plane_v2(obs_cloud,n_align,n_x);
	  		}
	  		
	  		
	  		//POINTCLOUD TRANSLATION (Z-AXIS)--------------------------------------------------------------------------
	  		camera_offs_z/=ground_points;
	  		Eigen::Affine3f offset_transform = Eigen::Affine3f::Identity();
			//std::cout << "Camera height w.r.t. ground: " << -camera_offs_z << std::endl;
    		offset_transform.translation() <<0.0,0.0,-camera_offs_z; //added translation on z-axis
    		pcl::transformPointCloud (*input_cloud, *input_cloud, offset_transform); // Apply traslation
    		pcl::transformPointCloud (*ground_cloud, *ground_cloud, offset_transform);
    		pcl::transformPointCloud (*obs_cloud, *obs_cloud, offset_transform);
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
  			float stddev = 0.4f; //stddev of optimal step length
	  		float mean = max_step_length/2; //mean of optimal step length
	  		float exponential_component;
  			float min_obs_dist;
  			float max_obs_dist;
  			float min_score;
  			float score;
  			float dist;
  			uint8_t r,g,b;
  			uint32_t rgb;
	  		for(int i=0;i<ground_cloud->points.size();i++){
	    		condition_1 = ground_cloud->points[i].x>-bound_1 && ground_cloud->points[i].x<-bound_2;
	    		condition_2 = ground_cloud->points[i].x<bound_1 && ground_cloud->points[i].x>bound_2;
	    		condition_3 = ground_cloud->points[i].y < high_obs_bound_y;
	    		min_score=1.0f;
	    		score=1.0f;
	    		if(condition_3){
	    			if(condition_1) { //left track
	    				for(int j=0;j<ltrack_obs_cloud->points.size();j++){
	    					if(ltrack_obs_cloud->points[j].y<high_obs_bound_y){//second condition added 18/01/2024
		    					dist=sqrt(pow(ground_cloud->points[i].x-ltrack_obs_cloud->points[j].x,2)+pow(ground_cloud->points[i].y-ltrack_obs_cloud->points[j].y,2));
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
	    				exponential_component = exp(-(pow((ground_cloud->points[i].y-mean) / stddev,2.0)));
	    				b = (uint8_t)255*min_score* exponential_component;
	    				rgb = (uint32_t)b;
	    				ground_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
	    				tracks[0].push_back(ground_cloud->points[i]);	
	    			}
	    			if(condition_2) { //right track
	    				for(int j=0;j<rtrack_obs_cloud->points.size();j++){
	    					if(rtrack_obs_cloud->points[j].y<high_obs_bound_y){ //second condition added 18/01/2024
		    					dist=sqrt(pow(ground_cloud->points[i].x-rtrack_obs_cloud->points[j].x,2)+pow(ground_cloud->points[i].y-rtrack_obs_cloud->points[j].y,2));
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
	   					exponential_component = exp(-(pow((ground_cloud->points[i].y-mean) / stddev,2.0)));
	   					r = (uint8_t)255*min_score* exponential_component;
	   					rgb = (uint32_t)r<<16;
	   					ground_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
	   					tracks[1].push_back(ground_cloud->points[i]);
	   				}
	   				if(!(condition_1 || condition_2)) { //point not on tracks
	   					rgb = (uint32_t)0;
	   					ground_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
	   				}
	   			}
	   			else {
	   				rgb = (uint32_t)0;
	   				ground_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
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
	  	 			for(int j=0; j< ground_cloud->points.size();j++){
		  				if(best_window[i][k].x ==ground_cloud->points[j].x &&  best_window[i][k].y ==ground_cloud->points[j].y && best_window[i][k].z ==ground_cloud->points[j].z){
		  					uint8_t r = 255, g = 255, b = 0;
		    					uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		    					ground_cloud->points[j].rgb = *reinterpret_cast<float*>(&rgb);
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
	  		pcl::toPCLPointCloud2(*ground_cloud, outcloud3); //ground only
	  		pub3.publish (outcloud3);
	  		
	  		//publish obstacle shape of swing leg in sagittal plane
	  		std::vector<float> o_p;
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
	  			//std::sort(ltrack_obs_cloud->points.begin(), ltrack_obs_cloud->points.end(), cmp_xyzrgb());
	  			std::cout<<"Sending left track obs"<<std::endl;
	  			for(int i=0; i<ltrack_obs_cloud->points.size(); i++) {
	  				o_p.push_back(ltrack_obs_cloud->points[i].y);
	  				o_p.push_back(ltrack_obs_cloud->points[i].z);
	  			}
	  		
	  		}
	  		if(swing_leg==1){
	  			//std::sort(rtrack_obs_cloud->points.begin(), rtrack_obs_cloud->points.end(), cmp_xyzrgb());
	  			std::cout<<"Sending right track obs"<<std::endl;
	  			for(int i=0; i<rtrack_obs_cloud->points.size(); i++) {
	  				o_p.push_back(rtrack_obs_cloud->points[i].y);
	  				o_p.push_back(rtrack_obs_cloud->points[i].z);
	  			}
	  		}
	  		std_msgs::Float32MultiArray outp; //obstacle array: even indexes-> y coordinate   odd indexes->z coordinate
  			outp.layout.dim.push_back(std_msgs::MultiArrayDimension());
  			outp.layout.dim[0].size = o_p.size();
  			outp.layout.dim[0].stride = 1;
  			outp.layout.dim[0].label = "y-z"; // or whatever name you typically use to index
  			outp.data.clear();
  			outp.data.insert(outp.data.end(), o_p.begin(), o_p.end());
  			pub4.publish(outp);
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
