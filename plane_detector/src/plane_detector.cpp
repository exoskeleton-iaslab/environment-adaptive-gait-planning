
#include "utility.h"
using namespace std::chrono;

//global vars
ros::Publisher pub,pub2,pub3,pub4,pub5,pub6,pub7;
float dist_bt_feet = 0.095f; // [m]
float feet_width = 0.15f;
float feet_length = 0.29f;
float max_step_length = 0.5f;
float max_step_height = 0.4f;
bool saved = true; //set as false to save point clouds as txt files


//this value should be retrieved by a topic
int swing_leg=-1; // "-1" means its the first step, which can be done with both left or right leg. "0" means left leg. "1" means right leg.


void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

  auto start= high_resolution_clock::now();

  // Create the filtering object: downsample the dataset using a leaf size of 2.5 cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.02f, 0.02f, 0.02f); 
  sor.filter (*cloud_filtered_blob);
  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
  
  
  
  
  //plane detection setup
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (3000);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  
  if(!saved){
  	 printPC(cloud_filtered, "initial_cloud.txt");
  	 
  }
 
  
  //pointcloud translation vars
  float height_offs;
  bool done = false; 
  //obstacle calculation vars 
  bool valid_move = true;
  float max_obs_height=-10.0f;
  float min_obs_y = 10.0f;
  float max_obs_y = 0.0f;
  float min_obs_dist = 0.075f;
  float max_obs_dist = 0.0f;
  
  
  //COME METTERE A POSTO: ACCEDERE AGLI INDICI CORRETTI (height*width, nel mio caso il centro è cloud->at(320*240), un punto 10 pixel alla sua dx è cloud->at(320*(240+10)). salvare indice di questi punti e poi creare un vettore per riallineare.
  
  
  int i = 0, nr_points = (int) cloud_filtered->points.size();
  pcl::IndicesPtr remaining (new std::vector<int>);
  remaining->resize (nr_points);
  for (size_t i = 0; i < remaining->size (); i++) { (*remaining)[i] = static_cast<int>(i); }
  
 
	
float tilt_ang;
	
	
  // While 90% of the original cloud is still there 
  while (remaining->size () > 0.9* nr_points) //so that we find only 1 plane. This mechanism should be upgraded if used outdoor
  {
    // Segment the largest planar component from the remaining cloud
    seg.setIndices (remaining);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) break;
                                     
    Eigen::Vector3f n_z(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f n_plane(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
    n_plane.normalize();
  
  
    tilt_ang = rotate_point_cloud_plane(cloud_filtered, n_z, n_plane);    
  
  
         

    // Extract the ground plane inliers
    std::vector<int>::iterator it = remaining->begin();
    
    for (size_t i = 0; i < inliers->indices.size ();i++)
    {
      int curr = inliers->indices[i];
      // Remove it from further consideration.
      while (it != remaining->end() && *it < curr) { ++it; }
      if (it == remaining->end()) break;
      if (*it == curr) {
 	it = remaining->erase(it);
      }
    }
    i++;
  }
  
  
  
  
  
  std::cout << "Camera tilt angle is " << tilt_ang << " degrees." << std::endl;
  
  
  
  
  std::cout << "Found " << i << " plane(s)." << std::endl; //Just to be sure that only one plane is detected.
  
  
  std::vector<pcl::PointXYZRGB> obstacle_points; //obstacle points that are near the tracks
  std::vector<pcl::PointXYZRGB> tracks; //points that could be stepped on(represented as tracks)
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pc(new pcl::PointCloud<pcl::PointXYZ>); //ALL obstacle points
  obs_pc->header = cloud_filtered->header; //needed to display in the right frame
    

  // Color all the non-planar things.
  for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it)
  {
    uint8_t r = 0, g = 255, b = 0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rgb);
    
    (*obs_pc).push_back(pcl::PointXYZ(cloud_filtered->at(*it).x,cloud_filtered->at(*it).y,cloud_filtered->at(*it).z));
    
    if(sqrt(pow(cloud_filtered->at(*it).x,2)+ pow(cloud_filtered->at(*it).y,2)) < (max_step_length + 1.5f) && 
  	  ((cloud_filtered->at(*it).x>-(dist_bt_feet + feet_width + 0.03f) && (cloud_filtered->at(*it).x<-(dist_bt_feet-0.03f))) || 
  	  (cloud_filtered->at(*it).x<(dist_bt_feet+ feet_width+0.03f) && (cloud_filtered->at(*it).x>(dist_bt_feet-0.03f)))) ){   	  
  	  obstacle_points.push_back(cloud_filtered->at(*it)); //points nearby feet trajectory
  	  //testing conditions (optional)
  	  if(cloud_filtered->at(*it).z> max_obs_height) max_obs_height=cloud_filtered->at(*it).z;
  	  if(cloud_filtered->at(*it).y> max_obs_y) max_obs_y=cloud_filtered->at(*it).y;
  	  if(cloud_filtered->at(*it).y< min_obs_y && cloud_filtered->at(*it).y>0.03) min_obs_y=cloud_filtered->at(*it).y;
  	  
  	  
     }
  }
  
if(!saved){
  	 printPC(cloud_filtered, "obstacle_cloud.txt");
  	 
  }

std::cout<<"Min obs pos y: "<<min_obs_y<<std::endl;
std::cout<<"Max obs pos y: "<<max_obs_y<<std::endl;

  
  pc = cloud_filtered; //from now on this pointer will be used
  

  
  //crop whole pointcloud
  pcl::CropBox<pcl::PointXYZRGB> box;
  box.setMin(Eigen::Vector4f(-5.0f, -2.0f, -2.5f, 1.0f));
  box.setMax(Eigen::Vector4f( 5.0f, 2.0f, 2.5f, 1.0f));
  box.setInputCloud(pc);
  box.filter(*pc);
  
  //crop obstacle pointcloud
  pcl::CropBox<pcl::PointXYZ> box2;
  box2.setMin(Eigen::Vector4f(-5.0f, -2.0f, -2.5f, 1.0f));
  box2.setMax(Eigen::Vector4f( 5.0f, 2.0f, 2.5f, 1.0f));
  box2.setInputCloud(obs_pc);
  box2.filter(*obs_pc);
  
  //THIS WOULD BE THE POSITION TO PLACE A CLUSTERING ALGORITHM SO THAT OBSTACLES CAN BE DIFFERENTIATED
  
  //the following loop will calculate the score for each track point
  for(int j=0; j< pc->points.size();j++){
  	if(sqrt(pow(pc->points[j].x,2)+pow(pc->points[j].y,2)) < max_step_length &&
  	   ((pc->points[j].x>-(dist_bt_feet+ feet_width) && (pc->points[j].x<-(dist_bt_feet))) || 
  	   (pc->points[j].x<(dist_bt_feet+ feet_width) && (pc->points[j].x>(dist_bt_feet)))))   { //point on tracks
  		
  		float min_dist=10.0f;
  		//pcl::PointXYZRGB closest_point;
  		float multiplier;
  		uint32_t rgb = *reinterpret_cast<uint32_t*>(&pc->points[j].rgb);
  		if((rgb&0x0000ff00)==0x0000ff00) multiplier=0.0f; //obstacle point, score is 0
  		else{
  			if(!done){ //this condition applies Z-axis traslation and sets min and max dist. w.r.t. obstacles
				Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
				height_offs = pc->points[j].z;
				std::cout << "Height Offset: " << height_offs << std::endl;
    				transform_3.translation() <<0.0,0.0,-height_offs; //added translation on x-axis
    				pcl::transformPointCloud (*pc, *pc, transform_3); // Apply traslation
    				pcl::transformPointCloud (*obs_pc, *obs_pc, transform_3); // Apply traslation
    				//Min and Max Dist. Calculation (MUST BE IMPROVED TO SET DIFFERENT DISTANCES FOR DIFFERENT OBSTACLES)
    				max_obs_height-=height_offs;
    				if(max_obs_height<0.0f) max_obs_height=0.0f;
    				if(max_obs_height> 0.2f) {
    					valid_move=false;
    					std::cout<<"Invalid move, obstacle height: " << max_obs_height << std::endl;
    					break;
    				}
    				else {
    					if(max_obs_height>0.1f) {
    						min_obs_dist+= max_obs_height - 0.1f;
    					}
    					max_obs_dist = min_obs_dist + 0.05f;
    					std::cout<<"Obstacle height: "<<max_obs_height<< ", Min dist chosen: "<<min_obs_dist<<std::endl;
    				}
    				
    				done=true;
			}	
			
			//find nearest obstacle point
	  		for(int k=0; k< obstacle_points.size(); k++){
	  		   float tmp_dist=sqrt(pow(pc->points[j].x-obstacle_points[k].x,2)+pow(pc->points[j].y-obstacle_points[k].y,2));
	  		   if(tmp_dist<min_dist) {
	  		   	min_dist = tmp_dist; 
	  		   }
	  		}
	  		//give linear score based on nearest obstacle dist
	  		if(min_dist<min_obs_dist) {
	  			multiplier =0.0f;
	  		}
	  		else {multiplier =  min_dist>max_obs_dist? 1.0f: min_dist/max_obs_dist;}
	  	}
	  	float stddev = 0.3f; //stddev of optimal step length
	  	float mean = 0.4f; //mean of optimal step length
	  	float exponential_component = exp(-(pow((pc->points[j].y-mean) / stddev,2.0)));
	  	//score is saved as point color intensity (btwn 0 and 255)
  		uint8_t r = (uint8_t)255*multiplier * exponential_component, g = 0, b = (uint8_t)255*multiplier* exponential_component;
  		if(pc->points[j].x<0)  rgb= ((uint32_t)b);
  		else rgb = ((uint32_t)r << 16);
  		pc->points[j].rgb = *reinterpret_cast<float*>(&rgb);
  		tracks.push_back(pc->points[j]);
  	}
  	
  }
  

 //SLIDING WINDOW CALCULATION
  std::vector<pcl::PointXYZRGB> best_window[2];
  float window_y_position =0.0f;
  int l_i=0;
  int u_i=0;
  float max_score[2] ={0.0f,0.0f};
  float score[2]={0.0f,0.0f};
  float max_score_win_y_pos[2] ={0.0f, 0.0f}; //heel
 
 if(valid_move && min_obs_dist <5.0 && tracks.size()>20){
  

  //Sorting pre-calculation
  std::sort(tracks.begin(), tracks.end(), cmp_xyzrgb());
  std::sort(obstacle_points.begin(), obstacle_points.end(), cmp_xyzrgb()); //servirà per il calcolo dell'altezza
  
  

  //Foothold selection
  while(window_y_position<=max_step_length-feet_length){
  	long score_sum[2]={0,0};
  	bool invalid[2]={false,false};
  	std::vector<pcl::PointXYZRGB> window[2];
  	while(l_i < tracks.size() && tracks[l_i].y<window_y_position) l_i++;
  	if(l_i> u_i) u_i = l_i;
  	while(u_i < tracks.size() && tracks[u_i].y<= window_y_position+feet_length) u_i++;
  	for(int k = l_i; k<=u_i; k++){
  		if(tracks[k].x<0) window[0].push_back(tracks[k]);
  		else window[1].push_back(tracks[k]);
  	 	uint32_t value = *reinterpret_cast<uint32_t*>(&tracks[k].rgb);
  		if(value==0) {
  		     if(tracks[k].x < 0) {
  		     	score_sum[0]=0;
  		     	invalid[0] = true;
  		     }
  		     else{
  		        score_sum[1]=0;
  		     	invalid[1] = true;
  		     }
  			
  		}
  		else{
  			if(tracks[k].x < 0 && invalid[0]==false) score_sum[0]+= static_cast<long>(value);
  			else if (tracks[k].x > 0 && invalid[1]==false)
  			 { 
  			 	value = value>>16; 
  			 	score_sum[1]+= static_cast<long>(value); 
  			 }		
  		}
  	
  	}
  	//Verify if selected windows are compliant
  	for(int i=0; i < 2 ; i++) {
  		if(window[i].size()>0){
	  		score[i] = score_sum[i] / static_cast<float>(window[i].size());
		  	if(score[i] > 0 && score[i] > max_score[i] && 
		  	   abs(window[i][window[i].size()-1].y - (window_y_position+feet_length))<0.03f &&
		  	   abs(window[i][0].y - window_y_position)<0.03f)  {
		  	   
		  		max_score[i] = score[i];
		  		max_score_win_y_pos[i]= window_y_position;
		  		best_window[i].clear();
		  		best_window[i]=window[i];
		  	}
		  	window_y_position+=0.01f;		
  		}
  	}
  }

  if(swing_leg = -1) {
  	if(max_score[0] > max_score[1]) swing_leg = 0;
  	else swing_leg = 1;
  }
  
  //Color selected foothold points
  std::cout<<"Min foot pos y for leg "<< swing_leg << " is: "<<max_score_win_y_pos[swing_leg]<<std::endl;
  for(int i=0; i<2;i++) {
	  for(int k=0; k<best_window[i].size();k++){
	  	int colored_points=0;
	  	 for(int j=0; j< pc->points.size();j++){
	  		if(best_window[i][k].x ==pc->points[j].x &&  best_window[i][k].y ==pc->points[j].y && best_window[i][k].z ==pc->points[j].z){
	  			uint8_t r = 255, g = 255, b = 0;
	    			uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	    			pc->points[j].rgb = *reinterpret_cast<float*>(&rgb);
	    			colored_points++;
	    			if(colored_points==best_window[i].size()) break;
	  		}
	  	}
	  }
  }	  
 } 	  
 
 
 
 
 
 float pivot_tip=0.0;	  

  //Clustering (might be smart to turn all of this into an utility fuction)-------------------------------------------------------------------------------------------------------------
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (obs_pc);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.03); // 3 cm
  ec.setMinClusterSize (15);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (obs_pc);
  ec.extract (cluster_indices);
   
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_obs_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  final_obs_cloud->header = cloud_filtered->header;
  
   int j = 0;
   float mean_x, mean_y=0.0;
   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
   {
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
     float min_x = 10.0;
     float min_y = 10.0;
     float max_x = -10.0;
     float max_y = -10.0;
     for (const auto& idx : it->indices) {
       cloud_cluster->push_back ((*obs_pc)[idx]); 
       
       
       pcl::PointXYZRGB p = pcl::PointXYZRGB(j*(255/cluster_indices.size()),0,0);
       p.x = (*obs_pc)[idx].x;
       p.y = (*obs_pc)[idx].y;
       p.z = (*obs_pc)[idx].z;
       mean_y+=p.y;
       mean_x+=p.x;
       if(p.x < min_x) min_x = p.x;
       if(p.x > max_x) max_x = p.x;
       if(p.y < min_y) min_y = p.y;
       if(p.y > max_y) max_y = p.y;
       final_obs_cloud->push_back(p); 
     }
     cloud_cluster->width = cloud_cluster->size();
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = true;
     mean_x = mean_x / cloud_cluster->size();
     mean_y = mean_y / cloud_cluster->size();
     if(min_y<0.05 && mean_y<0.15  && ((mean_x > dist_bt_feet && mean_x< dist_bt_feet + feet_width) || (mean_x < -dist_bt_feet && mean_x> - dist_bt_feet - feet_width)) ){
     
     	std::cout << "Support foot detected!" << std::endl;
     	//calculation of CoM given supp foot
     	//float supp_heel = max_x - foot_length
     	pivot_tip = max_y;
     	if((mean_x > dist_bt_feet && mean_x< dist_bt_feet + feet_width)) swing_leg=0;
     	else swing_leg=1;
     
     }
     std::cout << "PointCloud representing the " << j <<"-th Cluster: " << cloud_cluster->size() << " data points." << std::endl;
     std::cout << "Boundary of " << j <<"-th Cluster: X=[ " << min_x << ", " << max_x<<" ]   Y=["<<min_y<< ", " << max_y<<" ]" << std::endl;
     std::cout << "Centroid of " << j <<"-th Cluster: [ " << mean_x << ", " << mean_y<<" ]"<< std::endl;
     j++;
   }
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  if(!saved){
  	 printPC(cloud_filtered, "final_cloud.txt");
  	 
  }
  auto stop= high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  std::cout<<"Total time: " << duration.count()/1000 << " milliseconds" << std::endl;
  saved=true;
  pcl::PCLPointCloud2 outcloud, outcloud2;
  pcl::toPCLPointCloud2(*cloud_filtered, outcloud);
  pub.publish (outcloud); // Publish the segmented plane + obstacles in green
  pcl::toPCLPointCloud2 (*final_obs_cloud, outcloud2);
  pub2.publish(outcloud2); //Publish only obstacles (clustered)
  
  
  std::vector<float> o_p;
  //publish obstacle shape of right track
  for(int i=0; i< obstacle_points.size();i++){
  	if(obstacle_points[i].x<(dist_bt_feet+ feet_width+0.03f) && (obstacle_points[i].x>(dist_bt_feet-0.03f))){
  		o_p.push_back(obstacle_points[i].y);
  		o_p.push_back(obstacle_points[i].z - height_offs);
  	}
  }
  std_msgs::Float32MultiArray outp;
  outp.layout.dim.push_back(std_msgs::MultiArrayDimension());
  outp.layout.dim[0].size = o_p.size();
  outp.layout.dim[0].stride = 1;
  outp.layout.dim[0].label = "y-z"; // or whatever name you typically use to index
  outp.data.clear();
  outp.data.insert(outp.data.end(), o_p.begin(), o_p.end());
  pub3.publish(outp);
  std_msgs::Float32 msg;
  msg.data = max_score_win_y_pos[1];
  pub4.publish(msg);
  msg.data = -height_offs;
  pub5.publish(msg);
  msg.data = pivot_tip;
  pub6.publish(msg);
  std_msgs::Bool bmsg;
  bmsg.data= (swing_leg==0); // true means left leg
  pub7.publish(bmsg);
  

}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "obstacles");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("obstacles", 1);
  pub2 = nh.advertise<pcl::PCLPointCloud2> ("clusters", 1);
  pub3 = nh.advertise<std_msgs::Float32MultiArray> ("obstacle_shape_raw", 1);
  pub4 =nh.advertise<std_msgs::Float32> ("foothold_raw", 1);
  pub5 = nh.advertise<std_msgs::Float32> ("CoM_height_raw", 1);
  pub6 = nh.advertise<std_msgs::Float32> ("pivot_tip_raw", 1);
  pub7 = nh.advertise<std_msgs::Bool> ("swing_leg", 1);

  // Spin
  ros::spin ();
}
