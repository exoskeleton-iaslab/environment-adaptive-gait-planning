#include "utility.h"


//PIVOT AND COM HEIGHT ARGUMENTS SHOULD BE ADDED TO CALCULATE ADAPTIVE STEP LENGTH
std::vector<float> fs_simulator(std::vector<Point> left_obs, std::vector<Point> right_obs, int swing_leg, float foot_length,float pivot) {
	float max_sl=1.0 +pivot; //fixed
	float stddev = 0.4f; //stddev of optimal step length
	float mean = 3*max_sl/4; //mean of optimal step length
	std::vector<float> res; //should return foothold position and swing leg (if the input swing leg argument is -1)
	std::vector<Point> left_track;
	std::vector<Point> right_track; // Contains Point(y position, score)
	
	std::cout<<"Start populating tracks"<<std::endl;
	for(int i=0;i<=max_sl*100;i++){
		left_track.push_back(Point(i*0.01,0.0));
		right_track.push_back(Point(i*0.01,0.0));
		
	}
	std::cout<<"Populated tracks"<<std::endl;
	if(swing_leg==0) { //only left track
		for(int i=0; i< left_track.size();i++){
			float min_score=1.0f;
			float score;
			if(left_track[i].getx()<pivot) {
				left_track[i].set(left_track[i].getx(), 0);
			}
			else{
				for(int j=0;j<left_obs.size();j++){
					if(left_obs[j].gety()>0){
						float dist= abs(left_track[i].getx() - left_obs[j].getx());
						float min_obs_dist = left_obs[j].gety();
						float max_obs_dist = min_obs_dist + 0.05;
						if(dist<min_obs_dist) {
				  			min_score =0.0f;
				  			break;
				  		}					
				  		else {score =  dist>max_obs_dist? 1.0f: dist/max_obs_dist;}
				  		if(score<min_score) min_score = score;
			  		}
			  		else score=1.0f;
				}
				float exponential_component = exp(-(pow((left_track[i].getx()-mean) / stddev,2.0)));
				float final_score = 255*min_score* exponential_component;
				left_track[i].set(left_track[i].getx(), final_score);
	    	}
		}
		
		
		std::cout<<"Left track:" << std::endl;
		for(int i=0;i<left_track.size();i++){
		if(i!= left_track.size()-1) std::cout<< left_track[i].getx() << ", " << left_track[i].gety() << ", ";
		else std::cout << left_track[i].getx() << ", " << left_track[i].gety() << std::endl;
		}
		
		int pos_idx=0;
		int temp_idx;
		float window_score;
		float best_window_score=0;
		float best_window_pos; //located on the heel;
		while(left_track[pos_idx].getx()<=max_sl - foot_length){
			temp_idx=pos_idx;
			window_score=0;
			while(left_track[temp_idx].getx() - left_track[pos_idx].getx() <= foot_length) {
				if(left_track[temp_idx].gety()>0){
					window_score += left_track[temp_idx].gety();
					temp_idx++;
				}
				else{
					window_score=0;
					break;
				}
				
			
			}
			if(window_score>0) window_score = window_score / (temp_idx - pos_idx);
			if (window_score > best_window_score){
				best_window_score=window_score;
				best_window_pos = left_track[pos_idx].getx();
			}
			pos_idx++;
		}
		if (best_window_score>0) res.push_back(best_window_pos);
		else std::cout << "Foothold not found for left leg" <<std::endl;
		
		
		
	}
	
	
	
	else if (swing_leg==1){ //only right track
		for(int i=0; i< right_track.size();i++){
			float min_score=1.0f;
			float score;
			if(right_track[i].getx()<pivot) {
				right_track[i].set(right_track[i].getx(), 0);
			}
			else{
				for(int j=0;j<right_obs.size();j++){
					if(right_obs[j].gety()>0){
						float dist= abs(right_track[i].getx() - right_obs[j].getx());
						float min_obs_dist = right_obs[j].gety();
						float max_obs_dist = min_obs_dist + 0.05;
						if(dist<min_obs_dist) {
				  			min_score =0.0f;
				  			break;
				  		}					
				  		else {score =  dist>max_obs_dist? 1.0f: dist/max_obs_dist;}
				  		if(score<min_score) min_score = score;
			  		}
			  		else score = 1.0f;
				}
				float exponential_component = exp(-(pow((right_track[i].getx()-mean) / stddev,2.0)));
				//std::cout<< exponential_component << std::endl;
				float final_score = 255*min_score* exponential_component;
				right_track[i].set(right_track[i].getx(), final_score);
	    	}
		}
		
		std::cout<<"Right track:" << std::endl;
		for(int i=0;i<right_track.size();i++){
		if(i!= right_track.size()-1) std::cout<< right_track[i].getx() << ", "<< right_track[i].gety() << ", ";
		else std::cout <<right_track[i].getx() << ", "<< right_track[i].gety() << std::endl;
		}
		
		int pos_idx=0;
		int temp_idx;
		float window_score;
		float best_window_score=0;
		float best_window_pos; //located on the heel;
		while(right_track[pos_idx].getx()<=max_sl - foot_length){
			temp_idx=pos_idx;
			window_score=0;
			while(right_track[temp_idx].getx() - right_track[pos_idx].getx() <= foot_length) {
				if(right_track[temp_idx].gety()>0){
					window_score += right_track[temp_idx].gety();
					temp_idx++;
				}
				else{
					window_score=0;
					break;
				}
				
			
			}
			if(window_score>0) window_score = window_score / (temp_idx - pos_idx);
			if (window_score > best_window_score){
				best_window_score=window_score;
				best_window_pos = right_track[pos_idx].getx();
			}
			pos_idx++;
		}
		if (best_window_score>0) res.push_back(best_window_pos);
		else std::cout << "Foothold not found for right leg" <<std::endl;
		
		
		
	}
	
	
	
	else { //both
		std::cout<< "Start processing both tracks" << std::endl;
		float dist;
		float min_obs_dist;
		float max_obs_dist;
		float exponential_component;
		float final_score;
		for(int i=0; i< left_track.size();i++){
			float min_score=1.0f;
			float score;
			if(left_track[i].getx()<pivot) {
				left_track[i].set(left_track[i].getx(), 0);
				break;
			}
	    	for(int j=0;j<left_obs.size();j++){
	    		if(left_obs[j].gety()>0){
					dist= abs(left_track[i].getx() - left_obs[j].getx());
					min_obs_dist = left_obs[j].gety();
					max_obs_dist = min_obs_dist + 0.05;
					if(dist<min_obs_dist) {
			  			min_score =0.0f;
			  			break;
				  	}					
				  	else {score =  dist>max_obs_dist? 1.0f: dist/max_obs_dist;}
				 	if(score<min_score) min_score = score;
				}
				else score=1.0f;
	    	}
	    	exponential_component = exp(-(pow((left_track[i].getx()-mean) / stddev,2.0)));
	    	//std::cout<< exponential_component << std::endl;
	    	final_score = 255*min_score* exponential_component;
	    	//std::cout<< final_score << std::endl;
	    	left_track[i].set(left_track[i].getx(), final_score);
		}
		for(int i=0; i< right_track.size();i++){
			float min_score=1.0f;
			float score;
			if(right_track[i].getx()<pivot) {
				right_track[i].set(right_track[i].getx(), 0);
				break;
			}
	    	for(int j=0;j<right_obs.size();j++){
				if(right_obs[j].gety()>0){
					dist= abs(right_track[i].getx() - right_obs[j].getx());
					min_obs_dist = right_obs[j].gety();
					max_obs_dist = min_obs_dist + 0.05;
					if(dist<min_obs_dist) {
				  		min_score =0.0f;
				  		break;
				  	}					
				  	else {score =  dist>max_obs_dist? 1.0f: dist/max_obs_dist;}
				  	if(score<min_score) min_score = score;	
			  	}
			  	else score=1.0;
		  	}
	    	float exponential_component = exp(-(pow((right_track[i].getx()-mean) / stddev,2.0)));
	    	float final_score = 255*min_score* exponential_component;
	    	right_track[i].set(right_track[i].getx(), final_score);
		}
		
		std::cout<<"Left track:" << std::endl;
		for(int i=0;i<left_track.size();i++){
		if(i!= left_track.size()-1) std::cout<< left_track[i].getx() << ", " << left_track[i].gety() << ", ";
		else std::cout << left_track[i].gety() << std::endl;
		}
		
		std::cout<<"Right track:" << std::endl;
		for(int i=0;i<right_track.size();i++){
		if(i!= right_track.size()-1) std::cout<< right_track[i].getx() << ", "<< right_track[i].gety() << ", ";
		else std::cout << right_track[i].gety() << std::endl;
		}
		
		
		int pos_idx=0;
		int temp_idx;
		float window_score;
		float best_left_window_score=0;
		float best_right_window_score=0;
		float best_left_window_pos; //located on the heel
		float best_right_window_pos; //located on the heel
		std::cout<<"Score given, now I pass to window sliding"<<std::endl;
		while(left_track[pos_idx].getx()<=max_sl - foot_length){
			temp_idx=pos_idx;
			window_score=0;
			while(left_track[temp_idx].getx() - left_track[pos_idx].getx() <= foot_length) {
				if(left_track[temp_idx].gety()>0){
					window_score += left_track[temp_idx].gety();
					temp_idx++;
				}
				else{
					window_score=0;
					break;
				}
			
			}
			if(window_score>0) window_score = window_score / (temp_idx - pos_idx);
			if (window_score > best_left_window_score){
				best_left_window_score=window_score;
				best_left_window_pos = left_track[pos_idx].getx();
			}
			pos_idx++;
		}
		
		pos_idx=0;
		while(right_track[pos_idx].getx()<=max_sl - foot_length){
			temp_idx=pos_idx;
			window_score=0;
			while(right_track[temp_idx].getx() - right_track[pos_idx].getx() <= foot_length) {
				if(right_track[temp_idx].gety()>0){
					window_score += right_track[temp_idx].gety();
					temp_idx++;
				}
				else{
					window_score=0;
					break;
				}
			}
			if(window_score>0) window_score = window_score / (temp_idx - pos_idx);
			if (window_score > best_right_window_score){
				best_right_window_score=window_score;
				best_right_window_pos = right_track[pos_idx].getx();
			}
			pos_idx++;
		}
		
		if(best_left_window_score >= best_right_window_score && best_left_window_score>0) {
			res.push_back(best_left_window_pos);
			res.push_back(0); //left leg
		
		
		}
		else if(best_right_window_score >= best_left_window_score && best_right_window_score>0) {
			res.push_back(best_right_window_pos);
			res.push_back(1); //right leg
		
		
		}
		else std::cout << "Foothold not found for both legs. Left leg score: " << best_left_window_score<<" , right leg score: "<< best_right_window_score <<std::endl;
	
	
	
	
	}
	return res;


}
