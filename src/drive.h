/* Helper functions for path finding */

#include <math.h>   
#define PI 3.14159265

const double MAX_SPEED = 49.5;
const double MPH_TO_MPS = 0.44704;

const int lanecnt = 3;
vector <vector <vector <double>>>  targets(3);


int desired_lane = 1;
int fastest_lane;
double desired_speed = MAX_SPEED;
double Test_old_speed = 0;
double closest_target;

// build new
void surrounding_vehicles_new(vector<vector <double>> sensor_fusion, double * ego_vehicle){
  double lane_speed[lanecnt];  
  
  for (int i = 0; i < lanecnt; i++){
    lane_speed[i] = MAX_SPEED;
    targets[i].clear();
  }  
  closest_target = 1000;
  
  vector <vector <double>> tar_neg;  
  vector <double> tar; 
  tar.push_back(-100);
  tar.push_back(0);
  for (int i = 0; i < lanecnt; i++){
    tar_neg.push_back(tar);
  }
  
  vector <vector <double>> tar_pos;  
  tar[0] = 1000;
  tar[1] = MAX_SPEED;
  for (int i = 0; i < lanecnt; i++){
    tar_pos.push_back(tar);
  }
  
  for ( int i = 0; i < sensor_fusion.size(); i++ ) {
    
    //double target_s = sensor_fusion[i][5];
    double target_distance = sensor_fusion[i][5] - ego_vehicle[0];      
    if (abs(target_distance)>100)
      continue;    
    
    float d = sensor_fusion[i][6];
    double target_vx = sensor_fusion[i][3];
    double target_vy = sensor_fusion[i][4];
    double target_speed = sqrt(target_vx*target_vx + target_vy*target_vy) / MPH_TO_MPS;
    
    //vector <double> tar;  
    tar[0] = target_distance;
    tar[1] = target_speed;
    
    //lane 0 .. 2
    int lane = trunc(d/4);
    //std::cout << "target added on lane "<< lane <<  ":  d " << d <<"  tar[0] " << tar[0] << "  tar[1] " << tar[1] <<std::endl;
    
    
    if (tar[0] < 0){
      if (tar[0] > tar_neg[lane][0]){
        tar_neg[lane][0] = tar[0];
        tar_neg[lane][1] = tar[1];
      }
    }
    else{
      tar_pos[lane][0] = tar[0];
      if (tar[0] < closest_target)
        closest_target = tar[0];
      targets[lane].push_back(tar);

      if ((target_distance > 0)&&(target_speed < lane_speed[lane]))
        lane_speed[lane] = target_speed;  
    }
  }
  fastest_lane = 1; //  0 .. 2
  
  for (int i = 0; i < lanecnt; i++){
    //std::cout << "lane_speed "<< i << " " << lane_speed[i] << " lane_speed[fastest_lane-1] " << lane_speed[fastest_lane-1] <<std::endl;
    if (lane_speed[i]>lane_speed[fastest_lane])
      fastest_lane = i;
  }
  
  //targets always have one negative and at least one positive entry
  for (int i = 0; i < lanecnt; i++){
    targets[i].push_back(tar_neg[i]);
    if (tar_pos[i][0] == 1000)
      targets[i].push_back(tar_pos[i]);
  }
  sort(targets[0].begin(), targets[0].end()); 
  sort(targets[1].begin(), targets[1].end()); 
  sort(targets[2].begin(), targets[2].end()); 
  //std::cout << "Checked targets "<<std::endl;
  
}


int check_gap(int lane, double v_ego){
  int rtvalue = 1;
/*  
  if (targets[lane].size() > 0){
    if (targets[lane][0][0] > 0){
      vector <double> new_entry;
      new_entry.push_back(-100);
      new_entry.push_back(0);
      targets[lane].insert(targets[lane].begin(), new_entry);
      std::cout << "SHOULD NEVER HAVE BEEN CALLED - added negative target"<< std::endl;
    
    }
*/
  
    int amount = targets[lane].size();
    /*
	std::cout << "CHECK GAPE to lane "<< lane <<std::endl;
    for (int i = 0; i < std::min(2, amount); i++){
      std::cout << "check target["<<lane<<"]["<<i<<"][0] "<< targets[lane][i][0] <<  "  targets[lane][i][1] "<< targets[lane][i][1] << std::endl;
    }
	*/
    
    double delta_v;
    delta_v = v_ego - targets[lane][0][1];
    //target from behind too close
    if (targets[lane][0][0] > delta_v - 10 ){
      rtvalue = 1000;
      
    }
    // no target in front
    /*
    if (targets[lane].size() == 1)
      return rtvalue;  
      */
    //first target in front too close
    delta_v = v_ego - targets[lane][1][1];
    if (targets[lane][1][0] < (10 + 2 * delta_v))
      rtvalue = 2000;
    // check if enough space in front to accelerate and lanechange
    
    // check if braking and lanechange is an option

  //}
  return rtvalue;  
}


double getspeed(int lane, double v_ego){
  double returnspeed = MAX_SPEED;
  //std::cout << "Get Speed!" << std::endl; 
  for (int i = 1; i < targets[lane].size(); i++){
    //std::cout << "targets["<<lane<<"]["<<i<<"][0] "<< targets[lane][i][0] <<  "  targets[lane][i][1] "<< targets[lane][i][1] << std::endl; 

    if (targets[lane][i][0] < 30){
      //std::cout << "Set speed" << std::endl; 
      returnspeed = targets[lane][i][1]; 
      //break;     

    }
    else if (targets[lane][i][0] < 60){
      double delta_v;
      delta_v = v_ego - targets[lane][i][1];
      if (delta_v > 15){
        //std::cout << "Set speed" << std::endl; 
        returnspeed = targets[lane][i][1] + delta_v / 2; 
        //break;
      }
    }
    else
      break;

  }
  return returnspeed;  
}



void decide_action(double * ego_vehicle){
  
  double speed_lanes[lanecnt] = {MAX_SPEED, MAX_SPEED, MAX_SPEED};
  double mindist[lanecnt] = {100, 100, 100};
  int target_amount[lanecnt] = {0, 0, 0};
  double lanecost[lanecnt] = {1, 1, 1};
  int ego_lane = int(ego_vehicle[6]);

  const double middle_lane = 0.98;
  const double fast_lane = 0.9;  
  
  desired_speed = std::min(getspeed(ego_lane, ego_vehicle[4]), getspeed(desired_lane, ego_vehicle[4]));
  
  // targets ion front more than 120m away (ego and middle)
  if ((targets[ego_lane][1][0] >= 120)&&(targets[1][1][0] >= 120)){
    desired_lane = 1;
    desired_speed = getspeed(desired_lane, ego_vehicle[4]);
    return;
  }  
  
  //if previous action isn't finished
  // or target in front further away than 100m
  if ((ego_lane != desired_lane)||(targets[ego_lane][1][0] >= 60)){
    return;
  }

  
  for (int i = 0; i< lanecnt; i++){
    for (int j = 1; j < targets[i].size() ; j++){
      double tmp_spd = MAX_SPEED;
      //if (targets[i][j][0] < 0)
      //  continue;

      target_amount[i]++;
      if (targets[i][j][0] < 40)
        tmp_spd = targets[i][j][1];
      else if (targets[i][j][0] < 75)     
        tmp_spd = targets[i][j][1] + (MAX_SPEED - targets[i][j][1]) * 75 / (targets[i][j][0] - 40);
      // > 60 tmp_spd == MAX_SPEED

      if (tmp_spd < speed_lanes[i])
        speed_lanes[i] = tmp_spd;


    }
    mindist[i] = std::min(targets[i][1][0], 100.0);
    lanecost[i] *= exp((-speed_lanes[i])/MAX_SPEED); // 0.367 .. 1
    lanecost[i] *= (1 + target_amount[i]/20);
    lanecost[i] *= (3 - 2 * (mindist[i]/100));    
  }
  lanecost[fastest_lane] *= fast_lane;
  lanecost[1] *= middle_lane;
  
  // sort lane cost
  int cheaplanes[3] = {0,1,2};
 
/*  
  for (int i = 0; i < lanecnt; i++){
    std::cout << "1. lanecost   "<< i <<  "  : "<< lanecost[i] << std::endl;
    std::cout << "1. cheaplanes "<< i <<  "  : "<< cheaplanes[i] << std::endl;
  }
*/  

  for (int i = 0; i<lanecnt; i++){
    for (int j = i+1; j<lanecnt; j++){
      if (lanecost[cheaplanes[j]] < lanecost[cheaplanes[i]]){
        //std::cout << "lanecost   "<< j <<  " : "<< lanecost[j]  << " < lanecost   "<< i <<  " : "<< lanecost[i] << std::endl;
        int tmp = cheaplanes[i];    
        cheaplanes[i] = cheaplanes[j];
        cheaplanes[j] = tmp;
      }
    }
  }
  /*
  for (int i = 0; i < lanecnt; i++){
    std::cout << "2. lanecost   "<< i <<  "  : "<< lanecost[i] << std::endl;
    std::cout << "2. cheaplanes "<< i <<  "  : "<< cheaplanes[i] << std::endl;
  }
  */

  for (int i = 0; i < lanecnt; i++){
    int checklane;
    int gap;
    if (cheaplanes[i] == ego_lane){ 
      desired_lane = ego_lane;
      break;
    }
    else if (cheaplanes[i] < ego_lane)
      checklane = ego_lane-1;    
    else
      checklane = ego_lane+1;
    gap = check_gap(checklane, ego_vehicle[4]);
    //std::cout << "check_gap "<< checklane <<  "  gap "<< gap << std::endl;
    
    if (gap > 10)
      continue;
    else{
      desired_lane = checklane;
      break;
    }
  }  
  //std::cout << "desired_speed "<< desired_speed <<  "  desired_lane "<< desired_lane << std::endl;
  

}

