#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <math.h>

#include "drive.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int counter = 0;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          /***********************************************************/

          
          const double MAX_ACC = 0.3;
          const double TSTEP = 0.02;
          const double LANE_WIDTH = 4.0;
          
          
          double ego_vehicle[7] = {0};
          
          ego_vehicle[0] = car_s;
          ego_vehicle[1] = car_d;
          ego_vehicle[2] = car_x;
          ego_vehicle[3] = car_y;
          if (abs(Test_old_speed-car_speed)>0.2){
			std::cout << "Happened again! " << " Test_old_speed " << Test_old_speed<< "   car_speed " << car_speed<< std::endl;
            ego_vehicle[4] = Test_old_speed;
		  }
          // weird behavior where car_speed is reset during acceleration...
            //std::cout << "Happened again! " << " Test_old_speed " << Test_old_speed<< "   car_speed " << car_speed<< std::endl;
          else
            ego_vehicle[4] = car_speed;
          ego_vehicle[5] = car_yaw;
          
          
          // Check Current Ego State
          // 0 .. 2
          ego_vehicle[6] = trunc(car_d/4);
          

          
          //surrounding_cars(sensor_fusion, ego_vehicle);
          //decide_action(ego_vehicle);
          surrounding_vehicles_new(sensor_fusion, ego_vehicle);
          decide_action(ego_vehicle);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          vector<double> path_x;
          vector<double> path_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          int prev_size = previous_path_x.size();

          
          
          double current_speed = ego_vehicle[4];
          double nec_acc = 0;
          if (abs(current_speed - desired_speed)<=MAX_ACC)
            current_speed = desired_speed;
          else if (current_speed < desired_speed)
            nec_acc = MAX_ACC/30;
          else{
            //std::cout << "Braking "<<std::endl;
            nec_acc = -MAX_ACC/20;
          }
          
          
         // std::cout << "desired_lane "<< desired_lane << " current_speed " << current_speed << " desired_speed " << desired_speed << std::endl;
          

        // basic implementation of spline usage and movement taken from
        // https://github.com/csharpseattle
        
        // We need at least 2 points to create the spline
          if (prev_size < 2)
          {
            // no previous points available
            // use current position and one additional point based on the heading
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            path_x.push_back(prev_car_x);
            path_x.push_back(car_x);
            path_y.push_back(prev_car_y);
            path_y.push_back(car_y);
          }
          else
          {

            // use last two points of the previously available path
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            path_x.push_back(ref_x_prev);
            path_x.push_back(ref_x);
            path_y.push_back(ref_y_prev);
            path_y.push_back(ref_y);
          }

          
          // add more points far in front of the vehicle
          // Change them from Frenet to Cartesian MAP coordinate system
          for (unsigned int i = 60; i <= 180; i+=30)
          {
            std::vector<double> next_wp = getXY(car_s + i,
                                                ((LANE_WIDTH / 2) + LANE_WIDTH * (desired_lane)),
                                                map_waypoints_s,
                                                map_waypoints_x,
                                                map_waypoints_y);
            path_x.push_back(next_wp[0]);
            path_y.push_back(next_wp[1]);
          }

          // Change points from MAP to VEHICLE coordinate system
          for (unsigned int i = 0; i < path_x.size(); ++i)
          {
            double shift_x = path_x[i] - ref_x;
            double shift_y = path_y[i] - ref_y;

            path_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            path_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // create the spline
          tk::spline s;
          s.set_points(path_x, path_y);

          // add remaining points from old path to new path
          for (unsigned int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //
          // calculate the euclidian distace of the spline
          // 100 meters out.
          //
          double target_x = 100.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          //
          // increase the number of points in the path
          // to 50.
          //
          double x_previous_step = 0;
          for (unsigned int i = 1; i <= 75 - previous_path_x.size(); ++i)
          {
            //
            // generate interpolated points along the spline.
            // N = amount of points necessary to cover 100 meters ahead
            // 
            if (abs(current_speed - desired_speed) <= abs(nec_acc)){
              current_speed = desired_speed;
              nec_acc = 0;
            }            
              
            current_speed = current_speed + nec_acc*10;
            car_speed = current_speed;
            
            double N = (target_dist / (0.02 * current_speed * MPH_TO_MPS));
            double x_point = x_previous_step + (target_x / N);
            double y_point = s(x_point);

            x_previous_step = x_point;

            //
            // Tranlate and rotate back to world space.
            //
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          //std::cout << "Test_old_speed "<< Test_old_speed <<std::endl;
          Test_old_speed = current_speed;

          
          /***********************************************************/


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567; // //3000; //
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
