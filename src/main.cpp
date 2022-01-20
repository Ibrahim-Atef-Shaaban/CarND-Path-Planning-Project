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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  
  // Tunning distance parameter for the future path points, this is distance
  // between each path point of the 3 points planned in the future
  int tunning_dist = 30;
  
  //Maximum Number of pathpoints
  int n_pathpoints = 50;
          
  //Set my starting lane to 1, possible values are 0, 1, 2, representing Left, Middle, Right
  int my_lane = 1;
  int lane_size = 4; // 4 meters
  
  // Reference Velocity Limit is 50 MPH, I've set it with a small difference for safety
  double speed_lim = 49.0; //in MPH
  double my_velo = 0; //in MPH

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
  
  
  h.onMessage([&my_lane,&n_pathpoints,&tunning_dist,&my_velo,&speed_lim,&lane_size,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          int prev_size = previous_path_x.size();
          /**
           * define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
         if (prev_size > 0) {
              car_s = end_path_s;
            }
          // Warning flags for surrounding objects
          bool warning_right = false;
          bool warning_front = false;
          bool warning_left  = false;
          
          // Iterating over the objects detected by sensor fusion
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            // FIrst check if the objects is in our driving direction, else, skip it
            float d = sensor_fusion[i][6];
            int object_lane = 99; // invalid number init
            if (d > 0 && d < 4){
              object_lane = 0;
            }
            else if (d > 4 && d < 8){
              object_lane = 1;
            }
            else if (d > 8 && d < 12){
              object_lane = 2;
            }
            else {
              continue;
            }
            
            // Get the object's data
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            //if using previous points can give a value out
            check_car_s += ((double)prev_size*0.02*check_speed);
            
            
            
            if (object_lane == my_lane){
              // Object in my lane, check the distance, and raise the warning if it is within 30 S distance.
               warning_front |= check_car_s > car_s && check_car_s - car_s < 35;
            }
            else if (my_lane - object_lane == 1){
              // Object on left lane, check the distance, and raise the warning if it is within 30 S distance.
              warning_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
            }
            else if (object_lane - my_lane == 1){
              // Object on right lane, check the distance, and raise the warning if it is within 30 S distance.
              warning_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
            }
          }
          // Let's check for a possible lane change, if needed
          if (warning_front){
            if ( my_lane > 0 && warning_left == false){
              my_lane -=1; //Go Left
            }
            else if ( my_lane != 2 && warning_right == false){
              my_lane +=1; //Go right
            }
            else{
              my_velo -= 0.2;//decelerate, as we are totally surrounded
            }
          }
          else {
            // if we are not in the middle lane, move to it, to give more options to bypass other vehicles
            if ( ( my_lane == 0 && !warning_right ) || ( my_lane == 2 && !warning_left ) ) {
                  my_lane = 1; // Back to center.
              }
            if (my_velo < speed_lim - 0.225){
            my_velo += 0.2;
            }
          }
          
          
          
                
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if the memory storage is almost empty, just use the car's current data
          if ( prev_size < 2 ) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // push the latest 2 path points
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          // Now, let's set the next path points, and push them to the x and y vectors
          vector<double> next_wp0 = getXY(car_s + tunning_dist  , 2 + lane_size*my_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + tunning_dist*2, 2 + lane_size*my_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + tunning_dist*3, 2 + lane_size*my_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // At this point, we have now 5 points, 2 in the past, and 3 planned in the future
          
          // Transferring coordinate to local coordinates(realtive to the car pos).
          for ( int i = 0; i < ptsx.size(); i++ ) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
          
          // Create the spline.
          tk::spline s;
          
          //Set the x and y path points to the spline
          s.set_points(ptsx, ptsy);
          
          // Output path points from previous path for continuity.
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // First, copy over the left previous path points into our path, instead of
          // building path from scrath, we can pick up where we left off
          for ( int i = 0; i < prev_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          
          // Calculate distance y position on tunning_dist m ahead.
          double target_x = double(tunning_dist);
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;
          
          
          for( int i = 1; i < n_pathpoints - prev_size; i++ ) {
              
              double N = target_dist/(0.02*my_velo/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }


          json msgJson;

          

          
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

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}