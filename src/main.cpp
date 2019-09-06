#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"
#include <cmath>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const double MAX_SPEED = 49.5;
const double MAX_ACCELERATION = .2;

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

  // Car current lane. Init at middle lane.
  int current_lane = 1;

  // Car current vel. Init at 0
  double current_vel = 0.0;

  h.onMessage([&current_vel, &current_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int path_size = previous_path_x.size();

          // First Check all cars positions and 
          // check if cars are close and in our lane, on left or on the right lane
          bool car_ahead = false;
          bool car_left = false;
          bool car_righ = false;
          double distance_threshold = 30;
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
              float d = sensor_fusion[i][6];
              int car_lane = -999;
              // center left lane 0  : 2
              // center middle lane 1  : 6
              // center right lane 2  : 10
              if ( 0 < d && d < 4 ) { // lane is always 4m wide
                car_lane = 0;
              } else if ( 4 < d && d < 8 ) {
                car_lane = 1;
              } else if ( 8 < d && d < 12 ) {
                car_lane = 2;
              }
              if (car_lane < 0) {
                std::cout << "check car lane - something went wrong !" << std::endl;
                continue;
              }
              // Find selected car speed.
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double speed = sqrt(vx*vx + vy*vy);
              double neighbor_car_s = sensor_fusion[i][5];
              // distance interpolating neighbor car s position
              neighbor_car_s += ((double)path_size*0.02*speed);

              if ( car_lane == current_lane ) {
                // Car is in our lane.
                car_ahead |= neighbor_car_s > car_s && neighbor_car_s - car_s < distance_threshold;
              } else if ( car_lane - current_lane == -1 ) {
                // Car is on our left
                car_left |= car_s - distance_threshold < neighbor_car_s && car_s + distance_threshold > neighbor_car_s;
              } else if ( car_lane - current_lane == 1 ) {
                // Car is on our right
                car_righ |= car_s - distance_threshold < neighbor_car_s && car_s + distance_threshold > neighbor_car_s;
              }
          }

          // handle our speed depending on neighbor cars
          double speed_adjustment = 0;
          if ( car_ahead ) { // Car ahead
            if ( !car_left && current_lane > 0 ) {
              // if there is no car left and there is a left lane.
              current_lane--; // Change lane left.
            } else if ( !car_righ && current_lane != 2 ){
              // if there is no car right and there is a right lane.
              current_lane++; // Change lane right.
            } else {
              speed_adjustment -= MAX_ACCELERATION;
            }
          } else {
            if ( current_lane != 1 ) { // if we are not on the center lane.
              if ( ( current_lane == 0 && !car_righ ) || ( current_lane == 2 && !car_left ) ) {
                current_lane = 1; // Back to center.
              }
            }
            if ( current_vel < MAX_SPEED ) {
              speed_adjustment += MAX_ACCELERATION;
            }
          }          
          // ---------------

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