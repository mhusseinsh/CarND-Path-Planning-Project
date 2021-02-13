#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <fstream>
#include <iostream>
#include <string>
#include <uWS/uWS.h>
#include <vector>

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

  // Start in lane 1 (middle lane)
  int ego_car_lane = 1;

  // Reference velocity
  double ref_vel = 0.0; // mph

  // Define const values for lane information
  const int LEFT_LANE = 0;
  const int MIDDLE_LANE = 1;
  const int RIGHT_LANE = 2;

  const int ORIGIN_LANE_BOUNDARY = 0;
  const int LEFT_LANE_BOUNDARY = 4;
  const int MIDDLE_LANE_BOUNDARY = 8;
  const int RIGHT_LANE_BOUNDARY = 12;

  // Maximum velocity and acceleration
  const double MAX_VELOCITY = 49.5;
  const double MAX_ACC = .224;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &ego_car_lane,
               &ref_vel, &MAX_VELOCITY, &MAX_ACC](uWS::WebSocket<uWS::SERVER> ws, char *data,
                         size_t length, uWS::OpCode opCode) {
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

          // Size of previous path points
          int prev_size = previous_path_x.size();

          // Preventing collitions.
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          bool car_front = false;
          bool car_left = false;
          bool car_right = false;

          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            int other_car_lane = -1;
            // Check if other car is in the left lane (between 0 and 4)
            if (d > ORIGIN_LANE_BOUNDARY && d < LEFT_LANE_BOUNDARY) {
              other_car_lane = LEFT_LANE;
            }
            // Check if other car is in the middle lane (between 4 and 8)
            else if (d > LEFT_LANE_BOUNDARY && d < MIDDLE_LANE_BOUNDARY) {
              other_car_lane = MIDDLE_LANE;
            }
            // Check if other car is in the right lane (between 8 and 12)
            else if (d > MIDDLE_LANE_BOUNDARY && d < RIGHT_LANE_BOUNDARY) {
              other_car_lane = RIGHT_LANE;
            } else {
              continue;
            }
            // Find car speed.
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double)prev_size * 0.02 * check_speed);

            // If the other car is on the same lane
            if (other_car_lane == ego_car_lane) {
              // If the car is close to my car
              car_front |= check_car_s > car_s && (check_car_s - car_s) < 30;
            }
            // If the other car is on the left of my car
            else if (other_car_lane - ego_car_lane == -1) {
              // If the car is close to my car
              car_left |=
                  car_s - 30 < (check_car_s && car_s) + 30 > check_car_s;
            }
            // If the other car is on the right of my car
            else if (other_car_lane - ego_car_lane == 1) {
              // If the car is close to my car
              car_right |=
                  car_s - 30 < (check_car_s && car_s) + 30 > check_car_s;
            }
          }

          // Take Action
          // If vehicle is on the same lane and close to my car
          if (car_front) {
            // Priority for left side, If no car is on the left side, then do a
            // left lane change
            if (!car_left && ego_car_lane > LEFT_LANE) {
              ego_car_lane--;
            }
            // Else, go to right side, If no car is on the right side, then do a
            // right lane change
            else if (!car_right && ego_car_lane != RIGHT_LANE) {
              ego_car_lane++;
            }
            // If no lane change, then decrease speed for maintaining safety
            // distance
            else {
              ref_vel -= MAX_ACC;
            }
          } else {
            // Priority is to always drive in middle lane - If our car is not in
            // the middle lane
            if (ego_car_lane != MIDDLE_LANE) {
              // Go back to the middle lane
              if ((ego_car_lane == LEFT_LANE && !car_right) ||
                  (ego_car_lane == RIGHT_LANE && !car_left)) {
                ego_car_lane = MIDDLE_LANE;
              }
            }
            // Accelerate if no vehicles are front ahead
            if (ref_vel < MAX_VELOCITY) {
              ref_vel += MAX_ACC;
            }
          }

          // Calculate Trajectory
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Check if there are previous points
          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Using the latest two points
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Target Points
          vector<double> next_wp0 =
              getXY(car_s + 30, 2 + 4 * ego_car_lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 =
              getXY(car_s + 60, 2 + 4 * ego_car_lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 =
              getXY(car_s + 90, 2 + 4 * ego_car_lane, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Transform to car coordinate system
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create the spline.
          tk::spline s;
          s.set_points(ptsx, ptsy);

          // Ouptut from previous path
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate distance with 30 meters ahead
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          for (int i = 1; i < 50 - prev_size; i++) {
            // Avoid exceeding speed limit
            if (ref_vel > MAX_VELOCITY) {
              ref_vel = MAX_VELOCITY;
            }
            // Avoid driving at 0 speed
            else if (ref_vel < MAX_ACC) {
              ref_vel = MAX_ACC;
            }
            double N = target_dist / (0.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
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

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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