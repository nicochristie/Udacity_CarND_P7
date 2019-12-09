#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "ThirdParty/spline.h"
#include "ThirdParty/fsm.h"
#include "helpers.h"
#include "json.hpp"
#include "myfsm.h"

//#define PRINT_DEBUG

using namespace std;

// for convenience
using json = nlohmann::json;
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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

  int currLane = MIDDLE_LANE;   // Sim starts in lane 1
  bool blockedLeft = false;     // Any object blocking left?
  bool blockedFront = false;    // Any object blocking front?
  bool blockedRight = false;    // Any object blocking right?
  double currVelocity = 0.0;    // Reference velocity (mph)
  double blckVelocity = 0.0;    // Blocking object velocity

  // FSM States setup
  // State machine with 4 states (I am not including the 'Prepare change lane' states)
  fsm.add_transitions({
    //  From state,          To state,              Triggers,
    //  Conditions,
    //  Actions
    {
        States::KeepLane,    States::KeepLane,      Triggers::ClearAhead,
        [&] { return !blockedFront; },
        [&] { if (currVelocity < TARGET_VELOCITY) { currVelocity += min(abs(TARGET_VELOCITY - currVelocity), MAX_VEL_INC); }}
    }, {
        States::Follow,      States::KeepLane,      Triggers::ClearAhead,
        [&] { return !blockedFront; },
        [&] { if (currVelocity < TARGET_VELOCITY) { currVelocity += min(abs(TARGET_VELOCITY - currVelocity), MAX_VEL_INC); }}
    }, {
        States::ChangeLeft,  States::KeepLane,      Triggers::ClearAhead,
        [&] { return !blockedFront; },
        [&] {}
    }, {
        States::ChangeRight, States::KeepLane,      Triggers::ClearAhead,
        [&] { return !blockedFront; },
        [&] {}
    }, {
        States::KeepLane,    States::ChangeLeft,    Triggers::BlockAhead,
        [&] { return blockedFront && !blockedLeft; },
        [&] { currLane--; }
    }, {
        States::Follow,      States::ChangeLeft,    Triggers::BlockAhead,
        [&] { return blockedFront && !blockedLeft; },
        [&] { currLane--; }
    }, {
        States::KeepLane,    States::ChangeRight,   Triggers::BlockAhead,
        [&] { return blockedFront && !blockedRight; },
        [&] { currLane++; }
    }, {
        States::Follow,      States::ChangeRight,   Triggers::BlockAhead,
        [&] { return blockedFront && !blockedRight; },
        [&] { currLane++; }
    }, {
        States::ChangeRight, States::Follow,        Triggers::BlockAhead,
        [&] { return blockedFront; },
        [&] { currVelocity -= min(abs(blckVelocity - currVelocity), MAX_VEL_DEC); }
    }, {
        States::ChangeLeft,  States::Follow,        Triggers::BlockAhead,
        [&] { return blockedFront; },
        [&] { currVelocity -= min(abs(blckVelocity - currVelocity), MAX_VEL_DEC); }
    }, {
        States::KeepLane,    States::Follow,        Triggers::BlockAhead,
        [&] { return true; },
        [&] { currVelocity -= min(abs(blckVelocity - currVelocity), MAX_VEL_DEC); }
    }, {
        States::Follow,      States::Follow,        Triggers::BlockAhead,
        [&] { return true; },
        [&] { currVelocity -= min(abs(blckVelocity - currVelocity), MAX_VEL_DEC); }
    } });
  fsm.add_debug_fn(fsm_info);

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
    &blockedLeft, &blockedFront, &blockedRight, &currLane, &currVelocity, &blckVelocity]
    (uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      //auto sdata = string(data).substr(0, length);
      //cout << sdata << endl;
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

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            /**
             * TODO: define a path made up of (x,y) points that the car will visit
             *   sequentially every .02 seconds
             */
            int prev_size = previous_path_x.size();
            if (prev_size > 0) { car_s = end_path_s; }

            // Get the surrounding objects' relative positions and determine if the car
            // needs to change lanes and if, or to which side it can (giving 'left' a switch preference)
            blockedLeft = false;
            blockedFront = false;
            blockedRight = false;
            blckVelocity = TARGET_VELOCITY;
            double targetSpeed = TARGET_VELOCITY;

            for (int i = 0; i < sensor_fusion.size(); i++) {
              // Check if the object is in a valid lane
              float d = sensor_fusion[i][6];
              int ol = UNKNOWN_LANE;

              // Determine exactly which lane the detected object is in
              if (LEFT_LANE_MIN_D < d && d < LEFT_LANE_MAX_D) { ol = LEFT_LANE; }
              else if (MIDDLE_LANE_MIN_D < d && d < MIDDLE_LANE_MAX_D) { ol = MIDDLE_LANE; }
              else if (RIGHT_LANE_MIN_D < d && d < RIGHT_LANE_MAX_D) { ol = RIGHT_LANE; }

              if (ol == UNKNOWN_LANE) { continue; } // error condition, skip object

              // Since the object is in a valid lane, get its trajectory
              // Get the speed of the detected object and correct with previous trajectory values
              double ovx = sensor_fusion[i][3];
              double ovy = sensor_fusion[i][4];
              double os = sensor_fusion[i][5];
              os += (double)prev_size * 0.02 * distance(ovx, ovy, 0, 0);

              // Get (rectangular (which is good in this case!)) margin bounds for our car.
              // Detected object is on the next adjacent left lane
              if (ol == currLane - 1) { blockedLeft |= car_s - MIN_MARGIN_TO_OBJECTS < os && os < car_s + MIN_MARGIN_TO_OBJECTS; }
              // Detected object on the same lane
              else if (ol == currLane) { blockedFront |= car_s < os && os - car_s < MIN_MARGIN_TO_OBJECTS; if (blockedFront) blckVelocity = os; }
              // Detected object is on the next adjacent right lane
              else if (ol == currLane + 1) { blockedRight |= car_s - MIN_MARGIN_TO_OBJECTS < os && os < car_s + MIN_MARGIN_TO_OBJECTS; }
            }

            blockedLeft |= currLane == LEFT_LANE;   // Block a lane change to the left if the car already is at the leftmost lane
            blockedRight |= currLane == RIGHT_LANE; // Block a lane change to the right if the car already is at the rightmost lane
            if (!blockedLeft) blockedRight = true;  // Prefer left side overtakes if the left side is available for lane change
            targetSpeed = min(blckVelocity, TARGET_VELOCITY);

            // Once the surrounding conditions are observed and analysed, transition the FSM and execute an action
            if (blockedFront) { fsm.execute(Triggers::BlockAhead); }    // Execute 'BlockAhead' trigger on FSM
            else { fsm.execute(Triggers::ClearAhead); }                 // Execute 'ClearAhead' trigger on FSM

#ifdef PRINT_DEBUG
            switch (currLane) {
            case LEFT_LANE:
              std::cout << '|';
              if (blockedFront) std::cout << '0'; else std::cout << '^';
              if (blockedRight) std::cout << '0'; else std::cout << '^';
              std::cout << ' ' << '|' << '\n';
              std::cout << "|^  |\n";
              break;
            case MIDDLE_LANE:
              std::cout << '|';
              if (blockedLeft) std::cout << '0'; else std::cout << '^';
              if (blockedFront) std::cout << '0'; else std::cout << '^';
              if (blockedRight) std::cout << '0'; else std::cout << '^';
              std::cout << '|' << '\n';
              std::cout << "| ^ |\n";
              break;
            case RIGHT_LANE:
              std::cout << '|' << ' ';
              if (blockedLeft) std::cout << '0'; else std::cout << '^';
              if (blockedFront) std::cout << '0'; else std::cout << '^';
              std::cout << '|' << '\n';
              std::cout << "|  ^|\n";
              break;
            default:
              break;
            }
#endif // PRINT_DEBUG

            // 3. TRAJECTORY: Calculate trajectory for the car to follow
            // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // later interpolate waypoints with spline and fill in more waypoints

            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // If previous size is almost empty, use car as starting reference
            if (prev_size < 2) {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
            // Use previous path's points as starting reference
            else {
              // Redefine reference state as previous path endpoint
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Use two points that make the path tangent to the previous path's endpoint
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            // Add evenly spaced waypoints in standard XY coordinates (from Frenet coordinates)
            vector<double> next_wp0 = getXY(car_s + 1 * MIN_MARGIN_TO_OBJECTS, (LANE_WIDTH * currLane + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 2 * MIN_MARGIN_TO_OBJECTS, (LANE_WIDTH * currLane + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 3 * MIN_MARGIN_TO_OBJECTS, (LANE_WIDTH * currLane + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for (int i = 0; i < ptsx.size(); i++) {
              // Shift car angle reference to 0 degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
            }

            // Create a spline
            tk::spline s;

            // Set (x,y) points to the spline
            s.set_points(ptsx, ptsy);

            // Define the actual (x,y) points that will be used for the planner
            vector<double> next_x_vals, next_y_vals;

            // Start with all of the previous path points from last time
            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate how to break up spline points so that the desired refrence velocity is kept
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = distance(target_x, target_y, 0, 0);

            double x_add_on = 0;

            // Fill up the rest of the path planner after filling it with previous points
            // Always 50 points will be output
            for (int i = 0; i <= 50 - previous_path_x.size(); i++) {

              if (currVelocity > targetSpeed) { currVelocity = targetSpeed; }
              else if (currVelocity < MAX_VEL_INC) { currVelocity = MAX_VEL_INC; }

              double N = target_dist / (0.02 * currVelocity / 2.24);
              double x_point = x_add_on + target_x / N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Rotate back to normal after rotating earlier
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\"," + msgJson.dump() + "]";

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }  // end "telemetry" if
        }
        else {
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
    char* message, size_t length) {
      ws.close();
      std::cout << "Disconnected" << std::endl;
    });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}