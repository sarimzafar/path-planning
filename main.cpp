#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "perception.h"
#include "movement.h"

using namespace std;

// for convenience
using json = nlohmann::json;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  Vehicle vehicle = Vehicle(0,0,0,0,0,0);
  double ref_speed = 0.0;
  int lane = 1;
  	
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
  	map_waypoints_dx.push_back(d_x); // normal component to the waypoint in X-dir
  	map_waypoints_dy.push_back(d_y); // normal compoennt to the waypoint in Y-dir
  }

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &vehicle, &ref_speed, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
		// Define initial params
	const double max_speed = 48;
	const double max_acc = 0.25;
	
	if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          	vehicle.updateVehicleState(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"]);
		
			vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
			
			bool too_close = false;
			int newLane = getLane(sensor_fusion, vehicle, previous_path_x, previous_path_y);
			
			if (newLane == -1) {
				too_close = true;
			}
			
			if (newLane != lane && newLane >= 0) {
				cout << "New Lane - " << newLane << endl;
				lane = newLane;
			}

			if (too_close) {
				ref_speed -= max_acc;
			} else if(ref_speed < max_speed) {
				ref_speed += max_acc;
			}

			json msgJson;

          	// // Get the size of the previous path
			int prev_size = previous_path_x.size();

			// Spaced waypoints
			vector<double> ptsX;
			vector<double> ptsY;

			// Define the reference frame
			double ref_x = (vehicle.getVehicleXY())[0];
			double ref_y = (vehicle.getVehicleXY())[1];
			double ref_yaw = deg2rad((vehicle.getVehicleSpeedYaw())[1]);

			// Just starting
			if (prev_size < 2) {
				double prev_car_x = (vehicle.getVehicleXY())[0] - cos((vehicle.getVehicleSpeedYaw())[1]);
				double prev_car_y = (vehicle.getVehicleXY())[1] - sin((vehicle.getVehicleSpeedYaw())[1]);

				// Starting waypoint - is the last waypoint
				ptsX.push_back(prev_car_x);
				ptsX.push_back((vehicle.getVehicleXY())[0]);

				ptsY.push_back(prev_car_y);
				ptsY.push_back((vehicle.getVehicleXY())[1]);
			
			} else { // Use the previous path to plan the next path
				// Make sure the reference is now the end of the previous path
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];

				// Reference for the path before that
				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];

				// angle the vehicle is heading in right now
				ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

				// Add these waypoints
				ptsX.push_back(ref_x_prev);
				ptsX.push_back(ref_x);

				ptsY.push_back(ref_y_prev);
				ptsY.push_back(ref_y);
			}

			// Find the number of discritizing points from the current position and 30m ahead

			vector<double> next_wp0 = getXY((vehicle.getVehicleSD())[0] + 50, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp1 = getXY((vehicle.getVehicleSD())[0] + 100, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp2 = getXY((vehicle.getVehicleSD())[0] + 150, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsX.push_back(next_wp0[0]);
			ptsX.push_back(next_wp1[0]);
			ptsX.push_back(next_wp2[0]);

			ptsY.push_back(next_wp0[1]);
			ptsY.push_back(next_wp1[1]);
			ptsY.push_back(next_wp2[1]);

			for(int i = 0; i<ptsX.size(); ++i) {
				// Go through all the points and make sure they are with respect to last waypoint as center
				double shift_x = ptsX[i]-ref_x;
				double shift_y = ptsY[i]-ref_y;

				// No idea what happened here
				ptsX[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
				ptsY[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
			}

			tk::spline s;

			// Sort the vectors
			// sort(ptsX.begin(), ptsX.end());
			// sort(ptsY.begin(), ptsY.end());

			s.set_points(ptsX, ptsY); // Use spline to fit a spline through the points

			// Final points
			vector<double> next_x_vals;
			vector<double> next_y_vals;

			// First add all the previous path points
			for(int i = 0; i < previous_path_x.size(); i++) {
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			double last_x = 50.0;
			double last_y = s(last_x);

			double dist = distance(0, 0, last_x, last_y);
			double delta_x = 0;

			for (int i=0; i<50-previous_path_x.size(); i++) {
				// Add future points here
				double N = (dist/(.02*ref_speed/2.24));
				double x = delta_x + (last_x)/N;
				double y = s(x);

				delta_x = x;

				double x_ref = x;
				double y_ref = y;

				x = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
				y = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

				x += ref_x;
				y += ref_y; 

				next_x_vals.push_back(x);
				next_y_vals.push_back(y);
			}
			
			
			// Write to the controller
			msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-( -- LOL!
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
