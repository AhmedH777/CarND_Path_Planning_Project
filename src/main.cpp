#include <uWS/uWS.h>
#include "json.hpp"
#include "Road.h"

// for convenience
using nlohmann::json;

// Define Start lane
//int lane = 1;

double timeStep = 0;

Road Road_c = Road();

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) {
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

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_;

  in_map_.open(map_file_.c_str(), std::ifstream::in);

  if (!in_map_.is_open()) {
	  std::cerr << "error: file open failed " << map_file_ << ".\n";
	  return 1;
  }

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
			timeStep += 1;
          // j[1] is the data JSON object
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

		  // Previous data size
		  int prev_size = previous_path_x.size();

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

		  /*##################################### Parsing ###################################################*/
		  /*############################### Parse Ego Vehicle ##########################################*/
		  ego_vehicle_input ego_veh_in;

		  ego_veh_in.timeStep = timeStep;

		  ego_veh_in.cartesian_pos.x = j[1]["x"];
		  ego_veh_in.cartesian_pos.y = j[1]["y"];

		  ego_veh_in.fernet_pos.s = j[1]["s"];
		  ego_veh_in.fernet_pos.d = j[1]["d"];

		  ego_veh_in.yaw = j[1]["yaw"];
		  ego_veh_in.speed = j[1]["speed"];

		  ego_veh_in.end_path_s = j[1]["end_path_s"];
		  ego_veh_in.end_path_d = j[1]["end_path_d"];

		  ego_veh_in.prev_size = prev_size;

		  vector<double> prev_path_x = previous_path_x;
		  vector<double> prev_path_y = previous_path_y;

		  ego_veh_in.previous_path_x = prev_path_x;
		  ego_veh_in.previous_path_y = prev_path_y;


		  /*############################### Parse Sensor Fusion #######################################*/
		  unordered_map<int, sensor_fusion_input> current_vehicles;
		  for (int i = 0; i < sensor_fusion.size(); i++)
		  {
				  sensor_fusion_input current_input;

				  current_input.timeStep = timeStep;
				  current_input.id = sensor_fusion[i][0];
				  current_input.cartesian_pos.x = sensor_fusion[i][1];
				  current_input.cartesian_pos.y = sensor_fusion[i][2];
				  current_input.vx = sensor_fusion[i][3];
				  current_input.vy = sensor_fusion[i][4];
				  current_input.fernet_pos.s = sensor_fusion[i][5];
				  current_input.fernet_pos.d = sensor_fusion[i][6];

				  current_vehicles[sensor_fusion[i][0]] = current_input;

		  }

		  Road_c.update(timeStep, current_vehicles, ego_veh_in, map_waypoints_x, map_waypoints_y, map_waypoints_s);
		  //Road_c.display();

		  /*##########################################################################################*/
          msgJson["next_x"] = Road_c.Ego_veh_c.next_x_vals;
          msgJson["next_y"] = Road_c.Ego_veh_c.next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
		timeStep = 0;
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
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}