#include"Road.h"
#include"costs.h"
#include <algorithm> 


Road::Road()
{
	this->Lanes[0] = Lane(0);
	this->Lanes[1] = Lane(1);
	this->Lanes[2] = Lane(2);
	this->generate_traj = true;
}

void Road::update(double timeStep,
				  unordered_map<int,sensor_fusion_input>& vehicles_input,
				  ego_vehicle_input& ego_veh,
				  vector<double>& map_waypoints_x,
				  vector<double>& map_waypoints_y,
				  vector<double>& map_waypoints_s)
{
	this->timeStep = timeStep;
	
	Ego_veh_c.update(ego_veh);

	// Update Vehicles in Lanes
	unordered_map<int, sensor_fusion_input>::iterator it;
	for (it = vehicles_input.begin(); it != vehicles_input.end(); it++)
	{
		Vehicle current_vehicle =  Vehicle(it->second);

		// Update Relative Data
		current_vehicle.cart_pos.x -= Ego_veh_c.cart_pos.x;
		current_vehicle.cart_pos.y -= Ego_veh_c.cart_pos.y;

		current_vehicle.fernet_pos.s -= Ego_veh_c.fernet_pos.s;
		current_vehicle.fernet_pos.d -= Ego_veh_c.fernet_pos.d;
		
		add_vehicle_to_lane(current_vehicle);
	}

	// Clean up the UnUpdated Vehicles
	post_process_lanes();

	// Plan Next Ego Manuever
	plan_ego_manuever(map_waypoints_x,
					  map_waypoints_y,
					  map_waypoints_s );

	// Generate planned trajectory
	//if (this->generate_traj == true && Ego_veh_c.end_path_s < (map_waypoints_s.back() - 200))
	//{
		Ego_veh_c.generate_trajectory(Ego_veh_c.next_x_vals,
									  Ego_veh_c.next_y_vals,
									  map_waypoints_x,
									  map_waypoints_y,
									  map_waypoints_s,
									  Ego_veh_c.lane_id);
	//}
	//else
	//{
		//this->generate_traj = false;
		//std::cout << "End of Map data" << std::endl;
	//}


}

void Road::add_vehicle_to_lane(Vehicle& input_veh)
{
	double input_veh_d = input_veh.fernet_pos.d + Ego_veh_c.fernet_pos.d;

	if (input_veh_d > 0 && input_veh_d < 4)
	{
		Lanes[0].add_vehicle(input_veh);
	}
	else if (input_veh_d > 4 && input_veh_d < 8)
	{
		Lanes[1].add_vehicle(input_veh);
	}
	else if (input_veh_d > 8 && input_veh_d < 12)
	{
		Lanes[2].add_vehicle(input_veh);
	}
}

void Road::post_process_lanes()
{
	unordered_map<int, Lane>::iterator it;

	for (it = Lanes.begin(); it != Lanes.end(); it++)
	{
		it->second.post_process(this->timeStep);
	}
}

void Road::plan_ego_manuever(vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s)
{
	double car_s;

	if (this->Ego_veh_c.traj_size > 0)
	{
		car_s = this->Ego_veh_c.end_path_s - this->Ego_veh_c.fernet_pos.s;
	}
	else
	{
		car_s = 0;
	}

	//######################################################################################
	unordered_map<int,double> costs;
	vector<int> possible_lanes;
	int current_ego_lane = Ego_veh_c.lane_id;

	// ########################### Detect Possible lanes for manuever #################################################
	switch (current_ego_lane)
	{
		case 0:
			possible_lanes.push_back(1);
			possible_lanes.push_back(0);
			break;
		case 1:
			possible_lanes.push_back(2);
			possible_lanes.push_back(0);
			possible_lanes.push_back(1);
			break;
		case 2:
			possible_lanes.push_back(1);
			possible_lanes.push_back(2);
			break;
	}

	// ########################### Calculate cost for each possible lane #################################################
	vector<int>::iterator lane_it;
	for (lane_it = possible_lanes.begin(); lane_it != possible_lanes.end(); lane_it++)
	{
		double current_cost = 0;
		double lane_vel_cost = 0;
		double collis_cost = 0;
		double cost_keep_lane = 0;
		double cost_nearest_veh_dist = 0;
		double cost_path_acc = 0;
		
		// Calculate weights
		double weight = 1;
		if (this->Ego_veh_c.lane_id != *lane_it)
		{
			weight = 10;
		}

		// Calculate Predicted Trajectory given Lane
		vector<double> next_xs;
		vector<double> next_ys;

		Ego_veh_c.generate_trajectory(next_xs,
									  next_ys,
									  map_waypoints_x,
									  map_waypoints_y,
									  map_waypoints_s,
									  *lane_it);

		// Calculate Costs
		lane_vel_cost += lane_velocity_cost(Lanes[*lane_it], 49.5, 50);
		collis_cost += collision_cost(Lanes[*lane_it],car_s);
		cost_keep_lane += keep_lane_cost(Lanes[*lane_it], Ego_veh_c.lane_id);
		cost_nearest_veh_dist += nearest_veh_distance_cost(Lanes[*lane_it]);
		cost_path_acc += path_accelration_cost(next_xs,next_ys,Ego_veh_c.speed);

		current_cost = lane_vel_cost + (weight * collis_cost) + (10 * cost_nearest_veh_dist) + (5 * cost_path_acc); //+ cost_keep_lane;

		costs[*lane_it] = current_cost;
		
	//std::cout << "lane_vel_cost = " << (lane_vel_cost) << "  collision_cost = " << (weight * collis_cost) << std::endl;

	}
	//std::cout << "###################################" << std::endl;
	//std::cout << std::endl;


	
	// ########################### Choose Lane with minimum cost #################################################
	unordered_map<int, double>::iterator it;
	int lane_id;
	double min_cost = 100;

	for (it = costs.begin(); it != costs.end(); it++)
	{
		if (it->second < min_cost)
		{
			lane_id = it->first;
			min_cost = it->second;
		}
	}

	// Prefer Keeping Lanes
	if (costs[this->Ego_veh_c.lane_id] != min_cost)
	{
		this->Ego_veh_c.lane_id = lane_id;
	}
	
	// ########################### Adjust Ego Speed based on Front Veh #################################################
	/*
	if ((Lanes[Ego_veh_c.lane_id].nearest_to_ego_veh != NULL))
	{
		std::cout << "Nearest Veh pred s = " << Lanes[Ego_veh_c.lane_id].nearest_to_ego_veh->predicted_s << "  Car_s = " << car_s << std::endl;

	}
	*/
	double Ego_current_ref_vel = 0;
	if ((Lanes[Ego_veh_c.lane_id].nearest_to_ego_veh != NULL) && ((Lanes[Ego_veh_c.lane_id].nearest_to_ego_veh->predicted_s - car_s) < 20 ))
	{
		Ego_current_ref_vel = Lanes[Ego_veh_c.lane_id].nearest_to_ego_veh->speed;
	}
	else
	{
		Ego_current_ref_vel = 49.5;
	}


	if (this->Ego_veh_c.ref_vel < Ego_current_ref_vel)
	{
		this->Ego_veh_c.ref_vel += 0.224;
	}
	else
	{
		this->Ego_veh_c.ref_vel -= 0.224;
	}
	
	
}

void Road::display() {

	
	std::cout << "############ TimeStep " << this->timeStep << "############" << std::endl;
	std::cout << "Ego Vehicle " << "Lane " << this->Ego_veh_c.lane_id << std::endl;
	unordered_map<int, Lane>::iterator lane_it;
	for (lane_it = Lanes.begin(); lane_it != Lanes.end(); lane_it++)
	{
		std::cout << "Lane " << lane_it->first << std::endl;
		std::cout << "-------" << std::endl;

		unordered_map<int, Vehicle>::iterator veh_it;
		for (veh_it = lane_it->second.lane_vehicles.begin(); veh_it != lane_it->second.lane_vehicles.end(); veh_it++)
		{

			std::cout << "Vehicle[" << veh_it->first <<"]  " << "pred_s = " << veh_it->second.predicted_s << "s = "<<veh_it->second.fernet_pos.s<< "  d = " << veh_it->second.fernet_pos.d << "  x = " << veh_it->second.cart_pos.x << "  y = " << veh_it->second.cart_pos.y << " speed = " << veh_it->second.speed << std::endl;

		}
		if (lane_it->second.nearest_to_ego_veh != NULL)
		{
			std::cout << "Nearest_Vehicle[" << lane_it->second.nearest_to_ego_veh->id << "]" << std::endl;
		}
		std::cout << "----------------------------------" << std::endl;
		
	}
	

	/*
	vector<vector<string>> road;

	for (int i = 0; i < 50; ++i) {
		vector<string> road_lane;
		for (int ln = 0; ln < 3; ++ln) {
			road_lane.push_back("     ");
		}
		road.push_back(road_lane);
	}

	int s_min = 0;
	int s_max = 50;

	for (int i = 0; i < 3; i++)
	{
		unordered_map<int, Vehicle>::iterator it;

		for (it = Lanes[i].lane_vehicles.begin(); it != Lanes[i].lane_vehicles.end(); it++)
		{
			int v_id = it->second.id;

			if (s_min <= it->second.fernet_pos.s && it->second.fernet_pos.s < s_max) {
				string marker = "";

				std::stringstream oss;
				std::stringstream buffer;
				buffer << " ";
				oss << v_id;

				for (int buffer_i = oss.str().length(); buffer_i < 3; ++buffer_i) {
					buffer << "0";
				}
				buffer << oss.str() << " ";
				marker = buffer.str();
				
				road[int(it->second.fernet_pos.s - s_min)][int(i)] = marker;
			}
		}
	}
	
	std::ostringstream oss;
	oss << "+Meters ======================+ step: " << this->timeStep << std::endl;
	int i = s_min;

	for (int lj = 0; lj < road.size(); ++lj) {
		if (i % 20 == 0) {
			std::stringstream buffer;
			std::stringstream dis;
			dis << i;

			for (int buffer_i = dis.str().length(); buffer_i < 3; ++buffer_i) {
				buffer << "0";
			}

			oss << buffer.str() << dis.str() << " - ";
		}
		else {
			oss << "      ";
		}
		++i;
		for (int li = 0; li < road[0].size(); ++li) {
			oss << "|" << road[lj][li];
		}
		oss << "|";
		oss << "\n";
	}

	std::cout << oss.str();
	*/
}

Road::~Road()
{

}
