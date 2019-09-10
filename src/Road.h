#ifndef ROAD_H
#define ROAD_H

#include"Lane.h"

class Road
{
	public:
		unordered_map<int, Lane> Lanes;
		EgoVehicle				 Ego_veh_c;
		double					 timeStep;
		bool                     generate_traj;

		Road();
		~Road();
		void display();
		void update(double timeStep, unordered_map<int, sensor_fusion_input>& vehicles_input, ego_vehicle_input& ego_veh, vector<double>& map_waypoints_x,vector<double>& map_waypoints_y,vector<double>& map_waypoints_s);
	
	private:
		void post_process_lanes();

		void add_vehicle_to_lane(Vehicle& input_veh);

		void plan_ego_manuever(vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s);


};

#endif // ROAD_H