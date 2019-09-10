#ifndef LANE_H
#define LANE_H

#include "Vehicle.h"

class Lane
{
	public:
		// Members
		int	      id;
		int		  num_vehicles;
		Vehicle*  min_speed_veh;
		Vehicle*  max_speed_veh;
		Vehicle* nearest_to_ego_veh;

		lane_tags name;
		unordered_map<int,Vehicle> lane_vehicles;

		Lane();
		Lane(int id);

		~Lane();

		void add_vehicle(Vehicle& input_veh);

		void post_process(double current_timeStep);

		int get_id(lane_tags& name);

};

#endif // LANE_H

