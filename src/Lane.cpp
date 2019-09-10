#include"Lane.h"

Lane::Lane()
{

}

Lane::Lane(int lane_id)
{
	min_speed_veh		= NULL;
	max_speed_veh		= NULL;
	nearest_to_ego_veh	= NULL;
	id					= lane_id;
	num_vehicles		= 0;
	name				= (lane_tags)id;
}

void Lane::add_vehicle(Vehicle& input_vehicle)
{
	int input_veh_id = input_vehicle.id;

	if (lane_vehicles.find(input_veh_id) == lane_vehicles.end())
	{
		input_vehicle.lane_id = this->id;

		lane_vehicles[input_veh_id] = input_vehicle;
		
		this->num_vehicles++;
		
		if (lane_vehicles[input_veh_id].fernet_pos.s > 0)
		{
			// Assign Nearest Veh to Ego
			if (this->nearest_to_ego_veh == NULL)
			{
				this->nearest_to_ego_veh = &lane_vehicles[input_veh_id];
			}
			else if (input_vehicle.fernet_pos.s < this->nearest_to_ego_veh->fernet_pos.s)
			{
				this->nearest_to_ego_veh = &lane_vehicles[input_veh_id];
			}

			// Assign slowest car in lane
			if (this->min_speed_veh == NULL)
			{
				this->min_speed_veh = &lane_vehicles[input_veh_id];
			}
			else if (input_vehicle.speed < this->min_speed_veh->speed)
			{
				this->min_speed_veh = &lane_vehicles[input_veh_id];
			}

			// Assign fastest car in lane
			if (this->max_speed_veh == NULL)
			{
				this->max_speed_veh = &lane_vehicles[input_veh_id];
			}
			else if (input_vehicle.speed > max_speed_veh->speed)
			{
				this->max_speed_veh = &lane_vehicles[input_veh_id];
			}
		}


	}
	else
	{
		lane_vehicles[input_veh_id].update(input_vehicle);

		if (lane_vehicles[input_veh_id].fernet_pos.s > 0)
		{
			// Assign Nearest Veh to Ego
			if (this->nearest_to_ego_veh == NULL)
			{
				this->nearest_to_ego_veh = &lane_vehicles[input_veh_id];
			}
			else if (input_vehicle.fernet_pos.s < this->nearest_to_ego_veh->fernet_pos.s)
			{
				this->nearest_to_ego_veh = &lane_vehicles[input_veh_id];
			}

			// Assign slowest car in lane
			if (this->min_speed_veh == NULL)
			{
				this->min_speed_veh = &lane_vehicles[input_veh_id];
			}
			else if (input_vehicle.speed < this->min_speed_veh->speed)
			{
				this->min_speed_veh = &lane_vehicles[input_veh_id];
			}

			// Assign fastest car in lane
			if (this->max_speed_veh == NULL)
			{
				this->max_speed_veh = &lane_vehicles[input_veh_id];
			}
			else if (input_vehicle.speed > this->max_speed_veh->speed)
			{
				this->max_speed_veh = &lane_vehicles[input_veh_id];
			}
		}

	}
}

void Lane::post_process(double current_timeStep)
{
	unordered_map<int, Vehicle>::iterator it;
	vector<int> erase_ids;
	
	for (it = lane_vehicles.begin(); it != lane_vehicles.end(); it++)
	{
		if (it->second.timeStep != current_timeStep)
		{
			erase_ids.push_back(it->first);
		}
	}

	for (vector<int>::iterator it = erase_ids.begin(); it != erase_ids.end(); it++)
	{
		if (this->nearest_to_ego_veh != NULL && this->nearest_to_ego_veh->id == *it)
		{
			this->nearest_to_ego_veh = NULL;
		}
		if (this->min_speed_veh != NULL && this->min_speed_veh->id == *it)
		{
			this->min_speed_veh = NULL;
		}
		if (this->max_speed_veh != NULL && this->max_speed_veh->id == *it)
		{
			this->max_speed_veh = NULL;
		}

		lane_vehicles.erase(*it);
	}

	if (this->nearest_to_ego_veh != NULL && this->nearest_to_ego_veh->fernet_pos.s < 0)
	{
		this->nearest_to_ego_veh = NULL;
	}
	if (this->min_speed_veh != NULL && this->min_speed_veh->fernet_pos.s < 0)
	{
		this->min_speed_veh = NULL;
	}
	if (this->max_speed_veh != NULL && this->max_speed_veh->fernet_pos.s < 0)
	{
		this->max_speed_veh = NULL;
	}

}

int Lane::get_id(lane_tags& name)
{
	return (int)this->name;
}

Lane::~Lane()
{

}