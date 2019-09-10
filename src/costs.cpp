#include"costs.h"

double speed_cost(double speed,double target_speed,double speed_limit)
{
	double cost = 0;
	double stop_cost = 0.8;

	if (speed < target_speed)
	{
		cost = stop_cost * ((target_speed - speed) / target_speed);
	}
	else if ((speed > target_speed) && (speed < speed_limit))
	{
		cost = (speed - target_speed) / (speed_limit - target_speed);
	}
	else
	{
		cost = 1;
	}

	return cost;
}

double lane_velocity_cost(Lane& lane,double target_speed,double speed_limit)
{
	if (lane.nearest_to_ego_veh != NULL && lane.nearest_to_ego_veh->fernet_pos.s < 100)
	{
		return speed_cost(lane.nearest_to_ego_veh->speed, target_speed, speed_limit);
	}
	else
	{
		return 0;
	}
	
}

double nearest_veh_distance_cost(Lane& lane)
{
	double nearest_distance = 100;

	if (lane.nearest_to_ego_veh != NULL && lane.nearest_to_ego_veh->fernet_pos.s < 100)
	{
		nearest_distance = lane.nearest_to_ego_veh->fernet_pos.s;
	}

	return (1 - exp(-1 / nearest_distance));
}


double path_accelration_cost(const vector<double>& x_vals, const vector<double>& y_vals, const double current_speed)
{
	double distance_prev;
	double distance;
	double speed_prev = current_speed;

	for (int i = 0; i < (x_vals.size() - 1); i++)
	{
		distance_prev = sqrt((x_vals[i] * x_vals[i]) + (y_vals[i] + y_vals[i]));
		distance      = sqrt((x_vals[i+1] * x_vals[i+1]) + (y_vals[i+1] + y_vals[i+1]));

		double speed = (distance - distance_prev) / 0.02;
		double acc = (speed - speed_prev) / 0.02;
		
		speed_prev = speed;

		if (acc > 10)
		{
			return 1;
		}
	}

	return 0;
}

double collision_cost(Lane& lane, double ego_veh_s)
{
	unordered_map<int, Vehicle>::iterator it;

	for (it = lane.lane_vehicles.begin(); it != lane.lane_vehicles.end(); it++)
	{
		if ((it->second.predicted_s > 0) && ((it->second.predicted_s - ego_veh_s) < 20))
		{
			return 1;
		}
	}
	
	return 0;
}

double keep_lane_cost(Lane& lane, int ego_lane_id)
{
	if (lane.id == ego_lane_id)
	{
		return 0;
	}
	else
	{
		return 0.5;
	}
}