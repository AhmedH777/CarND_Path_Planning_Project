#ifndef COSTS_H
#define COSTS_H

#include"Lane.h"

double speed_cost(double speed, double target_speed, double Buffer_vel);
double lane_velocity_cost(Lane& lane, double target_speed, double speed_limit);

double nearest_veh_distance_cost(Lane& lane);

double path_accelration_cost(const vector<double>& x_vals, const vector<double>& y_vals, const double current_speed);

double collision_cost(Lane& lane, double ego_veh_s);

double keep_lane_cost(Lane& lane, int ego_lane_id);
#endif // COSTS_H