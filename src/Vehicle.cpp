#include "Vehicle.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"

// ################################## Vehicle ##########################################
Vehicle::Vehicle()
{

}
Vehicle::Vehicle(sensor_fusion_input& input_data)
{
	this->timeStep		= input_data.timeStep;
	this->id			= input_data.id;
	this->v_x			= input_data.vx;
	this->v_y			= input_data.vy;
	this->cart_pos		= input_data.cartesian_pos;
	this->fernet_pos	= input_data.fernet_pos;
	this->lane_id       = -1;
	this->acc			= 0;
	this->yaw			= 0;
	this->predicted_s	= input_data.fernet_pos.s;
	this->speed			= sqrt( (this->v_x * this->v_x) + ( this->v_y * this->v_y) );
}

void Vehicle::update(Vehicle& input_vehicle)
{
	double prev_speed			= this->speed;
	cartesian_coords cart_prev  = this->cart_pos;

	this->timeStep		= input_vehicle.timeStep;
	this->id			= input_vehicle.id;
	this->v_x			= input_vehicle.v_x;
	this->v_y			= input_vehicle.v_y;
	this->cart_pos		= input_vehicle.cart_pos;
	this->fernet_pos	= input_vehicle.fernet_pos;
	this->speed			= sqrt((this->v_x * this->v_x) + (this->v_y * this->v_y));
	this->acc			= (this->speed - prev_speed) / 0.02;
	this->yaw			= atan2((this->cart_pos.y - cart_prev.y), (this->cart_pos.x - cart_prev.x));


	this->predict(50);
}

void Vehicle::predict(int prediction_span)
{
	// Predict Vehicle S
	this->predicted_s = this->fernet_pos.s + ((double)prediction_span * 0.02F * this->speed);
}

Vehicle::~Vehicle()
{

}


// ################################## Ego Vehicle ##########################################
EgoVehicle::EgoVehicle()
{
	this->lane_id = 1;
	this->ref_vel = 0;
}

/*
EgoVehicle::EgoVehicle(ego_vehicle_input& ego_veh)
{
	this->timeStep		= ego_veh.timeStep;
	this->cart_pos		= ego_veh.cartesian_pos;
	this->fernet_pos	= ego_veh.fernet_pos;
	
	this->end_path_s	= ego_veh.end_path_s;
	this->end_path_d	= ego_veh.end_path_d;

	this->speed			= ego_veh.speed;
	this->yaw			= ego_veh.yaw;

	this->v_x			= this->speed * cos(this->yaw);
	this->v_y			= this->speed * sin(this->yaw);

	this->lane_id		= 1;
}
*/
void EgoVehicle::update(ego_vehicle_input& ego_veh)
{
	this->timeStep = ego_veh.timeStep;
	this->cart_pos = ego_veh.cartesian_pos;
	this->fernet_pos = ego_veh.fernet_pos;

	this->end_path_s = ego_veh.end_path_s;
	this->end_path_d = ego_veh.end_path_d;

	this->speed = ego_veh.speed;
	this->yaw = ego_veh.yaw;

	this->v_x = this->speed * cos(this->yaw);
	this->v_y = this->speed * sin(this->yaw);

	this->previous_path_x = ego_veh.previous_path_x;
	this->previous_path_y = ego_veh.previous_path_y;

	this->traj_size = ego_veh.prev_size;
}

void EgoVehicle::generate_trajectory(vector<double>& next_x_s,
									 vector<double>& next_y_s,
									 const vector<double>& map_waypoints_x,
									 const vector<double>& map_waypoints_y,
									 const vector<double>& map_waypoints_s,
									 const int lane)
{
	double car_s;

	if (this->traj_size > 0)
	{
		car_s = this->end_path_s;
	}
	else
	{
		car_s = this->fernet_pos.s;
	}
	// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30 m
	// Later we will interlpolate these waypoints with a spline to fill in the gaps between points

	vector<double> ptsx;
	vector<double> ptsy;

	// Reference x , y , yaw states
	// Either we will reference the starting point as where the car is or at the previous path's end point

	double ref_x = this->cart_pos.x;
	double ref_y = this->cart_pos.y;
	double ref_yaw = deg2rad(this->yaw);

	// if previous size is almost empty, use the car as starting reference
	if (this->traj_size < 2)
	{
		// Use two points that make the path tangent to the car
		double prev_car_x = this->cart_pos.x - cos(this->yaw);
		double prev_car_y = this->cart_pos.y - sin(this->yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(this->cart_pos.x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(this->cart_pos.y);
	}
	// Use the previous path's end point as starting refrence
	else
	{
		// Redefine reference state as previous path end point
		ref_x = previous_path_x[this->traj_size - 1];
		ref_y = previous_path_y[this->traj_size - 1];

		double ref_x_prev = this->previous_path_x[this->traj_size - 2];
		double ref_y_prev = this->previous_path_y[this->traj_size - 2];
		ref_yaw = atan2((ref_y - ref_y_prev), (ref_x - ref_x_prev));

		// Use two points that make the path tangent to the previous path's end point
		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	// In Fernet add evenly 30 m spaced points ahead of starting reference
	//if (car_s + 30 < 6824)
	//{
		vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		ptsx.push_back(next_wp0[0]);
		ptsy.push_back(next_wp0[1]);
	//}
	//if (car_s + 60 < 6824)
	//{
		vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		ptsx.push_back(next_wp1[0]);
		ptsy.push_back(next_wp1[1]);
	//}
	//if (car_s + 90 < 6824)
	//{
		vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		ptsx.push_back(next_wp2[0]);
		ptsy.push_back(next_wp2[1]);
	//}


	for (int i = 0; i < ptsx.size(); i++)
	{
		// Shift car reference angle to 0 degrees
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
		ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);

	}

	// Create a spline
	tk::spline s;

	// Set (x,y) points to the spline
	// TODO :: Handle spline exceptions;
	s.set_points(ptsx, ptsy);

	// Define the actual (x,y) points we will use for the planner
	//vector<double> next_x_s;
	//vector<double> next_y_s;

	next_x_s.clear();
	next_y_s.clear();

	// Start with all the previous path points from the last time
	for (int i = 0; i < this->previous_path_x.size(); i++)
	{
		next_x_s.push_back(this->previous_path_x[i]);
		next_y_s.push_back(this->previous_path_y[i]);
	}

	// Calculate how to break up spline points so that we travel at our desired reference velocity
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

	double x_add_on = 0;

	for (int i = 1; i <= (50 - this->previous_path_x.size()); i++)
	{
		double N = (target_dist / (0.02F * this->ref_vel / 2.24));
		double x_point = x_add_on + (target_x / N);
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// Rotate back to normal coordinates from Ego coordinates
		x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		next_x_s.push_back(x_point);
		next_y_s.push_back(y_point);
	}

}

EgoVehicle::~EgoVehicle()
{

}