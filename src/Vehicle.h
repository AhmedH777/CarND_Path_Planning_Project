#ifndef VEHICLE_H
#define VEHICLE_H

#include"types.h"

class Vehicle
{
public:

	// Members
	cartesian_coords cart_pos;
	fernet_coords    fernet_pos;
	double			 predicted_s;
	double		     yaw;
	double           timeStep;
	double			 speed;
	double			 v_x;
	double           v_y;
	double			 acc;
	int			     lane_id;
	int				 id;

	// Constructor
	Vehicle();
	Vehicle(sensor_fusion_input& input_data);
	virtual void update(Vehicle& input_vehicle);
	void predict(int prediction_span);

	// Destructor
	~Vehicle();

};

class EgoVehicle : public Vehicle
{
public:

	// Members
	double					end_path_s;
	double					end_path_d;
	vector<double>			next_x_vals;
	vector<double>			next_y_vals;
	vector<double>			previous_path_x;
	vector<double>			previous_path_y;
	double					ref_vel;
	int						traj_size;

	// Constructor
	EgoVehicle();
	//EgoVehicle(ego_vehicle_input& ego_veh);
	
	void update(ego_vehicle_input& ego_veh);

	void generate_trajectory(vector<double>& next_x_s,
							 vector<double>& next_y_s, 
							 const vector<double>& map_waypoints_x,
							 const vector<double>& map_waypoints_y,
							 const vector<double>& map_waypoints_s,
							 const int lane);
	// Destructor
	~EgoVehicle();

};
#endif //VEHICLE_H