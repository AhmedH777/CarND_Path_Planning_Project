#ifndef TYPES_H
#define TYPES_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <string>
#include "math.h"

using std::vector;
using std::string;
using std::unordered_map;


enum lane_tags
{
	Left = 0,
	Middle,
	Right
};


struct cartesian_coords
{
	double x;
	double y;
};

struct fernet_coords
{
	double s;
	double d;
};

struct sensor_fusion_input
{
	double timeStep;
	int id;
	cartesian_coords cartesian_pos;
	fernet_coords	 fernet_pos;
	double vx;
	double vy;
};

struct ego_vehicle_input
{
	double				timeStep;
	cartesian_coords	cartesian_pos;
	fernet_coords		fernet_pos;
	double				speed;
	double				yaw;
	double				end_path_s;
	double				end_path_d;
	vector<double>      previous_path_x;
	vector<double>      previous_path_y;
	int                 prev_size;
};




#endif // TYPES_H