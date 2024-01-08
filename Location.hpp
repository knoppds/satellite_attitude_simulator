#ifndef LOCATION_HPP
#define LOCATION_HPP

#include <cmath>
#include <string>
#include <iostream>
#include "Helper_Functions.hpp"

class Location {

public:

    double global_x;    // global x coordinate
    double global_y;    // global y coordinate
    double global_z;    // global z coordinate

    double local_x;     // local x coordinate
    double local_y;     // local y coordinate
    double local_z;     // local z coordinate
    double local_r;     // local radius
    double local_theta; // local theta
    double local_phi;   // local phi

    Location(double x, double y, double z); // constructor (global cartesian coords)

    void compute_local_coords(double rot_mat[3][3]); // compute local coordinates from global coordinates

    void rotate_local_coords(); // rotate local coordinates after completing maneuvers (for console display purposes only, has no functional significance otherwise)

    void compute_global_coords(double rot_mat_T[3][3]); // compute global coordinates from local coordinates

    void update_local_spherical(std::string coord, double value); // update the value of a local spherical coordinate

    void convert_to_spherical(double x, double y, double z, double &rho, double &theta, double &phi); // convert cartesian coordinates to spherical coordinates

};

#endif