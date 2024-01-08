#include "Location.hpp"

// constructor (global cartesian coords)
Location::Location(double x, double y, double z) {

    // store input global cartesian coordinates
    global_x = x;
    global_y = y;
    global_z = z;

    // initialize with default local coordinates
    local_x     = x;
    local_y     = y;
    local_z     = z;
    convert_to_spherical(local_x, local_y, local_z, local_r, local_theta, local_phi);
}

// compute local coordinates from global coordinates
void Location::compute_local_coords(double rot_mat[3][3]) {

    // overwrite local cartesian coordinates with global coordinate values
    local_x = global_x;
    local_y = global_y;
    local_z = global_z;
    
    // apply rotation matrix to local coordinates
    apply_rotation(rot_mat, local_x, local_y, local_z);

    // update local spherical coordinates
    convert_to_spherical(local_x, local_y, local_z, local_r, local_theta, local_phi);
}

// rotate local coordinates after completing maneuvers (for console display purposes only, has no functional significance otherwise)
void Location::rotate_local_coords() {

    // once rotations are complete, points are always aligned with positive z axis (updating values for display purposes on console, rotations already exist in the updated rotation matrix)
    local_x = 0.0;
    local_y = 0.0;
    local_z = local_r;

    // update local spherical coordinates
    convert_to_spherical(local_x, local_y, local_z, local_r, local_theta, local_phi);
}

// compute global coordinates from local coordinates
void Location::compute_global_coords(double rot_mat_T[3][3]) {
    
    // overwrite global cartesian coordinates with local coordinates
    global_x = local_x;
    global_y = local_y;
    global_z = local_z;
    
    // apply transpose of rotation matrix to global coordinates
    apply_rotation(rot_mat_T, global_x, global_y, global_z);
}

// update the value of a local spherical coordinate
void Location::update_local_spherical(std::string coord, double value) {
    
    // update the value specified by coord argument
    if      (coord == "r"    ) {local_r     = value;}
    else if (coord == "theta") {local_theta = value;}
    else if (coord == "phi"  ) {local_phi   = value;}
    else {
        
        // something went wrong, exit program
        std::cout << "ERROR: Invalid coord in Location::update_local_spherical()" << std::endl;
        exit(1);
    }

    // update local cartesian coordinates
    local_x = local_r * sin(local_theta) * cos(local_phi);
    local_y = local_r * sin(local_theta) * sin(local_phi);
    local_z = local_r * cos(local_theta);
}

// convert cartesian coordinates to spherical coordinates
void Location::convert_to_spherical(double x, double y, double z, double &rho, double &theta, double &phi) {

    // compute spherical coordinates from cartesian coordinates
    rho   = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    theta = acos(z / rho);
    phi   = fmod(atan2(y, x) + 2 * M_PI, 2 * M_PI); // adjustment ensures phi is always in range [0, 2pi)
}