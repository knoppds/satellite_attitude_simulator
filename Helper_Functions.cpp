#include "Helper_Functions.hpp"

// compute the smallest roll angle required to align target point for a single subsequent pitch or yaw maneuver
void compute_efficient_roll(double local_phi, double &roll_angle, double &phi_offset, std::string &next_maneuver, int &next_sign) {

    /* We need to determine the minimum amount of roll required to aligh the satellite such that a single
    pitch or yaw maneuver can be executed afterwards to finish aligning the satellite with the target point.
    Typically roll is about x axis, pitch is about y axis, and yaw is about z axis. However, I am using a
    different convention (shown below) for simplicity of operations since I am directly considering spherical
    coord phi as roll angle and spherical coord theta as pitch or yaw.

    Visualization of satellite local spherical coord system and the corresponding satellite roll, pitch, and yaw axes:
    
             z (satellite roll, because camera points this way) - spherical coords theta angle is measured from this axis
             |
             |
             |
             |
             |_____________ y (satellite yaw)
            /
           /
          /
         /
        x (satellite pitch) - spherical coords phi angle is measured from this axis
    */

    // define the roll angle and the next rotation to be applied to the satellite based on the local phi angle (minimizing magnitude of the roll angle)
    if      (local_phi <=       M_PI / 4.0) {roll_angle = local_phi                   ; next_maneuver = "Yaw"  ; next_sign =  1;} // the positive x axis, positive phi side
    else if (local_phi <= 3.0 * M_PI / 4.0) {roll_angle = local_phi -       M_PI / 2.0; next_maneuver = "Pitch"; next_sign = -1;} // the positive y axis
    else if (local_phi <= 5.0 * M_PI / 4.0) {roll_angle = local_phi -       M_PI      ; next_maneuver = "Yaw"  ; next_sign = -1;} // the negative x axis
    else if (local_phi <= 7.0 * M_PI / 4.0) {roll_angle = local_phi - 3.0 * M_PI / 2.0; next_maneuver = "Pitch"; next_sign =  1;} // the negative y axis
    else if (local_phi >  7.0 * M_PI / 4.0) {roll_angle = local_phi - 2.0 * M_PI      ; next_maneuver = "Yaw"  ; next_sign =  1;} // the positive x axis, negative phi side
    else {
        
        // something went wrong, exit program
        std::cout << "ERROR: Invalid local_phi in Helper_Functions::compute_efficient_roll()" << std::endl; 
        exit(1);
    }

    // compute the phi offset so that the spherical coord math works out when using a more efficient maneuver instead of positive-yaw-only (i.e. instead of sweeping out spherical coordinate angles explicitly)
    phi_offset = local_phi - roll_angle;

}

// compute the rotation matrix for a given angle and rotation maneuver
void compute_rotation_matrix(double rot_mat[3][3], double angle, const std::string axis) {

    // if the angle is negative, add 2pi to make it positive
    if (angle < 0.0) {angle += 2.0 * M_PI;}

    // define the rotation matrix based on the angle and rotation maneuver
    if      (axis == "Pitch") {rot_mat[0][0] =         1.0; rot_mat[1][0] =         0.0; rot_mat[2][0] =         0.0;
                               rot_mat[0][1] =         0.0; rot_mat[1][1] =  cos(angle); rot_mat[2][1] = -sin(angle);
                               rot_mat[0][2] =         0.0; rot_mat[1][2] =  sin(angle); rot_mat[2][2] =  cos(angle);}
    else if (axis == "Yaw"  ) {rot_mat[0][0] =  cos(angle); rot_mat[1][0] =         0.0; rot_mat[2][0] =  sin(angle);
                               rot_mat[0][1] =         0.0; rot_mat[1][1] =         1.0; rot_mat[2][1] =         0.0;
                               rot_mat[0][2] = -sin(angle); rot_mat[1][2] =         0.0; rot_mat[2][2] =  cos(angle);}
    else if (axis == "Roll" ) {rot_mat[0][0] =  cos(angle); rot_mat[1][0] = -sin(angle); rot_mat[2][0] =         0.0;
                               rot_mat[0][1] =  sin(angle); rot_mat[1][1] =  cos(angle); rot_mat[2][1] =         0.0;
                               rot_mat[0][2] =         0.0; rot_mat[1][2] =         0.0; rot_mat[2][2] =         1.0;}
}

// multiply two rotation matrices
void multiply_rot_mats(double rot_mat_1[3][3], double rot_mat_2[3][3], double rot_mat_out[3][3]) {

    // make sure rot_mat_out is initialized to zero (otherwise the += operation below will not work properly)
    for (int i = 0; i < 3; i++) {     // i represents row
        for (int j = 0; j < 3; j++) { // j represents col
            rot_mat_out[i][j] = 0.0;
        }
    }

    // multiply rot_mat_1 with rot_mat_2 and store in rot_mat_out
    for (int i = 0; i < 3; i++) {                                        // i represents row of mat1 and row of out_mat
        for (int j = 0; j < 3; j++) {                                    // j represents col of mat2 and col of out_mat
            for (int k = 0; k < 3; k++) {                                // k represents col of mat1 and row of mat2
                rot_mat_out[i][j] += rot_mat_1[i][k] * rot_mat_2[k][j];  // for element i, j of out_mat, multiple entire row i (all k's) of mat1 with entire col j (all k's) of mat2 element-wise and sum
            }
        }
    }
}

// copy a rotation matrix
void copy_rot_mat(double rot_mat_in[3][3], double rot_mat_out[3][3]) {

    // copy rot_mat_in to rot_mat_out
    for (int i = 0; i < 3; i++) {     // i represents row
        for (int j = 0; j < 3; j++) { // j represents col
            rot_mat_out[i][j] = rot_mat_in[i][j];
        }
    }
}

// transpose a rotation matrix
void transpose_rot_mat(double rot_mat_in[3][3], double rot_mat_out[3][3]) {

    // transpose rot_mat_in and store in rot_mat_out
    for (int i = 0; i < 3; i++) {     // i represents row
        for (int j = 0; j < 3; j++) { // j represents col
            rot_mat_out[i][j] = rot_mat_in[j][i];
        }
    }
}

// apply a rotation matrix to a set of coordinates
void apply_rotation(double rot_mat[3][3], double &x, double &y, double &z) {

    // create some temp variables to store the original values of x, y, and z
    double x_temp = x;
    double y_temp = y;
    double z_temp = z;

    // apply rot_mat to x, y, and z
    x = rot_mat[0][0] * x_temp + rot_mat[0][1] * y_temp + rot_mat[0][2] * z_temp;
    y = rot_mat[1][0] * x_temp + rot_mat[1][1] * y_temp + rot_mat[1][2] * z_temp;
    z = rot_mat[2][0] * x_temp + rot_mat[2][1] * y_temp + rot_mat[2][2] * z_temp;
}

// determine which planet is in the satellite's current focused octant
void determine_focused_planet(double x, double y, double z, std::string &planet) {

    // Round the coordinates to the nearest 2nd decimal place (avoids misidentification due to floating-point error)
    double rounded_x = std::round(x * 100) / 100; x = rounded_x;
    double rounded_y = std::round(y * 100) / 100; y = rounded_y;
    double rounded_z = std::round(z * 100) / 100; z = rounded_z;

    // determine which planet is in the satellite's current focused octant
    if      (x > 0.0 && y > 0.0 && z > 0.0) {planet = "GRACE (+x, +y, +z)";}
    else if (x > 0.0 && y < 0.0 && z > 0.0) {planet =  "BRAY (+x, -y, +z)";}
    else if (x > 0.0 && y > 0.0 && z < 0.0) {planet = "PRICE (+x, +y, -z)";}
    else if (x > 0.0 && y < 0.0 && z < 0.0) {planet =   "MIG (+x, -y, -z)";}
    else if (x < 0.0 && y > 0.0 && z > 0.0) {planet =  "WIEM (-x, +y, +z)";}
    else if (x < 0.0 && y < 0.0 && z > 0.0) {planet =  "TURK (-x, -y, +z)";}
    else if (x < 0.0 && y > 0.0 && z < 0.0) {planet =  "MROW (-x, +y, -z)";}
    else if (x < 0.0 && y < 0.0 && z < 0.0) {planet = "SEBAS (-x, -y, -z)";}
    else {planet = "N/A (on octant boundary)";}
}