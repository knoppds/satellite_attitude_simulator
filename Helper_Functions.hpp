#ifndef HELPER_FUNCTIONS_HPP
#define HELPER_FUNCTIONS_HPP

#include <string>
#include <iostream>
#include <cmath>

void compute_efficient_roll(double local_phi, double &roll_angle, double &phi_offset, std::string &next_maneuver, int &next_sign); // compute the smallest roll angle required to align target point for a single subsequent pitch or yaw maneuver

void compute_rotation_matrix(double rot_mat[3][3], double angle, const std::string axis); // compute the rotation matrix for a given angle and rotation maneuver

void multiply_rot_mats(double rot_mat_1[3][3], double rot_mat_2[3][3], double rot_mat_out[3][3]); // multiply two rotation matrices

void copy_rot_mat(double rot_mat_in[3][3], double rot_mat_out[3][3]); // copy a rotation matrix

void transpose_rot_mat(double rot_mat_in[3][3], double rot_mat_out[3][3]); // transpose a rotation matrix

void apply_rotation(double rot_mat[3][3], double &x, double &y, double &z); // apply a rotation matrix to a set of coordinates

void determine_focused_planet(double x, double y, double z, std::string &planet); // determine which planet is in the satellite's current focused octant

#endif