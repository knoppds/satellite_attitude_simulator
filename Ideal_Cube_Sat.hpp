#ifndef IDEAL_CUBE_SAT_HPP
#define IDEAL_CUBE_SAT_HPP

#include <string>
#include <chrono>
#include <thread>
#include "Console_Manager.hpp"
#include "Reaction_Wheel.hpp"
#include "Location.hpp"
#include "Helper_Functions.hpp"

class Ideal_Cube_Sat {

private:

    double FPS;             // max frames per second of console refresh
    double mass;            // kg
    double size;            // m
    double inertia;         // kg*m^2
    double omega_roll;      // rad/s
    double omega_pitch;     // rad/s
    double omega_yaw;       // rad/s
    double zoom_rate;       // coordinate units/s
    double rot_mat[3][3];   // rotation matrix to go from global to local cartesian coordinate system
    double rot_mat_T[3][3]; // transpose of rotation matrix to go from local to global cartesian coordinate system

public:

    Console_Manager console_man; // console manager object

    Reaction_Wheel reaction_wheel_roll;  // roll  control reaction wheel object (identical for all axis) (defined in this program as rotation about +z axis, see Helper_Functions.cpp for rationalle)
    Reaction_Wheel reaction_wheel_pitch; // pitch control reaction wheel object (identical for all axis) (defined in this program as rotation about +x axis, see Helper_Functions.cpp for rationalle)
    Reaction_Wheel reaction_wheel_yaw;   // yaw   control reaction wheel object (identical for all axis) (defined in this program as rotation about +y axis, see Helper_Functions.cpp for rationalle)

    Location curr_point; // current focused position of the satellite

    Location targ_point; // target focused position of the satellite

    void print_info(std::string message); // prints current satellite info to the console

    void get_new_target(); // get user input for new target point

    void reorient(); // perform sequence of attitude maneuvers to reorient sattelite to target point

    void execute_maneuver(std::string maneuver, int sign, std::string coord, double angle, double offset, double &omega, double alpha, double t_accel, double t_coast, double t_decel); // execute a single maneuver

    void adjust_zoom(); // adjust the zoom level of the satellite

    Ideal_Cube_Sat(); // constructor

};

#endif