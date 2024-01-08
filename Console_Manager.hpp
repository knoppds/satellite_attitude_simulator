#ifndef CONSOLE_MANAGER_HPP
#define CONSOLE_MANAGER_HPP

#include <iostream>
#include <string>
#include <cmath>
#include <Windows.h>
#include <iomanip>
#include "Helper_Functions.hpp"

class Console_Manager {

private:

    HANDLE consoleHandle; // handle to the console window

public: 

    Console_Manager(); // constructor declaration

    void get_new_target(double &new_x, double &new_y, double &new_z); // get new target point from user

    // print the current state of the satellite
    void update(double x_targ            , double y_targ             , double z_targ           , 
                double x_curr            , double y_curr             , double z_curr           , 
                double x_l_targ          , double y_l_targ           , double z_l_targ         , 
                double x_l_curr          , double y_l_curr           , double z_l_curr         , 
                double r_targ            , double theta_targ         , double phi_targ         , 
                double r_curr            , double theta_curr         , double phi_curr         , 
                double rot_mat[3][3]     , double rot_mat_T[3][3]    ,
                double omega_roll        , double omega_pitch        , double omega_yaw        , 
                double rw_saturation_roll, double rw_saturation_pitch, double rw_saturation_yaw, 
                double zoom_dist         , std::string message);

    void clear_buffer(); // clear the console buffer (fill with spaces)

    double value_rounder(double value, int precision); // rounds values to a specified precision, ensures no negative zeros are printed

};

#endif