#ifndef REACTION_WHEEL_HPP
#define REACTION_WHEEL_HPP

#include <cmath>

class Reaction_Wheel {

private:

    double max_torque;           // N*m
    double max_angular_momentum; // N*m*s

public:

    double saturation;           // % (current angular momentum saturation of the reaction wheel)
    double time_to_max_momentum; // s (time required to reach max angular momentum at max torque)
    double max_sat_theta_acc;    // rad (max angle the satellite will rotate from the reaction wheel accelerating to max angular momentum at max torque)
    double max_sat_omega;        // rad/s (max angular velocity the satellite will rotate at from the reaction wheel accelerating to max angular momentum at max torque)
    double max_sat_alpha;        // rad/s^2 (angular acceleration the satellite will experience during the reaction wheel accelerating to max angular momentum at max torque)

    Reaction_Wheel(double sat_inertia); // constructor

    void compute_maneuver(double angle, double &t_accel, double &t_coast, double &t_decel, double &alpha); // compute the time required to complete a maneuver and acceleration during accel/decel phases

};

#endif