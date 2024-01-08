#include "Reaction_Wheel.hpp"

/* Assumptions: 
    - motor will always apply either max torque (positively or negatively) or zero torque
    - any changes in motor torque happen instantly
    - acceleration/deceleration torques are equal
    - reaction wheel has no efficiency losses
    - electric motor battery has sufficient capacity for any maneuver and is continuously replenished by solar array
    - any gyroscopic effects from the reaction wheel rotation are ignored
    - reaction torque on satellite from reaction wheel occurs at satellite centroid
*/

// constructor
Reaction_Wheel::Reaction_Wheel(double sat_inertia) {

    // initialize reaction wheel parameters
    max_torque           = 0.012;
    max_angular_momentum = 0.03;
    saturation           = 0.0;

    // compute how long it takes to saturate the reaction wheel at max torque (see README png for derivation)
    time_to_max_momentum = max_angular_momentum / max_torque;

    // compute the maximum angular acceleration of the satellite while accelerating (see README png for derivation)
    max_sat_alpha = max_torque / sat_inertia;

    // compute the maximum angular velocity of the satellite from the reaction wheel accelerating to max angular momentum at constant max torque (see README png for derivation)
    max_sat_omega = max_sat_alpha * time_to_max_momentum;

    // compute how much rotation the satellite will experience from the reaction wheel accelerating to max angular momentum at constant max torque (see README png for derivation)
    max_sat_theta_acc = 0.5 * max_sat_alpha * time_to_max_momentum * time_to_max_momentum;
}

// compute the time required to complete a maneuver and acceleration during accel/decel phases
void Reaction_Wheel::compute_maneuver(double angle, double &t_accel, double &t_coast, double &t_decel, double &alpha) {
    
    // handle condition when the angle is zero (no maneuver required)
    if (angle == 0.0) {

        // no maneuver required
        t_accel = 0.0;
        t_coast = 0.0;
        t_decel = 0.0;
        return;
    }

    // take the absolute value of the angle
    double angle_abs = std::abs(angle);

    // compute the signed angular acceleration of the satellite during the maneuver
    alpha = max_sat_alpha * (angle / angle_abs);
    
    // if there is any coasting time between the acceleration and deceleration phases of the maneuver given the commanded angle change
    if (angle_abs > (max_sat_theta_acc * 2)) {

        // full acceleration and deceleration phases are required
        t_accel = time_to_max_momentum;
        t_decel = time_to_max_momentum;

        // compute the coasting time required between accel and decel phases based on the remaining angle to be rotated and the max angular velocity
        t_coast = (angle_abs - max_sat_theta_acc * 2) / max_sat_omega;
    }
    else {

        // partial, but still equal, acceleration and deceleration phases are required
        t_accel =  sqrt(angle_abs / max_sat_alpha);
        t_decel = t_accel;
        t_coast = 0.0;
    }
}