#include "Ideal_Cube_Sat.hpp"

/* Assumptions:
    - The satellite is a perfect cube with uniform mass distribution (equation for a cube is used to compute inertia)
    - The satellite optically zooms in and out along the local z-axis to focus at some distance from the satellite
    - The satellite optically zooms at a constant rate, starts and stops instantly, and has no range limit
*/

// define constructor
Ideal_Cube_Sat::Ideal_Cube_Sat(): // member variables that are external classes which take arguments must be initialized in the initializer list otherwise they will be initialized with default constructor first anyway (just how C++ works)

    // initialize satellite properties
    mass(16.0), 
    size(0.2),
    inertia(mass * (size * size) / 6.0),

    // initialize the reaction wheels (same specification for all 3 axes)
    reaction_wheel_roll( inertia), 
    reaction_wheel_pitch(inertia), 
    reaction_wheel_yaw(  inertia), 

    // initialize the current point at default starting location (global cartesian coords)
    curr_point(0.0, 0.0, 1.0), 

    // initialize the target point at default starting location (global cartesian coords)
    targ_point(0.0, 0.0, 1.0),

    // initialize the console manager object
    console_man()

{ // now can initialize member variables that are not external classes with arguments

    // initialize the max frames per second of console refresh
    FPS = 60.0;

    // initialize satellite current rotational motion state
    omega_roll  = 0.0;
    omega_pitch = 0.0;
    omega_yaw   = 0.0;

    // initialize satellite zoom rate
    zoom_rate = 1.5;

    // initialize the default rotation matrix (starts out as identity matrix - the same reference frame as global coordinate system)
    rot_mat[0][0] = 1.0; rot_mat[1][0] = 0.0; rot_mat[2][0] = 0.0;
    rot_mat[0][1] = 0.0; rot_mat[1][1] = 1.0; rot_mat[2][1] = 0.0;
    rot_mat[0][2] = 0.0; rot_mat[1][2] = 0.0; rot_mat[2][2] = 1.0;

    // initialize the default transposed rotation matrix (starts out as identity matrix - the same reference frame as global coordinate system)
    rot_mat_T[0][0] = 1.0; rot_mat_T[1][0] = 0.0; rot_mat_T[2][0] = 0.0;
    rot_mat_T[0][1] = 0.0; rot_mat_T[1][1] = 1.0; rot_mat_T[2][1] = 0.0;
    rot_mat_T[0][2] = 0.0; rot_mat_T[1][2] = 0.0; rot_mat_T[2][2] = 1.0;
}

 // prints current satellite info to the console
void Ideal_Cube_Sat::print_info(std::string message) {

    // update the console output
    console_man.update(targ_point.global_x           , targ_point.global_y            , targ_point.global_z          , 
                       curr_point.global_x           , curr_point.global_y            , curr_point.global_z          , 
                       targ_point.local_x            , targ_point.local_y             , targ_point.local_z           , 
                       curr_point.local_x            , curr_point.local_y             , curr_point.local_z           , 
                       targ_point.local_r            , targ_point.local_theta         , targ_point.local_phi         ,
                       curr_point.local_r            , curr_point.local_theta         , curr_point.local_phi         ,
                       rot_mat                       , rot_mat_T                      ,
                       omega_roll                    , omega_pitch                    , omega_yaw                    , 
                       reaction_wheel_roll.saturation, reaction_wheel_pitch.saturation, reaction_wheel_yaw.saturation, 
                       curr_point.local_r            , message);

}

// get user input for new target point
void Ideal_Cube_Sat::get_new_target() {

    // declare new user input values
    double new_x, new_y, new_z;

    // use console manager to get new target point from user
    console_man.get_new_target(new_x, new_y, new_z);
    
    // redifine target point based on the new user coordinates
    targ_point = Location(new_x, new_y, new_z);
}

// perform sequence of attitude maneuvers to reorient sattelite to target point
void Ideal_Cube_Sat::reorient() {

    // convert the target point's global coords to local coords by applying the satellite's current rotation matrix
    targ_point.compute_local_coords(rot_mat);
    
    double roll_angle = 0.0;   // roll angle to be applied to the satellite
    double phi_offset = 0.0;   // offset to the roll angle for spherical coordinate math when using a more efficient maneuver sequence (i.e. not rolling full phi angle and using something else other than a positive yaw maneuver next)
    std::string next_maneuver; // string label of next rotation maneuver needed after roll maneuver
    int next_sign;             // sign convention of next rotation maneuver

    // compute the most efficient roll angle and the subsequent rotation maneuver
    compute_efficient_roll(targ_point.local_phi, roll_angle, phi_offset, next_maneuver, next_sign);

    // compute the time required to complete the roll maneuver and the angular acceleration during accel/decel phases
    double t_accel, t_coast, t_decel, alpha;
    reaction_wheel_roll.compute_maneuver(roll_angle, t_accel, t_coast, t_decel, alpha);

    // execute the roll maneuver
    execute_maneuver("Roll", 1, "phi", roll_angle, phi_offset, omega_roll, alpha, t_accel, t_coast, t_decel);

    // check if the next maneuver is Pitch or Yaw
    if (next_maneuver == "Pitch") {

        // compute maneuver parameters and exectue
        reaction_wheel_pitch.compute_maneuver(targ_point.local_theta, t_accel, t_coast, t_decel, alpha);
        execute_maneuver(next_maneuver, next_sign, "theta", targ_point.local_theta, 0, omega_pitch, alpha, t_accel, t_coast, t_decel);
    }

    else if (next_maneuver == "Yaw") {

        // compute maneuver parameters and exectue
        reaction_wheel_yaw.compute_maneuver(targ_point.local_theta, t_accel, t_coast, t_decel, alpha);
        execute_maneuver(next_maneuver, next_sign, "theta", targ_point.local_theta, 0, omega_yaw, alpha, t_accel, t_coast, t_decel);
    }

    else {

        // something went wrong, exit program
        std::cout << "ERROR: Invalid next maneuver in Ideal_Cube_Sat::reorient()" << std::endl;
        exit(1);
    }

    // adjust the satellite's zoom level
    adjust_zoom();

    // initialize some temp rotation matrices
    double maneuver_rot_mat[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0 ,0.0, 0.0}};
    double rot_mat_temp[3][3]     = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0 ,0.0, 0.0}};
    double rot_mat_T_temp[3][3]   = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0 ,0.0, 0.0}};

    // compute the rotation matrix of the first maneuver
    compute_rotation_matrix(maneuver_rot_mat, roll_angle, "Roll");

    // update the satellite's current rotation matrix
    multiply_rot_mats(maneuver_rot_mat, rot_mat, rot_mat_temp);
    copy_rot_mat(rot_mat_temp, rot_mat);

    // compute the rotation matrix of the second maneuver
    compute_rotation_matrix(maneuver_rot_mat, next_sign * targ_point.local_theta, next_maneuver);

    // update the satellite's current rotation matrix
    multiply_rot_mats(maneuver_rot_mat, rot_mat, rot_mat_temp);
    copy_rot_mat(rot_mat_temp, rot_mat);

    // update the satellite's current rotation matrix transpose
    transpose_rot_mat(rot_mat, rot_mat_T_temp);
    copy_rot_mat(rot_mat_T_temp, rot_mat_T);

    // update the satellite's current point and target point after completing maneuvers
    curr_point.rotate_local_coords();
    targ_point.rotate_local_coords();
}

// execute a single rotation maneuver
void Ideal_Cube_Sat::execute_maneuver(std::string maneuver, int sign, std::string coord, double angle, double offset, double &omega, double alpha, double t_accel, double t_coast, double t_decel) {
    
    /* Note: sign is only used for non-roll maneuvers. 
       In polar coordinates, theta angle is always positive, but depending on orientation of satellite, may need a negative Pitch or Yaw maneuver to achieve the change in theta angle. 
       Sign is used in this member function only for display purposes in the console output (actual satellite rotation is tracked via rotation matrix member variables).
    */

    // initialize loop variables
    auto   t_start    = std::chrono::high_resolution_clock::now(); // get current time
    double t_elapsed  = 0.0;                                       // initialize elapsed time
    double t_in_phase = 0.0;                                       // initialize time in current phase of maneuver
    bool   startup    = true;                                      // initialize startup flag
    std::string message;                                           // initialize custom message to be displayed in console

    // simulate the realtime motion of the satellite during the maneuver (add 1s to display initial starting message and 1s to display final message after maneuver is complete)
    while (t_elapsed < (t_accel + t_coast + t_decel + 2.0)) {
        
        // get current time and compute elapsed time
        auto t_now = std::chrono::high_resolution_clock::now();
        t_elapsed  = std::chrono::duration<double>(t_now - t_start).count();
        
        // simulate maneuver
        if (t_elapsed < 1.0) { // startup message

            message = "Executing " + maneuver + " Maneuver:";

        } else if (t_elapsed < (1 + t_accel)) { startup = false; // satellite accelerating

            message = "Executing " + maneuver + " Maneuver: Accelerating...";

            // update the time in current phase
            t_in_phase = t_elapsed - 1.0;
            
            // update satellite angular velocity
            omega = alpha * t_in_phase;

            // update satellite current point phi angle (in local spherical coords)
            curr_point.update_local_spherical(coord, offset + alpha * t_in_phase * t_in_phase / 2.0);

        } else if (t_elapsed < (1 + t_accel + t_coast)) { startup = false; // satellite coasting

            message = "Executing " + maneuver + " Maneuver: Coasting...";

            // update the time in current phase
            t_in_phase = t_elapsed - t_accel - 1.0;
            
            // update satellite angular velocity
            omega = alpha * t_accel;

            // update satellite current point phi angle (in local spherical coords)
            curr_point.update_local_spherical(coord, offset + alpha * t_accel * t_accel / 2.0 + omega * t_in_phase);

        } else if (t_elapsed < (1 + t_accel + t_coast + t_decel)) { startup = false; // satellite decelerating

            message = "Executing " + maneuver + " Maneuver: Decelerating...";

            // update the time in current phase
            t_in_phase = t_elapsed - t_accel - t_coast - 1.0;
            
            // update satellite angular velocity
            omega = alpha * t_accel - alpha * t_in_phase;

            // update satellite current point phi angle (in local spherical coords)
            curr_point.update_local_spherical(coord, offset + (alpha * t_accel * t_accel / 2.0) + (alpha * t_accel * t_coast) + (alpha * t_accel * t_in_phase - alpha * t_in_phase * t_in_phase / 2.0));

        } else { startup = false; // maneuver complete (still computing everything though as a means of error checking, even though know the correct final values already)

            message = "Executing " + maneuver + " Maneuver: Complete";
            
            // update satellite angular velocity
            omega = alpha * t_accel - alpha * t_decel;

            // update satellite current point phi angle (in local spherical coords)
            curr_point.update_local_spherical(coord, offset + (alpha * t_accel * t_accel / 2.0) + (alpha * t_accel * t_coast) + (alpha * t_accel * t_decel - alpha * t_decel * t_decel / 2.0));
        }

        // only update satellite information if out of startup phase
        if (startup == false) {

            // compute the new global coords after the change in local coords from above
            curr_point.compute_global_coords(rot_mat_T);

            // if maneuver is not Roll, apply the sign convention before updating the console output
            if (maneuver != "Roll") {
                omega *= sign;
            }

            // update reaction wheel momentum saturation percentage (follows sign convention of maneuver, can be between -100% and 100%)
            if      (maneuver == "Roll" ) {reaction_wheel_roll.saturation  = 100.0 * omega / reaction_wheel_roll.max_sat_omega; }
            else if (maneuver == "Pitch") {reaction_wheel_pitch.saturation = 100.0 * omega / reaction_wheel_pitch.max_sat_omega;}
            else if (maneuver == "Yaw"  ) {reaction_wheel_yaw.saturation   = 100.0 * omega / reaction_wheel_yaw.max_sat_omega;  }
        }

        // update the console output
        print_info(message);

        // Calculate the sleep duration in milliseconds based on FPS
        auto sleep_duration = std::chrono::milliseconds(static_cast<long long>(1000.0 / FPS));

        // Sleep for the calculated duration
        std::this_thread::sleep_for(sleep_duration);
    }
}

// adjust the zoom level of the satellite
void Ideal_Cube_Sat::adjust_zoom() {

    // compute how much the zoom level needs to change
    double r_zoom = targ_point.local_r - curr_point.local_r;

    // compute how long it will take to zoom to the new target point
    double t_zoom = std::abs(r_zoom) / zoom_rate;

    // initialize loop variables
    auto   t_start    = std::chrono::high_resolution_clock::now(); // get current time
    double t_elapsed  = 0.0;                                       // initialize elapsed time
    double r_start    = curr_point.local_r;                        // initialize starting rho distance
    std::string message;                                           // initialize message to be displayed in console

    // simulate the realtime motion of the satellite during the maneuver (add 1s to display final message after maneuver is complete)
    while (t_elapsed < (t_zoom + 1.0)) {
        
        // get current time and compute elapsed time
        auto t_now = std::chrono::high_resolution_clock::now();
        t_elapsed  = std::chrono::duration<double>(t_now - t_start).count();

        // simulate zoom
        if (t_elapsed < t_zoom) { // satellite zooming

            message = "Adjusting Optical Zoom...";

            // update satellite current point r distance (in local spherical coords)
            curr_point.update_local_spherical("r", r_start + r_zoom * t_elapsed / t_zoom);

        } else { // zoom complete

            message = "Optical Zoom Complete";

            // update satellite current point r distance (in local spherical coords)
            curr_point.update_local_spherical("r", targ_point.local_r);
        }

        // compute the new global coords after the change in local coords
        curr_point.compute_global_coords(rot_mat_T);

        // update the console output
        print_info(message);

        // Calculate the sleep duration in milliseconds based on FPS
        auto sleep_duration = std::chrono::milliseconds(static_cast<long long>(1000.0 / FPS));

        // Sleep for the calculated duration
        std::this_thread::sleep_for(sleep_duration);

    }

}