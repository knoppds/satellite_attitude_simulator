
#include "Ideal_Cube_Sat.hpp"

int main() {

    // initialize the satellite object
    Ideal_Cube_Sat sat;

    // Write initial data to the console
    sat.print_info("Welcome to the Ideal Cube Satellite Simulator!");

    // main update loop
    while (true) {
        
        // get user input for new target point
        sat.get_new_target();

        // execute attitude maneuvers
        sat.reorient();

        // write current data to the console (removes any residual messages from maneuvers)
        sat.print_info("");

    }
    
    return 0;
}