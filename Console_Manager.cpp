#include "Console_Manager.hpp"

// constructor
Console_Manager::Console_Manager() {

    // get handle to the console window
    consoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
}

// get new target point from user
void Console_Manager::get_new_target(double &new_x, double &new_y, double &new_z)
{
    // loop until valid input is received
    while (true){

        // prompt user and wait to parse cin for new values
        std::cout << "Enter the new target coordinates (x y z): ";
        std::cin >> new_x >> new_y >> new_z;

        if (std::cin.fail()) { // if format of cin is wrong, ask the user to try again

            // display error message
            std::cerr << "Invalid input. Please enter 3 signed numbers seperated by spaces (x y z)." << std::endl;

            // clear the error state
            std::cin.clear();

            // discard invalid input from the input buffer
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            continue;

        } else if (new_x == 0.0 && new_y == 0.0 && new_z == 0.0) { // if user specified (0, 0, 0), ask the user to try again

            // display error message
            std::cerr << "(0 0 0) is not a supported input. Please enter 3 signed numbers seperated by spaces (x y z)." << std::endl;

            // clear the error state
            std::cin.clear();

            // discard invalid input from the input buffer
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            continue;
        }

        // clear the console buffer
        clear_buffer();

        // if get here then the input is valid, break out of the loop
        break;
    }
}

// print the current state of the satellite
void Console_Manager::update(double x_targ            , double y_targ             , double z_targ           , 
                             double x_curr            , double y_curr             , double z_curr           , 
                             double x_l_targ          , double y_l_targ           , double z_l_targ         , 
                             double x_l_curr          , double y_l_curr           , double z_l_curr         , 
                             double r_targ            , double theta_targ         , double phi_targ         , 
                             double r_curr            , double theta_curr         , double phi_curr         , 
                             double rot_mat[3][3]     , double rot_mat_T[3][3]    ,
                             double omega_roll        , double omega_pitch        , double omega_yaw        , 
                             double rw_saturation_roll, double rw_saturation_pitch, double rw_saturation_yaw, 
                             double zoom_dist         , std::string message)
{

    // set the cursor position to the top left corner of the console window
    SetConsoleCursorPosition(consoleHandle, {0, 0});

    // get the currently focused planet string using helper function
    std::string planet;
    determine_focused_planet(x_curr, y_curr, z_curr, planet);

    // Convert angular velocity into deg/s for display purposes
    omega_roll  *= 180 / M_PI;
    omega_pitch *= 180 / M_PI;
    omega_yaw   *= 180 / M_PI;

    // Convert spherical coordinates into deg for display purposes
    theta_targ *= 180 / M_PI;
    theta_curr *= 180 / M_PI;
    phi_targ   *= 180 / M_PI;
    phi_curr   *= 180 / M_PI;

    // define the precision for all floating-point output
    int precision = 2;

    // Set the precision for all floating-point output
    std::cout << std::fixed << std::setprecision(precision);

    // set the width for each value field output (adds whitespace padding)
    const int width = 5 + precision;

    // print all information to the console (commented out lines are for debugging purposes)
    std::cout << "Target    Focus Point [-]:        X:" << std::setw(width) << value_rounder(x_targ            , precision) << "     Y:" << std::setw(width) << value_rounder(y_targ             , precision) << "   Z:" << std::setw(width) << value_rounder(z_targ           , precision) << std::endl;
    std::cout << "Sattelite Focus Point [-]:        X:" << std::setw(width) << value_rounder(x_curr            , precision) << "     Y:" << std::setw(width) << value_rounder(y_curr             , precision) << "   Z:" << std::setw(width) << value_rounder(z_curr           , precision) << std::endl; std::cout << std::endl;
    // std::cout << "Target    Local Focus Point [-]:  X:" << std::setw(width) << value_rounder(x_l_targ          , precision) << "     Y:" << std::setw(width) << value_rounder(y_l_targ           , precision) << "   Z:" << std::setw(width) << value_rounder(z_l_targ         , precision) << std::endl;
    // std::cout << "Sattelite Local Focus Point [-]:  X:" << std::setw(width) << value_rounder(x_l_curr          , precision) << "     Y:" << std::setw(width) << value_rounder(y_l_curr           , precision) << "   Z:" << std::setw(width) << value_rounder(z_l_curr         , precision) << std::endl; std::cout << std::endl;
    // std::cout << "Target    Spherical Coords [-]:   R:" << std::setw(width) << value_rounder(r_targ            , precision) << " Theta:" << std::setw(width) << value_rounder(theta_targ         , precision) << " Phi:" << std::setw(width) << value_rounder(phi_targ         , precision) << std::endl;
    // std::cout << "Sattelite Spherical Coords [-]:   R:" << std::setw(width) << value_rounder(r_curr            , precision) << " Theta:" << std::setw(width) << value_rounder(theta_curr         , precision) << " Phi:" << std::setw(width) << value_rounder(phi_curr         , precision) << std::endl; std::cout << std::endl;
    // std::cout << "Sattelite Rotation Matrix [-]:      " << std::setw(width) << value_rounder(rot_mat[0][0]     , precision) << "       " << std::setw(width) << value_rounder(rot_mat[0][1]      , precision) << "     " << std::setw(width) << value_rounder(rot_mat[0][2]    , precision) << std::endl;
    // std::cout << "                                    " << std::setw(width) << value_rounder(rot_mat[1][0]     , precision) << "       " << std::setw(width) << value_rounder(rot_mat[1][1]      , precision) << "     " << std::setw(width) << value_rounder(rot_mat[1][2]    , precision) << std::endl;
    // std::cout << "                                    " << std::setw(width) << value_rounder(rot_mat[2][0]     , precision) << "       " << std::setw(width) << value_rounder(rot_mat[2][1]      , precision) << "     " << std::setw(width) << value_rounder(rot_mat[2][2]    , precision) << std::endl; std::cout << std::endl;
    // std::cout << "Sattelite Rotation Matrix T [-]:    " << std::setw(width) << value_rounder(rot_mat_T[0][0]   , precision) << "       " << std::setw(width) << value_rounder(rot_mat_T[0][1]    , precision) << "     " << std::setw(width) << value_rounder(rot_mat_T[0][2]  , precision) << std::endl;
    // std::cout << "                                    " << std::setw(width) << value_rounder(rot_mat_T[1][0]   , precision) << "       " << std::setw(width) << value_rounder(rot_mat_T[1][1]    , precision) << "     " << std::setw(width) << value_rounder(rot_mat_T[1][2]  , precision) << std::endl;
    // std::cout << "                                    " << std::setw(width) << value_rounder(rot_mat_T[2][0]   , precision) << "       " << std::setw(width) << value_rounder(rot_mat_T[2][1]    , precision) << "     " << std::setw(width) << value_rounder(rot_mat_T[2][2]  , precision) << std::endl; std::cout << std::endl;
    std::cout << "Sat. Angular Velocity [deg/s]: Roll:" << std::setw(width) << value_rounder(omega_roll        , precision) << " Pitch:" << std::setw(width) << value_rounder(omega_pitch        , precision) << " Yaw:" << std::setw(width) << value_rounder(omega_yaw        , precision) << std::endl;
    std::cout << "Reaction Wheel Saturation [%]: Roll:" << std::setw(width) << value_rounder(rw_saturation_roll, precision) << " Pitch:" << std::setw(width) << value_rounder(rw_saturation_pitch, precision) << " Yaw:" << std::setw(width) << value_rounder(rw_saturation_yaw, precision) << std::endl; std::cout << std::endl;
    std::cout << "Optical Zoom Distance [-]: " << zoom_dist << std::endl;
    std::cout << std::endl;
    std::cout << "Planet in Current Focused Octant: " << planet << "                                                  " << std::endl; // added whitespace padding to overwrite any residual text
    std::cout << std::endl;
    if (message != "") {
        std::cout << message << "                                                  " << std::endl; // added whitespace padding to overwrite any residual text
        std::cout << std::endl;
    }
}

// clear the console buffer (fill with spaces)
void Console_Manager::clear_buffer() {

    // get the console screen buffer info
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    COORD topLeft = {0, 0};
    GetConsoleScreenBufferInfo(consoleHandle, &csbi);

    // fill the entire buffer with spaces
    DWORD length = csbi.dwSize.X * csbi.dwSize.Y;
    DWORD written;
    FillConsoleOutputCharacter(consoleHandle, ' ', length, topLeft, &written);
    FillConsoleOutputAttribute(consoleHandle, csbi.wAttributes, length, topLeft, &written);

    // set the cursor position to the top left corner of the console window
    SetConsoleCursorPosition(consoleHandle, topLeft);
}

// rounds values to a specified precision, ensures no negative zeros are printed
double Console_Manager::value_rounder(double value, int precision) {

    // round the value to the specified precision
    value = round(value * pow(10, precision)) / pow(10, precision);

    // if the value is zero, redefine to ensure it is positive zero
    if (value == 0.0) {
        value = 0.0;
    }

    return value;
}