/*
* This file contains function definitions for the "robot_arm" class
*
* The robot arm is a line with no width in the environment that is described by the end effector x,y coordinates
* and the theta_degrees value. 
*/
#include "robot_arm.h"

// specify the specific functions you are using rather than all of std
using std::cout;
using std::cerr;
using std::endl;
using ::atan;
using std::min;
using std::max;

namespace despot {

// constructors
robot_arm::robot_arm() {
    /* 
    * Constructor with no arguments - initializes the coords to something arbitrary 
    * This will conflict when initializing multiple arms
    * This will also cause errors if any function is called due to incorrect environment_height and environment_length
    * 
    * NOTE: IF USING EMPTY CONSTRUCTOR - SET THE PROPER VALUES - using:
    *   - init_robot_arm
    */
    coords_m.x = 0;
    coords_m.y = 10;
    coords_m.theta_degrees = 0;

    debug_m = DEFAULT_DEBUG_FLAG_g;
}

robot_arm::robot_arm(robotArmCoords coords, bool debug, bool &error) {
    /*
    * This is the complete constructor that properly initializes the robot arm attributes
    * args:
    *   - x: the x position of the robot arm end effector
    *   - y: the y position of the robot arm end effector
    *   - theta_degrees: the angle in degrees that the line that is the robot arm forms
    *   - height: height of the environment
    *   - length: length of the environment
    *   - debug: boolean that if true will allow verbose printing to the iostream for debugging
    * returns: by pass by reference
    *   - error - if there was an error while setting up the robot arm return error = true, else false
    */
    error = init_robot_arm(coords, debug);
}

robot_arm::robot_arm(const robot_arm &arm_to_copy) {
    /*
    * This is the copy constructor of the robot_arm class 
    * args:
    *   - arm_to_copy: arm object that you are replicating
    */ 
    coords_m = arm_to_copy.coords_m;
    old_coords_m = arm_to_copy.old_coords_m;
    debug_m = arm_to_copy.debug_m;
    bool error =  !is_valid_pos();
    if (error) {
        cerr << "ERROR: IN ROBOT_ARM COPY CONSTRUCTOR: invalid robot arm object was used to copy!!!!"  << endl;
        exit(1);  
    }
}

/*
****************************************************************************************
* BEGIN: GET FUNCTIONS
****************************************************************************************
*/
int robot_arm::get_x() const{
    return coords_m.x;
}

int robot_arm::get_y() const{
    return coords_m.y;
}

int robot_arm::get_theta_degrees() const{
    return coords_m.theta_degrees;
}

robotArmCoords robot_arm::get_coords() const{
    return coords_m;
}


float robot_arm::get_theta_radians() const{
    float theta_radians = THETA_MULTIPLIER * coords_m.theta_degrees;
    return theta_radians;
}

/*
* END: GET FUNCTIONS
*/

/*
* BEGIN: SET FUNCTIONS
*/
bool robot_arm::init_robot_arm(robotArmCoords coords, bool debug) {
    /*
    * Initialize the robot arm as in the full constructor. 
    * args:
    *   - x: set the coords.x
    *   - y: set the coords.y
    *   - theta_degrees: set the coords.theta_degrees
    *   - height: set the environment height
    *   - length: set the environment length 
    *   - debug: set the debugging flag
    * Returns:
    *   - error: boolean if thee was an error while initializing the robot arm 
    * NOTE: if there was an error robot arm initialized to some defaults that may not be correct !!!! - re init robot arm !
    */
    coords_m = coords;
    debug_m = debug;

    bool error =  !this->is_valid_pos();
    if (error) {
        coords_m.x = 0;
        coords_m.y = 0;
        coords_m.theta_degrees = 0;

        debug_m =  DEFAULT_DEBUG_FLAG_g;
    }

    return error;
}

bool robot_arm::set_coords(robotArmCoords new_coords) {
    /*
    * This function sets the coordinates for the robot arm to the new_coords
    * Keep a temporary account of the current coords of the robot arm, set the new_coords
    * check if valid. If invalid then change back to the temp_curr_coords and return error. 
    * Else return that it did not error. 
    * 
    * NOTE: the old_coords will also be set to the new coords if it goes through
    * 
    * args:
    *   - new_coords: structure of type robotArmCoords to set the robot arm's coords to
    * returns:
    *   - boolean value: True if there was an error, Else False if there was no error
    */

    robotArmCoords temp_curr_coords;
    temp_curr_coords = coords_m;
    coords_m = new_coords;
    bool new_coords_valid = this->is_valid_pos();

    if (debug_m && !new_coords_valid) {
        cout << "IN SET COORDS setting: " << new_coords.x << ", " << new_coords.y << ", " << new_coords.theta_degrees << endl;
        cout << "env height: " << ENV_HEIGHT_g << ", env length: " << ENV_HEIGHT_g; 
        cout << "error: THE NEW COORDINATES WERE DEEMED INVALID" << endl;
    }

    if (new_coords_valid) {
        old_coords_m = coords_m;
        return false;
    } else {
        // revert back and return error
        coords_m = temp_curr_coords;
        return true;
    }   
}
/*
****************************************************************************************
* END: SET FUNCTIONS
****************************************************************************************
*/

/*
****************************************************************************************
* BEGIN: STATE CHECKING FUNCTIONS
****************************************************************************************
*/
bool robot_arm::is_valid_pos() const{
    /*
    * This function determines whether the current position of the robot arm is valid
    * returns:
    *   - boolean value: True if the position of the robot arm is valid, false otherwise
    */
    float theta_radians_min;
    float theta_radians_max;

    if (coords_m.x == 0) {
        theta_radians_min = -M_PI/2;
        theta_radians_max = M_PI/2;
    } else {
        theta_radians_min = -1 * atan((float) coords_m.y/ (float) coords_m.x);
        theta_radians_max = -1 * atan( ((float) (ENV_HEIGHT_g - coords_m.y))/((float) (0 - coords_m.x)));
    }
    float curr_theta_radians = this->get_theta_radians();

    // check the x, y and theta bounds
    bool x_in_bounds = (coords_m.x >= 0 && coords_m.x <= ENV_LENGTH_g);
    bool y_in_bounds = (coords_m.y >= 0 && coords_m.y <= ENV_HEIGHT_g);
    bool theta_rad_in_bounds = (curr_theta_radians >= theta_radians_min && curr_theta_radians <= theta_radians_max);

    if (debug_m) {
        cout << endl << endl << "Testing the position: " << coords_m.x << ", " << coords_m.y << ", " << coords_m.theta_degrees << endl;  
        if (!x_in_bounds) {
            cout << "X was out of bounds: " << coords_m.x << endl;
            cout << (coords_m.x >= 0) << endl;
            cout << (coords_m.x <= ENV_LENGTH_g) << endl;
        }
        if (!y_in_bounds) {
            cout << "Y was out of bounds: " << coords_m.y << endl;
        }
        if (!theta_rad_in_bounds) {
            cout << "The valid theta range is " << theta_radians_min << " to " << theta_radians_max << endl;
            cout << "The valid theta DEGREE range is " << theta_radians_min/THETA_MULTIPLIER << " to " << theta_radians_max/THETA_MULTIPLIER << endl;
            cout << "THETA RADIANS was out of bounds: " << coords_m.theta_degrees << endl;
        }
    }

    bool is_in_bounds = (x_in_bounds && y_in_bounds && theta_rad_in_bounds);
    return is_in_bounds;
}

void robot_arm::find_startpt_slope(environmentCoords &start_pt, float &slope) const{
    /*
    * This function returns the slope that the line takes in the environment in units of pixels/pixels
    * and the start point - y intercept of the line in the environment (where the x coordinate of the env is 0)
    * 
    * args:
    *   - start_pt: a struct of type environmentCoords that is passed by reference and is set to the value of the 
    *               y intercept of the line the robot arm makes
    *   - slope: a float that is passed by reference and set to the value that the slope of the robot arm makes
    *            in the units of pixels/pixels.
    */
    float theta_radians = this->get_theta_radians();

    float delta_x = coords_m.x;
    float delta_y = delta_x * tanf(theta_radians);

    // due to the angle if theta > 0 then the start point will be lower and vice versa 
    float start_x = 0;
    float start_y = coords_m.y + delta_y;

    // find the slope and the intercept
    if (coords_m.x == 0) {
        slope = 0;
    } else {
        slope = (start_y - coords_m.y)/(-1 * coords_m.x);
    }
    start_pt.x = start_x;
    start_pt.y = start_y;
}
/*
****************************************************************************************
* END: STATE CHECKING FUNCTIONS
****************************************************************************************
*/

/*
****************************************************************************************
* BEGIN: STATE MUTATING FUNCTIONS
****************************************************************************************
*/

void robot_arm::step(robotArmActions action, int xy_step_size, int theta_deg_step_size, bool &is_out_of_bounds, float &cost) {
    /*
    * This function steps the robot arm in the environment with the given action. It returns if 
    * the step caused the robot to go out of bounds. 
    * 
    * NOTE: any collision checking with other arms is abstracted to the container of robot_arms
    *  
    * args:
    *   - action: the action that the robot should take of type robotArmActions defined in surgical_utils.h
    *   - xy_step_size: the step size for the veritcal and horizontal actions
    *   - theta_deg_step_size: the step size for the angle change actions in theta_degrees 
    * returns by pass by reference:
    *   - is_out_of_bounds: boolean pass by reference - True if action causes robot arm to go out of bounds
    *   - cost: float pass by reference - the cost of taking the action with the robot arm
    * 
    * NOTE: The step function auto rollsback if there was an error - the robot arm's state is as it was before the step. 
    */
    int delta_x;
    int delta_y;
    int delta_theta_degrees;
    action_to_movements(action, xy_step_size, theta_deg_step_size, delta_x, delta_y, delta_theta_degrees);

    // populate the old_coords before evolving coords state in case of a rollback 
    old_coords_m = coords_m;

    coords_m.x += delta_x;
    coords_m.y += delta_y;
    coords_m.theta_degrees += delta_theta_degrees;

    is_out_of_bounds = !this->is_valid_pos();
    if (is_out_of_bounds) {
        // rollback to prevent the arm from entering an invalid state
        this->state_rollback();
        cost = MOVEMENT_ERROR;
        return;
    }
    cost = sqrt(pow(coords_m.x - old_coords_m.x, 2) + pow(coords_m.y - old_coords_m.y, 2)) + abs(coords_m.theta_degrees - old_coords_m.theta_degrees); 
}

void robot_arm::state_rollback() {
    coords_m = old_coords_m;
}
/*
****************************************************************************************
* END: STATE MUTATING FUNCTIONS
****************************************************************************************
*/
// miscellaneous helper functions
void robot_arm::printState() {
    /*
    * This function prints the entire state of the robot arm 
    */
    cout << "(x, y, theta degrees, theta radians): " << "(" << coords_m.x << ", " << coords_m.y << ", " << 
        coords_m.theta_degrees << ", " << get_theta_radians() << ") " << endl;
    cout << "Environment height: " << ENV_HEIGHT_g << ", Environment Length: " << ENV_LENGTH_g << 
        ", debug: " << debug_m << endl;
}

} // end namespace despot


