/*
* This is a file to provide global helper funtions tied to surgical_utils.h for the surgical simulator.
*/
#include "surgical_utils.h"

namespace despot {

void action_to_movements(robotArmActions action, int xy_step_size, int theta_deg_step_size, int &delta_x, int &delta_y, int &delta_theta_degrees) {
    /*
    * This function converts the robot arm action to a movement in terms of delta_x, delta_y, delta_theta_degrees
    * 
    * args:
    *   - action: of type robotArmActions specifies the action that the robot arm will take
    *   - xy_step_size: the size of steps in the x and y direction
    *   - theta_deg_step_size: the step_size in theta_degrees
    * returns:
    *   - delta_x: pass by reference - the change in the x value of the robot arm that the action corresponds to
    *   - delta_y: pass by reference - the change in the y value of the robot arm that the action corresponds to
    *   - delta_theta_degrees: pass by reference - the change in the theta_degrees value of the robot arm that the action corresponds to
    */

    // set all the values to zero initially 
    delta_x = 0;
    delta_y = 0;
    delta_theta_degrees = 0;

    switch(action) {
        case xRight:
            delta_x = xy_step_size;
            break;
        case xLeft:
            delta_x = -xy_step_size;
            break;
        case yUp:
            delta_y = xy_step_size;
            break;
        case yDown:
            delta_y = -xy_step_size;
            break;
        case thetaUp:
            delta_theta_degrees = theta_deg_step_size;
            break;
        case thetaDown:
            delta_theta_degrees = - theta_deg_step_size;
            break;
        case stay:
            break;
    }
}   

bool cmp_floats(float a, float b) {
    /*
    * Compares two floats up to the float_compare_precision
    * 
    * NOTE: do not need to use when comparing START POINTS OR SLOPES - since the start points and slopes are 
    * "based in integers" - integer that is the angle in degrees times a defined constant - they should be 
    * able to be compared wihtout worrying about precision as if they are the same they stem from the same 
    * base integers. 
    */ 
    if (abs(a - b) < float_compare_precision) {
        return true;
    } else {
        return false;
    }
}

} // end namespace despot