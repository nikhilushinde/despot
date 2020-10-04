/*
* This file contains function definitions for the "obstacle" class
* Comment flags used:
*   - MEM:ALLOC - places that dynamic memory is being explicitly allocated on the heap
*   - MEM:DELETE - places that dynamic memory is being explicitly deleted from the heap
*/

#include "obstacle.h"

using std::cout;
using std::endl;
using std::max_element;
using std::min_element;
using std::max;
using std::min;

namespace despot {
/*
* ********************************************************************************
* BEGIN: Obstacle CONSTRUCTORS
* ********************************************************************************
*/
obstacle::obstacle() {
    /*
    * Default obstacle constructor:
    *   - initialize all the numerical attributes to be zero 
    *   - initialize all pointers/lists to point to the null pointer
    * NOTE: Must call the init_obstacle function after this to properly be able to use obstacle
    */ 
    orig_center_m.x = 0;
    orig_center_m.y = 0; 
    center_m = orig_center_m;
    old_center_m = center_m;

    radius_m = 0;
    k_m = 0;

    debug_m = DEFAULT_DEBUG_FLAG_g;
}

obstacle::obstacle(environmentCoords orig_center, int radius, float k, const deflectionDirection *deflection_directions, bool debug, const robot_arm arms[], bool &error) {
    /*
    * Full constructor of the obstacle function that initializes the obstacle
    */ 
    // TODO: SEE IF YOU NEED TO RETURN AN ERROR VIA PASS BY REFERENCE
    error = init_obstacle(orig_center, radius, k, deflection_directions, debug, arms);
}

obstacle::obstacle(const obstacle &obstacle_to_copy) {
    /*
    * Copy constructor for the obstacle class
    * args:
    *   - obstacle_to_copy: object to copy from
    */ 
    orig_center_m = obstacle_to_copy.orig_center_m;
    center_m = obstacle_to_copy.center_m;
    old_center_m = obstacle_to_copy.old_center_m;

    radius_m = obstacle_to_copy.radius_m;
    k_m = obstacle_to_copy.k_m;

    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        deflection_directions_m[arm_num] = obstacle_to_copy.deflection_directions_m[arm_num];
        old_deflection_directions_m[arm_num] = obstacle_to_copy.old_deflection_directions_m[arm_num];
    }

    debug_m = obstacle_to_copy.debug_m;
}

/*
* ********************************************************************************
* END: Obstacle CONSTRUCTORS
* ********************************************************************************
*/
/*
* ********************************************************************************
* BEGIN: Obstacle DESTRUCTORS
* ********************************************************************************
*/
obstacle::~obstacle() {
    /*
    * Called when the obstacle is deleted
    */
}

/*
* ********************************************************************************
* END: Obstacle DESTRUCTORS
* ********************************************************************************
*/
/*
* ********************************************************************************
* BEGIN: Obstacle SET functions
* ********************************************************************************
*/

//TODO: CREATE A FUNCTION THAT ONLY TAKES ORIG CENTER, DEFLECTION DIRECTIONS AND SETS IT PROPERLY 
//TODO: CREATE A FUNCTION THAT JUST SETS THE DEFLECTION DIRECTION AND ALLOWS THINGS TO SETTLE PROPERLY 
//TODO: CHECK THAT YOU ARE ROLLING BACK POSITIONS AND THE ARRAYS PROPERLY WHEN RETURNING ERRORS!!!!!

bool obstacle::init_obstacle(environmentCoords orig_center, int radius, float k, const deflectionDirection deflection_directions[], bool debug, const robot_arm arms[]) {
    /*
    * Initialize the obstacle. 
    * NOTE: MAKE SURE TO CALL THIS AFTER THE DEFAULT CONSTRUCTOR to properly use the obstacle
    * args:
    *   - orig_center: the original center that the obstacle is deformed from 
    *   - radius: the obstacle radius in pixels
    *   - k: the spring constant of the imaginary obstacle springs - a cost multiplier on obstacle deformations
    *   - num_robot_arms: the number of robot arms that the obstacle will interact with. 
    *   - deflection_directions: list of deflection directions that is num_robot_arms long 
    *   - height: environment_height
    *   - length: environment_length
    *   - debug: the debug flag
    *   - arms: list of robot_arms to configure the initial obstacle position 
    */ 
    radius_m = radius;
    k_m = k;
    debug_m = debug;

    // init deflection direction lists
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
        deflection_directions_m[i] = deflection_directions[i];
        old_deflection_directions_m[i] = deflection_directions[i];
    }

    orig_center_m = orig_center;
    center_m = orig_center;
    old_center_m = orig_center;
    
    bool error;
    float cost;
    // do a step to deformed center here to "settle" the environment - return error if there is one
    step_to_deformed_center(arms, error, cost);

    // set the old_center to the center that the obstacle settles on after stepping
    old_center_m = center_m;
    return error;
}

bool obstacle::set_coords(environmentCoords coords, const deflectionDirection deflection_directions[], const robot_arm arms[]) {
    /*
    * NOTE: Use this method only when setting obstacle positions non sequentially after the first initialization - eg: while reinitializing
    * a state in A star. Do not use it for steps or initializing an obstacle. 
    * 
    * Set the coordinates of the obstacle using the deflection directions and the arms
    * Methodology:
    *   - 1. Set the coordinate of the obstacle to the exact given coordinate
    *   - 2. Set the specified deflection directions exactly as well 
    *   - 3. Check that no arms are intersecting the obstacle in this position 
    * args:
    *   - coords: the new coordinates of the obstacle
    *   - deflection_directions: the new deflection directions to set for the obstacle
    *   - arms: list of the arms in the environment to check for collision with the obstacle. 
    * returns:
    *   - boolean error: true if there was a collision with the arms in the set position, or the limit error
    */ 
    bool error = false;

    // return error if out of bounds if limiting the obstacle positions
    if (OBSTACLE_LIMIT_FLAG_g) {
        if (coords.y < radius_m || coords.y > ENV_HEIGHT_g - radius_m) {
            error = true;
            return error;
        }
    }

    center_m = coords;
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        deflection_directions_m[i] = deflection_directions[i];
    }

    environmentCoords above_stuck_center;
    environmentCoords below_stuck_center;
    bool is_not_intersecting = true;
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        /* To make sure no arms are intersecting the obstacle:
        *   - for each arm check that the stuck coordinate from deflecting above and below is either above or equal to
        *     the above deflection direction or below or equal to the below deflection direction (otherwise there is an intersection with the arm)
        */
        if (arms[i].get_x() < center_m.x - radius_m) {
            continue;
        }
        above_stuck_center = single_arm_stuck_center(arms[i], obs_above);
        below_stuck_center = single_arm_stuck_center(arms[i], obs_below);

        is_not_intersecting = ((center_m.y >= above_stuck_center.y) && (center_m.y <= below_stuck_center.y));
        if (!is_not_intersecting) {
            return !is_not_intersecting;
        }
    }

    return error;

}

void obstacle::set_deflection_directions(const robot_arm arms[]) {
    /*
    * Sets the deflection directions for arms that are within range of the obstacle
    * args:
    *   - arms: list of arms with respect to which to set the deflection directions
    * Methodology:
    *   - If self.deflection is None set deflection by comparing to original center
    *    If any self.deflection is set then since all arms move at once we compare to the last 
    *    snapshot of the center (self.x, self.y)
    * 
    *    set all at once as arms move all at once
    */ 
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        old_deflection_directions_m[i] = deflection_directions_m[i];
    }

    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        if (arms[i].get_x() < orig_center_m.x - radius_m) {
            // set None
            deflection_directions_m[i] = obs_none;
        } else if (old_deflection_directions_m[i] == obs_none) {
            
            if (center_m.x <= arms[i].get_x()) {
                // set relationship if robot has crossed the obstacle center
                environmentCoords robot_start_pt;
                float robot_slope, arm_pt_y;
                
                arms[i].find_startpt_slope(robot_start_pt, robot_slope);
                arm_pt_y = robot_slope*center_m.x + robot_start_pt.y;
                if (arm_pt_y > center_m.y) {
                    deflection_directions_m[i] = obs_below;
                } else {
                    deflection_directions_m[i] = obs_above;
                }
            } else if (center_m.x - radius_m <= arms[i].get_x() && arms[i].get_x() < center_m.x) {
                // set relationship if the robot has not crossed the obstacle center yet
                if (arms[i].get_y() > center_m.y) {
                    deflection_directions_m[i] = obs_below;
                } else {
                    deflection_directions_m[i] = obs_above;
                }
            }
        } 
    }
    return;
}

bool obstacle::manual_set_deflection_directions(const robot_arm arms[], const deflectionDirection deflection_directions[]) {
    /*
    * Manually set the deflection directions list of the obstacle. 
    * Methodology:
    *   - 1. manually set the deflection directions list
    *   - 2. allow the environment to "settle" as dictated by the robot arms
    *       - 2.1 check for any invalid states - collisions with the arms, squishing by the arms, limit violations
    *   
    * returns: 
    *   error: boolean value that is true if there was an error
    * 
    * NOTE: The final deflection directions may turn out to be different after the environment settles with the robot arms
    * NOTE: If there was an error the state is rolled back to the previous state by the step_to_deformed_center function
    */ 
    bool error = false;
    /* 1. SET DEFLECTION DIRECTIONS */
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        old_deflection_directions_m[i] = deflection_directions_m[i];
        deflection_directions_m[i] = deflection_directions[i];
    }

    /* 2. ALLOW THE ENVIRONMENT TO SETTLE - also checks for any invalid states*/
    float cost;
    step_to_deformed_center(arms, error, cost);
    if (error) {
        return error;
    }
    
    return error;
}



// SIMPLE SET FUNCTIONS
void obstacle::set_orig_center(environmentCoords center) {
    orig_center_m = center;
}
void obstacle::set_radius(int radius) {
    radius_m = radius;
}
void obstacle::set_k(float k) {
    k_m = k;
}
void obstacle::set_debug(bool debug) {
    debug_m = debug;
}

/*
* ********************************************************************************
* END: Obstacle SET functions
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: Obstacle GET functions
* ********************************************************************************
*/
// SIMPLE GET FUNCTIONS
void obstacle::get_deflection_directions(deflectionDirection *ret_deflection_directions, int arraySize) const{
    /*
    * Fills in the array pointer with the deflection directions of the obstacle, arraySize given to prevent 
    * illegal memory accesses. 
    */ 
    for (int i = 0; i < arraySize; i++) {
        ret_deflection_directions[i] = deflection_directions_m[i];
    }
}

environmentCoords obstacle::get_orig_center() const{
    environmentCoords ret_center = orig_center_m;
    return ret_center;
}
environmentCoords obstacle::get_center() const{
    environmentCoords ret_center = center_m;
    return ret_center;
}
int obstacle::get_radius() const{
    return radius_m;
}
float obstacle::get_k() const{
    return k_m;
}
bool obstacle::get_debug() const{
    return debug_m;
}
/*
* ********************************************************************************
* END: Obstacle GET functions
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: Obstacle OTHER functions
* ********************************************************************************
*/

float obstacle::deformation_cost() const{
    /* Cost to move from old_x, old_y to x,y using the spring energy model
    * Uses difference in potential energy of the spring to calculate cost. 
    *   NOTE: TODO: - change this to change the cost model. 
    */
    float old_potential_energy = (0.5) * k_m * pow((old_center_m.y - orig_center_m.y), 2);
    float new_potential_energy = (0.5) * k_m * pow((center_m.y - orig_center_m.y), 2);
    float stage_cost = abs(new_potential_energy - old_potential_energy);

    return stage_cost;
}

environmentCoords obstacle::single_arm_stuck_center(robot_arm arm, deflectionDirection deflection_direction) const{
    /*
    * IMPORTANT NOTE: ALIAS FOR: single_arm_stuck_center
    * (self.deflection_directions MUST BE already set correctly for the new arm position 
    * before this function call)
    * 
    * This finds the deflected center of the obstacle with the obstacle stuck to the arm
    * if the deflection direction is above the obstacle center cannot have a y value lower than 
    * this position 
    * If the deflection direction is below the obstacle center cannot have a y value that is 
    * greater than this position 
    * 
    * args:
    *   - robot_arm: robot arm object
    *   - environment height 
    *   - deflection_direction - corresponding to that arm object
    * returns:
    *   - deflected_center: deflected center of this object 
    */
    cout << "IN STUCK CENTER FUNCTION: the arm coordinate given here is (" << arm.get_x() << "," << arm.get_y() << endl;
    environmentCoords stuck_center;
    stuck_center.x = 0;
    stuck_center.y = 0;

    if (orig_center_m.x - radius_m > arm.get_x()) {
        stuck_center = orig_center_m;
        return stuck_center;
    }
    /* COMPUTE DEFLECTION: find the intersection of te line and the obstacle if the obstacle center was on the line. 
    then find the midpoint on the arc between the two interssection points. Then based on the deflection direction change
    the center so tht the midpoint is shifted such that it is on the line.*/
    // m = slope, b = y intercept, (x_c, y_c) - new center on the line
    environmentCoords start_pt;
    float m, b, x_c, y_c;
    arm.find_startpt_slope(start_pt, m);

    cout << "THE SLOPE OF THE ARM IS: " << m << " and the start point is: (" << start_pt.x  << ',' << start_pt.y << ')' << endl;
    b = start_pt.y;
    x_c = orig_center_m.x;
    y_c = m*x_c + b;

    // PERPENDICULAR LINE: to find the arc midpoint find the intersection of the perpendicular line through the center
    float perp_m, perp_b, a_qf, b_qf, c_qf, pos_perpint_x, pos_perpint_y, neg_perpint_x, neg_perpint_y;
    if (m!=0) {
        perp_m = -1/m;
        perp_b = (x_c/m) + y_c;
        a_qf = 1 + pow(perp_m, 2);
        b_qf = 2*(perp_m*(perp_b - y_c) - x_c);
        c_qf = pow(x_c, 2) + pow((perp_b - y_c), 2) - pow(radius_m, 2);

        pos_perpint_x = (-b_qf + sqrt(pow(b_qf, 2) - 4*a_qf*c_qf))/(2*a_qf);
        pos_perpint_y = perp_m*pos_perpint_x + perp_b;
        neg_perpint_x = (-b_qf - sqrt(pow(b_qf, 2) - 4*a_qf*c_qf))/(2*a_qf);
        neg_perpint_y = perp_m*neg_perpint_x + perp_b;
    } else {
        pos_perpint_x = x_c;
        pos_perpint_y = y_c + radius_m;
        neg_perpint_x = x_c;
        neg_perpint_y = y_c - radius_m;
    }

    cout << "negperpint y: " <<  neg_perpint_y << ", posperpint y: " << pos_perpint_y << endl;
    environmentCoords arc_mid_point; 
    if (deflection_direction == obs_above) {
        // if the obstacle was to deflect above you want to move the lower point up 
        if (neg_perpint_y < pos_perpint_y) {
            arc_mid_point.x = neg_perpint_x;
            arc_mid_point.y = neg_perpint_y;
        } else {
            arc_mid_point.x = pos_perpint_x;
            arc_mid_point.y = pos_perpint_y;
        }
    } else {
        // if the obstacle was to deflect below you want to move the upper point down 
        if (neg_perpint_y > pos_perpint_y) {
            arc_mid_point.x = neg_perpint_x;
            arc_mid_point.y = neg_perpint_y;
        } else {
            arc_mid_point.x = pos_perpint_x;
            arc_mid_point.y = pos_perpint_y;
        }
    }
    float y_shift; // what you are shifting the obstacle by to get it to that deformed position 
    /* CASE 1: case when both extremum of the obstacle (center - radius and center + radius) are before the robot arm x*/
    if (x_c - radius_m < arm.get_x() && x_c + radius_m < arm.get_x()) {
        y_shift = (m * arc_mid_point.x + b) - arc_mid_point.y;
    } else {
    /* CASE 2: case when one center - radius piont before the robot line */
        // if the robot endpiont crosses the obstacle center in x use the same arc_mid_point method
        if (arm.get_x() >= arc_mid_point.x) {
            y_shift = (m * arc_mid_point.x + b) - arc_mid_point.y;
        } else {
        // if robot endpoint does not cross the arc midpoint in x use the idea that the center must stay radius away from the arm endpoint x 
            float delta_x = (float) abs(x_c - arm.get_x());
            float delta_y = (float) abs(y_c - arm.get_y());
            
            //this delta is from the robot arm not the center - as you are shifting the center not the robot arm
            float new_delta_y = abs(sqrt(pow(radius_m, 2) - pow(delta_x, 2)));
            if (deflection_direction == obs_above) {
                y_shift = new_delta_y + arm.get_y() - y_c;
            } else {
                y_shift = -new_delta_y + arm.get_y() - y_c;
            }
        }
    }
    // y_shift is the shift with respect to the obstacle center placed on the robot arm line
    stuck_center.x = x_c;
    stuck_center.y = y_c + y_shift;
    return stuck_center;
}

environmentCoords obstacle::single_arm_deformed_center(robot_arm arm, deflectionDirection deflection_direction, bool &error) const{
    /*
    * Returns the proposed center of the obstacle if it were deformed by the given robot arm in the given deflection direction
    * Methodology:
    *   - 1. Calculate the center of the obstacle if it were stuck to the robot arm while deflecting in the given direction
    *   - 2. See if you need to deform given this found center. 
    *   - 3. Check limits and decide if an error has occurred (if limit flag set to true)
    */ 
    error = false;
    environmentCoords proposed_center;
    // store the stuck center in deflected_center 
    proposed_center = single_arm_stuck_center(arm, deflection_direction);
    cout << "The STUCK CENTER to arm is (" << proposed_center.x << "," << proposed_center.y << ")" << endl;

    environmentCoords deflected_center;
    deflected_center.x = proposed_center.x;
    deflected_center.y = proposed_center.y;
    // once you find the deflected stuck center - you have to decide whether to deflect
    if (deflection_direction == obs_above) {
        // if the deflection direction is above: deflect if the deflected center is above the original point
        if (deflected_center.y < orig_center_m.y) {
            deflected_center.x = orig_center_m.x;
            deflected_center.y = orig_center_m.y;
        }
    } else {
        // if the deflection is below: deflect if the deflected center is below the original point
        if (deflected_center.y > orig_center_m.y) {
            deflected_center.x = orig_center_m.x;
            deflected_center.y = orig_center_m.y;
        }
    }

    // if limiting the position that the obstacles can take in the environment
    if (OBSTACLE_LIMIT_FLAG_g) {
        error = (deflected_center.y < radius_m || deflected_center.y > ENV_HEIGHT_g - radius_m);
    }
    cout << "The RETURNED DEFLECTED CENTER to arm is (" << deflected_center.x << "," << deflected_center.y << ")" << endl;
    
    return deflected_center;
}

bool obstacle::is_collision(robot_arm arm, deflectionDirection deflection_direction, environmentCoords obs_center) {
    /*
    * Returns true if there is a collision with the specified arm. 
    * METHODOLOGY: 
    *   - Take the obs_center set it as the orig_center of the obstacle, calculate the deformed center, if the 
    *   calculated deformed center is different than the center that you set then there was an intersection with 
    *   robot_arm that deformed the obstacle.
    */
    environmentCoords temp_store_orig_center = orig_center_m;
    orig_center_m = obs_center;

    bool error; 
    environmentCoords deflected_center;
    deflected_center = single_arm_deformed_center(arm, deflection_direction, error);

    // reset the orig_center properly
    orig_center_m = temp_store_orig_center;

    // if the deflected_center is the same as the obs_center then there was no collision 
    if (obs_center.x == deflected_center.x && obs_center.y == deflected_center.y) {
        return false;
    } else {
        return true;
    }
}

void obstacle::printState() const{
    /*
    * Function to print the state of the obstacle
    */ 
    cout << "original center: (" << orig_center_m.x << "," << orig_center_m.y << ")  center:  (" << center_m.x << "," << center_m.y << ")" << endl;
    cout << "k value: " << k_m << "  radius: " << radius_m << endl; 
    cout << "Deflection directions: [";
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        if (deflection_directions_m[i] == obs_none) {
            cout << "None, ";
        } else if (deflection_directions_m[i] == obs_above) {
            cout << "Above, ";
        } else if (deflection_directions_m[i] == obs_below) {
            cout << "Below, ";
        }
    }
    cout << "]" << endl;
}

/*
* ********************************************************************************
* END: Obstacle OTHER functions
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: STATE MUTATION functions
* ********************************************************************************
*/
void obstacle::step_to_deformed_center( const robot_arm arms[], bool &error, float &cost) {
    /*
    * Given the new state of the robot arms calculates the new deformed center of the obstacle, sets it and 
    * checks if there are any errors. 
    * 
    * args:
    *   - arms: list of the arm states
    * returns: pass by reference:
    *   - error: true if there was an error - squishing an obstacle or boundary condition violations if checking them (limit == true)
    *   - cost: cost of obstacle deformations
    * 
    * NOTE: step function auto rollbacks if there was an error
    */ 
    set_deflection_directions(arms);
    old_center_m.x = center_m.x;
    old_center_m.y = center_m.y;

    cout << endl << endl;
    cout << "In function OBSTACLE STEP TO DEFORMED CENTER " << endl;
    cout << "Finding each deformed center" << endl;
    environmentCoords proposed_center;
    float all_proposed_ys[NUM_ROBOT_ARMS_g];
    bool temp_error = false;
    
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        //proposed_center = single_arm_stuck_center(arms[i], deflection_directions[i]);
        //cout << "IN STEP: The STUCK CENTER to arm indexed at: " << i << " is (" << proposed_center.x << "," << proposed_center.y << ")" << endl;
        proposed_center = single_arm_deformed_center(arms[i], deflection_directions_m[i], temp_error);
        error = temp_error;
        cout << "IN STEP: The INITIAL DEFORMED CENTER to arm indexed at: " << i << " is (" << proposed_center.x << "," << proposed_center.y << ")" << endl;
        cout << endl << endl;
        
        all_proposed_ys[i] = proposed_center.y;
        if (error) {
            cost = OBSTACLE_ERROR_COST;
            state_rollback();
            return;
        }
    }

    // calcuate the upper most deflection in the above direction
    float uppermost_y = orig_center_m.y;
    // calculate the lower most deflection in the below direction
    float lowermost_y = orig_center_m.y;
    
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        if (deflection_directions_m[i] == obs_above) {
            uppermost_y = max(uppermost_y, all_proposed_ys[i]);
        } else if (deflection_directions_m[i] == obs_below) {
            lowermost_y = min(lowermost_y, all_proposed_ys[i]);
        }   
    }

    bool upper_equal_lower_center, upper_equal_orig, lower_equal_orig;
    upper_equal_lower_center = (uppermost_y == lowermost_y);
    upper_equal_orig = (uppermost_y == orig_center_m.y);
    lower_equal_orig = (lowermost_y == orig_center_m.y);

    environmentCoords deflected_center;
    deflected_center.x = orig_center_m.x;
    // if there is a conflict or one of the upper or lower proposed centers is not the origin then there must be an error
    if ((!upper_equal_lower_center) &&  (!(upper_equal_orig || lower_equal_orig))) {
        error = true;
        cost = OBSTACLE_ERROR_COST;
        state_rollback();
        return;
    } else {
        if (lower_equal_orig) {
            deflected_center.y = uppermost_y;
        } else {
            deflected_center.y = lowermost_y;
        }
    }

    cout << "Checking for a collision with the arms" << endl;
    // check for a collision
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        error = is_collision(arms[i], deflection_directions_m[i], deflected_center);
        if (error) {
            cout << "FOUND COLLISION WITH ARM indexed at: " << i << endl << endl;
            cost = OBSTACLE_ERROR_COST;
            state_rollback();
            return;
        } 
    }
    cout << "In end of OBSTACLE STEP" << endl;
    cout << "CURRENT CENTER: (" << center_m.x  << "," << center_m.y << ")" << endl;
    cout << "DEFLECTED CENTER: (" << deflected_center.x << "," << deflected_center.y << ")" << endl;
    cout << endl << endl;
    center_m = deflected_center;
    cost = deformation_cost();
    return;
}


void obstacle::state_rollback() {
    /* Rollback to the obstacle state at the last time step */
    center_m.x = old_center_m.x;
    center_m.y = old_center_m.y;
    rollback_deflection_directions();
    return;
}

void obstacle::rollback_deflection_directions() {
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
        deflection_directions_m[i] = old_deflection_directions_m[i];
    }
    return;
}
/*
* ********************************************************************************
* END: STATE MUATATION functions
* ********************************************************************************
*/
} // end namespace despot