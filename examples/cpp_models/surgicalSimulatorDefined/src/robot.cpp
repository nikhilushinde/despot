/*
* File containing function descriptions for the robot class that is described in "robot.h"
* Comment flags used:
*   - MEM:ALLOC - places that dynamic memory is being explicitly allocated on the heap
*   - MEM:DELETE - places that dynamic memory is being explicitly deleted from the heap
*/

#include "robot.h"

using std::cout;
using std::cerr;
using std::endl;
using std::max_element;
using std::distance;
using std::min;

namespace despot {

/*
* ********************************************************************************
* BEGIN: Robot Constructors
* ********************************************************************************
*/
robot::robot() {
    /*
    * This is the default constructor for the robot class does not properly
    * set the other paramters. 
    * NOTE: must call the following functions after this to properly initialize
    *   - 3. initialize the robot_arms
    */
    if (NUM_ROBOT_ARMS_g <= 0) {
        cerr << "The number of robot arms must be set to greater than 0 to initialize the robot" << endl;
        exit(1);
    }
    debug_m = DEFAULT_DEBUG_FLAG_g;

    // set the robot arms to the default robot arm coordinates that are defined in the defined parameters file
    robotArmCoords arm_coords[NUM_ROBOT_ARMS_g];
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        arm_coords[arm_num].x = all_robot_x_g[arm_num];
        arm_coords[arm_num].y = all_robot_y_g[arm_num];
        arm_coords[arm_num].theta_degrees = all_robot_theta_deg_g[arm_num];
    }
    // initialize the robot arms with these default values
    bool error = init_robot(arm_coords, debug_m);
    if (error) {
        cerr << "The default parameters set for the robot arms in the defined parameters file were invalid" << endl;
        exit(1);
    }
}

robot::robot(const robotArmCoords arm_coords[], bool debug, bool &error) {
    /*
    * This is the full constructor for the robot
    * args:
    *   - arm_coords: the arm coordinates of all the robot arms
    *   - debug: boolean debug flag
    * returns: by pass by reference
    *   - error: true if there is an error, else returns false. 
    */
    if (NUM_ROBOT_ARMS_g <= 0) {
        cerr << "The number of robot arms must be set to greater than 0 to initialize the robot" << endl;
        exit(1);
    }
    error = init_robot(arm_coords, debug);
}

robot::robot(const robot &robot_to_copy) {
    /*
    * Copy constructor for the robot class. 
    * args:
    *   - robot_to_copy: the robot object to copy
    */ 
    debug_m = robot_to_copy.debug_m;
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        // this uses the robot_arm classes copy constructor implicitly
        arms_m[arm_num] = robot_to_copy.arms_m[arm_num];

        robot_arm_relations_m[arm_num] = robot_to_copy.robot_arm_relations_m[arm_num];
        old_robot_arm_relations_m[arm_num] = robot_to_copy.old_robot_arm_relations_m[arm_num];
    }

}
/*
* ********************************************************************************
* END: Robot Constructors
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: Robot Destructor functions
* ********************************************************************************
*/
robot::~robot() {
    /*
    * Destructor for robot objects - delete all dynamically allocated memory
    */ 
}

/*
* ********************************************************************************
* END: Robot Destructor functions
* ********************************************************************************
*/


/*
* ********************************************************************************
* BEGIN: Initialization and Set Functions and Get Functions
* ********************************************************************************
*/
bool robot::init_robot(const robotArmCoords arm_coords[], bool debug) {
    /*
    * This function completely initializes the robot - make sure to call this after the default constructor to properly set robot attributes
    * args:
    *   - arm_coords: the arm coordinates of all the robot arms
    *   - debug: boolean debug flag
    * returns:
    *   - error: true if there is an error, else returns false. 
    * NOTE: If this errors - MUST REINITIALIZE the robot object for it to function properly 
    */
    bool error = false;

    // allocate lists and set the robot arms and relations correctly 
    error = set_arms(arm_coords);
    if (error) {
        // if error set the other attributes back to 0
        debug = DEFAULT_DEBUG_FLAG_g;
    }
    return error;
}

bool robot::set_arms(const robotArmCoords arm_coords[]) {
    /*
    * Sets the arm coordinates. If the arms existed before this then if there is an error in setting the 
    * arms roll the state back to the old state that the arms had. 
    * 
    * If the arms did not exist before this (this function is called in initializing the arms) then delete the 
    * lists upon encountering an error:
    * 
    * Function layout
    *   - 1. Set the arm coords
    *   - 2. check for inter robot_arm collisions 
    *   - 3. Set the arm relations 
    *       - 3.1 Set the old arm relations
    * 
    * args:
    *   - arm_coords: arm coordinates to set of type robotArmCoords
    * returns:
    *   - error: true if there was an error, else false 
    */ 

    if (debug_m) {
        cout << endl << endl << "IN ROBOT SET ARMS FUNCTION:  " << endl << endl;
        cout << "INITIAL CHECKS" << endl;
        cout << "SETTING ARM COORDINATES" << endl;
    }
    /* 1: SETTING ARM COORDINATES */
    bool error = false;
    // if arms has not been set, dynamically allocate the arms of the robot
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num ++) {
        error = arms_m[arm_num].set_coords(arm_coords[arm_num]);
        if (error) {
            // roll back all the arms up to and INCLUDING the arm that you just set. 
            for (int i = 0; i < arm_num + 1; i ++)  {
                arms_m[i].state_rollback();
            }
            return error;
        }
    }

    
    if (debug_m) {
        cout << "CHECKING FOR INTER ARM COLLISIONS" << endl;
    }
    /* 2. CHECK FOR INTER ARM COLLISIONS */
    error = is_any_collision();
    if (error) { // there was a collision
        // roll back all the arms
        for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
            arms_m[i].state_rollback();
        }
        return error;
    }

    if (debug_m) {
        cout << "SETTING ARM RELATIONS" << endl;
    }
    /* 3. SETTING THE ARM RELATIONS */
    // set the arm relations - the error from this is the same as the if statement at the beginning so won't be relevant
    error = set_arm_relations(); // TODO: CHANGE THE SET ARM RELATIONS FUNCTION TO REFLECT THE NON DYNAMIC LIST USAGE
    if (error) {
        // roll back all the arms
        for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
            arms_m[i].state_rollback();
        }
        return error;
    }
    /* 3.1 set the old arm relations to match the current ones */
    // Set the old robot arm relations to match the newly set ones since the environment has not seen anything else 
    // set the old arm relations;
    for (int j = 0; j < NUM_ROBOT_ARMS_g; j ++) {
        old_robot_arm_relations_m[j] = robot_arm_relations_m[j];
    }

    return error;
}

bool robot::set_arm_relations() {
    /*
    * Sets the robot arm relations. The order of the arms in the vertical axis. 
    * If an arm is not in the environment x == 0, then it is set to the NOT_ARM_INDEX defined constant
    */
    bool error = false;
    float all_start_ys[NUM_ROBOT_ARMS_g];
    environmentCoords start_pt;
    float slope;
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num ++) {
        if (arms_m[arm_num].get_x() <= 0) {
            all_start_ys[arm_num] = NO_YS;
        } else {
            arms_m[arm_num].find_startpt_slope(start_pt, slope);
            all_start_ys[arm_num] = start_pt.y;
        }
    }

    float *max_y_value_ptr;
    int max_y_armindex;
    int set_rel_index = 0;
    // set the default value for all the indices in the arm relations array 
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
        robot_arm_relations_m[i] = NOT_ARM_INDEX;
    }

    // set the actual arm relations
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
        max_y_value_ptr = max_element(all_start_ys, all_start_ys + NUM_ROBOT_ARMS_g);
        if (*max_y_value_ptr == NO_YS) {
            break;
        }
        max_y_armindex = distance(all_start_ys, max_y_value_ptr);
        if (arms_m[max_y_armindex].get_x() > 0) {
            robot_arm_relations_m[set_rel_index] = max_y_armindex;
            set_rel_index += 1;
        }
        // set the value of that index to -1 so it is not selected as the max in the next round
        all_start_ys[max_y_armindex] = NO_YS; 
    }

    return error;
}

/*
* ********************************************************************************
* END: Initialization and Set Functions and Get Functions
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: OTHER FUNCTIONS
* ********************************************************************************
*/
bool robot::is_2arm_collision(const robot_arm &arm1, const robot_arm &arm2) const {
    /*
    * This function checks if there is a collision between the two given arms
    * args:
    *   - arm: robot_arm object that you are checking against for a collision
    * returns:
    *   - boolean value: True if there is a collision and False is if there is no collision
    */
    environmentCoords startPt1;
    float slope1;
    float y1;

    environmentCoords startPt2;
    float slope2;
    float y2;

    arm1.find_startpt_slope(startPt1, slope1);
    arm2.find_startpt_slope(startPt2, slope2);
    y1 =  startPt1.y;
    y2 = startPt2.y;

    if (slope1 == slope2) {
        // lines of the same slope only intersect if they are on top of each other - have the same start point
        // they are not intersecting if the x values are 0
        return (y1 == y2 && arm1.get_x() != 0 && arm2.get_x() != 0);
    } 

    float intersection_x = (y2 - y1)/(slope1 - slope2);
    bool is_collision = ((intersection_x < arm1.get_x()) && (intersection_x < arm2.get_x()) && (intersection_x > 0));
    return is_collision;
}

bool robot::is_any_collision() const{
    /*
    * This function checks if there are any collisions between any of the robot_arms in the robot
    */ 
    bool found_collision = false;
    for (int arm1_idx = 0; arm1_idx < NUM_ROBOT_ARMS_g; arm1_idx ++) {
        // TODO: CHANGE THIS FOR ARM2 INDEX TO START AT 0 and go up to the index itself - prevents double checking
        for (int arm2_idx = 0; arm2_idx < NUM_ROBOT_ARMS_g; arm2_idx ++) {
            if (arm2_idx != arm1_idx) {
                found_collision = is_2arm_collision(arms_m[arm1_idx], arms_m[arm2_idx]);
                if (found_collision) {
                    return true;
                }
            }
        }
    }
    return found_collision;
}


bool robot::violate_above_relation(const robot_arm &above_arm, const robot_arm &below_arm) const{
    /*
    * This function takes in two arms and checks if the above_arm is entirely above the below_arm
    * args:
    *   - above_arm: robot_arm that is supposed to be above the other arm
    *   - below_arm: robot_arm that is supposed to be below the other arm 
    * return:
    *   - boolean: true if the above_arm is NOT above the below_arm else false
    */ 
    if (above_arm.get_x() == 0 || below_arm.get_x() == 0) {
        return false;
    }
    environmentCoords above_start_pt, below_start_pt;
    float above_slope, below_slope;

    above_arm.find_startpt_slope(above_start_pt, above_slope);
    below_arm.find_startpt_slope(below_start_pt, below_slope);

    float min_x;
    min_x = (float) min(above_arm.get_x(), below_arm.get_x());
    float above_y = (min_x * above_slope) + above_start_pt.y;
    float below_y = (min_x * below_slope) + below_start_pt.y;
    // check that both the above start point and the above calculated point are above the corresponding below points
    // TODO: SEE IF THIS HAS TO BE GREATER THAN OR EQUAL TO ?
    if (above_y > below_y && above_start_pt.y > below_start_pt.y) {
        return false;
    } else {
        return true;
    }
}

void robot::step(const robotArmActions actions[], int xy_step_size, int theta_deg_step_size, bool &error, float &cost) {
    /*
    * Robot takes a step in the environment. 
    * Methodology:
    *   - 1. Step all the arms
    *   - 2. Check if there is a collision - if there is roll back individual arms
    *   - 3. check arm order and make sure it hasn't switched for every arm in the list and ensure that those that were below it are still below it (except the last arm)
    *   - 4. set the arm relations using set_arm_relations()
    * 
    * args:
    *   - actions: list of actions of type robotArmActions where the index of the action in the list corresponds to which robot arm it is moving from arms
    *   - xy_step_size: integer step size for vertical and horizontal movement of end effector
    *   - theta_deg_step_size: integer step size for change in the theta in degrees of the end effector
    * returns: by pass by reference
    *   - error: true if an error occurred, else false
    *   - cost: the cost of taking the specified actions
    * 
    * NOTE: the step function auto rolls back if there was an error 
    * 
    * TODO: REMOVE EXTRA LINES OF CODE - might want to change the cost structure and the way errors are treated ?
    */ 

    if (debug_m) {
        cout << endl << endl;
        cout << "IN FUNCTION ROBOT STEP " << endl << "STEPPING ALL THE ARMS" << endl;
    }
    /* 1. STEP ALL THE ARMS */
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        old_robot_arm_relations_m[i] = robot_arm_relations_m[i];
    }
    bool is_out_of_bounds = false;
    float arm_cost = 0;
    float totalCost = 0;
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
        arms_m[i].step(actions[i], xy_step_size, theta_deg_step_size, is_out_of_bounds, arm_cost);
        totalCost += arm_cost;
        if (is_out_of_bounds) {
            // there was an error - rollback and return 
            for (int roll_back_num = 0; roll_back_num < i + 1; roll_back_num ++) {
                arms_m[roll_back_num].state_rollback();
            }
            error = true;
            //cost = totalCost;
            cost = ROBOT_MOVEMENT_ERROR_COST;
            return; 
        }
    }
    cost = totalCost;
    
    if (debug_m) {
        cout << "CHECKING FOR COLLISION" << endl;
    }
    /* 2. CHECK FOR COLLISION */
    error = is_any_collision();
    if (error) {
        // roll back all the arms and return error 
        for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
            arms_m[i].state_rollback();
        }
        // additional cost for collision
        // TODO: MIGHT WANT TO CHANGE THE COST STRUCTURE
        //cost += MOVEMENT_ERROR;
        cost = ROBOT_MOVEMENT_ERROR_COST;
        return;
    }

    if (debug_m) {
        cout << "CHECKING THAT THE ARM RELATIONS ARE PRESERVED" << endl;
    }
    int above_index; // index of the arm that should be above
    int below_index; // index of the arm that should be below
    /* 3. CHECK THAT ARM RELATIONS WERE NOT VIOLATED */
    for (int a = 0; a < NUM_ROBOT_ARMS_g; a ++) {
        above_index = robot_arm_relations_m[a];
        if (above_index == NOT_ARM_INDEX) {
            // you reached the lowest arm so break the for loop
            break;
        }
        // go down the robot_arm_relations list - for every arm make sure the arms below it are still below it/ have x = 0
        for (int b = a + 1; b < NUM_ROBOT_ARMS_g; b ++) {
            below_index = robot_arm_relations_m[b];
            if (below_index == NOT_ARM_INDEX) {
                // you reached the lowest arm so break the for loop
                break;
            }
            error = violate_above_relation(arms_m[above_index], arms_m[below_index]);
            if (error) {
                // there was an error rollback and return error 
                for (int rollback_index = 0; rollback_index < NUM_ROBOT_ARMS_g; rollback_index ++) {
                    arms_m[rollback_index].state_rollback();
                }
                cost = ROBOT_MOVEMENT_ERROR_COST;
                return;
            }
        }
    }

    if (debug_m) {
        cout << "SETTING THE ARM RELATIONS" << endl << endl;
    }
    /* 4. SET THE ARM RELATIONS - given that there were no errors */
    set_arm_relations();
    return;
}

void robot::state_rollback() {
    /*
    * Rolls back all the internal arms and also sets the current arm relations to the old arm relations
    */ 
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
        robot_arm_relations_m[i] = old_robot_arm_relations_m[i];
        arms_m[i].state_rollback();
    }   
}

void robot::printState() const{
    /* Prints the state of the robot arm */
    cout << "Robot Environment height: " << ENV_HEIGHT_g << ", Robot environment length: " << ENV_LENGTH_g << endl;
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i ++) {
        cout << "ARM NUM: " << i << endl;
        arms_m[i].printState();
    }
    cout << "Robot arm relations: ["; 
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        cout << robot_arm_relations_m[i] << ", ";
    }
    cout << "]" << endl;
}

/*
* ********************************************************************************
* END: OTHER FUNCTIONS
* ********************************************************************************
*/

} // end namespace despot 