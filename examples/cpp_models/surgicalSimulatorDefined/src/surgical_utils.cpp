/*
* This is a file to provide global helper funtions tied to surgical_utils.h for the surgical simulator.
*/
#include "surgical_utils.h"

namespace despot {


void action_to_movements_g(robotArmActions action, int xy_step_size, int theta_deg_step_size, int &delta_x, int &delta_y, int &delta_theta_degrees) {
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


void int_to_action_array_g(int actionNum, robotArmActions *ret_action_array, int ret_action_array_size) {
    /*
    * Convert specified integer to an array of actions - where the action at index i corresponds to the action the robot's ith robot arm will take.
    * returns: by pass by reference:
    *   - the array of robot actions to take
    */ 
    if (ret_action_array_size != NUM_ROBOT_ARMS_g) {
        std::cerr << "ERROR: invalid return array size in IntToActions" << std::endl;
        exit(1);
    }

    int num_actions_per_arm = 6; // as we do not include the stay action for these purposes
    int arm_to_move_index = static_cast<int>(actionNum/num_actions_per_arm);
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        if (arm_num == arm_to_move_index) {
            ret_action_array[arm_to_move_index] = static_cast<robotArmActions>(actionNum%num_actions_per_arm);
        } else {
            ret_action_array[arm_num] = stay;
        }
    }
    return;
}

bool isReverseAction_g(ACT_TYPE act1, ACT_TYPE act2)  {
    /*
    * checks if act2 undoes the effects of act1
    * args:
    *   - act1: action number 1 - of ACT_TYPE not robotArmActions 
    *   - act2: action number 2 - of ACT_TYPE not robotArmActions
    * returns:
    *   - boolean: true if the act1 reverses act2 else 
    *
    */ 
    robotArmActions act_array1[NUM_ROBOT_ARMS_g];
    robotArmActions act_array2[NUM_ROBOT_ARMS_g];

    int_to_action_array_g(act1, act_array1, NUM_ROBOT_ARMS_g);
    int_to_action_array_g(act2, act_array2, NUM_ROBOT_ARMS_g);

    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num ++) {
        if ((act_array1[arm_num] == stay && act_array2[arm_num] != stay ) ||
        (act_array2[arm_num] == stay && act_array1[arm_num] != stay )) {
            return false;
        } 
        bool isReverse = ((act_array1[arm_num] == xRight && act_array2[arm_num] == xLeft) ||
            (act_array1[arm_num] == xLeft && act_array2[arm_num] == xRight) ||
            (act_array1[arm_num] == yUp && act_array2[arm_num] == yDown) ||
            (act_array1[arm_num] == yDown && act_array2[arm_num] == yUp) ||
            (act_array1[arm_num] == thetaUp && act_array2[arm_num] == thetaDown) ||
            (act_array1[arm_num] == thetaDown && act_array2[arm_num] == thetaUp));
        return isReverse; // return immediately as only 1 non stay action. 
    }

    return false;
}


int total_num_actions_g() {
    /*
    * The total number of actions possible for the robot. - this is what is returned by SurgicalDespot's NumActions functions and the max number that should be input to 
    * int_to_action_array
    */ 
    return 6*NUM_ROBOT_ARMS_g;
}


bool cmp_floats_g(float a, float b) {
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

int random_number_to_index_g(const double probabilityDistrib[], int probabilityDistrib_size, double randomNum) {
    /*
    * Used for deterministic observations. Returns the that results from using the specified random number 
    * with the given discrete probability distribution. 
    * args:
    *   - probabilityDistrib: probability distribution - the index in the array corresopnds to the probability associated with that index
    *                         when being selected using the random number
    *   - probabilityDistrib_size: the size of the probabilityDistrib array for memory safety
    *   - randomNum: random number to use to generate the deterministic actions. - NOTE: float (0, 1)
    * return:
    *   - ret_index: index corresponding to the selected index in probabilityDistrib
    */ 

    float trackedProb = 0;
    int ret_index = -1;
    for (int index = 0; index < probabilityDistrib_size; index++) {
        if (index == 0) {
            trackedProb = probabilityDistrib[0];
        } else {
            trackedProb += probabilityDistrib[index];
        }
        // ensure that an action with zero probability id not being selected
        if (randomNum <= trackedProb && probabilityDistrib[index] != 0) {
            // this is the index to return 
            ret_index = index;
            return ret_index;
        }
    }
    return ret_index;
}

} // end namespace despot