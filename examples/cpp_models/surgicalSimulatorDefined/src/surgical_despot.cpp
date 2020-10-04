/*
* Description: This file creates the class functions for the class described in "surgical_despot.h" - function descriptions for the environment to despot interface
*/

#include "surgical_despot.h"

using std::cerr;
using std::cout;
using std::endl;

namespace despot {

/*
* ***********************************************************************************
* SurgicalDespot Functions: declared in surgical_despot.h
* ***********************************************************************************
*/

SurgicalDespot::SurgicalDespot() {
    /*
    * Default constructor function 
    */ 
}

int SurgicalDespot::NumActions() const{
    /*
    * NOTE: Change this to alter the set of allowable actions you want to use 
    * This function returns the number of possible actions that can be taken in the environment. 
    * There are 7 actions that each arm can take - 0: xRight, 1: xLeft, 2: yUp, 3: yDown, 4:thetaUp, 5:thetaDown, 6:stay
    * 
    * In addition we only allow one arm to move at a time for now. 
    * However the action of all arms staying in the same place is not allowed. 
    * 
    * Methodology: 
    * eg: 2 arms - 0, 1, 2, 3, 4, 5 - respective actions on the first arm and no action on the second arm
    * 6, 7, 8, 9, 10, 11 - subtract 6 and the respective action is then used for the second arm and no action on the first
    */
    return 6*NUM_ROBOT_ARMS_g;
}

void SurgicalDespot::IntToActions(int actionNum, robotArmActions *ret_action_array, int ret_action_array_size) const {
    /*
    * Convert specified integer to an array of actions - where the action at index i corresponds to the action the robot's ith robot arm will take.
    * returns: by pass by reference:
    *   - the array of robot actions to take
    */ 
    if (ret_action_array_size != NUM_ROBOT_ARMS_g) {
        cerr << "ERROR: invalid return array size in IntToActions" << endl;
        exit(1);
    }

    int num_actions_per_arm = 6; // as we do not include the stay action for these purposes
    int arm_to_move_index = static_cast<int>(actionNum/num_actions_per_arm);
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        if (arm_num == arm_to_move_index) {
            ret_action_array[arm_to_move_index] = static_cast<robotArmActions>(actionNum%num_actions_per_arm);
        }   
        ret_action_array[arm_num] = stay;
    }
    return;
}




} // end namespace despot