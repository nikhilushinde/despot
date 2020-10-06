/*
* Description: This file creates the class functions for the class described in "surgical_despot.h" - function descriptions for the environment to despot interface
* NOTE: this file only handles class uncertainty. 
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


bool SurgicalDespot::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const {
    /*
    * Deterministic simulative model that simulates the state taking a determinsitic step with the action and observing
    * the environment with its observation model. This is made deterministic by the use of the specified random number. 
    * 
    * args:
    *   - state: the environment state that you are taking the step in: here an object of class environment
    *   - rand_num: random number 
    *   - action: a number: [0, this->NumActions()) representing an action in the environment
    * return: by pass by reference:
    *   - reward: the reward from that action
    *   - obs: observation - must be an integer
    */ 

    // set up
    environment *environment_state = static_cast<environment*>(&state);
    robotArmActions action_list[NUM_ROBOT_ARMS_g];
    int actionNum = static_cast<int>(action);
    IntToActions(actionNum, action_list, NUM_ROBOT_ARMS_g);
    
    // environment step
    bool error = false; // returns if the action caused an error 
    float step_cost = 0;
    environment_state->step(action_list, error, step_cost);
    
    // environment observe
    float observed_obstacle_classes[NUM_OBSTACLES_g];
    environment_state->observe_classes(observed_obstacle_classes, NUM_OBSTACLES_g, rand_num);

    // convert observation to integer 
    obs = environment_state->class_observations_to_obstype(observed_obstacle_classes, NUM_OBSTACLES_g);
    
    // set reward and terminal status
    bool at_terminal_state = false;
    if (environment_state->at_goal()) {
        at_terminal_state = true;
        reward = TERMINAL_REWARD - step_cost;
    } else {
        reward = 0 - step_cost;
    }

    return at_terminal_state;

}


double SurgicalDespot::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {
    /*
    * This function returns the probability of seeing observation obs from state s given that 
    * we got to state s using action a. 
    * args:
    *   - obs: the observation seen from state gotten to by action
    *   - state: state from which finding the probability of observing obs
    *   - action: action taken to get to state
    */ 

   // TODO: FINISH THIS FUNCTION
}


State* SurgicalDespot::CreateStartState(std::string type = "DEFAULT") const {
    /*
    * Creates a random start state of type "environment". This state is allocated
    * in dynamic memory and the pointer to this state is returned. 
    * args:
    *   - type: string indicating the type of method used to generate the random start state 
    *           can be used for different priors on the random start state
    * returns:
    *   - pointer of type state to the random state object that was created in dynamic memory
    */ 

   // TODO: FINISH THIS FUNCTION
}

Belief* SurgicalDespot::InitialBelief(const State* start, std::string type = "DEFAULT") const {
    /*
    * Creates the initial belief for the problem. 
    * args:
    *   - start: a start state that can be used to bias the initial belief
    *   - type: a string that can be used to select different types of priors for the belief
    * returns:
    *   - belief: The type belief is in the despot code
    */ 

    // TODO: FINISH THIS FUNCTION
}





} // end namespace despot