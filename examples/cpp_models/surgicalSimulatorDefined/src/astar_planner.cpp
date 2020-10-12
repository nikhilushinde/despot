#include "astar_planner.h"

/*
* This file includes the functions for the a star planner class defined in 
* astar_planner.h
*/

using std::max;
using std::cout;
using std::endl;
using std::cerr;
using std::find;

namespace despot {

astar_planner::astar_planner() {
    /* 
    * the constructor for astar_planner 
    */ 
    open_set_m = pqdict_env_double(false); // initialize it to be a min pq

}

double astar_planner::heuristic(const environment &environment_state) const {
    /*
    * Finds the heurisitic cost given the current state of the environment. 
    * Specifically, it computes the least cost to reach the goal, and the minimum 
    * accurued cost to move all the obstacles away from the vicinity of the goal
    * args:
    *   - environment_state: the state for which to calculate the heuristic
    * returns:
    *   - heuristic value 
    */
    
    double min_deformation_cost;
    double deformation_cost;
    double delta_x, delta_y;
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        delta_x = environment_state.robObj_m.arms_m[arm_num].get_x() - environment_state.get_goal_coord().x;
        delta_y = environment_state.robObj_m.arms_m[arm_num].get_y() - environment_state.get_goal_coord().y;

        deformation_cost = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        if (arm_num == 0) {
            min_deformation_cost = deformation_cost;
        } else if (deformation_cost < min_deformation_cost) {
            min_deformation_cost = deformation_cost;
        }
    }

    return max(static_cast<double>(0), min_deformation_cost - environment_state.get_goal_radius());
}

void astar_planner::get_children(const environment &parent_state, environment *ret_child_states, ACT_TYPE *corresponding_act_nums, double *corresponding_costs, int &num_children_returned, int ret_array_size) const {
    /*
    * Gets the children of a given parent state
    * args:
    *   - ret_child_states: the array of child states 
    *   - corresponding_act_nums: array containing the action types that correspond to the child state at that same index in ret_child_states
    *                            these are the actions that the parent took to get to the corresponding child states
    *   - num_children_returned: the number of children in the list that are populated with proper child states
    *   - ret_array_size: the size of the return array for memory purposes.
    * 
    * Methodology:
    *   - make copies of the parent state and step them. Only add feasible actions that do not error to the list of children.  
    */
   
    if (ret_array_size < total_num_actions_g()) {
        cerr << "Error: in get children the return array size was too small";
        exit(1);
    }

    bool error = false;
    float cost = 0;
    robotArmActions action_array[NUM_ROBOT_ARMS_g];

    int ret_count_num = 0;

    for (int act_num = 0; act_num < total_num_actions_g(); act_num++) {
        environment potential_child_environment = parent_state;
        int_to_action_array_g(act_num, action_array, NUM_ROBOT_ARMS_g);
        potential_child_environment.step(action_array, error, cost);

        if (!error) {
            ret_child_states[ret_count_num] = potential_child_environment;
            corresponding_act_nums[ret_count_num] = act_num;
            corresponding_costs[ret_count_num] = static_cast<double>(cost);
            ret_count_num++;
        }
    }

    num_children_returned = ret_count_num;

}

bool astar_planner::is_goal(const environment &environment_state) {
    /*
    * checks if the specified environment state is at a goal state
    * args:
    *   - environment_state to check
    * returns:
    *   - true if at a terminal state
    */ 
    return environment_state.at_goal();
}

void astar_planner::get_path(vector<ACT_TYPE>& ret_all_path_actions){
    /*
    * Run this after running the plan_a_star function. Returns the path from the start_environment_state
    * to a goal state based on the dictionaries/values found in the plan_a_star function. 
    * returns: pass by reference
    *   - ret_all_path_actions: vector that contains all the actions in forward order to get to the goal state. 
    */ 

    if (closed_set_m.empty()) {
        cerr << "ERROR: In plan a star get path: get_path called without first planning" << endl;
        exit(1);
    }

    environment current_state_ptr;
    current_state_ptr = goal_state_m;

    while (true) {
        if (start_state_m == current_state_ptr) {
            return;
        }
        // the parent dict stores the parent of the key state and the action taken to get from the parent to the key state. 
        ret_all_path_actions.insert(ret_all_path_actions.begin(), parent_dict_m[current_state_ptr].second);
        current_state_ptr = parent_dict_m[current_state_ptr].first;
    }

}

void astar_planner::plan_a_star(const environment &start_environment_state) {
    /*
    * Does the entire A star planning algorithm.
    * args:
    *   - start_environment_state: the environment object indicating the starting state that a star plans from 
    * 
    * Methodology:
    *   - Does A star planning - uses the cost as a metric to get the value of each state so the COST IS USED FOR THE VALUES not reward
    */
    bool verbose = true;

    // clear everything
    g_dict_m.clear();
    parent_dict_m.clear();
    open_set_m.clear();
    closed_set_m.clear();

    // Initialize the first values in all the dictionaries 
    g_dict_m[start_environment_state] = 0.0;
    parent_dict_m[start_environment_state].first = start_environment_state;
    parent_dict_m[start_environment_state].second = FALSE_ACTION;

    open_set_m.set(start_environment_state, 0.0);
    
    start_state_m = start_environment_state;
    bool goal_reached = false;

    int iter_count = 0;
    environment current_state;
    double current_val;

    int ret_child_array_size = total_num_actions_g();
    int num_returned_children;
    environment returned_child_states[ret_child_array_size];
    double returned_child_costs[ret_child_array_size];
    ACT_TYPE returned_child_act_nums[ret_child_array_size];
    
    int open_set_true_size = 1;

    while(!goal_reached) {
        if (iter_count % 1000 == 0 && verbose) {
            cout << "Iteration: " << iter_count << endl;
            //open_set_m.printChecker();
            cout << "closed set size: " << closed_set_m.size() << endl;
        }

        if (open_set_m.empty()) {
            cerr << "ERROR: no path found, open set is empty" << endl;
            exit(1);
        }

        // pop top of open set and add to closed set
        current_state = open_set_m.top_key_m;
        current_val = open_set_m.top_val_m;
        open_set_m.erase(current_state);
        closed_set_m.push_back(current_state);

        if (is_goal(current_state)) {
            goal_reached = true;
            goal_state_m = current_state;
            break;
        }

        get_children(current_state, returned_child_states, returned_child_act_nums, returned_child_costs, num_returned_children, ret_child_array_size);

        for (int child_num = 0; child_num < num_returned_children; child_num++ ) {
            // if the child is not in the closed set
            if (find(closed_set_m.begin(), closed_set_m.end(), returned_child_states[child_num]) == closed_set_m.end() ) {
                double new_cost = g_dict_m[current_state] + returned_child_costs[child_num];
                
                // if child not in g initialize it and if the new cost is better update the dictionaries accordingly
                if (g_dict_m.find(returned_child_states[child_num]) == g_dict_m.end()) {
                    g_dict_m[returned_child_states[child_num]] = new_cost;
                    parent_dict_m[returned_child_states[child_num]].first = current_state;
                    parent_dict_m[returned_child_states[child_num]].second = returned_child_act_nums[child_num];
                    open_set_m.set(returned_child_states[child_num], new_cost + heuristic(returned_child_states[child_num]));
                } else if (new_cost < g_dict_m[returned_child_states[child_num]]) {
                    g_dict_m[returned_child_states[child_num]] = new_cost;
                    parent_dict_m[returned_child_states[child_num]].first = current_state;
                    parent_dict_m[returned_child_states[child_num]].second = returned_child_act_nums[child_num];
                    open_set_m.set(returned_child_states[child_num], new_cost + heuristic(returned_child_states[child_num]));
                } 
            } 
        }
        
        iter_count++;
    }
}



} // end namespace