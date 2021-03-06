#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include "environment.h"
#include "pq_dict.h"

using std::vector;
using std::map;

/*
* This file defines a planner that does A star in the surgical environment to 
* get the robot arm to the goal. 
*
* NOTE: RESTRICTED TO USE WITH ONLY ONE ROBOT ARM - otherwise will not work. 

* NOTE: IDEA: MAKE A MAP WHEN PROCESSING ALL THE PARTICLES AND IF ANY OF THE PARTICLES ARE THE
* SAME THEN YOU JUST REUSE RATHER THAN REDO EVERYTHING.  
*/
namespace despot {

#define FALSE_ACTION -1 // action where there is no parent child relationship - used at the end of the parent dictionary

class astar_planner {
public:
    astar_planner(); // default constructor

    double heuristic(const environment& environment_state) const; //calculates the heuristic cost given a certain state
    void get_children(const environment &parent_state, environment *ret_child_states, ACT_TYPE *corresponding_act_nums, double *corresponding_costs, int &num_children_returned, int ret_array_size) const; // gets the children of the parent state 
    bool is_goal(const environment &environment_state); // checks if the environment state is a goal state
    void get_path(vector<ACT_TYPE>& ret_all_path_actions); // gets the optimal path from the q values that have been found via a star
    double get_goal_cost(); // gets the non discounted cost of getting to the goal state. 
    double get_discounted_goal_value(vector<ACT_TYPE>& all_path_actions); // computes the discounted value to get to the goal from the start state. 
    void plan_a_star(const environment &start_environment_state, bool verbose = false); // uses a star to plan path to goal. 

    // DEBUGGING functions
    double get_state_cost(const environment &environment_key); //returns the cost stored in g_dict_m of the given state

private: // variables
    environment goal_state_m; // the goal state that was found - assigned in plan_a_star
    environment start_state_m; // the starting state fro which to plan a star

    map<environment, double> g_dict_m; // contains the value of the key state
    map<environment, std::pair<environment, ACT_TYPE>> parent_dict_m;// the value is the parent of the key state and the action taken from the parent to get to the key state
    pqdict_env_double open_set_m; // set of open states in the A star algorithm as the key with their values
    vector<environment> closed_set_m; // vector of states that have already been explored. 
};
} // end namespace despot

#endif