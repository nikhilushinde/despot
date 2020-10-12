#include "astar_planner.h"
#include "render_sim.h"

/*
* This file contains test code to test the a star planner for the environment class
*/
using namespace despot;
using std::vector;

int main() {
    environment test_environment;
    astar_planner planner;

    planner.plan_a_star(test_environment);
    vector<ACT_TYPE> actions_to_goal;
    planner.get_path(actions_to_goal);

    // render the path to the goal
    // initialize the render_sim object
    render_sim Renderer;
    // render the state 
    Renderer.render_environment(test_environment);

    robotArmActions action_array[NUM_ROBOT_ARMS_g];
    ACT_TYPE current_action;
    bool error; 
    float cost;
    for (int i = 0; i < actions_to_goal.size(); i++) {
        // step the environment 
        current_action = actions_to_goal[i];
        int_to_action_array_g(current_action, action_array, NUM_ROBOT_ARMS_g);
        test_environment.step(action_array, error, cost);
        // render the environment
        Renderer.render_environment(test_environment, 50);
    }   

    return 0;
}