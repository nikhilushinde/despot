#include "astar_planner.h"
#include "render_sim.h"

/*
* This file contains test code to test the a star planner for the environment class
*/
using namespace despot;
using std::vector;
using std::cout;
using std::endl;

int main() {
    environment test_environment;
    astar_planner planner;

    planner.plan_a_star(test_environment, true);
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

    cout << "get NON discounted goal cost: " << planner.get_goal_cost() << endl;
    cout << "get discounted value: " << planner.get_discounted_goal_value(actions_to_goal) << endl;
    cout << "the terminal reward alone is: " << TERMINAL_REWARD_g << endl;
    cout << "the discount factor is: " << Globals::Discount(1) << endl;
    cout << "environment is at goal: " << planner.is_goal(test_environment) << endl;

    return 0;
}