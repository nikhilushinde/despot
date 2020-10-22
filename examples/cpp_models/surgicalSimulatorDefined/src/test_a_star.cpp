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
    double all_values_along_path[actions_to_goal.size()];

    cout << "STARTING STATE: " << endl;
    test_environment.printState();
    cout << endl;

    cout << "Printing all values along path: ";
    for (int i = 0; i < actions_to_goal.size(); i++) {
        // step the environment 
        current_action = actions_to_goal[i];
        int_to_action_array_g(current_action, action_array, NUM_ROBOT_ARMS_g);
        test_environment.step(action_array, error, cost);
        // render the environment
        Renderer.render_environment(test_environment, 50);

        all_values_along_path[i] = planner.get_state_cost(test_environment);
        if (current_action == 0) {
			cout << "xRight: ";
		} else if (current_action == 1) {
			cout << "xLeft: ";
		} else if (current_action == 2) {
			cout << "yUp: ";
		} else if (current_action == 3) {
			cout << "yDown: ";
		} else if (current_action == 4) {
			cout << "thetaUp: ";
		} else if (current_action == 5) {
			cout << "thetaDown: ";
		}
        cout << all_values_along_path[i] << ", ";
    }   
    cout << endl << endl;

    cout << "get NON discounted goal cost: " << planner.get_goal_cost() << endl;
    cout << "get discounted value: " << planner.get_discounted_goal_value(actions_to_goal) << endl;
    cout << "the terminal reward alone is: " << TERMINAL_REWARD_g << endl;
    cout << "the discount factor is: " << Globals::Discount(1) << endl;
    cout << "environment is at goal: " << planner.is_goal(test_environment) << endl;
    cout << endl << endl << endl;

    environment env;
    // get the values of all the surrounding states
    for (int action = 0; action < 6; action++) {
        // get the surrounding state and print everything about it 
        cout << "SURROUNDING STATE AFTER ACTION: ";
        if (action == 0) {
			cout << "xRight: ";
		} else if (action == 1) {
			cout << "xLeft: ";
		} else if (action == 2) {
			cout << "yUp: ";
		} else if (action == 3) {
			cout << "yDown: ";
		} else if (action == 4) {
			cout << "thetaUp: ";
		} else if (action == 5) {
			cout << "thetaDown: ";
		}
        cout << endl;

        astar_planner childplanner;

        environment child_env = env;
        int_to_action_array_g(action, action_array, NUM_ROBOT_ARMS_g);
        child_env.step(action_array, error, cost);

        child_env.robObj_m.printState();
        cout << endl;
        childplanner.plan_a_star(child_env, false);
        actions_to_goal.clear();
        childplanner.get_path(actions_to_goal);

        cout << "cost to step to child: " << cost << endl;
        cout << "get NON discounted goal cost: " << childplanner.get_goal_cost() << endl;
        cout << "get discounted value: " << childplanner.get_discounted_goal_value(actions_to_goal) << endl;
        cout << "the terminal reward alone is: " << TERMINAL_REWARD_g << endl;

        cout << endl << endl << endl;
        
    }


    return 0;
}