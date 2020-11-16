/*
* This file contains a NAIVE dynamic version of A star that uses rapid replanning and A star to plan in the surgical environment
*
* Methodology: Plan with Astar assuming some parameterization of the environment. Continue along that path until you see some 
* observation that does not align with your parameterization of the environment. For the Naive approach clip your parameterization
* to deterministically be the observed parameterization of the environment and then replan and continue. 
* 
* NOTE: by clipping to the parameter returned by the observation this assumes that the obstacles can only return their true class
* when observed - may cause problems when this is not true and is actually stochastic. 
*/

#include "defined_parameters.h"
#include "surgical_utils.h"
#include "environment.h"
#include "render_sim.h"
#include "astar_planner.h"

using std::cout; 
using std::endl;

namespace despot{
     
class naiveClippedAstarReplanner {

public: 
    naiveClippedAstarReplanner(environment* start_environment) {
        true_environment = start_environment;
    }

    void printAction(ACT_TYPE action, float step_value) {
        /*
        * Prints the action and its step cost to the terminal output
        */  
        cout << "Selected action: "; 
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
        } else {
            cout << "unknown action: " << action;
        }

        cout << ", step value: " << step_value << endl;
    } 

    void printObstacleK(float obstacle_ks[NUM_OBSTACLES_g]) {
        /*
        * Prints the list of the obstacle ks
        */ 
        cout << "The obstacle ks are: "; 
        for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
            cout << obstacle_ks[obs_num] << ", ";
        }
        cout << endl;
    }


    void planOnline(float init_obstacle_ks[NUM_OBSTACLES_g]) {
        /*
        * Planning function that plans with Astar, follows policy online until it gets an observation that contradicts
        * the deterministic parameters that it calculated its Astar path with. Only observing it sets its parameters to 
        * match its observations, replans, and then continues. 
        */ 

        // Initialize attributes
        astar_planner planner;
        render_sim renderer; 
        int render_wait_time = 100;
        bool planner_verbosity = false;

        // initialize constants
        double rand_max = 1; 
        double rand_min = 0;
        int rand_seed = 13;
        // seed random number generator
        //srand(rand_seed);

        // initialize planning environment and current obstacle ks
        float current_obstacle_ks[NUM_OBSTACLES_g] = {0};
        for (int i = 0; i < NUM_OBSTACLES_g; i++) {
            current_obstacle_ks[i] = init_obstacle_ks[i];
        }
        environment planning_environment = environment(*true_environment);
        planning_environment.set_obstacle_ks(current_obstacle_ks);

        bool replan = true; 
        renderer.render_environment(*true_environment, render_wait_time);
        std::vector<ACT_TYPE> action_plan; 

        float total_value = 0; 
        while (!true_environment->at_goal()) {
            if (replan || action_plan.empty()) {
                // use Astar to plan in the plannning_environment
                planner.plan_a_star(planning_environment, planner_verbosity);
                planner.get_path(action_plan);
                replan = false;
            }

            // pop the first action
            ACT_TYPE current_action = action_plan.front();
            action_plan.erase(action_plan.begin());
            robotArmActions action_array[NUM_ROBOT_ARMS_g];
            int_to_action_array_g(current_action, action_array, NUM_ROBOT_ARMS_g);

            // step in the environment
            bool error = false;
            float step_cost = 0;
            float observed_obstacle_classes[NUM_OBSTACLES_g];
            true_environment->step(action_array, error, step_cost);
            // observe
            double rand_num = (double) rand() / RAND_MAX * (rand_max - rand_min) + rand_min;
            true_environment->observe_classes(observed_obstacle_classes, NUM_OBSTACLES_g, rand_num);
            // track the value
            total_value += -(step_cost);
            if (true_environment->at_goal()) {
                total_value += TERMINAL_REWARD_g;
                printAction(current_action, TERMINAL_REWARD_g - step_cost); 
            } else {
                printAction(current_action, -step_cost);
            }
            cout << "Observation: "; 
            printObstacleK(observed_obstacle_classes);

            renderer.render_environment(*true_environment, render_wait_time);

            // also step the planning environment
            float plan_cost;
            bool plan_error;
            planning_environment.step(action_array, plan_error, plan_cost);

            for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
                if (observed_obstacle_classes[obs_num] != DEFAULT_NOTOBSERVED_OBS_K_g && 
                    observed_obstacle_classes[obs_num] != current_obstacle_ks[obs_num]) {
                    // set replan to true and set the environment obs ks to correspond to those seen, leaving unobserved obstacles the same
                    replan = true;
                    for (int i = 0; i < NUM_OBSTACLES_g; i++) {
                        if (observed_obstacle_classes[i] != DEFAULT_NOTOBSERVED_OBS_K_g) {
                            current_obstacle_ks[i] = observed_obstacle_classes[i];
                        }
                    } 
                    planning_environment.set_obstacle_ks(current_obstacle_ks);
                    action_plan.clear(); 
                    break;
                }
            }
            cout << "Planning environment obstacle ks: "; 
            printObstacleK(current_obstacle_ks);

            cout << endl << endl;
        } // end while loop

    cout << "The total value of all the actions is: " << total_value << endl;
    }

private: 
    environment* true_environment; 
};
}

using namespace despot; 
int main() {
    environment true_environment; 
    float init_obstacle_ks[NUM_OBSTACLES_g] = {0};//{1e-11, 1e6, 1e-11, 1e-11}; 
    naiveClippedAstarReplanner replanner = naiveClippedAstarReplanner(&true_environment);
    replanner.planOnline(init_obstacle_ks);

    return 0;
}
