#include <iostream>
#include "astar_planner.h"
#include "environment.h"
#include <thread>

using std::cout;
using std::endl;
using std::cerr;
using std::vector;

using namespace despot;

static void multi_thread_Astar_getbestaction(const environment high_environment,  const environment both_environment, ACT_TYPE &best_action) {
    /*
    * Function to enable multi thread planning with A star
    * args:
    *   - planning_environment: environment object on which to do A star
    * returns: by pass by reference
    *   - best_value: the upper bound value for the planning environment based on A star.
    */
    

    double action_values[total_num_actions_g()];
    float initial_step_costs[total_num_actions_g()];
    for (int action = 0; action < total_num_actions_g(); action++) {
        astar_planner high_planner;
        astar_planner both_planner; 

        environment high = high_environment;
        environment both = both_environment;
        robotArmActions action_array[NUM_ROBOT_ARMS_g];
        int_to_action_array_g(action, action_array, NUM_ROBOT_ARMS_g);

        bool error; 
        float init_cost = 0;
        high.step(action_array, error, init_cost);
        initial_step_costs[action] += init_cost;
        init_cost = 0;
        both.step(action_array, error, init_cost);
        initial_step_costs[action] += init_cost; 

        high_planner.plan_a_star(high);
        action_values[action] = -high_planner.get_goal_cost() + TERMINAL_REWARD_g;
        both_planner.plan_a_star(both);
        action_values[action] += -both_planner.get_goal_cost() + TERMINAL_REWARD_g;
        action_values[action] += -initial_step_costs[action];

        action_values[action] /= 2;
    }   

    best_action = std::distance(action_values, std::max_element(action_values, action_values + total_num_actions_g()));
    return;
}

int main() {
    astar_planner planner;
    environment high_environment;
    environment both_environment;

    float high_obs_ks[NUM_OBSTACLES_g] = {1e-11, 1e6};//{1e6, 1e6};
    float both_obs_ks[NUM_OBSTACLES_g] = {1e6, 1e-11};
    high_environment.set_obstacle_ks(high_obs_ks);
    both_environment.set_obstacle_ks(both_obs_ks);

    // define the range and number of steps you want in your heatmap of the values
    int start_x = 0;
    int start_y = 10; 
    int start_theta = -10; 

    int num_x_steps = 13;
    int num_y_steps = 12; 
    int num_theta_steps = 5; 

    ACT_TYPE astar_actions[num_x_steps*num_y_steps*num_theta_steps];
    std::vector<std::thread> all_particle_threads;

    const deflectionDirection deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g] = {{obs_above}, {obs_below}};
    int counter = 0;
    for (int theta_num = 0; theta_num < num_theta_steps; theta_num++) {
        for (int x_num = 0; x_num < num_x_steps; x_num++) {
            for (int y_num = 0; y_num < num_y_steps; y_num++) {
                int current_x = start_x + (x_num*XY_STEP_SIZE_g); 
                int current_y = start_y + (y_num*XY_STEP_SIZE_g); 
                int current_theta = start_theta + (theta_num*THETA_DEG_STEP_SIZE_g); 

                robotArmCoords current_coords[NUM_ROBOT_ARMS_g];
                current_coords[0].x = current_x;
                current_coords[0].y = current_y;
                current_coords[0].theta_degrees = current_theta;
                
                bool high_error = high_environment.set_robot_arms_autoset_obstacles(current_coords, deflection_directions);
                bool both_error = both_environment.set_robot_arms_autoset_obstacles(current_coords, deflection_directions);

                // if the coordinate state is invalid just put a -1 for the action as a placeholder and continue
                if (high_error || both_error) {
                    astar_actions[counter] = -1;
                } else {
                    all_particle_threads.push_back(std::thread(multi_thread_Astar_getbestaction, 
                        high_environment, both_environment, std::ref(astar_actions[counter])));
                }
                counter ++;
            }
        }
    }
    
    // join all the threads
    int num_threads = all_particle_threads.size();
    for (int i = 0; i < num_threads; i++) {
        all_particle_threads[i].join();
    }

    cout << "[";
    int print_counter = 0;
    for (int theta_num = 0; theta_num < num_theta_steps; theta_num++) {
        for (int x_num = 0; x_num < num_x_steps; x_num++) {
            for (int y_num = 0; y_num < num_y_steps; y_num++) {
                int current_x = start_x + (x_num*XY_STEP_SIZE_g); 
                int current_y = start_y + (y_num*XY_STEP_SIZE_g); 
                int current_theta = start_theta + (theta_num*THETA_DEG_STEP_SIZE_g); 

                cout << "((" << current_x << ", " << current_y << ", " << current_theta << "), ";
                cout << astar_actions[print_counter] << "), ";
                print_counter ++;
            }
        }
    }
    cout << "]" << endl;

    cout << "total print counter: " << print_counter << endl;



    return 0;
}