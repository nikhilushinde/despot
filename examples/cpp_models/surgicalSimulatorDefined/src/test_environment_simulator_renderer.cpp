#include "robot_arm.h"
#include "robot.h"
#include "obstacle.h"
#include "environment.h"
#include "surgical_utils.h"
#include "render_sim.h"
#include "camera.h"

using namespace std;
using namespace despot;

#define CHAR_TO_NUM_SUBTRACTOR 48 // subtract this from a character number that was entered to get the real number it corresponds to ( 0 = 48 )

int main() {
    /*
    All variables for the environment object initialization: 
    int height, int length, environmentCoords goal_coord, int goal_radius, int xy_step_size, 
    int theta_deg_step_size, int num_robot_arms, robotArmCoords *robot_arm_start_coords, bool obstacle_limit, 
    int obstacle_radius, int num_obstacles, float *obstacle_ks, float *obstacle_center_x, float *obstacle_center_y, 
    deflectionDirection **deflection_directions
    */
    bool error = false;

    // all variables needed to initialize the environment
    int environment_height = 200;
    int environment_length = 200;
    environmentCoords goal_coord;
    goal_coord.x = 170;
    goal_coord.y = 150;
    int goal_radius = 30;

    // all variables needed to initialize the environment robot
    int xy_step_size = 5;
    int theta_deg_step_size = 5;
    int num_robot_arms = 3;
    robotArmCoords robot_arm_start_coords[num_robot_arms];
    for (int i = 0; i < num_robot_arms; i++) {
        robot_arm_start_coords[i].x = 0;
        robot_arm_start_coords[i].y = i*10;
        robot_arm_start_coords[i].theta_degrees = 0;
    }
    
    // all variables needed to initialize the environment robot
    bool obstacle_limit = false;
    int obstacle_radius = 30;
    
    int num_obstacles = 3;
    float obstacle_ks[3] = {1, 1, 1};
    float obstacle_center_x[3] = {31, 95, 160};
    float obstacle_center_y[3] = {100, 75, 50};
    
    /*
    int num_obstacles = 0;
    float obstacle_ks[0];
    float obstacle_center_x[0];
    float obstacle_center_y[0];
    */
    deflectionDirection **init_deflection_directions;
    // MEM: ALLOC
    init_deflection_directions = new deflectionDirection *[num_obstacles];
    for (int obsnum = 0; obsnum < num_obstacles; obsnum++) {
        // MEM: ALLOC
        init_deflection_directions[obsnum] = new deflectionDirection[num_robot_arms];
        for (int armnum = 0; armnum < num_robot_arms; armnum++) {
            // initialize all the deflection directions to be none to start with 
            init_deflection_directions[obsnum][armnum] = obs_none;
        }
    }

    // camera parameters
    int cam_corresponding_arm_index = 0;
    float cam_fov_degrees = 180;
    float cam_angle_resolution_deg = 15;//30;
    float cam_max_distance = 50;

    cout << "Starting environment initialization" << endl;

    // initialize the environment object
    environment env;
    /*
    error = env.init_environment(environment_height, environment_length, goal_coord, goal_radius, xy_step_size, theta_deg_step_size,
    num_robot_arms, robot_arm_start_coords, obstacle_limit, obstacle_radius, num_obstacles, obstacle_ks, obstacle_center_x, 
    obstacle_center_y, init_deflection_directions, cam_corresponding_arm_index, cam_fov_degrees,
    cam_angle_resolution_deg, cam_max_distance);
    */

    cout << "Initialized the environment" << endl;

    // delete the init_deflection_directions pointers
    for (int obsnum = 0; obsnum < num_obstacles; obsnum++) {
        // MEM: DELETE
        delete init_deflection_directions[obsnum];
    }
    // MEM: DELETE
    delete init_deflection_directions;

    if (error) {
        cout << "ERROR: There was an error while initializing the environment!!!!" << endl;
        return 1;
    }


    // print the state of the environment
    env.printState();

    // initialize the render_sim object
    render_sim Renderer;
    // render the state 
    Renderer.render_environment(env);

    robotArmActions action;
    robotArmActions default_action[num_robot_arms];
    for (int i = 0; i < num_robot_arms; i++) {
        default_action[i] = stay;
    }

    bool break_game = false;
    float cost = 0;;
    float totalCost = 0;

    int controlled_arm = 0;
    char c;
    robotArmActions act;

    cout << endl << endl;

    while (!break_game) {
        cout << "Enter character: ";
        cin >> c;
        cout << endl;

        if (c == 'w') {
            act = yUp;
            default_action[controlled_arm] = act;
            cout << "Took action: Up" << endl;
        } else if (c == 's') {
            act = yDown;
            default_action[controlled_arm] = act;
            cout << "Took action: Down" << endl;
        } else if (c == 'a') {
            act = xLeft;
            default_action[controlled_arm] = act;
            cout << "Took action: Left" << endl;
        } else if (c == 'd') {
            act = xRight;
            default_action[controlled_arm] = act;
            cout << "Took action: Right" << endl;
        } else if (c == 'k') {
            act = thetaUp;
            default_action[controlled_arm] = act;
            cout << "Took action: Theta Increase" << endl;
        } else if (c == 'l') {
            act = thetaDown;
            default_action[controlled_arm] = act;
            cout << "Took action: Theta Decrease" << endl;
        } else if (c =='x') {
            break_game = true;
            cout << "No action taken: breaking the game loop" << endl;
        } else if (c == '1' || c == '2' || c == '3'){
            controlled_arm = (int) c - CHAR_TO_NUM_SUBTRACTOR - 1; // set index of arm to control
            cout << "Changed control to arm at index: " << controlled_arm << endl;
        } else if (c == 'o') {
            cout << "OBSERVING: " << endl;
            float observed_classes[NUM_OBSTACLES_g];
            double rand_num = static_cast<double>(rand())/static_cast<double>(RAND_MAX);
            cout << "The random number was: " << rand_num << endl;
            env.observe_classes(observed_classes, NUM_OBSTACLES_g, rand_num);
            
            cout << "Observed class index array: ";
            for (int i = 0; i < NUM_OBSTACLES_g; i++) {
                cout << observed_classes[i] << ", ";
            }

            OBS_TYPE obs = env.class_observations_to_obstype(observed_classes, NUM_OBSTACLES_g);
            cout << endl;
            cout << "Observed class as OBS_TYPE:  " <<  obs << endl; 
            cout << endl;

            double obsProb = env.get_class_obs_prob(obs, 0);
            cout << "Probability of the seem observation: " << obs << " is: " << obsProb  << endl;

            cout << "True class number should be: " << env.class_observations_to_obstype(all_obstacle_ks_g, NUM_OBSTACLES_g) << endl;

            continue;
        } else {
            cout << "INVALID ENTRY!!" << endl << endl << endl;
        }

        env.step(default_action, error, cost);

        // reset the default action
        for (int i = 0; i < num_robot_arms; i ++) {
            default_action[i] = stay;
        }

        totalCost += cost;
        env.printState();
        // render the state 
        cout << "Starting the render of the environment" << endl;
        Renderer.render_environment(env);
        cout << "Finished the render of the environment" << endl;

        if (error) {
            cout << "Action resulted in an error" << endl;
            cout << "Action reverted: state back to: " << endl;
        }
        cout << endl;
        cout << "Cost incurred: " << cost << ", Total cost now: " << totalCost << endl;

        cout << "#############################################################################################" << endl << endl;
        error = false;
    }

}