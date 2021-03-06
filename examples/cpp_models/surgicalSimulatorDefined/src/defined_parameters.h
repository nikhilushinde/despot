#ifndef DEFINED_PARAMETERS_H
#define DEFINED_PARAMETERS_H

/* 
DESCRIPTION: this file contains user defined parameters that set
various parameters of the toy surgical simulator environment. 

NOTE: All variables defined in this file end in "_g" to 
indicate that they are globally defined varibles. 
*/
#include <string>

namespace despot {

/*
* ********************************************
* Non-modifiable parameters: parameters that should not be modified by the user
* ********************************************
*/
// Deflection direction constants: Used to define the relationship between arms and obstacles
enum deflectionDirection {
    obs_below = -1, // the obstacle deflects above the robot arm 
    obs_none = 0, // the obstacle deflects below the robot arm 
    obs_above = 1 // the obstacle has not established any relationship to the robot arm 
};
/*
* ********************************************
* Non-modifiable parameters: parameters that should not be modified by the user
* ********************************************
*/

// define the global parameters in this file
/* 
* ********************************************
* Environment characteristics 
* ********************************************
*/
#define ENV_LENGTH_g 200 // environment length
#define ENV_HEIGHT_g 200 // environment height
#define GOAL_X_g 160 // x coordinate of the goal location 
#define GOAL_Y_g 100 // y coordinate of the goal location
#define GOAL_RADIUS_g 30 // radius of goal location centered at (GOAL_X_g, GOAL_Y_g)
#define XY_STEP_SIZE_g 15 // size of the x,y steps that the robot arm can take 
#define THETA_DEG_STEP_SIZE_g 5 // size of the theta step size in degrees

#define DEFAULT_DEBUG_FLAG_g false // set the  debug flag for all the objects in the environment

/* 
* ********************************************
* Obstacle characteristics 
* ******************************************** 
*/
#define NUM_OBSTACLES_g 2 // number of obstacles in the environment
#define NUM_OBS_K_CLASSES_g 5 // number of possible classes for the k values of obstacles - NOTE MUST BE GREATER THAN 1
#define NUM_OBS_CenterY_CLASSES_g 5 // number of possible classes for the initial y positions of the obstacles
#define OBSTACLE_LIMIT_FLAG_g false // boolean indicating if the obstacles are to be limited vertically by the environment
static const float all_possible_obs_ks_g[NUM_OBS_K_CLASSES_g] = {1e-11, 10, 100, 10000, 1e6};;//{0.00000000001, 10, 100, 10000, 100000};//{0, 1, 5, 10, 100};//{0.001, 1, 5, 10, 100};//{0.001, 1, 10,  1000,  5000}; // values of all possible k 
static const float all_possible_obs_ys_g[NUM_OBS_CenterY_CLASSES_g] = {180, 30, 100, 110, 150}; // values of all possible initial y centers of the obstacles
#define DEFAULT_NOTOBSERVED_OBS_K_g 0 // default value to use for obstacle k when the obstacle is not observed. - should not be contained in all_possible_obs_ks_g

// // used for obstacle initialization
// #define OBSTACLE_RADIUS_g 30 // define single obstacle radius for all the obstacles
// //static const int all_obstacle_radiuses_g[NUM_OBSTACLES_g] = {30, 30, 30}; // the radiuses of each obstacle
// static const float all_obstacle_ks_g[NUM_OBSTACLES_g] = {10000, 10000, 100000, 10};//{1000, 10, 5000}; // the k value of each obstacle
// static const float all_obstacle_center_x_g[NUM_OBSTACLES_g] = {31, 71, 111, 151}; // the x value of each obstacle
// static const float all_obstacle_center_y_g[NUM_OBSTACLES_g] = {170, 140, 120, 100}; // the initial y value of each obstacle

// used for obstacle initialization
#define OBSTACLE_RADIUS_g 70 // define single obstacle radius for all the obstacles
//static const int all_obstacle_radiuses_g[NUM_OBSTACLES_g] = {30, 30, 30}; // the radiuses of each obstacle
static const float all_obstacle_ks_g[NUM_OBSTACLES_g] = {1e6, 1e-11};// the k value of each obstacle
static const float all_obstacle_center_x_g[NUM_OBSTACLES_g] = {81, 81}; // the x value of each obstacle
static const float all_obstacle_center_y_g[NUM_OBSTACLES_g] = {150, 50}; // the initial y value of each obstacle


// probability of observing the true obstacle class after interactions 
#define TRUE_CLASS_OBSERVATION_PROB_g 1//1//0.9//1//0.9

// Initial Belief bias 
#define INITIAL_BELIEF_TYPE_g "CLASS_SET" // options: {"DEFAULT", "TRUE_CLASS", "CLASS_SET"}
// "DEFAULT" - uniform prior over all obstacle class configurations for particles 

// "TRUE_CLASS" - indicates to usCLEARe the biasing strategy that changes the probability of a particle in the initial belief based on the true obstacle class of the true environment
#define TRUE_INITIAL_BIAS_PROBABILITY_g 0.8  //1//0.5 // dictates the probability of the true class in the biased initial belief

// "CLASS_SET" - indicates to use the initial belief where for every obstacle the classes are biased to some given probability distribution - specified below
// use this for high bias distribution 
static const double class_set_init_belief_biased_distrib_g[NUM_OBS_K_CLASSES_g] = {0.5, 0, 0, 0, 0.5};//{0.05, 0.05, 0.1, 0.1, 0.7};
// use this for low bias distribution
//static const double class_set_init_belief_biased_distrib_g[NUM_OBS_K_CLASSES_g] = {0.7, 0.1, 0.1, 0.05, 0.05};

/* 
* ********************************************
* Robot characteristics 
* ******************************************** 
*/
/*
// 3 arm defined parameters
#define NUM_ROBOT_ARMS_g 3 // the number of robot arms in the environment

// used for robot arm initialization
static const int all_robot_x_g[NUM_ROBOT_ARMS_g] = {0, 0, 0}; // the starting x coordinates of all the robot arms
static const int all_robot_y_g[NUM_ROBOT_ARMS_g] = {0, 10, 20}; // the starting y coordinates of all the robot arms
static const int all_robot_theta_deg_g[NUM_ROBOT_ARMS_g] = {0, 0, 0}; // the starting theta values in degrees of all the robot arms

// Define deflection directions: relates the obstacles and robot arms
static const deflectionDirection all_initial_deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g] = {{obs_none, obs_none, obs_none},
{obs_none, obs_none, obs_none},
{obs_none, obs_none, obs_none}};
*/

// 1 arm defined parameters
#define NUM_ROBOT_ARMS_g 1 // the number of robot arms in the environment

// used for robot arm initialization
static const int all_robot_x_g[NUM_ROBOT_ARMS_g] = {30};//{0}; // the starting x coordinates of all the robot arms
static const int all_robot_y_g[NUM_ROBOT_ARMS_g] = {100};//{80}; // the starting y coordinates of all the robot arms
static const int all_robot_theta_deg_g[NUM_ROBOT_ARMS_g] = {-5}; // the starting theta values in degrees of all the robot arms

// Define deflection directions: relates the obstacles and robot arms
//static const deflectionDirection all_initial_deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g] = {{obs_above}, {obs_above}, {obs_above}, {obs_above}};
static const deflectionDirection all_initial_deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g] = {{obs_none},{obs_none}};

/* 
* ********************************************
* Camera characteristics 
* ******************************************** 
*/
#define CAM_CORRESPONDING_ARM_IDX_g 0
#define CAM_FOV_DEG_g 179
#define CAM_ANGLE_RESOLUTION_DEG_g 15
#define CAM_MAX_DISTANCE_g 50

// calculated camera attributes 
static const float half_fov_deg_g = CAM_FOV_DEG_g/2.0;
static const int cam_num_scan_angles_g = ((int)(half_fov_deg_g/CAM_ANGLE_RESOLUTION_DEG_g)*2) + 1;


// reward related constants - only give the magnitude of the values here. 
static const bool USE_CONSTANT_MOVEMENT_COST_g = true; // defines if the cost of all actions is the same constant
#define CONSTANT_MOVEMENT_COST_g 10 // defines the numerical cost of each action if they are to be constant
// TODO: add the _g suffix to these global variables
#define MOVEMENT_ERROR 10000000 // cost for taking an action that resulted in an out of bounds error or collision error 
#define ROBOT_MOVEMENT_ERROR_COST MOVEMENT_ERROR  // constant cost to return if there was an error in the robot movement 
static const float OBSTACLE_ERROR_COST = MOVEMENT_ERROR; //define constant cost for error in obstacle manipulation

static const double DOUBLE_BACK_PENALTY_g = -1e4; // penalty that is added to the value if the action chosen doubles back 

#define TERMINAL_REWARD_g 1e8//1000000000


// results related parameters
static const std::string results_folder_name_g = "//home/nikhilushinde/Documents/research/arclab/cpp_despot/despot/examples/cpp_models/surgicalSimulatorDefined/results/astar_test/";


/*
NOTE: WHAT ATTRIBUTES TO DELETE FROM OBJECTS AND REPLACE WITH THESE PARAMETERS
- ENVIRONMENT HEIGHT
- ENVIRONMENT LENGTH 
- NUM ROBOT ARMS
- NUM OBSTACLES
- OBSTACLE_LIMIT_g
- NUM SCAN ANGLES


specify the remaining parameters during initialization - trickle down from the high 
level function calls
*/ 

} // end namespace despot


#endif