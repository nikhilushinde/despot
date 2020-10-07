/*
* This file describes the "environment" object. This object is the all encompassing object that contains
* the obstacles, robot (which contains the robot_arm objects). It is the overall simulator and interacts
* with commands from the outside world. 
*
* The functions for this class are defined in environment.cpp
*/

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "surgical_utils.h"
#include "robot_arm.h"
#include "robot.h"
#include "obstacle.h"
#include "camera.h"

namespace despot {

// structure that captures the distinguishing information of the state of an environment
struct environmentState {
    robot robObj;
    obstacle obstacles[NUM_OBSTACLES_g];
};

// TODO: REMOVE WHEN DONE TESTING:
typedef uint64_t OBS_TYPE;
typedef int ACT_TYPE;

class environment: public State {
public:
    // constructor
    environment(); // default constructor of the environment
    /* 
    Do not allow non defined parameters to specify the environment as it will lead to conflicts 
    environment(int height, int length, environmentCoords goal_coord, int goal_radius, int xy_step_size, 
        int theta_deg_step_size, int num_robot_arms, const robotArmCoords *robot_arm_start_coords, bool obstacle_limit, 
        int obstacle_radius, int num_obstacles, const float *obstacle_ks, const float *obstacle_center_x, const float *obstacle_center_y, 
        const deflectionDirection deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g], int cam_corresponding_arm_index, int cam_fov_degrees, int cam_angle_resolution_deg, 
        int cam_max_distance, bool &error);
    */
    environment(const environment &environment_to_copy); // copy constructor 

    // destructor
    ~environment(); // destructor to destroy environment object
    
    // init and set functions
    bool init_environment(int height, int length, environmentCoords goal_coord, int goal_radius, int xy_step_size, 
        int theta_deg_step_size, int num_robot_arms, const robotArmCoords *robot_arm_start_coords, bool obstacle_limit, 
        int obstacle_radius, int num_obstacles, const float *obstacle_ks, const float *obstacle_center_x, const float *obstacle_center_y, 
        const deflectionDirection deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g], int cam_corresponding_arm_index, int cam_fov_degrees, int cam_angle_resolution_deg, 
        int cam_max_distance); // initialize the environment after calling the default constructor
    void init_state(environmentState x); // initialize the environment by by setting the environment's robot and obstacles to this environment
    
    // get state
    void get_state(environmentState &x) const; // get the current state of the environment returned by pass by value as an environmentState
    void delete_state(environmentState &x) const;// delete the dynamically allocated parts of the state of the environment
    float get_intersection_area() const; // the intersection of the obstacles with the goal area to calculate area based costs
    environmentCoords get_goal_coord() const;
    int get_goal_radius() const;

    // robot related get functions
    int get_num_robot_arms() const;
    int get_cam_num_scan_angles() const;
    void get_all_robot_arm_coords(robotArmCoords *ret_robot_arm_coords, int ret_array_len) const; // returns all the coordinates of the robot arm end effectors 
    void get_all_robot_start_end_pts(environmentCoords *ret_start_pts, environmentCoords *ret_end_pts, int ret_array_len) const; // return an array of the robot arms' start and end points - the index in the array corresponds to which robot arm in the robot

    // obstacle related get functions
    int get_obstacle_radius() const;
    int get_num_obstacles() const;
    void get_obstacle_ks(float *ret_obstacle_ks, int ret_array_len) const; // returns the obstacle k values via pass by value - array must be num_obstacles
    void get_current_obstacle_centers(environmentCoords *ret_obstacle_centers, int ret_array_len) const;

    void observe_points(cameraIntersectionPoint *ret_cam_intersection_points, int ret_array_size) const; // observes using the camera's scan_environment function and returns intersectoin points with the environment and the intersection types
    void observe_dists(cameraIntersectionDistance *ret_cam_intersection_dists, int ret_array_size) const; // observes using the camera's scan_environment_dists function and returns the intersection distances and types 
    int randomNumToInt(const double probabilityDistrib[], int probabilityDistrib_size, double randomNum) const; // to convert a random number to an integer based on a discrete probability distribution 
    void observe_classes(float *ret_observed_classes, int ret_array_size, double rand_num) const; //observes the classes of the obstacles that the robot just moved in the last round.  
    OBS_TYPE class_observations_to_obstype(const float observed_obstacle_classes[], int array_size) const; // convert obstacle observations to an integer/obs_type for despot
    double get_class_obs_prob(OBS_TYPE obs, ACT_TYPE action) const; // get the probability of the observation given the observation and the action to get the current state
    // other functions

    // set functions 
    void set_obstacle_ks(const float obstacle_ks[NUM_OBSTACLES_g]); // sets the obstacle ks of the obstacles in the environment
    bool set_robot_arms_autoset_obstacles(const robotArmCoords new_arm_states[NUM_ROBOT_ARMS_g], const deflectionDirection deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g]); // sets the robot arm positions and the obstacles conform with it based on deflection directions

    // state mutation function
    void step(const robotArmActions *actions, bool &error, float &cost); //step the robot in the environment using the given actions
    void state_rollback(); // rollback the state of the environment - rollback each obstacle and the robot
    bool reset(); // reset the positions of things in the environment

    // state checking functions
    bool at_goal() const; // checks if the environment is in a goal state
    void printState() const; // print the state of the environment


    // variable attributes 

    float obstacle_center_x_m[NUM_OBSTACLES_g]; // list of the x values of the centers of all the obstacles
    float obstacle_center_y_m[NUM_OBSTACLES_g]; // list of the original y values of the centers of all the obstacles

private:
    /* NOTE: End everything in _m to indicate that these variables are attributes the environment class */
    // environment attributes 
    int environment_height_m;
    int environment_length_m;

    environmentCoords goal_coord_m; // the location of the goal in the environment
    int goal_radius_m; // radius that defines the goal area around the goal_coord. 

    // robot and robot arm attributes
    int xy_step_size_m;
    int theta_deg_step_size_m;
    int num_robot_arms_m; // the number of robot arms that the robot has 
    robotArmCoords robot_arm_start_coords_m[NUM_ROBOT_ARMS_g]; // list of the starting coordinates for all the robot arms in the robot
    robot robObj_m; // the actual robot object

    // obstacle attributes 
    bool obstacle_limit_m; // boolean flag that if true limits obstacles so that part of the obstacles cannot be out of bounds
    int obstacle_radius_m; // radius of the obstacle 
    int num_obstacles_m; // the number of obstacles in the environment
    float obstacle_ks_m[NUM_OBSTACLES_g]; // list of the k value of each of the obstacles
    deflectionDirection init_deflection_directions_m[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g]; // the initial deflection directions used to initialize the obstacles
    obstacle obstacles_m[NUM_OBSTACLES_g]; // list of all the obstacle objects that are in the environment
    bool class_observable_obstacles_m[NUM_OBSTACLES_g]; // boolean list where true indicates that the obstacle's class is observable due to interaction in the last time step. 

    bool debug_m; // the boolean debugging flag
    camera cam_m; // the camera object. 
};

} // end namespace despot

#endif