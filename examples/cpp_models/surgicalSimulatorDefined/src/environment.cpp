/*
* This file contains function descriptions for the "environment" object.
* Comment flags used:
*   - MEM:ALLOC - places that dynamic memory is being explicitly allocated on the heap
*   - MEM:DELETE - places that dynamic memory is being explicitly deleted from the heap 
*/
#include "environment.h"

using std::cout;
using std::cerr;
using std::endl;
using std::min;

namespace despot {

/*
* ********************************************************************************
* BEGIN: Environment CONSTRUCTORS
* ********************************************************************************
*/
environment::environment() {
    /*
    * This is the default constructor of the environment object. Initializes all numerical attributes to zero
    * and all lists and pointers to point to the null pointer.
    * 
    * This initializes the environment to 
    *  
    * NOTE: the init_environment function must be called after this to use the environment object properly 
    */ 

    environmentCoords goal_coord;
    goal_coord.x = GOAL_X_g;
    goal_coord.y = GOAL_Y_g;
    
    // initialize the robot related lists for initialization
    robotArmCoords robot_arm_start_coords[NUM_ROBOT_ARMS_g];
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        robot_arm_start_coords[arm_num].x = all_robot_x_g[arm_num];
        robot_arm_start_coords[arm_num].y = all_robot_y_g[arm_num];
        robot_arm_start_coords[arm_num].theta_degrees = all_robot_theta_deg_g[arm_num];
    }

    // initialize the obstacle related lists for initialization
    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        
    }


    bool error = init_environment(ENV_HEIGHT_g, ENV_LENGTH_g, goal_coord, GOAL_RADIUS_g, XY_STEP_SIZE_g, THETA_DEG_STEP_SIZE_g, 
    NUM_ROBOT_ARMS_g, robot_arm_start_coords, OBSTACLE_LIMIT_FLAG_g, OBSTACLE_RADIUS_g, NUM_OBSTACLES_g, all_obstacle_ks_g, 
    all_obstacle_center_x_g, all_obstacle_center_y_g, all_initial_deflection_directions, CAM_CORRESPONDING_ARM_IDX_g, CAM_FOV_DEG_g, 
    CAM_ANGLE_RESOLUTION_DEG_g, CAM_MAX_DISTANCE_g);

    if (error) {
        cerr << "There was an error in initializing the environment with the default parameters in the defined parameters file" << endl;
        exit(1);
    }
}

/*
environment::environment(int height, int length, environmentCoords goal_coord, int goal_radius, int xy_step_size, 
        int theta_deg_step_size, int num_robot_arms, const robotArmCoords *robot_arm_start_coords, bool obstacle_limit, 
        int obstacle_radius, int num_obstacles, const float *obstacle_ks, const float *obstacle_center_x, const float *obstacle_center_y, 
        const deflectionDirection deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g], int cam_corresponding_arm_index, int cam_fov_degrees, int cam_angle_resolution_deg, 
        int cam_max_distance, bool &error) {
    // Full constructor of the environment that initializes the object completely. 
    // returns:
    //   - error: by pass by reference. True if there is an error that has occurred while initializing the environment object
    

    error = init_environment(height, length, goal_coord, goal_radius, xy_step_size, theta_deg_step_size, 
    num_robot_arms, robot_arm_start_coords, obstacle_limit, obstacle_radius, num_obstacles, obstacle_ks, 
    obstacle_center_x, obstacle_center_y, deflection_directions, cam_corresponding_arm_index, cam_fov_degrees, 
    cam_angle_resolution_deg, cam_max_distance);
}
*/

environment::environment(const environment &environment_to_copy) {
    /*
    * Copy constructor for the environment class. 
    * args:
    *   - environment_to_copy: object to copy from
    */ 
    
    environment_height_m = environment_to_copy.environment_height_m;
    environment_length_m = environment_to_copy.environment_length_m;

    goal_coord_m = environment_to_copy.goal_coord_m;
    goal_radius_m = environment_to_copy.goal_radius_m;

    // copy robot related attributes
    xy_step_size_m = environment_to_copy.xy_step_size_m;
    theta_deg_step_size_m = environment_to_copy.theta_deg_step_size_m;
    num_robot_arms_m = environment_to_copy.num_robot_arms_m;
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        robot_arm_start_coords_m[arm_num] = environment_to_copy.robot_arm_start_coords_m[arm_num];
    }
    // use the copy constructor for the robot class implicityly 
    robObj_m = environment_to_copy.robObj_m;


    // copy the obstacle related attributes
    obstacle_limit_m = environment_to_copy.obstacle_limit_m;
    obstacle_radius_m = environment_to_copy.obstacle_radius_m;
    num_obstacles_m = environment_to_copy.num_obstacles_m;
    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        obstacle_ks_m[obs_num] = environment_to_copy.obstacle_ks_m[obs_num];
        for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
            init_deflection_directions_m[obs_num][arm_num] = environment_to_copy.init_deflection_directions_m[obs_num][arm_num];
        }

        // this uses the obstacle copy constructor implicitly 
        obstacles_m[obs_num] = environment_to_copy.obstacles_m[obs_num];
    }
    
    debug_m = environment_to_copy.debug_m;
    // this uses the camera copy constructor implicitly
    cam_m = environment_to_copy.cam_m;

}

/*
* ********************************************************************************
* END: Environment CONSTRUCTORS
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: Environment DESTRUCTORS
* ********************************************************************************
*/
environment::~environment() {
    /*
    * Destructor function for the environment object. 
    */ 
}

/*
* ********************************************************************************
* END: Environment DESTRUCTORS
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: Environment SET AND INIT FUNCTIONS
* ********************************************************************************
*/
bool environment::init_environment(int height, int length, environmentCoords goal_coord, int goal_radius, int xy_step_size, 
        int theta_deg_step_size, int num_robot_arms, const robotArmCoords *robot_arm_start_coords, bool obstacle_limit, 
        int obstacle_radius, int num_obstacles, const float *obstacle_ks, const float *obstacle_center_x, const float *obstacle_center_y, 
        const deflectionDirection deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g], int cam_corresponding_arm_index, int cam_fov_degrees, int cam_angle_resolution_deg, 
        int cam_max_distance) {
    /*
    * Initializes the environment object completely 
    * args:
    *   - height: environment_height
    *   - length: environment_length
    *   - goal_coods: the coordinates of the goal in the environment
    *   - goal_radius: the coordinates of the goal in the environment
    *   - num_robot_arms: the number of robot arms in the robot
    *   - robot_arm_start_coords: list of the y values that the robot arms will start at in the environment
    *   - obstacle_limit: boolean flag if the obstacles will be limited to be entirely in the environment
    *   - num_obstacles: the number of obstacles
    *   - obstacle_ks: the cost coefficients of the obstacles (obstacle weights)
    *   - obstacle_center_x: list of the x coordinates of the obstacle centers
    *   - obstacle_center_y: list of the y coordinates of the obstacle centers
    *   - deflection_directions: list of lists of the initial deflection directions to use while initializing the obstacles
    *       - outer list dimension: num_obstacles
    *       - inner list dimension: num_robot_arms
    * returns:
    *   - error: true if there was an error while initializing the environment
    * 
    * NOTE: If there is an error delete all dynamically allocated lists and return - no guarantees on any of the 
    * obstacle attribute values so you have to reinitialize. 
    */ 
    bool error = false;
    error = cam_corresponding_arm_index < num_robot_arms;

    // environment associated attributes
    this->environment_height_m = height;
    this->environment_length_m = length;

    this->goal_coord_m = goal_coord;
    this->goal_radius_m = goal_radius;

    // robot associated attributes - need the num_robot_arms for other initializations
    this->xy_step_size_m = xy_step_size;
    this->theta_deg_step_size_m = theta_deg_step_size;
    this->num_robot_arms_m = num_robot_arms;
    // MEM:ALLOC
    for (int i = 0; i < num_robot_arms; i++) {
        this->robot_arm_start_coords_m[i] = robot_arm_start_coords[i];
    }
    // initialize the robot object
    error = this->robObj_m.init_robot(this->robot_arm_start_coords_m, debug_m);
    if (error) {
        return error;
    }

    cout << "Initialized the robot in the environment" << endl;

    // obstacle associated attributes
    this->obstacle_limit_m = obstacle_limit;
    this->obstacle_radius_m = obstacle_radius;
    this->num_obstacles_m = num_obstacles;

    // TODO: THIS DOES NOT SEEM TO BE WORKING IN RETURNING AN ERROR CHECK THIS
    // check that there is no error that causes inter obstacle problems (eg: inter obstacle collisions) with the parameters
    for (int i = 0; i < num_obstacles; i++) {
        if (i == 0) {
            continue;
        }   
        if (obstacle_center_x[i] - obstacle_center_x[i - 1] < obstacle_radius) {
            error = true;
            return error;
        }
    }

    environmentCoords init_orig_center; // initial center to use to initialize obstacle
    for (int i = 0; i < num_obstacles; i++) {
        this->obstacle_ks_m[i] = obstacle_ks[i];
        this->obstacle_center_x_m[i] = obstacle_center_x[i];
        this->obstacle_center_y_m[i] = obstacle_center_y[i];
        init_orig_center.x = obstacle_center_x[i];
        init_orig_center.y = obstacle_center_y[i];
        error = this->obstacles_m[i].init_obstacle(init_orig_center, obstacle_radius, obstacle_ks[i], 
        deflection_directions[i], debug_m, this->robObj_m.arms_m);

        if (error) {
            return error;
        }
    }

    // store the initial deflection directions as an environment attribute for resets
    for (int obsnum = 0; obsnum < num_obstacles; obsnum++) {
        for (int armnum = 0; armnum < num_robot_arms; armnum++) {
            this->init_deflection_directions_m[obsnum][armnum] = deflection_directions[obsnum][armnum];
        }
    }

    // initialize the camera
    robotArmCoords init_camera_coords = robObj_m.arms_m[cam_corresponding_arm_index].get_coords();
    cam_m.init_camera(init_camera_coords, 
                    cam_corresponding_arm_index, cam_fov_degrees, cam_angle_resolution_deg, cam_max_distance);
    return error;
}

void init_state(environmentState x) {
    /*
    * Initializes the environment without any checks from a state x. The state contains a robot object
    * and obstacles that are used to set the state in the current environment. 
    */ 

    //TODO: FINISH THIS BY MAKING COPY CONSTRUCTORS FOR EVERYTHING and copying everything
    // NOTE: HANDLE THE DYNAMICALLY ALLOCATED ATTRIBUTES PROPERLY IN THE COPY CONSTRUCTOR BY CREATING NEW
    // DYNAMIC ALLOCATIONS AND THEN CREATING NEW OBJECTS TO POPULATE THEM - when needed
    return;
}

/*
* ********************************************************************************
* END: Environment SET AND INIT FUNCTIONS
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: Environment GET FUNCTIONS
* ********************************************************************************
*/
void environment::get_state(environmentState &x) const{
    /*
    * Get the state of the environment: This includes a copy of the robot object and a copy of the 
    * obstacle objects in the environment. 
    */ 

    // TODO: FINISH THIS FUNCTION - MAKE A COPY CONSTRUCTOR FOR EVERYTHING AND ONLY RETURN (WITH PASS BY REFERENCE)
    // pointers to the copied objects. 
    // TODO: HANDLE A WAY TO DELETE THESE TOO????
    return;
}
void environment::delete_state(environmentState &x) const{
    /*
    * Delete the dynamically allocated parts of the environment state. 
    */ 

    // TODO: FINISH THIS FUNCTION - need to finish get state and init_state functions first. 
    return;
}

float environment::get_intersection_area() const{
    /*
    * Used to calculate the intersection area between the goal region and the obstacles. May be used
    * in variants of the cost function or to asses whether the goal has been reached in the environment. 
    * returns:
    *   - area: a float indicated the area of intersection
    */ 
    float totalArea = 0;
    float a, b, d, x, z, y; // intermediate floats to use for area calculation
    environmentCoords obs_center_coord;
    for (int i = 0; i < num_obstacles_m; i++) {   
        obs_center_coord = obstacles_m[i].get_orig_center();
        d = sqrt(pow(obs_center_coord.x - goal_coord_m.x, 2) + pow(obs_center_coord.y - goal_coord_m.y, 2));
        if (d < obstacle_radius_m + goal_radius_m) {
            a = pow(goal_radius_m, 2);
            b = pow(obstacle_radius_m, 2);
            x = (a - b + pow(d, 2)) / (2 * d);
            z = pow(x, 2);
            y = sqrt(a - z);
            if (d <= abs(obstacle_radius_m - goal_radius_m)) {
                totalArea += M_PI * min(a,b);
            } else {
                totalArea += asin((float) y/goal_radius_m) + (b * asin((float) y/obstacle_radius_m)) - (y * (x + sqrt(z + b - a)));
            }
        }
    }
    return totalArea;
}

environmentCoords environment::get_goal_coord() const{
    return goal_coord_m;
}

int environment::get_goal_radius() const{
    return goal_radius_m;
}

// ROBOT RELATED GET FUNCTIONS
int environment::get_num_robot_arms() const{
    return num_robot_arms_m;
}
int environment::get_cam_num_scan_angles() const{
    return cam_m.get_num_scan_angles();
}
void environment::get_all_robot_arm_coords(robotArmCoords *ret_robot_arm_coords, int ret_array_len) const{
    /*
    * Return the robotArmCoords that define the state for each robot arm in the robot. Fill the array up to 
    * the min of ret_array_len and num_robot_arms.
    */ 

    for (int i = 0; i < min(ret_array_len, num_robot_arms_m); i++) {
        ret_robot_arm_coords[i] = robObj_m.arms_m[i].get_coords();
    }
    return;
}

void environment::get_all_robot_start_end_pts(environmentCoords *ret_start_pts, environmentCoords *ret_end_pts, int ret_array_len) const{
    /*
    * Return the start and end points for each robot arm in the robot. Fill the array up to the mi of the ret_array_len
    * and num_robot_arms. 
    * 
    * This will be largely used to draw the lines that represent the robot arms in the rendering function
    */
    float slope;
    for (int i = 0; i < min(ret_array_len, num_robot_arms_m); i++) {
        ret_end_pts[i].x = robObj_m.arms_m[i].get_x();
        ret_end_pts[i].y = robObj_m.arms_m[i].get_y();
        robObj_m.arms_m[i].find_startpt_slope(ret_start_pts[i], slope);
    }
    return;
}

// OBSTACLE RELATED GET FUNCTIONS
int environment::get_obstacle_radius() const{
    return obstacle_radius_m;
}
int environment::get_num_obstacles() const{
    return num_obstacles_m;
}
void environment::get_obstacle_ks(float *ret_obstacle_ks, int ret_array_len) const{
    /*
    * Return the cost coefficients of all the obstacles. only fill the return array up to the minimum of 
    * num_obstaclse and ret_array_len
    */ 
    for (int i = 0; i < min(ret_array_len, num_obstacles_m); i++) {
        ret_obstacle_ks[i] = obstacles_m[i].get_k();
    }
    return;
}
void environment::get_current_obstacle_centers(environmentCoords *ret_obstacle_centers, int ret_array_len) const{
    /*
    * Return the current center coordinates of the obstacles in the environment. Only fill up the return array up to 
    * the minimum of num_obstacles and ret_array_len. 
    */ 
    for (int i = 0; i < min(ret_array_len, num_obstacles_m); i++) {
        ret_obstacle_centers[i] = obstacles_m[i].get_center();
    }
    return;
}

// Camera observation functions
void environment::observe_points(cameraIntersectionPoint *ret_cam_intersection_points, int ret_array_size) const{
    /*
    * Calls the camera scan_environment function with the proper arguments
    */
    cam_m.scan_environment(num_robot_arms_m, robObj_m.arms_m, num_obstacles_m, obstacles_m, ret_cam_intersection_points, ret_array_size);
} 
void environment::observe_dists(cameraIntersectionDistance *ret_cam_intersection_dists, int ret_array_size) const{
    /*
    * Calls the camera scan_environment_distance function with the proper arguments
    */ 
    cam_m.scan_environment_distance(num_robot_arms_m, robObj_m.arms_m, num_obstacles_m, obstacles_m, ret_cam_intersection_dists, ret_array_size);
}


/*
* ********************************************************************************
* END: Environment GET FUNCTIONS
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: Environment SET FUNCTIONS
* ********************************************************************************
*/

void environment::set_obstacle_ks(const float obstacle_ks[NUM_OBSTACLES_g]) {
    /*
    * sets the k values of the obstacles to the values specified
    */ 
    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        obstacles_m[obs_num].set_k(obstacle_ks[obs_num]);
    }
    return;
}

bool environment::set_robot_arms_autoset_obstacles(const robotArmCoords new_arm_states[NUM_ROBOT_ARMS_g], const deflectionDirection deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g]) {
    /*
    * This function sets the robot arm coordinates to the specified new_arm_states. The obstacles are then 
    * set to the positions that they deflect to according to the deflection directions. The function 
    * returns if an error occurred while setting. 
    * 
    * NOTE: if an error occurs - rolls the environment (robot and obstacles) back to the state that they were in before this was called
    * args:
    *   - new_arm_states
    *   - deflection_directions
    * returns;
    *   - error: True if there was an error else false. 
    */ 
    bool error = false;
    error = robObj_m.set_arms(new_arm_states);

    if (error) {
        // rollback occurred in the set_arms function 
        return error;
    }

    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        // sets deflection directions and allows environment to settle - does not auto rollback
        error = obstacles_m[obs_num].manual_set_deflection_directions(robObj_m.arms_m, deflection_directions[obs_num]);
        if (error) {
            for (int rollback_obs_num = 0; rollback_obs_num < obs_num + 1; rollback_obs_num++) {
                obstacles_m[rollback_obs_num].state_rollback();
            }
            return error;
        }
    }
    return error;
}

/*
* ********************************************************************************
* END: Environment SET FUNCTIONS
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: Environment STATE mutation functions
* ********************************************************************************
*/
void environment::step(robotArmActions *actions, bool &error, float &cost) {
    /*
    * Steps in the enviornment by stepping the robot with the actions list. 
    * args:
    *   - actions: actions list. The action at index i is used to step robot_arm i of the robot
    * returns: by pass by reference
    *   - error: boolean value: true indicates that an error has occurred in taking the actions
    *   - cost: the float cost of the actions
    * 
    * NOTE: if an error is returned the environment state is the state before the action was taken. 
    * this is a feature of each of the step functions of the subclasses and if an error is found later down 
    * the line the rollback feature can be used to guarantee this. 
    */ 

    float temp_cost = 0;

    // step the robot arm 
    robObj_m.step(actions, xy_step_size_m, theta_deg_step_size_m, error, temp_cost);
    cost += temp_cost;
    if (error) {
        cout << "ERROR: error occurred in ROBOT object step" << endl;
        return;
    }

    // step the obstacles in response to the new robot position
    float total_obstacle_cost = 0;
    for (int i = 0; i < num_obstacles_m; i++) {
        obstacles_m[i].step_to_deformed_center(robObj_m.arms_m, error, temp_cost);
        total_obstacle_cost += temp_cost;

        if (error) {
            // rollback the robot and all obstacles up to and including the "ith" obstacle that you reached. Assign the cost to only the cost of having the obstacle error that was last returned
            cost = temp_cost;
            for (int rollback_idx = 0; rollback_idx < i+1; rollback_idx++) {
                obstacles_m[rollback_idx].state_rollback();
            }
            robObj_m.state_rollback();
            cout << "ERROR: error occurred after robot stepped and while the OBSTACLES were being stepped" << endl; 
            return;
        }
    }

    // step the camera to the appropriate index
    cam_m.step_to_coord(robObj_m.arms_m[cam_m.get_corresponding_arm_index()].get_coords());

    cost += total_obstacle_cost;
    return;
}

bool environment::reset() {
    /*
    * Resets to the environment to the state right after initialization. 
    */  
    // since we got here the initial parameters didn't cause an error so they should not here either. 
    bool error = init_environment(environment_height_m, environment_length_m, goal_coord_m, goal_radius_m, xy_step_size_m, theta_deg_step_size_m, 
    num_robot_arms_m, robot_arm_start_coords_m, obstacle_limit_m, obstacle_radius_m, num_obstacles_m, obstacle_ks_m, 
    obstacle_center_x_m, obstacle_center_y_m, init_deflection_directions_m, cam_m.get_corresponding_arm_index(), 
    cam_m.get_fov_deg(), cam_m.get_angle_resolution_deg(), cam_m.get_max_distance());
    return error;
}

bool environment::at_goal() const{
    /*
    * NOTE: Change thsi function according to objective
    * Checks if the environment is in a goal/terminal state - this one just checks if one of the arms is in the goal region
    * returns:
    *   - true if the environment is in a goal state else false
    */
    float dist_to_goal;
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        dist_to_goal = sqrt(pow(robObj_m.arms_m[arm_num].get_x() - goal_coord_m.x, 2) + pow(robObj_m.arms_m[arm_num].get_y() - goal_coord_m.y, 2));
        if (dist_to_goal <= goal_radius_m) {
            return true;
        }
    }
    return false;
}

void environment::printState() const{
    /*
    * Print the state of the environment - all the robot arms and all the obstacles
    */ 
    cout << "BEGIN ENVIRONMENT print: " << endl;
    cout << "ROBOT: " << endl;
    robObj_m.printState();
    cout << "OBSTACLES: " << endl;
    for (int i = 0; i < num_obstacles_m; i++) {
        cout << "Obstacle num: " << i << endl;
        obstacles_m[i].printState();
        cout << endl;
    }
    robotArmCoords camera_coords = cam_m.get_camera_coords();
    cout << "Camera coordinates: " << camera_coords.x << "," << camera_coords.y  << endl;
    cout << "camera max distance: " << cam_m.get_max_distance() << endl; 
    cout << "END ENVIRONMENT print"  << endl;
}
/*
* ********************************************************************************
* END: Environment STATE mutation functions
* ********************************************************************************
*/

} // end namespace despot