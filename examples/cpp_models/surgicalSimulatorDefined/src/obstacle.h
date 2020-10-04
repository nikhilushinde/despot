/*
* This file describes the "obstacle" class. These are circles in the environment that deform in response 
* to interaction by the robot. This class dictates how the obstacles react to the "robot" object. The objects of this
* class are contained in the "environment" class. 
*
* Functions for this class are defined in obstacle.cpp
*/

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "surgical_utils.h"
#include "robot_arm.h"
#include "robot.h"
#include <math.h>

namespace despot {

class obstacle{
public:
    // constructors
    obstacle(); // default obstacle constructor
    obstacle(environmentCoords orig_center, int radius, float k, const deflectionDirection deflection_directions[], bool debug, const robot_arm arms[], bool &error); // initialize the obstacle fully 
    obstacle(const obstacle &obstacle_to_copy); // copy constructor for the obstacle class

    // destructors
    ~obstacle(); // destroys the obstacle object

    // initialize and set functions and get functions
    bool init_obstacle(environmentCoords orig_center, int radius, float k, const deflectionDirection deflection_directions[], bool debug, const robot_arm arms[]); // initialize the obstacle as though calling the full constructor
    bool set_coords(environmentCoords coords, const deflectionDirection deflection_directions[], const robot_arm arms[]); // set the current coordinates of the obstacle to coords
    void set_deflection_directions(const robot_arm arms[]); // sets the deflection directions of the obstacle
    bool manual_set_deflection_directions(const robot_arm arms[], const deflectionDirection deflection_directions[]); // manually set the deflection directions of the obstacle and then "settle" the environment - return error results in an incorrect state

    void set_orig_center(environmentCoords center); 
    void set_radius(int radius);
    void set_k(float k);
    void set_debug(bool debug);

    // get functions
    void get_deflection_directions(deflectionDirection *ret_deflection_directions, int arraySize) const;
    environmentCoords get_orig_center() const;
    environmentCoords get_center() const;
    int get_radius() const;
    float get_k() const;
    bool get_debug() const;

    // other functions
    float deformation_cost() const; // calculates the deformation cost of the last time step
    environmentCoords single_arm_deformed_center(robot_arm arm, deflectionDirection deflection_direction, bool &error) const; // calculates the deflected center of the obstacle given a robot arm and the deflection direction
    environmentCoords single_arm_stuck_center(robot_arm arm, deflectionDirection deflection_direction) const; // finds the center of the obstacle assuming that the obstacle is stuck to the robot arm 
    bool is_collision(robot_arm arm, deflectionDirection deflection_direction, environmentCoords obs_center); // check if arm collides with the obstacle with the given center value
    void printState() const; // function to print the state of the obstacle

    // mutating functions
    void step_to_deformed_center(const robot_arm arms[], bool &error, float &cost); // function that steps the obstacles to the next position given the arms in the environment, returns if there was an error and the cost by pass by reference
    void state_rollback(); // rollback to the previous robot arm state. 


private:
    /* NOTE: Added suffix _m to all the member variables of the obstacle */
    void rollback_deflection_directions(); // just rollback deflection_directions to old_deflection_directions

    environmentCoords orig_center_m; // original center coordinate of the obstacle
    environmentCoords center_m; // the current deformed center coordinate of the obstacle
    environmentCoords old_center_m; // the center coordinate of the obstacle in the last time step

    int radius_m; // radius of the obstacle in pixels
    float k_m; // spring/deformation cost constant

    deflectionDirection deflection_directions_m[NUM_ROBOT_ARMS_g]; // list of deflection directions - dictates how the obstacle deflects relative to the robot arm at that index
    deflectionDirection old_deflection_directions_m[NUM_ROBOT_ARMS_g]; // deflection direction list at the last time step    
    bool debug_m; // debug flag

};

}  // end namespace despot

#endif