/*
* This file describes the "robot_arm" class. Objects of this class are contained in the "robot" class object. 
* These objects interact with the "obstacle" class in the environment. 
*
* Functions for this class are defined in robot_arm.cpp
*/
#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <iostream>
#include <math.h>
#include <array>
#include "surgical_utils.h"

namespace despot {

#define MOVEMENT_ERROR 10000000000 // cost for taking an action that resulted in an out of bounds error or collision error 

class robot_arm {
public:
    // constructors 
    robot_arm(); // simple no argument constructor: NOTE - need to set the attributes correctly after this
    robot_arm(robotArmCoords coords, bool debug, bool &error); // complete full argument constructor
    robot_arm(const robot_arm &arm_to_copy); // Copy constructor for the robot_arm class

    // get functions for access - constant functions
    int get_x() const;
    int get_y() const;
    int get_theta_degrees() const;
    robotArmCoords get_coords() const;
    float get_theta_radians() const;

    // set functions for access
    bool init_robot_arm(robotArmCoords coords, bool debug); // initializes the entire robot arm object - should be called after default constructor initialization
    bool set_coords(robotArmCoords new_coords); // sets the coordinates for the arms 

    // State Checking Functions - constant functions
    bool is_valid_pos() const; // returns if the position of the robot arm is valid/within bounds or not
    void find_startpt_slope(environmentCoords &start_pt, float &slope) const; // This function returns by assigning values to the pointers for the start point and the slope formed by the robot arm line
    // pass the object by value in check_collision as we don't want to mutate it or do anything to it

    // State Evolving Functions
    // step function that does not support collision checking - all collision checking with other arms is done in external function
    void step(robotArmActions action, int xy_step_size, int theta_deg_step_size, bool &is_out_of_bounds, float &cost);
    void state_rollback(); // rolls back the robot_arm's coords to the old_coords. 

    void printState() const; // prints the entire state of the robot_arm 


private:
    /* NOTE: all attributes of the class end with '_m' to signify being member variables of the class */
    // the end effector coordinates of the robot arm 
    robotArmCoords coords_m;
    // last end effector coordinats of the robot arm 
    robotArmCoords old_coords_m;
    
    bool debug_m;  // used for verbose printing for debugging 
};

} // end namespace despot

#endif