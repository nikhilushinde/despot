/*
* This file houses structures and static constants that are to be utilized throughout the toy 
* surgical simulator. 
*/

#ifndef SURGICAL_UTILS_H
#define SURGICAL_UTILS_H

#include <iostream>
#include <math.h>

// include all the defined parameters and constants to set the simulator correctly 
#include "defined_parameters.h"

// the despot related includes
#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>

namespace despot {

// constants to use in other files - NOT USER SET
static const float THETA_MULTIPLIER = M_PI/180.0;
static const float float_compare_precision = 0.005f;

// global variable to keep track of the images that have been generated to show the route - increments every time you save an image
static int saved_image_number_g = 0;


// enum to define the actions that the robot arm can take 
enum robotArmActions {
    xRight = 0, // end effector moves forward horizontally
    xLeft = 1, // end effector moves backward horizontally
    yUp = 2, // end effector moves up vertically
    yDown = 3, // end effector moves down vertically
    thetaUp = 4, // end effector increases the theta value
    thetaDown = 5, // end effector decreases the theta value
    stay = 6,
};

// structure that describes the end effector coordinate of the robot arm. theta_radians can be found via conversion with THETA_MULTIPLIER
struct robotArmCoords {
    int x;
    int y;
    int theta_degrees;
};

// (x,y)  point in the environment 
struct environmentCoords {
    float x;
    float y;
};


// _g suffix to refer to global function
void action_to_movements_g(robotArmActions action, int xy_step_size, int theta_deg_step_size, int &delta_x, int &delta_y, int &delta_theta_degrees); // go from an action to the exact change in x,y,theta that the robot needs to make
void int_to_action_array_g(int actionNum, robotArmActions *ret_action_array, int ret_action_array_size); // convert integer to list of actions of type robot arm actions
int total_num_actions_g(); // the total number of actions - integers up to this are valid actions as inputs to int_to_action_array
bool isReverseAction_g(ACT_TYPE act1, ACT_TYPE act2); // checks if act1 undoes act2
bool cmp_floats_g(float a, float b); // compare floats with some epsilon value
int random_number_to_index_g(const double probabilityDistrib[], int probabilityDistrib_size, double randomNum); // to convert a random number to an integer based on a discrete probability distribution 

} // end namespace despot

#endif

