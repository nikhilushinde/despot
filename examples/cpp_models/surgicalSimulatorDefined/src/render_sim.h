/*
* This is the header file that provides the interface to the functions to render
* the various objects in the surgicalSim environment. 
*/
#ifndef RENDER_SIM_H
#define RENDER_SIM_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "surgical_utils.h"
#include "robot_arm.h"
#include "obstacle.h"
#include "environment.h"

using std::string;

namespace despot {
#define RENDER_SIM_WINDOW_NAME "Surgical Simulator"

class render_sim {
public:
    render_sim(); // default constructor
    render_sim(const render_sim &render_sim_to_copy); // copy constructor

    void render_arm(const robot_arm &arm) const; // render the robot arm in an environment
    void render_robot(const robot_arm arms[], int num_robot_arms) const; // render the robot - all arms - in the environment
    void render_environment(const environment &env) const; // render the environment - all the robot arms and all the obstacles

private:
    string window_name_m;

    bool debug_m; // boolean debug flag
};
} // end namespace despot

#endif