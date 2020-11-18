/*
This file is used for saving and loading maps and converting the environment class to and from 
structures of common types 
HEADER FILE

map:
    - key: environment object 
    - value: ValuedAction - pair of action which is an integer and value which is a double 
*/
#ifndef SURGICAL_DESPOT_SAVELOAD 
#define SURGICAL_DESPOT_SAVELOAD
#include "defined_parameters.h"
#include "surgical_utils.h"
#include "environment.h"

using std::string;

namespace despot {

// struct that defines everything needed to construct the environment
struct environmentStruct {
    int robotX[NUM_ROBOT_ARMS_g]; 
    int robotY[NUM_ROBOT_ARMS_g];
    int robotThetaDegrees[NUM_ROBOT_ARMS_g];
    float obstacle_ks[NUM_OBSTACLES_g];
    int obs_deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g];
};

environment environmentStruct_to_environment_g(environmentStruct struct_to_convert); 
environmentStruct environment_to_environmentStruct_g(environment environment_to_convert);

void saveFile(std::map<environment, ValuedAction> &map_to_save, string saveFilename);
void loadFile(std::map<environment, ValuedAction> &ret_map_to_load, string loadFilename);
}
#endif 