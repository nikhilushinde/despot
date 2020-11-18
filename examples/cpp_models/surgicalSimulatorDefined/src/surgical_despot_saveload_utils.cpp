/*
This file is used for saving and loading maps and converting the environment class to and from 
structures of common types 

map:
    - key: environment object 
    - value: ValuedAction - pair of action which is an integer and value which is a double 
*/
#include "surgical_despot_saveload_utils.h"
#include <iostream>
#include <fstream>

using std::cerr; 
using std::endl;

namespace despot {

environment environmentStruct_to_environment_g(environmentStruct struct_to_convert) {
    /*
    * This function converts the environment struct type to an environment object that is returned
    * NOTE: 
    *   - this uses the environment height, environment length, goal radius, goal location that is set
    *   in the "defined_parameters.h" file as this is pointlessly repetitive to store. 
    */ 
    environment retEnvironment; 

    robotArmCoords robotCoords[NUM_ROBOT_ARMS_g];
    deflectionDirection setDeflectionDirs[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g];
    // set the robotCoords and deflection directions
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num ++) {
        robotCoords[arm_num].x = struct_to_convert.robotX[arm_num];
        robotCoords[arm_num].y = struct_to_convert.robotY[arm_num];
        robotCoords[arm_num].theta_degrees = struct_to_convert.robotThetaDegrees[arm_num];
    }
    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num ++) {
        for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num ++) {
            setDeflectionDirs[obs_num][arm_num] = static_cast<deflectionDirection>(struct_to_convert.obs_deflection_directions[obs_num][arm_num]);
        }    
    }

    bool error = retEnvironment.set_robot_arms_autoset_obstacles(robotCoords, setDeflectionDirs);
    retEnvironment.set_obstacle_ks(struct_to_convert.obstacle_ks);
    if (error) {
        cerr << "Error: the environment struct given in the environmentStruct_to_environment_g was invalid." << endl; 
        exit(1);
    }

    return retEnvironment;
}


environmentStruct environment_to_environmentStruct_g(environment environment_to_convert) {
    /*
    * This function converts environment structs to an environment object. 
    */ 
    environmentStruct retEnvironmentStruct;
    //float env_obstacle_ks[NUM_OBSTACLES_g];
    deflectionDirection env_deflection_directions[NUM_OBSTACLES_g][NUM_ROBOT_ARMS_g];

    environment_to_convert.get_obstacle_ks(retEnvironmentStruct.obstacle_ks, NUM_OBSTACLES_g); 
    environment_to_convert.get_current_deflection_directions(env_deflection_directions);
    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num ++) {
        for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num ++) {
            retEnvironmentStruct.obs_deflection_directions[obs_num][arm_num] = env_deflection_directions[obs_num][arm_num];
        }
    }
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num ++) {
        retEnvironmentStruct.robotX[arm_num] = environment_to_convert.robObj_m.arms_m[arm_num].get_x();
        retEnvironmentStruct.robotY[arm_num] = environment_to_convert.robObj_m.arms_m[arm_num].get_y();
        retEnvironmentStruct.robotThetaDegrees[arm_num] = environment_to_convert.robObj_m.arms_m[arm_num].get_theta_degrees();
    }

    return retEnvironmentStruct;
}

void saveEnvironmentStruct(std::ostream &out, environmentStruct struct_to_save) {
    /*
    * Saves the environment struct to file
    */ 
    // save the robot parameters
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        out.write((char *)&struct_to_save.robotX[arm_num], sizeof(int));
        out.write((char *)&struct_to_save.robotY[arm_num], sizeof(int));
        out.write((char *)&struct_to_save.robotThetaDegrees[arm_num], sizeof(int));
    }
    // save the obstacle parameters
    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        out.write((char *)&struct_to_save.obstacle_ks[obs_num], sizeof(float));
    }

    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
            out.write((char *)&struct_to_save.obs_deflection_directions[obs_num][arm_num], sizeof(int));
        }
    }
    return; 
}

void loadEnvironmentStruct(std::istream &in, environmentStruct &struct_to_load) {
    /*
    * Saves the environment struct to file
    */ 
    // save the robot parameters
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        in.read((char *)&struct_to_load.robotX[arm_num], sizeof(int));
        in.read((char *)&struct_to_load.robotY[arm_num], sizeof(int));
        in.read((char *)&struct_to_load.robotThetaDegrees[arm_num], sizeof(int));
    }
    // save the obstacle parameters
    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        in.read((char *)&struct_to_load.obstacle_ks[obs_num], sizeof(float));
    }

    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
            in.read((char *)&struct_to_load.obs_deflection_directions[obs_num][arm_num], sizeof(int));
        }
    }
    return; 
}

void saveFile(std::map<environment, ValuedAction> &map_to_save, string saveFilename) {
    /*
    * Saves the given map to a file. 
    */ 
    std::string filename = "/home/nikhilushinde/Documents/research/arclab/cpp_despot/despot/examples/cpp_models/surgicalSimulatorDefined/test.txt";
    std::ofstream out(filename);
    std::cout << "Map to save size: " << map_to_save.size() << endl;
    
    for (auto it = map_to_save.cbegin(); it != map_to_save.cend(); it++) {
        environmentStruct struct_to_save = environment_to_environmentStruct_g(it->first); 
        saveEnvironmentStruct(out, struct_to_save);
        out.write((char *)&it->second.action, sizeof(int));
        out.write((char *)&it->second.value, sizeof(double));

        //std::cout << "action: " << it->second.action << ", value: " << it->second.value << endl;
    }
    out.close();
}

void loadFile(std::map<environment, ValuedAction>&map_to_load, string loadFileName) {
    /*
    * Loads the map from file
    */ 
    std::string filename = "/home/nikhilushinde/Documents/research/arclab/cpp_despot/despot/examples/cpp_models/surgicalSimulatorDefined/test.txt";
    std::ifstream in(filename);
    
    while(!in.eof()) {
        environmentStruct struct_to_load; 
        loadEnvironmentStruct(in, struct_to_load);
        environment loaded_env = environmentStruct_to_environment_g(struct_to_load);
        ValuedAction loadedValuedAction; 

        in.read((char *)&loadedValuedAction.action, sizeof(int));
        in.read((char *)&loadedValuedAction.value,  sizeof(double));

        //std::cout << "action: " << loadedValuedAction.action << ", value: " << loadedValuedAction.value << endl;
        map_to_load[loaded_env] = loadedValuedAction;
    }
    return; 
}

}