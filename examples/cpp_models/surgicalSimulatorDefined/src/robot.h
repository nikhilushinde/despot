/*
* This file contains the description of the "robot" class. This class contains many objects of the class 
* "robot_arm". This class is contained in the "environment" class and interacts with objects of the class
* "obstacle". 
*
* Functions for this class are defined in robot.cpp
*/

#ifndef ROBOT_H
#define ROBOT_H

#include "surgical_utils.h"
#include "robot_arm.h"

namespace despot {

#define NOT_ARM_INDEX -1 // index to insert in the arm_relations list when there are more true arms than arms out in the environment
#define NO_YS -1 // y value to use when establishing arm relations and the robot arm end effector is not in the environment - x = 0 

class robot {
public:
    // constructors
    robot(); // default constructor - after this constructor you have to set  set_robot_arm_start_ys
    robot(const robotArmCoords arm_coords[], bool debug, bool &error); // constructor that that starts things properly 
    robot(const robot &robot_to_copy); // copy constructor for the robot class


    // destructor
    ~robot(); // the destructor function

    // initialization
    bool init_robot(const robotArmCoords arm_coords[], bool debug); // initialize the robot arms given the end effector coordinates

    // set and get functions
    bool set_arms(const robotArmCoords new_arm_states[]); // set the robot arms to these coordinates - set_arm_relations is called in this. 
    bool set_arm_relations(); // given that the arms are created and their y values are set - set the arm relations correctly according to this

    // other functions - constant functions
    bool is_2arm_collision(const robot_arm &arm1, const robot_arm &arm2) const; // check if there is a collision between 2 arms
    bool is_any_collision() const; // checks if there are any collisions between the robot's arms
    bool violate_above_relation(const robot_arm &above_arm, const robot_arm &below_arm) const; // check and return true if the robot_arm that is the above_arm is actually entirely above the below_arm. otherwise return false
    
    // other functions - non constant - mutating functions
    void step(const robotArmActions actions[], int xy_step_size, int theta_deg_step_size, bool &error, float &cost); //step the robot (the robot_arms) in the environment and use pass by reference to return error and cost 
    void state_rollback(); // rolls back the state of the robot - rolls back all the robot_arms

    void printState(); // print the state

    robot_arm arms_m[NUM_ROBOT_ARMS_g]; // list of robot arm objects
private:
    /* NOTE: all attributes of the class end with '_m' to signify being member variables of the class */
    bool debug_m; // debug flag

    int robot_arm_relations_m[NUM_ROBOT_ARMS_g]; // list of integers that store the relationship of the robot arms top to bottom
    int old_robot_arm_relations_m[NUM_ROBOT_ARMS_g]; // robot_arm_relations from the previous time step

};

} // end namespace despot

#endif 