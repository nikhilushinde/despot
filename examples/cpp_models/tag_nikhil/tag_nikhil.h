#ifndef TAG_NIKHIL_H
#define TAG_NIKHIL_H

// the despot related includes
#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>

// local includes 
#include "tag_state.h"

namespace despot {

/*
* TagState class: This class that serves as the state of the particles/dspomdp is defined in tag_state.h 
*                 with the functions defined in tag_state.cpp - contains the entire backend of the environment
*/

class TagNikhil: public DSPOMDP { 
// This class is based off of the DSPOMDP example classes in the examples/cpp_model folder. 
protected:
    mutable MemoryPool<TagStateNikhil> memory_pool_;
    std::vector <TagStateNikhil*> states_;

public:
    // actions via an enum - ensure they match with the TagState actions
    enum {
        NORTH = 0, 
        SOUTH = 1, 
        EAST = 2, 
        WEST = 3, 
        TAG = 4
    };

    static const int num_observations = NUM_OBSERVATIONS_TAG_NIKHIL_STATE;
    OBS_TYPE observation_array[num_observations]; // make sure to initialize the observation array in the constructor
    static const int num_actions = NUM_ACTIONS_TAG_NIKHIL_STATE;

public:
    // constructor
    TagNikhil();

    /* BEGIN: REQUIRED FUNCTIONS FOR DESPOT */

    /* returns the number of actions */
    int NumActions() const;

    /* Deterministic simulative model */
    bool Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs);

    /* Functions related to beliefs and starting states */
    /* Returns the probability of viewing the observation obs from state given you transitioned to it from action*/
    double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;
    /* Create a random start state - randomly choose a state of the environment to start in */
    State* CreateStartState(std::string type = "DEFAULT") const;
    /* The initial belief on the environment */
    Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

    /* Bound-related functions.*/
    double GetMaxReward() const;
    ValuedAction GetBestAction() const;
    // NOTE: ADD THE MORE ADVANCED BOUND RELATED FUNCTIONS

    /* Memory management.*/
    State* Allocate(int state_id, double weight) const; // The state_id is not really used for more the more advanced state representations here
    State* Copy(const State* particle) const;
    void Free(State* particle) const;
    int NumActiveParticles() const;

    /* END: REQUIRED FUNCTIONS FOR DESPOT */
    /* ======= Display Functions: ========*/
    /* These functions are not needed for performance but are used for debugging and displaying */
    /* Prints a state. */
    virtual void PrintState(const State& state, std::ostream& out = std::cout) const = 0;
    /*  Prints an observation. */
    virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const = 0;
    /* Prints an action. */
    virtual void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const = 0;
    /* Prints a belief. */
    virtual void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const = 0;
    /* Returns number of allocated particles. */
    virtual int NumActiveParticles() const = 0;

private:


};


} // end despot namespace. 

#endif