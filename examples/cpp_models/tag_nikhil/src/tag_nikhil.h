#ifndef TAG_NIKHIL_H
#define TAG_NIKHIL_H

// standard includes
#include <map>

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
friend class TagNikhilHistoryPolicy;
friend class TagNikhilManhattanUpperBound;

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
    bool Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const;

    /* Functions related to beliefs and starting states */
    /* Returns the probability of viewing the observation obs from state given you transitioned to it from action*/
    double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;
    /* Create a random start state - randomly choose a state of the environment to start in */
    State* CreateStartState(std::string type = "DEFAULT") const;
    /* The initial belief on the environment */
    Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

    /* Bound-related functions.*/
    double GetMaxReward() const;
    ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;
    ValuedAction GetBestAction() const;
    ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;
    // NOTE: ADD THE MORE ADVANCED BOUND RELATED FUNCTIONS

    /* Memory management.*/
    State* Allocate(int state_id, double weight) const; // The state_id is not really used for more the more advanced state representations here
    State* Copy(const State* particle) const;
    void Free(State* particle) const;
    int NumActiveParticles() const;

    void MostLikelyRobPosition(const vector<State*>& particles, int &retRobX, int &retRobY) const; // get the most likely robot position from a set of particles

    /* END: REQUIRED FUNCTIONS FOR DESPOT */
    /* ======= Display Functions: ========*/
    /* These functions are not needed for performance but are used for debugging and displaying */
    /* Prints a state. */
    void PrintState(const State& state, std::ostream& out = std::cout) const;
    /*  Prints an observation. */
    void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
    /* Prints an action. */
    void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;
    /* Prints a belief. */
    void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;

private:


};


} // end despot namespace. 

#endif