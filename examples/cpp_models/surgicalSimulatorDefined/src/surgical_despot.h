#ifndef SURGICAL_DESPOT_H
#define SURGICAL_DESPOT_H

/*
* Description: This file creates the class that acts like the interface between the environment class that describes the toy surgical environment
* and the despot solver. 
* NOTE: this file only handles class uncertainty. 
*/

// the despot related includes
#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include "environment.h"


namespace despot {

class SurgicalDespot: public DSPOMDP {
protected:
    mutable MemoryPool<environment> memory_pool_;
    std::vector <environment*> states_;
public: 
    SurgicalDespot(); // constructor

    /* BEGIN: REQUIRED FUNCTIONS FOR DESPOT */

    /* returns the number of actions */
    int NumActions() const;
    void IntToActions(int actionNum, robotArmActions *ret_action_array, int ret_action_array_size) const; // convert integer to action array that is taken by the environment

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

    void MostLikelyRobPosition(const std::vector <State*>&particles, int &retRobX, int &retRobY) const; // get the most likely robot position from a set of particles

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

}


#endif