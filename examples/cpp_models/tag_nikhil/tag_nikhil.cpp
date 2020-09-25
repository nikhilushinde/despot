#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;
#include "tag_state.h"
#include "tag_nikhil.h"

namespace despot {

TagNikhil::TagNikhil() {
    /*
    * Default constructor for the TagNikhil class. 
    */ 
    // initialize the observation array 
    for (int i = 0; i < num_observations; i++) {
        observation_array[i] = i;
    }
}

int TagNikhil::NumActions() const {
    /* Returns the total number of possible actions */
    return num_actions;
}

bool TagNikhil::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const {
    /*
    * This is the step function. The function is a const function meaning it does not change the TagNikhil object. 
    * The object takes the state object and steps that state object in the environment to mutate it to the next 
    * state upon taking the given action. 
    * args:
    *   - state: the current state in which to take the action - also mutates to return the new state of the 
    *            environment after taking the action 
    *   - rand_num: random number to make the state evolution deterministic
    *   - action: the action to take from the given current state 
    * returned: pass by reference:
    *   - state: state after the action from state
    *   - reward: the reward returned upon taking action from state
    *   - obs: the observation seen upon taking the action from state
    * returned:
    *   - bool: whether the state is a terminal state and the "game" should be ended. 
    */ 
    // dynamic cast since the inheritance is downwards (State is the parent class of TagStateNikhil and not vice versa)
    // use static casts for upwards inheritance. 
    TagStateNikhil *tagStateNikhil_state = dynamic_cast<TagStateNikhil*>(&state);
    bool terminal_state = tagStateNikhil_state->Step(action, rand_num, reward, obs);
    return terminal_state;
}


}