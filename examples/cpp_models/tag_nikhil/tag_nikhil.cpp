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
    TagStateNikhil *tagStateNikhil_state = static_cast<TagStateNikhil*>(&state);
    bool terminal_state = tagStateNikhil_state->Step(action, rand_num, reward, obs);
    return terminal_state;
}

double TagNikhil::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {
    /*
    * This function returns the probability of seeing observation obs from state s given that 
    * we got to state s using action a. 
    * args:
    *   - obs: the observation seen from state gotten to by action
    *   - state: state from which finding the probability of observing obs
    *   - action: action taken to get to state
    */ 
    const TagStateNikhil *tagStateNikhil_state = static_cast<const TagStateNikhil *>(&state);
    return tagStateNikhil_state->ObsProb(obs, action);
}

State* TagNikhil::CreateStartState(std::string type = "DEFAULT") const{
    /*
    * Creates a random start state of type TagNikhilState. This state is allocated
    * in dynamic memory and the pointer to this state is returned. 
    * args:
    *   - type: string indicating the type of method used to generate the random start state 
    *           can be used for different priors on the random start state
    * returns:
    *   - pointer of type state to the random state object that was created in dynamic memory
    */ 
    return new TagStateNikhil();
}

Belief* TagNikhil::InitialBelief(const State* start, std::string type = "DEFAULT") const {
    /*
    * Creates the initial belief for the problem. 
    * args:
    *   - start: a start state that can be used to bias the initial belief
    *   - type: a string that can be used to select different types of priors for the belief
    * returns:
    *   - belief: The type belief is in the despot code
    */ 
    // only support "DEFAULT" and "PARTICLE" belief types - uniform belief over all the positions
    if (type == "DEFAULT" || type == "PARTICLE") {
        vector <State*> particles;
        double uniform_particle_weight = (1/(NUM_XY_POS_TAG_NIKHIL_STATE * NUM_XY_POS_TAG_NIKHIL_STATE));
        TagStateNikhil *new_particle_state;
        for (int rob_num = 0; rob_num < NUM_XY_POS_TAG_NIKHIL_STATE; rob_num++) {
            for (int opp_num = 0; opp_num < NUM_XY_POS_TAG_NIKHIL_STATE; opp_num++) {
                new_particle_state = static_cast<TagStateNikhil *>(Allocate(-1, uniform_particle_weight));
                new_particle_state->InitStateFromInts(rob_num, opp_num);
                particles.push_back(new_particle_state);
            }
        }
        return new ParticleBelief(particles, this);
    } else {
        cerr << "[TagNikhil::InitialBelief] Unsupported belief type: " << type << endl;
		exit(1);
    }
}

State* TagNikhil::Allocate(int state_id, double weight) const {
    /*
    * REQUIRED: allocate memory for the dsepot algorithm using their specific memory handling
    * Sets the state id and the weight and returns a pointer to the state that has been allocated
    */ 
    TagStateNikhil* state_ptr = memory_pool_.Allocate();
	state_ptr->state_id = state_id;
	state_ptr->weight = weight;
	return state_ptr;
}

}