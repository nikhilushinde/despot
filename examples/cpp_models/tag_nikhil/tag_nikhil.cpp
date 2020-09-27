#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;
#include "tag_state.h"
#include "tag_nikhil.h"

namespace despot {

/*
* ***********************************************************************************
* Tag Default Policy class: to get a LOWER BOUND in  DESPOT. 
* ***********************************************************************************
*/

class TagNikhil_HistoryPolicy: public DefaultPolicy {
// This is a history based rollout policy to get the lower bounds for TagNikhil in DESPOT
private:
    const TagNikhil *m_TagNikhil; // a copy of the TagNikhil class to access its functions and the environment attributes

public: 
    TagNikhil_HistoryPolicy(const DSPOMDP* model, ParticleLowerBound* bound):
    DefaultPolicy(model, bound){
        /*
        * Constructor to initialize the TagNikhil History based default policy. 
        * args:
        *   - model: the DSPOMDP model - TagNikhil model
        *   - bound: a particle lower bound to use after the finite random number stream that is 
        *            used for this default policy runs out. 
        */
        m_TagNikhil = (static_cast<const TagNikhil*>(model));
    }

    ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams, History& history) const{
        /*
        * This function returns the action to take when using this history based policy. The action
        * is based on the current belief particles, a stream of random numbers that indicate the scenarios
        * and the history of actions that has been taken. 
        * returns:
        *   - action that should be taken
        */ 
        /*
        * Policy METHODOLOGY: If there is no history just take a random action. If we just 
        * saw the opponent then tag the opponent. Else take an action that does not undo your 
        * past actions and does not run into the walls. 
        */

        // If the history is empty then take any random move. 
        if (history.Size() == 0) {
            return Random::RANDOM.NextInt(m_TagNikhil->NumActions() - 1);
        }

        // If the last observation saw the opponent then Tag immediately 
        OBS_TYPE saw_opponent = 1;
        if (history.LastObservation() == saw_opponent) {
            return m_TagNikhil->TAG;
        }

        vector<ACT_TYPE>feasible_actions;
        // get the robot position - since here all the robots have the same position dont need to get the most likely robot position
        int robX, robY;
        m_TagNikhil->MostLikelyRobPosition(particles, robX, robY);

        // don't double back and don't go into walls
        TagStateNikhil tempEnv(robX, robY, 0, 0); // temporary environment to test actions on - to prevent running into walls
        int max_action_num = m_TagNikhil->NumActions();
        ACT_TYPE double_back_action = tempEnv.getOppositeAction(history.LastAction());
        for (int act_num = 0; act_num < max_action_num; act_num++) {
            if ((act_num != tempEnv.TAG) && (act_num != double_back_action) && (!tempEnv.isInvalidStep(act_num, robX, robY)) ) {
                feasible_actions.push_back(act_num);
            }
        }

        // return a random feasible action
        int ret_feasible_action_index = Random::RANDOM.NextInt(feasible_actions.size());
        return feasible_actions[ret_feasible_action_index];
    }
};

/*
* ***********************************************************************************
* Manhattan distance based UPPER bound class
* ***********************************************************************************
*/

class TagNikhilManhattanUpperBound: public ParticleUpperBound, public BeliefUpperBound {
protected: 
    const TagNikhil *m_TagNikhil; // copy of tag nikhil to have to reference its attributes
public: 
    TagNikhilManhattanUpperBound(const TagNikhil *model): m_TagNikhil(model) {
        /*
        * Constructor for the manhattan distance based upper bound class
        */ 
    }

    double ManhattanDistanceCalc(tagStateStruct environmentState) const{
        /*
        * Utility function that is used to find the manhattan distance between the robot and 
        * the opponent in the specified environment state. 
        */
        return (abs(environmentState.robX - environmentState.oppX) + abs(environmentState.robY - environmentState.oppY));
    }

    using ParticleUpperBound::Value;
    double Value(const State &s) {
        const TagStateNikhil& state = static_cast<const TagStateNikhil&>(s);
        double dist = ManhattanDistanceCalc(state.get_envState());
        double discountedValue = -(1 - Globals::Discount(dist)) / (1 - Globals::Discount())
				+ TAG_REWARD_SUCCESS_TAG_NIKHIL_STATE * Globals::Discount(dist);
        return discountedValue;
    }

    using BeliefUpperBound::Value;
    double Value(const Belief* belief) const {
        const vector<State*>& particles = static_cast<const ParticleBelief*>(belief)->particles();
        double totalValue = 0;

        State *particle;
        const TagStateNikhil *state;
        tagStateStruct environmentState;
        double dist, discountedValue;
        for (int i = 0; i < particles.size(); i++) {
            particle = particles[i];
            state = static_cast<const TagStateNikhil*>(particle);
            environmentState = state->get_envState();
            dist = ManhattanDistanceCalc(environmentState);
            discountedValue = -(1 - Globals::Discount(dist)) / (1 - Globals::Discount())
				+ TAG_REWARD_SUCCESS_TAG_NIKHIL_STATE * Globals::Discount(dist);
            totalValue += state->weight * discountedValue;
        }
        return totalValue;
    }
};


/*
* ***********************************************************************************
* TagNikhil Functions: declared in tag_nikhil.h
* ***********************************************************************************
*/

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

double TagNikhil::GetMaxReward() const {
    /*
    * Returns the maximum possible reward value achievable. 
    */ 
    return TAG_REWARD_SUCCESS_TAG_NIKHIL_STATE;
}

ValuedAction TagNikhil::GetBestAction() const {
    // TODO: FINISH THIS FUNCTION. 
}

/*
* ***********************************************************************************
* MEMORY MANAGEMENT FUNCTIONS
* ***********************************************************************************
*/

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

State* TagNikhil::Copy(const State* particle) const {
    /*
    * Creates a copy of the state whose pointer is input
    */ 
    // TODO: FINISH COPY CONSTRUCTOR FOR TagStateNikhil class before you can use this function
    cerr << " TODO: HAVE NOT COMPLETED THE COPY CONSTRUCTOR FOR TagStateNikhil " << endl;
	exit(1);

    TagStateNikhil* state_ptr = memory_pool_.Allocate();
    *state_ptr = *static_cast<const TagStateNikhil*>(particle);
    state_ptr->SetAllocated();
    return state_ptr;
}

void TagNikhil::Free(State* particle) const {
    memory_pool_.Free(static_cast<TagStateNikhil*>(particle));
}

int TagNikhil::NumActiveParticles() const{
    /*
    * The number of particles that have been allocated using DESPOT's memory management
    */ 
    return memory_pool_.num_allocated();
}

void TagNikhil::MostLikelyRobPosition(const vector<State*>& particles, int &retRobX, int &retRobY) const {
    /*
    * Find the most likely robot position given all the particles that represents the belief. 
    * args:
    *   - particles: the particles usually from the belief
    * returns: by pass by reference
    *   - retRobX
    *   - retRobY
    */ 
    map <tagCoordsStruct, float> m;
    tagCoordsStruct tempCoords; 
    tagStateStruct tempTagState;
    TagStateNikhil *tempTagStateNikhilptr;

    float maxElementWeight = 0;
    tagCoordsStruct maxElement;
    for (int i = 0; i < particles.size(); i++) {
        tempTagStateNikhilptr = static_cast<TagStateNikhil*>(particles[i]);
        tempTagState = tempTagStateNikhilptr->get_envState();
        tempCoords.x = tempTagState.robX;
        tempCoords.y = tempTagState.robY;

        m[tempCoords] += tempTagStateNikhilptr->weight;
        if (m[tempCoords] > maxElementWeight) {
            maxElementWeight = m[tempCoords];
            maxElement = tempCoords;
        }
    }

    retRobX = maxElement.x;
    retRobY = maxElement.y;
}

}