/*
* Description: This file creates the class functions for the class described in "surgical_despot.h" - function descriptions for the environment to despot interface
* NOTE: this file only handles class uncertainty. 
*/
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>


#include "surgical_despot.h"
#include <math.h> // ceil

// include all the bounding functions 
#include "surgical_despot_bounds.cpp"

using std::cerr;
using std::cout;
using std::endl;
using std::vector; 

namespace despot {

/*
* ***********************************************************************************
* SurgicalDespot Functions: declared in surgical_despot.h
* ***********************************************************************************
*/

SurgicalDespot::SurgicalDespot() {
    /*
    * Default constructor function 
    */ 
}

int SurgicalDespot::NumActions() const{
    /*
    * NOTE: Change this to alter the set of allowable actions you want to use 
    * This function returns the number of possible actions that can be taken in the environment. 
    * There are 7 actions that each arm can take - 0: xRight, 1: xLeft, 2: yUp, 3: yDown, 4:thetaUp, 5:thetaDown, 6:stay
    * 
    * In addition we only allow one arm to move at a time for now. 
    * However the action of all arms staying in the same place is not allowed. 
    * 
    * Methodology: 
    * eg: 2 arms - 0, 1, 2, 3, 4, 5 - respective actions on the first arm and no action on the second arm
    * 6, 7, 8, 9, 10, 11 - subtract 6 and the respective action is then used for the second arm and no action on the first
    */
    return total_num_actions_g();
}

bool SurgicalDespot::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const {
    /*
    * Deterministic simulative model that simulates the state taking a determinsitic step with the action and observing
    * the environment with its observation model. This is made deterministic by the use of the specified random number. 
    * 
    * args:
    *   - state: the environment state that you are taking the step in: here an object of class environment
    *   - rand_num: random number 
    *   - action: a number: [0, this->NumActions()) representing an action in the environment
    * return: by pass by reference:
    *   - reward: the reward from that action
    *   - obs: observation - must be an integer
    */ 

    // set up
    environment *environment_state = static_cast<environment*>(&state);


    robotArmActions action_list[NUM_ROBOT_ARMS_g];
    int actionNum = static_cast<int>(action);
    int_to_action_array_g(actionNum, action_list, NUM_ROBOT_ARMS_g);
    
    // environment step
    bool error = false; // returns if the action caused an error 
    float step_cost = 0;
    environment_state->step(action_list, error, step_cost);
    
    // environment observe
    float observed_obstacle_classes[NUM_OBSTACLES_g];
    environment_state->observe_classes(observed_obstacle_classes, NUM_OBSTACLES_g, rand_num);

    // convert observation to integer 
    obs = environment_state->class_observations_to_obstype(observed_obstacle_classes, NUM_OBSTACLES_g);
    
    // set reward and terminal status
    bool at_terminal_state = false;
    if (environment_state->at_goal()) {
        at_terminal_state = true;
        reward = TERMINAL_REWARD_g - step_cost;
    } else {
        reward = 0 - step_cost;
    }

    return at_terminal_state;

}


double SurgicalDespot::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {
    /*
    * This function returns the probability of seeing observation obs from state s given that 
    * we got to state s using action a. 
    * args:
    *   - obs: the observation seen from state gotten to by action
    *   - state: state from which finding the probability of observing obs
    *   - action: action taken to get to state
    */ 

    // TODO: FINISH THIS FUNCTION
    const environment *environment_state = static_cast<const environment*>(&state);
    return environment_state->get_class_obs_prob(obs, action);
}


State* SurgicalDespot::CreateStartState(std::string type) const {
    /*
    * Creates a random start state of type "environment". This state is allocated
    * in dynamic memory and the pointer to this state is returned. This creates a new
    * environment by sampling obstacle classes
    * args:
    *   - type: string indicating the type of method used to generate the random start state 
    *           can be used for different priors on the random start state
    * returns:
    *   - pointer of type state to the random state object that was created in dynamic memory
    */ 
    //double rand_num;
    //int current_obs_k_index;
    //float init_obstacle_ks[NUM_OBSTACLES_g];
    //double probabilityDistrib[NUM_OBS_K_CLASSES_g];
    //double uniform_probability = 1.0/static_cast<double>(NUM_OBS_K_CLASSES_g);
    //std::fill(probabilityDistrib, probabilityDistrib + NUM_OBS_K_CLASSES_g, uniform_probability);

    environment *new_env = new environment();

    /*
    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        rand_num = static_cast<double>(rand())/static_cast<double>(RAND_MAX);
        current_obs_k_index = random_number_to_index(probabilityDistrib, NUM_OBS_K_CLASSES_g, rand_num);
        init_obstacle_ks[obs_num] = all_possible_obs_ks_g[current_obs_k_index];
    }
    new_env->set_obstacle_ks(init_obstacle_ks);
    */

    float init_obstacle_ks[NUM_OBSTACLES_g];
    new_env->get_obstacle_ks(init_obstacle_ks, NUM_OBSTACLES_g);
    cout << "Created start state with initial obstacle ks: ";
    for (int i = 0; i < NUM_OBSTACLES_g; i++) {
        cout <<  init_obstacle_ks[i] << ", ";
    }
    cout << endl;
    cout << "is start state at goal: " << new_env->at_goal() << endl;
    cout << "the number of actions: " << NumActions() << endl;

    return  new_env;
}

Belief* SurgicalDespot::InitialBelief(const State* start, std::string type) const {
    /*
    * Creates the initial belief for the problem. Uses a uniform belief on all possible class configurations. 
    * args:
    *   - start: a start state that can be used to bias the initial belief
    *   - type: a string that can be used to select different types of priors for the belief
    * returns:
    *   - belief: The type belief is in the despot code
    */ 

    // TODO: FINISH THIS FUNCTION
    if (type == "DEFAULT" || type == "PARTICLE") {
        vector <State*> particles;
        double total_num_states = pow(NUM_OBS_K_CLASSES_g, NUM_OBSTACLES_g);
        double uniform_particle_weight = 1.0/total_num_states;
        cout << "The uniform particle weight is: " << uniform_particle_weight << endl;
        environment *new_particle_state;
        cout << "IN INITIAL BELIEF FOR SurgicalDespot" << endl;

        // TODO: CHANGE FROM HARD CODING FOR 3 OBSTACLES
        if (NUM_OBSTACLES_g != 1 && NUM_OBSTACLES_g != 2 && NUM_OBSTACLES_g != 3 && NUM_OBSTACLES_g != 4) {
            cerr << "INITIAL BELIEF ONLY HARD CODED FOR 3 or 4 OBSTACLES AT THE MOMENT" << endl;
            exit(1);
        }
        const environment *environment_start_state = static_cast<const environment *>(start);
        float start_state_obs_ks[NUM_OBSTACLES_g];
        environment_start_state->get_obstacle_ks(start_state_obs_ks, NUM_OBSTACLES_g);
        double particle_weight;

        
        // TODO: REMOVE THIS - FOR DEBUGGING - CUSTOM INITIAL BELIEF
        // custom initial belief prior for the 2 particle case JUST FOR DEBUGGING - REMOVE THIS PLEASE!!!!!!
        if (NUM_OBSTACLES_g == 2) {
            // have there just be 2 possible particles with 50 50 probability for testing. 
            environment *new_particle_state_1 = static_cast<environment *>(Allocate(-1, 0.5));
            float init_obstacle_ks_1[2] = {1e6, 1e-11};
            new_particle_state_1->set_obstacle_ks(init_obstacle_ks_1);
            environment *new_particle_state_2 = static_cast<environment *>(Allocate(-1, 0.5));
            float init_obstacle_ks_2[2] = {1e-11, 1e6}; // the true one
            new_particle_state_2->set_obstacle_ks(init_obstacle_ks_2);

            particles.push_back(new_particle_state_1);
            particles.push_back(new_particle_state_2);
            cout << "The number of particles created: " << particles.size() << endl;
            return new ParticleBelief(particles, this, NULL, false);
        } else {
            // force error so you remember to REMOVE this
            cerr << "in INITIAL BELIEF SURGICAL DESPOT - forgot to remove the testing code!!!!!!" << endl;
            exit(1);
        }
        // TODO: END REMOVE THIS - FOR DEBUGGING - CUSTOM INITIAL BELIEF
        

        // 1 obstacle case
        if (NUM_OBSTACLES_g == 1) {
            float init_obstacle_ks[NUM_OBSTACLES_g] = {0};
            for (int obs1 = 0; obs1 < NUM_OBS_K_CLASSES_g; obs1++) {
                if (INITIAL_BELIEF_TYPE_g == "TRUE_CLASS") {
                    // TRUE CLASS STRATEGY
                    particle_weight = 1;
                    // obstacle 1
                    if (all_possible_obs_ks_g[obs1] == start_state_obs_ks[0]) {
                        particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                    } else {
                        particle_weight = particle_weight * ((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                    }
                } else if (INITIAL_BELIEF_TYPE_g == "CLASS_SET") {
                    // CLASS SET STRATEGY
                    particle_weight = class_set_init_belief_biased_distrib_g[obs1];
                } else {
                    // DEFAULT
                    particle_weight = uniform_particle_weight;
                }

                // only add particle if it has a nonzero probability 
                if (particle_weight > 0) {
                    new_particle_state = static_cast<environment *>(Allocate(-1, particle_weight));
                    init_obstacle_ks[0] = all_possible_obs_ks_g[obs1];
                    new_particle_state->set_obstacle_ks(init_obstacle_ks);
                    particles.push_back(new_particle_state);
                }
            }
        }
        // 2 obstacle case
        else if (NUM_OBSTACLES_g == 2) {
            float init_obstacle_ks[NUM_OBSTACLES_g] = {0};
            for (int obs1 = 0; obs1 < NUM_OBS_K_CLASSES_g; obs1++) {
                for (int obs2 = 0; obs2 < NUM_OBS_K_CLASSES_g; obs2++) {
                    
                    if (INITIAL_BELIEF_TYPE_g == "TRUE_CLASS") {
                        // TRUE CLASS STRATEGY
                        particle_weight = 1;
                        // obstacle 1
                        if (all_possible_obs_ks_g[obs1] == start_state_obs_ks[0]) {
                            particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                        } else {
                            particle_weight = particle_weight * ((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                        }
                        // obstacle 2
                        if (all_possible_obs_ks_g[obs2] == start_state_obs_ks[1]) {
                            particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                        } else {
                            particle_weight = particle_weight *((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                        }

                    } else if (INITIAL_BELIEF_TYPE_g == "CLASS_SET") {
                        // CLASS SET STRATEGY
                        particle_weight = class_set_init_belief_biased_distrib_g[obs1] * class_set_init_belief_biased_distrib_g[obs2];
                    } else {
                        // DEFAULT
                        particle_weight = uniform_particle_weight;
                    }

                    // only add particle if it has a nonzero probability 
                    if (particle_weight > 0) {
                        new_particle_state = static_cast<environment *>(Allocate(-1, particle_weight));
                        init_obstacle_ks[0] = all_possible_obs_ks_g[obs1];
                        init_obstacle_ks[1] = all_possible_obs_ks_g[obs2];
                        new_particle_state->set_obstacle_ks(init_obstacle_ks);
                        particles.push_back(new_particle_state);
                    }
                }
            }
        }
        // 3 obstacle case
        else if (NUM_OBSTACLES_g == 3) {
            float init_obstacle_ks[NUM_OBSTACLES_g] = {0};
            for (int obs1 = 0; obs1 < NUM_OBS_K_CLASSES_g; obs1++) {
                for (int obs2 = 0; obs2 < NUM_OBS_K_CLASSES_g; obs2++) {
                    for (int obs3 = 0; obs3 < NUM_OBS_K_CLASSES_g; obs3++) {
                        
                        if (INITIAL_BELIEF_TYPE_g == "TRUE_CLASS") {
                            // TRUE CLASS STRATEGY
                            particle_weight = 1;
                            // obstacle 1
                            if (all_possible_obs_ks_g[obs1] == start_state_obs_ks[0]) {
                                particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                            } else {
                                particle_weight = particle_weight * ((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                            }
                            // obstacle 2
                            if (all_possible_obs_ks_g[obs2] == start_state_obs_ks[1]) {
                                particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                            } else {
                                particle_weight = particle_weight * ((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                            }
                            // obstacle 3
                            if (all_possible_obs_ks_g[obs3] == start_state_obs_ks[2]) {
                                particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                            } else {
                                particle_weight = particle_weight * ((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                            }
                            
                        } else if (INITIAL_BELIEF_TYPE_g == "CLASS_SET") {
                            // CLASS SET STRATEGY
                            particle_weight = class_set_init_belief_biased_distrib_g[obs1] * class_set_init_belief_biased_distrib_g[obs2] *
                                class_set_init_belief_biased_distrib_g[obs3];
                        } else {
                            // DEFAULT
                            particle_weight = uniform_particle_weight;
                        }

                        // only add particle if it has a nonzero probability 
                        if (particle_weight > 0) {
                            new_particle_state = static_cast<environment *>(Allocate(-1, particle_weight));
                            init_obstacle_ks[0] = all_possible_obs_ks_g[obs1];
                            init_obstacle_ks[1] = all_possible_obs_ks_g[obs2];
                            init_obstacle_ks[2] = all_possible_obs_ks_g[obs3];
                            new_particle_state->set_obstacle_ks(init_obstacle_ks);
                            particles.push_back(new_particle_state);
                        }
                    }
                }
            }
        } 
        // 4 obstacle case
        else if (NUM_OBSTACLES_g == 4) {
            float init_obstacle_ks[NUM_OBSTACLES_g] = {0};
            for (int obs1 = 0; obs1 < NUM_OBS_K_CLASSES_g; obs1++) {
                for (int obs2 = 0; obs2 < NUM_OBS_K_CLASSES_g; obs2++) {
                    for (int obs3 = 0; obs3 < NUM_OBS_K_CLASSES_g; obs3++) {
                        for (int obs4 = 0; obs4 < NUM_OBS_K_CLASSES_g; obs4++) {

                            if (INITIAL_BELIEF_TYPE_g == "TRUE_CLASS") {
                                // TRUE CLASS STRATEGY
                                particle_weight = 1;
                                // obstacle 1
                                if (all_possible_obs_ks_g[obs1] == start_state_obs_ks[0]) {
                                    particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                                } else {
                                    particle_weight = particle_weight * ((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                                }
                                // obstacle 2
                                if (all_possible_obs_ks_g[obs2] == start_state_obs_ks[1]) {
                                    particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                                } else {
                                    particle_weight = particle_weight * ((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                                }
                                // obstacle 3
                                if (all_possible_obs_ks_g[obs3] == start_state_obs_ks[2]) {
                                    particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                                } else {
                                    particle_weight = particle_weight * ((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                                }
                                // obstacle 4
                                if (all_possible_obs_ks_g[obs4] == start_state_obs_ks[3]) {
                                    particle_weight = particle_weight * TRUE_INITIAL_BIAS_PROBABILITY_g;
                                } else {
                                    particle_weight = particle_weight * ((1 - TRUE_INITIAL_BIAS_PROBABILITY_g)/(NUM_OBS_K_CLASSES_g - 1));
                                }
                                
                            } else if (INITIAL_BELIEF_TYPE_g == "CLASS_SET") {
                                // CLASS SET STRATEGY
                                particle_weight = class_set_init_belief_biased_distrib_g[obs1] * class_set_init_belief_biased_distrib_g[obs2] *
                                    class_set_init_belief_biased_distrib_g[obs3] * class_set_init_belief_biased_distrib_g[obs4];
                            } else {
                                // DEFAULT
                                particle_weight = uniform_particle_weight;
                            }

                            // only add particle if it has a nonzero probability 
                            if (particle_weight > 0) {
                                new_particle_state = static_cast<environment *>(Allocate(-1, particle_weight));
                                init_obstacle_ks[0] = all_possible_obs_ks_g[obs1];
                                init_obstacle_ks[1] = all_possible_obs_ks_g[obs2];
                                init_obstacle_ks[2] = all_possible_obs_ks_g[obs3];
                                init_obstacle_ks[3] = all_possible_obs_ks_g[obs4];
                                new_particle_state->set_obstacle_ks(init_obstacle_ks);
                                particles.push_back(new_particle_state);
                            }
                        }
                    }
                }
            }
        }

        // TODO: REMOVE THIS
        cout << "INITIAL BELIEF: the number of particles created: " << particles.size() << endl;
        // TODO: END REMOVE THIS

        return new ParticleBelief(particles, this);
    } else {
        cerr << "[TagNikhil::InitialBelief] Unsupported belief type: " << type << endl;
		exit(1);
    }
}

/*
* ***********************************************************************************
* MEMORY MANAGEMENT FUNCTIONS
* ***********************************************************************************
*/

State* SurgicalDespot::Allocate(int state_id, double weight) const {
    /*
    * REQUIRED: allocate memory for the dsepot algorithm using their specific memory handling
    * Sets the state id and the weight and returns a pointer to the state that has been allocated
    */ 
    environment* state_ptr = memory_pool_.Allocate();
	state_ptr->state_id = state_id;
	state_ptr->weight = weight;
	return state_ptr;
}

State* SurgicalDespot::Copy(const State* particle) const {
    /*
    * Creates a copy of the state whose pointer is input
    */ 
    environment* state_ptr = memory_pool_.Allocate();
    *state_ptr = *static_cast<const environment*>(particle);
    state_ptr->SetAllocated();
    return state_ptr;
}

void SurgicalDespot::Free(State* particle) const {
    memory_pool_.Free(static_cast<environment*>(particle));
}

int SurgicalDespot::NumActiveParticles() const{
    /*
    * The number of particles that have been allocated using DESPOT's memory management
    */ 
    return memory_pool_.num_allocated();
}

/*
* ***********************************************************************************
* Bounding Functions for heuristic search
* ***********************************************************************************
*/

double SurgicalDespot::GetMaxReward() const {
    /*
    * Returns the maximum possible reward value achievable. 
    */ 
    return TERMINAL_REWARD_g;
}

ScenarioUpperBound* SurgicalDespot::CreateScenarioUpperBound(std::string name, std::string particle_bound_name) const{
    /*
    * Create the Upper Bound for the TagNikhil class. 
    */ 
    if (name == "DEFAULT" || name == "TRIVIAL" || name == "EUCLIDEAN") {
        return new SurgicalDespotEuclideanUpperBound(this);
        //cout << "CREATE A STAR UPPER BOUND" << endl << endl;
        //return new SurgicalDespotAstarMultiThreadUpperBound(this);  
    } else if (name == "ASTAR") {
        return new SurgicalDespotAstarUpperBound(this);  
    } else {
        cerr << "Specified Upper Bound: " << name << " is NOT SUPPORTED!" << endl;
        exit(1);
    }
}


ValuedAction SurgicalDespot::GetBestAction() const {
    /*
    * This function returns the best action to be taken given nothing. This action is used to calculate the 
    * trivial lower bound after the number of random number in the stream expires. 
    */ 
    if (USE_CONSTANT_MOVEMENT_COST_g) {
        return ValuedAction(xRight, CONSTANT_MOVEMENT_COST_g);  // just moves the first arm  to the right
    } else {
        return ValuedAction(xRight, XY_STEP_SIZE_g);
    }
}

ScenarioLowerBound* SurgicalDespot::CreateScenarioLowerBound(string name, string particle_bound_name) const{
    /*
    * Creates the lower bound for the TagNikhil class. 
    */ 
    const DSPOMDP *model = this;
    if (name == "TRIVIAL" || name == "DEFAULT" || name == "SCHR") {
        // Use the smart history based default rollout policy to create the lower bound
        //return new TagNikhilHistoryPolicy(model, CreateParticleLowerBound(particle_bound_name)); 
        // the create particle lower bound function is in DSPOMDP or pomdp files - uses the GetBestAction to create a trivial lower bound
        
        //cout << "TODO: CHANGE THIS BACK TO A STAR AFTER TESTING SCHR" << endl;
        //return new SurgicalDespotCloserHistoryPolicy(model, CreateParticleLowerBound(particle_bound_name));
        
        //cout << "CREATED A STAR LOWER BOUND " << endl << endl;
        //return new SurgicalDespotAstarScenarioLowerBound(model);

        //cout << "CREATED MULTI THREADED closer history lower bound " << endl << endl;
        //return new SurgicalDespotCloserHistoryPolicy_multiThread(model);

        //cout << "create Astar multi threaded based lower bound" << endl;
        //return new SurgicalDespotAstarMultiThreadPolicy(model, CreateParticleLowerBound(particle_bound_name));
        //return new SurgicalDespotAstarPolicy(model, CreateParticleLowerBound(particle_bound_name));

        cout << "create Astar based parameter averaging policy" << endl;
        return new SurgicalDespotAstar_ParamAvgClass_LowerBound(model);
    
    } else if (name == "ASTAR") {
        cout << "CREATED A STAR LOWER BOUND " << endl << endl;
        return new SurgicalDespotAstarScenarioLowerBound(model);
    } else if (name == "SCHR") {
        return new SurgicalDespotCloserHistoryPolicy(model, CreateParticleLowerBound(particle_bound_name)); 
    } else if (name == "SHR") {
        return new SurgicalDespotHistoryPolicy(model, CreateParticleLowerBound(particle_bound_name));
    } else {
        cerr << "Specified Lower Bound " << name << " is NOT SUPPORTED!";
        exit(1);
    }
}



/*
* ***********************************************************************************
* Display functions
* ***********************************************************************************
*/

void SurgicalDespot::PrintState(const State& state, std::ostream& out) const{
    /*
    * Prints the environment state. Also renders the environment in opencv using the renderer
    */ 
    const environment* environment_state = static_cast<const environment *>(&state);
    environment_state->printState();
    renderer.render_environment(*environment_state);
}

void SurgicalDespot::PrintObs(const State& state, OBS_TYPE obs, std::ostream& out) const {
    /*
    * Print what the observation that is seen from the state means to the out stream. 
    */ 

    cout << "OBSERVATION: " << endl;
    cout << "Obs type uint 64: " << obs;

    const environment* environment_state = static_cast<const environment *>(&state);

    int digit_multiplier;
    if (NUM_OBS_K_CLASSES_g < 10) {
        digit_multiplier = 10;
    } else {
        digit_multiplier = 100;
    }
    int current_obstacle_k_index;
    cout << "Observed obstacle classes: ";
    for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
        // get the class observation from the OBS_TYPE number
        int mod_num = pow(digit_multiplier, obs_num + 1);
        current_obstacle_k_index = obs % mod_num;
        int div_num = pow(digit_multiplier, obs_num);
        current_obstacle_k_index = static_cast<int>(current_obstacle_k_index/div_num);
        current_obstacle_k_index -= 1; // NOTE: remember the 1 shift to have 0 as the default

        if (current_obstacle_k_index >= 0) {
            cout << all_possible_obs_ks_g[current_obstacle_k_index] << ", ";
        } else {
            cout <<  DEFAULT_NOTOBSERVED_OBS_K_g << ", ";
        }
    }
    cout << endl;

    cout << "Probability of observation: " << environment_state->get_class_obs_prob(obs, 0);
    
    /*
    // true state of the obstacles
    float true_environment_obstacle_ks[NUM_OBSTACLES_g];
    environment_state->get_obstacle_ks(true_environment_obstacle_ks, NUM_OBSTACLES_g);
    cout << "True obstacle classes: ";
    for (int i = 0; i < NUM_OBSTACLES_g; i++) {
        cout <<  true_environment_obstacle_ks[i] <<  ", ";
    }
    cout << endl;

    cout << "True obstacle classes as a OBS_TYPE: " << environment_state->class_observations_to_obstype(true_environment_obstacle_ks) << endl;
    */
}

void SurgicalDespot::PrintAction(ACT_TYPE action, std::ostream& out)  const {
    /*
    * Print what the action means to the outstream.
    */  
    robotArmActions action_array[NUM_ROBOT_ARMS_g];
    int_to_action_array_g(action, action_array, NUM_ROBOT_ARMS_g);
    cout << "ACTIONS: actions list: ";
    for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
        if (action_array[arm_num] == xRight) {
            cout << "xRight" << ", ";
        } else if (action_array[arm_num] == xLeft) {
            cout << "xLeft" << ", ";
        } else if (action_array[arm_num] == yUp) {
            cout << "yUp" << ", ";
        } else if (action_array[arm_num] == yDown) {
            cout << "yDown" << ", ";
        } else if (action_array[arm_num] == thetaUp) {
            cout << "thetaUp" << ", ";
        } else if (action_array[arm_num] == thetaDown) {
            cout << "thetaDown" << ", ";
        } else if (action_array[arm_num] == stay) {
            cout << "stay" << ", ";
        }
    }
    cout << endl;
    cout << "The action number was: " <<  action << endl;
    cout << "The found action array was: "; 
    for (int i = 0; i < NUM_ROBOT_ARMS_g; i++) {
        cout << action_array[i] << ", ";
    }
    cout << endl;
}

void SurgicalDespot::PrintBelief(const Belief& belief, std::ostream& out) const {
    /*
    * Print what the belief is to the outstream. 
    */ 
    // Methodology: keep track of how many particles belong to each type of obstacle k configurations with a map and print that. 

    const vector<State*>& particles = static_cast<const ParticleBelief&>(belief).particles();
    std::map<OBS_TYPE, std::pair<int, double>> obstacle_k_particles_map;// map from obstacle ks to number of particles with that characteristic, and total weight
    
    float current_obstacle_ks[NUM_OBSTACLES_g];
    for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
        const environment *environment_state = static_cast<const environment*>(particles[particle_num]);
        environment_state->get_obstacle_ks(current_obstacle_ks, NUM_OBSTACLES_g);

        double current_obsks_key = environment_state->class_observations_to_obstype(current_obstacle_ks, NUM_OBSTACLES_g);
        if (obstacle_k_particles_map.find(current_obsks_key) == obstacle_k_particles_map.cend()) {
            obstacle_k_particles_map[current_obsks_key].first = 1;
            obstacle_k_particles_map[current_obsks_key].second = environment_state->weight;
        } else {
            obstacle_k_particles_map[current_obsks_key].first += 1;
            obstacle_k_particles_map[current_obsks_key].second += environment_state->weight;
        }
    }

    // print the results
    int digit_multiplier;
    if (NUM_OBS_K_CLASSES_g < 10) {
        digit_multiplier = 10;
    } else {
        digit_multiplier = 100;
    }
    int current_obstacle_k_index;
    
    for (auto it = obstacle_k_particles_map.begin(); it != obstacle_k_particles_map.end(); it++) {
        OBS_TYPE obs = it->first;
        cout << "obstacle ks: {";
        for (int obs_num = 0; obs_num < NUM_OBSTACLES_g; obs_num++) {
            // get the class observation from the OBS_TYPE number
            int mod_num = pow(digit_multiplier, obs_num + 1);
            current_obstacle_k_index = obs % mod_num;
            int div_num = pow(digit_multiplier, obs_num);
            current_obstacle_k_index = static_cast<int>(current_obstacle_k_index/div_num);
            current_obstacle_k_index -= 1; // NOTE: remember the 1 shift to have 0 as the default

            if (current_obstacle_k_index >= 0) {
                cout << all_possible_obs_ks_g[current_obstacle_k_index] << ", ";
            } else {
                cout <<  DEFAULT_NOTOBSERVED_OBS_K_g << ", ";
            }
        }
        cout << "}: " << it->second.first << " : " << it->second.second;
        cout << "    ||    ";
    }
    cout << endl << endl << endl;

}

} // end namespace despot