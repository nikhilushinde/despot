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

using std::cerr;
using std::cout;
using std::endl;
using std::vector; 

namespace despot {

/*
* ***********************************************************************************
* Tag Default Policy class: to get a LOWER BOUND in  DESPOT. 
* ***********************************************************************************
*/
class SurgicalDespotHistoryPolicy: public DefaultPolicy {
/*
* This is a history based rollout policy to get the lower bounds for the environment in DESPOT
* Policy methodology: 
*   - If there is no history take a random action that does not run into walls. If there is a history
*   take actions that do not double back on the last action. 
*/
private:
    const SurgicalDespot *surgicalDespot_m; // pointer to the SurgicalDespot 

public:
    SurgicalDespotHistoryPolicy(const DSPOMDP* model, ParticleLowerBound* bound):
    DefaultPolicy(model, bound) {
        /*
        * Constructor to initialize the History based default policy
        * args:
        *   - model: DSPOMDP model - SurgicalDespot
        *   - bound: a particle lower bound to use after the finite random number streams used when the default policy runs out
        */ 
        surgicalDespot_m = static_cast<const SurgicalDespot *>(model);
    }

    bool isFeasibleAction(int action_num, environment& environment_state) const {
        /*
        * Returns if the specified action number is feasible: in regards that it does not cause 
        * the robot arms to go out of bounds. 
        * 
        * NOTE: Does not check for obstacle related collisions as those may be different in different particles
        */ 
        robotArmActions action_array[NUM_ROBOT_ARMS_g];
        // convert the integer to actions
        int_to_action_array_g(action_num, action_array, NUM_ROBOT_ARMS_g);

        bool error; 
        float cost;
        environment_state.robObj_m.step(action_array, XY_STEP_SIZE_g, THETA_DEG_STEP_SIZE_g, error, cost);
        
        // if error robot object auto rollbacks - else we roll it back ourself
        if (error) {
            return false;
        } else {
            environment_state.robObj_m.state_rollback();
            return true;
        }
    }

    ACT_TYPE Action(const vector<State*>& particles, RandomStreams &streams, History& history) const{
        /*
        * This function returns the action to take when using this history based policy. The action
        * is based on the current belief particles, a stream of random numbers that indicate the scenarios
        * and the history of actions that has been taken. 
        * returns:
        *   - action that should be taken
        * 
        * Methodlogy: described above
        * NOTE: this policy only moves the first/0th indexed arm
        */
       
        // Since all environments have the same robot location - just use the robot location in a particle
        robotArmCoords env_state_robot_coords[NUM_ROBOT_ARMS_g];
        const environment* environment_state = static_cast<const environment *>(particles[0]);
        // create a copy of the environment to get valid arm actions
        environment cpy_environment_state = environment(*environment_state);
        
        vector <ACT_TYPE> feasible_actions;

        // If the history is empty then take a random move that does not run into the wall
        if (history.Size() == 0) {
            for (int action_num = 0; action_num < surgicalDespot_m->NumActions(); action_num++) {
                if (isFeasibleAction(action_num, cpy_environment_state)) {
                    feasible_actions.push_back(action_num);
                }
            }
        } else {
            for (int action_num = 0; action_num < surgicalDespot_m->NumActions(); action_num++) {
                if (isFeasibleAction(action_num, cpy_environment_state) && 
                !isReverseAction_g(action_num, history.LastAction())) {
                    feasible_actions.push_back(action_num);
                }
            }
        }
        
        
        if (feasible_actions.size() == 0) {
            cerr << "ERROR: in the lower bound policy action - should always have some feasible actions" << endl;
            exit(1);
        }
        int ret_feasible_action_index = Random::RANDOM.NextInt(feasible_actions.size());
        return feasible_actions[ret_feasible_action_index];
    }   
};


/*
* ***********************************************************************************
* Default Policy class: to get a LOWER BOUND in  DESPOT. 
* ***********************************************************************************
*/
class SurgicalDespotCloserHistoryPolicy: public DefaultPolicy {
/*
* This is a history based rollout policy to get the lower bounds for the environment in DESPOT
* Policy methodology: 
*   - If there is no history take a random action that does not run into walls. If there is a history
*   take actions that do not double back on the last action. 
*   - When selecting the random action: select actions that lead you closer to the  goal coordinate
*   in the environment with some probability and select actions completely at random with some other probability
*/
private:
    const SurgicalDespot *surgicalDespot_m; // pointer to the SurgicalDespot 
    double probability_to_move_closer;
public:
    SurgicalDespotCloserHistoryPolicy(const DSPOMDP* model, ParticleLowerBound* bound):
    DefaultPolicy(model, bound) {
        /*
        * Constructor to initialize the History based default policy
        * args:
        *   - model: DSPOMDP model - SurgicalDespot
        *   - bound: a particle lower bound to use after the finite random number streams used when the default policy runs out
        */ 
        surgicalDespot_m = static_cast<const SurgicalDespot *>(model);
        probability_to_move_closer = 0.8;
    }

    bool isFeasibleAction(int action_num, environment& environment_state) const {
        /*
        * Returns if the specified action number is feasible: in regards that it does not cause 
        * the robot arms to go out of bounds. 
        * 
        * NOTE: Does not check for obstacle related collisions as those may be different in different particles
        */ 
        robotArmActions action_array[NUM_ROBOT_ARMS_g];
        // convert the integer to actions
        int_to_action_array_g(action_num, action_array, NUM_ROBOT_ARMS_g);

        bool error; 
        float cost;
        environment_state.robObj_m.step(action_array, XY_STEP_SIZE_g, THETA_DEG_STEP_SIZE_g, error, cost);
        
        // if error robot object auto rollbacks - else we roll it back ourself
        if (error) {
            return false;
        } else {
            environment_state.robObj_m.state_rollback();
            return true;
        }
    }

    bool isTowardsGoal(int action_num, const environment& environment_state) const{
        /*
        * Returns true if the action brings the robot closer to the goal and false otherwise
        */ 
        robotArmActions action_array[NUM_ROBOT_ARMS_g];
        int_to_action_array_g(action_num, action_array, NUM_ROBOT_ARMS_g);
        robotArmCoords env_state_robot_coords[NUM_ROBOT_ARMS_g];
        environment_state.get_all_robot_arm_coords(env_state_robot_coords, NUM_ROBOT_ARMS_g);
        environmentCoords goal_coords;
        goal_coords = environment_state.get_goal_coord();

        bool goalToRight; // boolean indicator - true if goal to the right of arm 
        bool goalToTop; // boolean indicator - true if goal above arm 
        for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
            if (action_array[arm_num] == stay) {
                continue;
            } else {
                goalToRight = env_state_robot_coords[arm_num].x < goal_coords.x;
                goalToTop = env_state_robot_coords[arm_num].y < goal_coords.y;

                if (goalToRight && action_array[arm_num] == xRight) {
                    return true;
                }  else if (!goalToRight && action_array[arm_num] == xLeft) {
                    return true;
                }

                if (goalToTop && action_array[arm_num] == yUp) {
                    return true;
                } else if (!goalToTop && action_array[arm_num] == yDown) {
                    return true;
                }
            }
        }

        return false;
    }

    ACT_TYPE Action(const vector<State*>& particles, RandomStreams &streams, History& history) const{
        /*
        * This function returns the action to take when using this history based policy. The action
        * is based on the current belief particles, a stream of random numbers that indicate the scenarios
        * and the history of actions that has been taken. 
        * returns:
        *   - action that should be taken
        * 
        * Methodlogy: described above
        * NOTE: this policy only moves the first/0th indexed arm
        */
       
        // Since all environments have the same robot location - just use the robot location in a particle
        robotArmCoords env_state_robot_coords[NUM_ROBOT_ARMS_g];
        const environment* environment_state = static_cast<const environment *>(particles[0]);
        // create a copy of the environment to get valid arm actions
        environment cpy_environment_state = environment(*environment_state);
        
        vector <ACT_TYPE> feasible_actions;
        vector <ACT_TYPE> towards_goal_feasible_actions;

        // If the history is empty then take a random move that does not run into the wall
        if (history.Size() == 0) {
            for (int action_num = 0; action_num < surgicalDespot_m->NumActions(); action_num++) {
                if (isFeasibleAction(action_num, cpy_environment_state)) {
                    feasible_actions.push_back(action_num);
                    if (isTowardsGoal(action_num, cpy_environment_state)) {
                        towards_goal_feasible_actions.push_back(action_num);
                    }
                }
            }
        } else {
            for (int action_num = 0; action_num < surgicalDespot_m->NumActions(); action_num++) {
                if (isFeasibleAction(action_num, cpy_environment_state) && 
                !isReverseAction_g(action_num, history.LastAction())) {
                    feasible_actions.push_back(action_num);
                    if (isTowardsGoal(action_num, cpy_environment_state)) {
                        towards_goal_feasible_actions.push_back(action_num);
                    }
                }
            }
        }
        
        
        if (feasible_actions.size() == 0) {
            cerr << "ERROR: in the lower bound policy action - should always have some feasible actions" << endl;
            exit(1);
        }

        double rand_num = Random::RANDOM.NextDouble(0, 1);
        if (rand_num < probability_to_move_closer && towards_goal_feasible_actions.size() > 0) {
            return towards_goal_feasible_actions[Random::RANDOM.NextInt(towards_goal_feasible_actions.size())];
        }
        int ret_feasible_action_index = Random::RANDOM.NextInt(feasible_actions.size());
        return feasible_actions[ret_feasible_action_index];
    }   
};
/*
* ***********************************************************************************
* A star based Default Policy class: to get a LOWER BOUND in  DESPOT. 
* ***********************************************************************************
*/
class SurgicalDespotAstarPolicy:  public DefaultPolicy {
/*
* This is an A star based default rollout policy to get the lower bounds for the environment in DESPOT
* Policy methodology: 
*   - Do A star on all the environments in the belief and based on that take the best action.  
*/
private: 
    const SurgicalDespot *surgicalDespot_m; // pointer to the SurgicalDespot DSPOMDP model
public:
    SurgicalDespotAstarPolicy(const DSPOMDP* model, ParticleLowerBound* bound):
    DefaultPolicy(model, bound) {
        /*
        * Constructor ot initialize the A star based default policy
        * args:
        *   - model: DSPOMDP model - SurgicalDespot
        *   - bound: a particle lower bound to use after the finite random number streams used when the default policy runs out
        */ 
        surgicalDespot_m = static_cast<const SurgicalDespot *>(model);
    }

    ACT_TYPE Action(const vector<State*>&particles, RandomStreams &streams, History& history) const {
        /*
        * This function returns the action to take when using the A star based policy. The action
        * is based on the current belief particles. 
        * 
        * Methodology: Do A star on all the environments in the blief and based on that take the 
        * best action based on the votes based off of the action that has the "most" particle weight
        * behind it 
        * - if two environments are the same - don't redo A star and just use the last A star's value
        */ 
        astar_planner planner; 

        // map to store the A star values of particles 
        std::map<environment, ACT_TYPE> astar_best_action_map;
        // create a list where the index is the action number and the value is the weight towards that  action
        double action_weight_array[surgicalDespot_m->NumActions()];
        ACT_TYPE particle_chosen_action; 
        std::vector<ACT_TYPE> astar_found_path_actions;
        
        cout << "At A star for particle number: " << endl; 
        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            if (particle_num%10 == 0 || particle_num == particles.size() - 1) {
                cout << particle_num << ", " << endl;
            } else {
                cout << particle_num << ", ";
            }
            
            const environment * environment_state = static_cast<const environment *>(particles[particle_num]);
            std::map<environment, ACT_TYPE>::iterator it = astar_best_action_map.find(*environment_state);
            if (it != astar_best_action_map.cend()) {
                // have already run a star on this environment
                particle_chosen_action = astar_best_action_map[*environment_state];
                action_weight_array[particle_chosen_action] += environment_state->weight;
            } else {
                // run a star on the environment 
                planner.plan_a_star(*environment_state, false);
                planner.get_path(astar_found_path_actions);
                particle_chosen_action = astar_found_path_actions[0];
                astar_best_action_map[*environment_state] = particle_chosen_action;
                action_weight_array[particle_chosen_action] += environment_state->weight;
            }
        }
        cout << endl;
        // chose the best action - the index in the action_weight_array that has the greatest weight
        return static_cast<ACT_TYPE>(std::distance(action_weight_array, std::max_element(action_weight_array, action_weight_array + surgicalDespot_m->NumActions())));

    }
};

/*
* ***********************************************************************************
* A star based Default Policy class with MULTI THREADING: to get a LOWER BOUND in  DESPOT. 
* ***********************************************************************************
*/

static void multi_thread_Astar_getbestaction(const environment &planning_environment, ACT_TYPE &best_action) {
    /*
    * Function to enable multi thread planning with A star
    * args:
    *   - planning_environment: environment object on which to do A star
    * returns: by pass by reference
    *   - best_action: the best action to take as the first action in that planning environment
    */
    astar_planner planner;
    vector<ACT_TYPE>astar_found_path_actions;
    planner.plan_a_star(planning_environment);
    planner.get_path(astar_found_path_actions);
    best_action = astar_found_path_actions[0];
    return;
}

class SurgicalDespotAstarMultiThreadPolicy:  public DefaultPolicy {
/*
* This is an A star based default rollout policy to get the lower bounds for the environment in DESPOT
* Policy methodology: 
*   - Do A star on all the environments in the belief and based on that take the best action.  
*/
private: 
    const SurgicalDespot *surgicalDespot_m; // pointer to the SurgicalDespot DSPOMDP model
public:
    SurgicalDespotAstarMultiThreadPolicy(const DSPOMDP* model, ParticleLowerBound* bound):
    DefaultPolicy(model, bound) {
        /*
        * Constructor ot initialize the A star based default policy
        * args:
        *   - model: DSPOMDP model - SurgicalDespot
        *   - bound: a particle lower bound to use after the finite random number streams used when the default policy runs out
        */ 
        surgicalDespot_m = static_cast<const SurgicalDespot *>(model);
    }


    ACT_TYPE Action(const vector<State*>&particles, RandomStreams &streams, History& history) const {
        /*
        * This function returns the action to take when using the A star based policy. The action
        * is based on the current belief particles. 
        * 
        * Methodology: Do A star on all the environments in the blief and based on that take the 
        * best action based on the votes based off of the action that has the "most" particle weight
        * behind it 
        * - if two environments are the same - don't redo A star and just use the last A star's value
        */ 
        astar_planner planner; 

        // map to store the A star values of particles 
        std::map<environment, ACT_TYPE> astar_best_action_map;
        // create a list where the index is the action number and the value is the weight towards that  action
        double action_weight_array[surgicalDespot_m->NumActions()];
        ACT_TYPE particle_chosen_action; 

        //create a list of threads
        std::thread all_particle_threads[particles.size()];
        int num_created_threads = 0; // number of threads created to decide for joining.
        ACT_TYPE default_init_action = 0; // default action to initialize the map

        // loop to spawn all threads
        cout << "Spawning A star threads: " << endl;
        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            if (particle_num%10 == 0 || particle_num == particles.size() - 1) {
                cout << particle_num << ", " << endl;
            } else {
                cout << particle_num << ", ";
            }

            const environment * environment_state = static_cast<const environment *>(particles[particle_num]);
            std::map<environment, ACT_TYPE>::iterator it = astar_best_action_map.find(*environment_state);
            if (it == astar_best_action_map.cend()) {
                // spawn a thread to run Astar for this environment
                astar_best_action_map[*environment_state] = default_init_action;
                all_particle_threads[num_created_threads] = std::thread(multi_thread_Astar_getbestaction, std::ref(*environment_state), std::ref(astar_best_action_map[*environment_state]));
                num_created_threads++;
            } else {
                // have already spawned a thread for this environment
                continue;
            }

        }

        // wait for all the threads to join
        cout << "Waiting for Astar threads to join: " << endl;
        for (int thread_num = 0; thread_num < num_created_threads; thread_num++) {
            if (thread_num%10 == 0 || thread_num == particles.size() - 1) {
                cout << thread_num << ", " << endl;
            } else {
                cout << thread_num << ", ";
            }
            all_particle_threads[thread_num].join();
        }

        // populate the array to decide the best action
        cout << "Assigning A star per particle number: " << endl;
        for (int i = 0; i < particles.size(); i++) {
            const environment * environment_state = static_cast<const environment *>(particles[i]);
            action_weight_array[astar_best_action_map[*environment_state]] += environment_state->weight;
        }

        ACT_TYPE policy_action = std::distance(action_weight_array, std::max_element(action_weight_array, action_weight_array + surgicalDespot_m->NumActions()));
        cout << "Found Astar policy action: " << policy_action;
        return policy_action;
    }
};
/*
* ***********************************************************************************
* Astar based SCENARIO LOWER BOUND - Multi threaded support
* ***********************************************************************************
*/
static void multi_thread_Astar_getbestactionvalue(const environment &planning_environment, ACT_TYPE &best_action, double &best_discounted_value) {
    /*
    * Function to enable multi thread planning with A star
    * args:
    *   - planning_environment: environment object on which to do A star
    * returns: by pass by reference
    *   - best_action: the best action to take as the first action in that planning environment
    *   - best_value: the discounted value of the Astar algorithm - including the terminal reward
    */
    astar_planner planner;
    vector<ACT_TYPE>astar_found_path_actions;
    planner.plan_a_star(planning_environment);
    planner.get_path(astar_found_path_actions);
    best_action = astar_found_path_actions[0];
    best_discounted_value = planner.get_discounted_goal_value(astar_found_path_actions);
    return;
}

class SurgicalDespotAstarScenarioLowerBound: public ScenarioLowerBound {
private:
    const SurgicalDespot *surgicalDespot_m;
public: 
    SurgicalDespotAstarScenarioLowerBound(const DSPOMDP* model): ScenarioLowerBound(model) {
        /*
        * Default constructor for the Astar based scenario lower bound
        */ 
        surgicalDespot_m = static_cast<const SurgicalDespot *>(model);
    }

    ValuedAction Value(const vector<State*>& particles, RandomStreams& streams, History& history) const {
        /*
        * Computes the value of the lower bound given the scenario that is expressed by the vector of weighted particles
        * also returns the first action that is needed to get that value using an A star based policy. The action
        *  
        * Methodology: Do A star on all the environments in the blief and based on that take the 
        * best action based on the votes based off of the action that has the "most" particle weight
        * behind it 
        * - if two environments are the same - don't redo A star and just use the last A star's value
        */ 
        astar_planner planner;

        // map to store the A star values of particles 
        std::map<environment, std::pair<ACT_TYPE, double>> astar_best_actionvalue_map;
        // create a list where the index is the action number and the value is the weight towards that  action
        double action_weight_array[surgicalDespot_m->NumActions()];

        //create a list of threads
        std::thread all_particle_threads[particles.size()];
        int num_created_threads = 0; // number of threads created to decide for joining.
        ACT_TYPE default_init_action = 0; // default action to initialize the map
        double default_init_value = 0; // default value to initialize the map

        // loop to spawn all threads
        //cout << "Spawning A star threads: " << endl;
        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            /*
            if (particle_num%10 == 0 || particle_num == particles.size() - 1) {
                cout << particle_num << ", " << endl;
            } else {
                cout << particle_num << ", ";
            }
            */

            const environment * environment_state = static_cast<const environment *>(particles[particle_num]);
            std::map<environment, std::pair<ACT_TYPE, double>>::iterator it = astar_best_actionvalue_map.find(*environment_state);
            if (it == astar_best_actionvalue_map.cend()) {
                // spawn a thread to run Astar for this environment
                astar_best_actionvalue_map[*environment_state].first = default_init_action;
                astar_best_actionvalue_map[*environment_state].second = default_init_value;
                all_particle_threads[num_created_threads] = std::thread(multi_thread_Astar_getbestactionvalue, std::ref(*environment_state), 
                    std::ref(astar_best_actionvalue_map[*environment_state].first), std::ref(astar_best_actionvalue_map[*environment_state].second));
                num_created_threads++;
            } else {
                // have already spawned a thread for this environment
                continue;
            }
        }

        // wait for all the threads to join
        //cout << "Waiting for Astar threads to join: " << endl;
        for (int thread_num = 0; thread_num < num_created_threads; thread_num++) {
            all_particle_threads[thread_num].join();
        }

        // populate the array to decide the best action and get the total value
        double totalDiscountedValue = 0;
        //cout << "Assigning A star per particle number: " << endl;
        for (int i = 0; i < particles.size(); i++) {
            const environment * environment_state = static_cast<const environment *>(particles[i]);
            action_weight_array[astar_best_actionvalue_map[*environment_state].first] += environment_state->weight;
            totalDiscountedValue += (environment_state->weight*astar_best_actionvalue_map[*environment_state].second);
        }

        ACT_TYPE best_action = std::distance(action_weight_array, std::max_element(action_weight_array, action_weight_array + surgicalDespot_m->NumActions()));
        ValuedAction retValuedAction(best_action, totalDiscountedValue);
        return retValuedAction; 
    }
};


/*
* ***********************************************************************************
* Euclidean distance based UPPER bound class
* ***********************************************************************************
*/
class SurgicalDespotEuclideanUpperBound: public ParticleUpperBound, public BeliefUpperBound {
/*
* This class creates an upper bound on the reward for the environment using the metric of Euclidean 
* distance of the closest arm of the robot from the goal coordinate minus the goal radius and the 
* potential number of steps it would take to get there. 
*/
protected: 
    const SurgicalDespot *surgicalDespot_m; // pointer to DSPOMDP model of SurgicalDespot
public: 
    SurgicalDespotEuclideanUpperBound(const SurgicalDespot *model): surgicalDespot_m(model){
        /*
        * Constructor for the manhattan distances based upper bound class
        */ 
        cout << "Creating the EUCLIDEAN UPPER BOUND" << endl;
    }

    double EuclideanDistanceCalc(const environment &environment_state) const {
        /*
        * Utility function used to find the Euclidean distance from the closest arm of the robot to the goal coordinate minus the goal radius
        */ 
        robotArmCoords env_state_robot_coords[NUM_ROBOT_ARMS_g];
        environmentCoords env_state_goal_coord;
        environment_state.get_all_robot_arm_coords(env_state_robot_coords, NUM_ROBOT_ARMS_g);

        double min_dist;
        double arm_dist; 
        float x;
        float y;
        env_state_goal_coord = environment_state.get_goal_coord();
        for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
            x = static_cast<float>(env_state_robot_coords[arm_num].x);
            y = static_cast<float>(env_state_robot_coords[arm_num].y);

            arm_dist = static_cast<double>(sqrt(pow(x - env_state_goal_coord.x, 2) + pow(y - env_state_goal_coord.y, 2))) 
                - static_cast<double>(environment_state.get_goal_radius());
            arm_dist = std::min(std::max(arm_dist, static_cast<double>(0)), min_dist);
        }
        return min_dist;
    }

    using ParticleUpperBound::Value; 
    double Value(const State &s) const{
        const environment* environment_state = static_cast<const environment*>(&s);
        double dist = EuclideanDistanceCalc(*environment_state);
        int steps_to_goal = ceil(dist/XY_STEP_SIZE_g);

        double discountedValue;
        if (USE_CONSTANT_MOVEMENT_COST_g) {
            discountedValue = (-(1 - Globals::Discount(steps_to_goal)) / (1 - Globals::Discount()) * CONSTANT_MOVEMENT_COST_g ) + 
                TERMINAL_REWARD_g * Globals::Discount(steps_to_goal);
        } else {
            // if not using constant cost just return discounted terminal reward assuming no stage costs.  
            discountedValue = TERMINAL_REWARD_g * Globals::Discount(steps_to_goal - 1);
        }
        
        return discountedValue;
    }

    using BeliefUpperBound::Value;
    double Value(const Belief* belief) const {
        /*
        * Methodology: For the value upper bound only consider the value of the terminal reward after the discounting
        */ 

        const vector<State*>& particles = static_cast<const ParticleBelief*>(belief)->particles();
        double totalValue = 0;

        State *particle;
        const environment *environment_state;
        
        double dist, discountedValue;
        for (int i = 0; i < particles.size(); i++) {
            particle = particles[i];
            environment_state = static_cast<const environment*>(particle);
            
            dist = EuclideanDistanceCalc(*environment_state);
            int steps_to_goal = ceil(dist/XY_STEP_SIZE_g);
            double discountedValue;
            if (USE_CONSTANT_MOVEMENT_COST_g) {
                discountedValue = (-(1 - Globals::Discount(steps_to_goal)) / (1 - Globals::Discount()) * CONSTANT_MOVEMENT_COST_g ) + 
                    TERMINAL_REWARD_g * Globals::Discount(steps_to_goal);
            } else {
                // if not using constant cost just return discounted terminal reward assuming no stage costs.  
                discountedValue = TERMINAL_REWARD_g * Globals::Discount(steps_to_goal - 1);
            }
            totalValue += discountedValue;
        }
        return totalValue;
    }
};

/*
* ***********************************************************************************
* Astar based UPPER bound class
* ***********************************************************************************
*/
class SurgicalDespotAstarUpperBound: public ParticleUpperBound, public BeliefUpperBound {
/*
* This class creates an upper bound on the reward for the environment using the metric of
* the nondiscounted Astar value that is computed from running the algorithm. 
*/
private:
    const SurgicalDespot *surgicalDespot_m; // pointer to the DSPOMDP model of SurgicalDespot
public:
    SurgicalDespotAstarUpperBound(const SurgicalDespot *model): surgicalDespot_m(model) {
        /*
        *Constructor fo the A star based upper bound class
        */ 
        cout << "Creating the A star based upper bound" << endl;
    }

    using ParticleUpperBound::Value;
    double Value(const State &s) const {
        // Required function
        astar_planner planner;
        const environment* environment_state = static_cast<const environment*>(&s);
        planner.plan_a_star(*environment_state);
        double nonDiscountedUpperBoundValue = planner.get_goal_cost();
        return nonDiscountedUpperBoundValue;
    }

    using BeliefUpperBound::Value;
    double Value(const Belief* belief) const {
        /*
        * Compute the nondiscounted value of each particle using A star and average them 
        * to compute the belief upper bound value. 
        */ 
        astar_planner planner;

        std::map<environment, double> astar_best_value_map;
        const vector<State *>&particles = static_cast<const ParticleBelief*>(belief)->particles();
        double totalValue = 0;

        const environment *environment_state;
        double currentValue;

        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            const environment * environment_state = static_cast<const environment *>(particles[particle_num]);
            std::map<environment, double>::iterator it = astar_best_value_map.find(*environment_state);
            if (it != astar_best_value_map.cend()) {
                // have already run a star on this environment
                currentValue = astar_best_value_map[*environment_state];
                totalValue += (currentValue * environment_state->weight);
            } else {
                // run a star on the environment 
                planner.plan_a_star(*environment_state);
                currentValue = planner.get_goal_cost() + TERMINAL_REWARD_g;
                astar_best_value_map[*environment_state] = currentValue;
                totalValue += (currentValue * environment_state->weight);
            }
        }
        return totalValue;
    }
};

/*
* ***********************************************************************************
* Astar based UPPER bound class with MULTI THREADING
* ***********************************************************************************
*/
// supporting function for multi threading
static void multi_thread_Astar_getbestvalue(const environment &planning_environment, double &best_value) {
    /*
    * Function to enable multi thread planning with A star
    * args:
    *   - planning_environment: environment object on which to do A star
    * returns: by pass by reference
    *   - best_value: the upper bound value for the planning environment based on A star.
    */
    astar_planner planner;
    planner.plan_a_star(planning_environment);
    best_value = planner.get_goal_cost() + TERMINAL_REWARD_g;
    return;
}


class SurgicalDespotAstarMultiThreadUpperBound: public ParticleUpperBound, public BeliefUpperBound {
/*
* This class creates an upper bound on the reward for the environment using the metric of
* the nondiscounted Astar value that is computed from running the algorithm. 
*/
private:
    const SurgicalDespot *surgicalDespot_m; // pointer to the DSPOMDP model of SurgicalDespot
public:
    SurgicalDespotAstarMultiThreadUpperBound(const SurgicalDespot *model): surgicalDespot_m(model) {
        /*
        *Constructor fo the A star based upper bound class
        */ 
        cout << "Creating the A star based upper bound" << endl;
    }

    using ParticleUpperBound::Value;
    double Value(const State &s) const {
        // Required function
        astar_planner planner;
        const environment* environment_state = static_cast<const environment*>(&s);
        planner.plan_a_star(*environment_state);
        double nonDiscountedUpperBoundValue = planner.get_goal_cost();
        return nonDiscountedUpperBoundValue;
    }

    

    using BeliefUpperBound::Value;
    double Value(const Belief* belief) const {
        /*
        * Compute the nondiscounted value of each particle using A star and average them 
        * to compute the belief upper bound value. 
        */ 
        astar_planner planner;

        std::map<environment, double> astar_best_value_map;
        const vector<State *>&particles = static_cast<const ParticleBelief*>(belief)->particles();
        double totalValue = 0;

        const environment *environment_state;
        double currentValue;

        std::thread all_particle_threads[particles.size()];
        int num_created_threads = 0;
        double default_init_value = 0;

        // spawn threads to calculate upper bound values with A star
        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            const environment *environment_state = static_cast<const environment *>(particles[particle_num]);
            std::map<environment, double>::iterator it = astar_best_value_map.find(*environment_state);

            if (it == astar_best_value_map.cend()) {
                // spawn a thread to run a star 
                astar_best_value_map[*environment_state] = default_init_value; 
                all_particle_threads[num_created_threads] = std::thread(multi_thread_Astar_getbestvalue, std::ref(*environment_state), std::ref(astar_best_value_map[*environment_state]));
                num_created_threads ++;
            } else { 
                // have already spawned a thread for this environment
                continue;
            }
        }

        // wait for spawned threads to finish
        for (int thread_num = 0; thread_num < num_created_threads; thread_num++) {
            all_particle_threads[thread_num].join();
        }

        // compute the total weighted value
        for (int i = 0; i < particles.size(); i++) {
            const environment * environment_state = static_cast<const environment *>(particles[i]);
            totalValue += (astar_best_value_map[*environment_state]*environment_state->weight);
        }

        return totalValue;
    }
};

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
        if (NUM_OBSTACLES_g != 3 && NUM_OBSTACLES_g != 4) {
            cerr << "INITIAL BELIEF ONLY HARD CODED FOR 3 or 4 OBSTACLES AT THE MOMENT" << endl;
            exit(1);
        }
        
        const environment *environment_start_state = static_cast<const environment *>(start);
        float start_state_obs_ks[NUM_OBSTACLES_g];
        environment_start_state->get_obstacle_ks(start_state_obs_ks, NUM_OBSTACLES_g);
        double particle_weight;

        if (NUM_OBSTACLES_g == 3) {
            float init_obstacle_ks[NUM_OBSTACLES_g];
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
                        new_particle_state = static_cast<environment *>(Allocate(-1, uniform_particle_weight));
                        init_obstacle_ks[0] = all_possible_obs_ks_g[obs1];
                        init_obstacle_ks[1] = all_possible_obs_ks_g[obs2];
                        init_obstacle_ks[2] = all_possible_obs_ks_g[obs3];
                        new_particle_state->set_obstacle_ks(init_obstacle_ks);
                        particles.push_back(new_particle_state);
                    }
                }
            }
        } else if (NUM_OBSTACLES_g == 4) {
            float init_obstacle_ks[NUM_OBSTACLES_g];
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

                            new_particle_state = static_cast<environment *>(Allocate(-1, uniform_particle_weight));
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
        
        cout << "Total number of particles created: " << particles.size() << endl;
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
        
        cout << "CREATED A STAR LOWER BOUND " << endl << endl;
        return new SurgicalDespotAstarScenarioLowerBound(model);

        //cout << "create Astar multi threaded based lower bound" << endl;
        //return new SurgicalDespotAstarMultiThreadPolicy(model, CreateParticleLowerBound(particle_bound_name));
        //return new SurgicalDespotAstarPolicy(model, CreateParticleLowerBound(particle_bound_name));
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
    // Dont implement as too much to print - just do nothing for this function
}

} // end namespace despot