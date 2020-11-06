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
* Default Policy class: to get a LOWER BOUND in  DESPOT - MULTI THREADED CLOSER HISTORY
* ***********************************************************************************
*/

bool isFeasibleAction(int action_num, const environment& environment_state) {
    /*
    * Returns if the specified action number is feasible: in regards that it does not cause 
    * the robot arms to go out of bounds. 
    * 
    * NOTE: Does not check for obstacle related collisions as those may be different in different particles
    */ 
    environment cpy_environment_state = environment_state;
    robotArmActions action_array[NUM_ROBOT_ARMS_g];
    // convert the integer to actions
    int_to_action_array_g(action_num, action_array, NUM_ROBOT_ARMS_g);

    bool error; 
    float cost;
    cpy_environment_state.robObj_m.step(action_array, XY_STEP_SIZE_g, THETA_DEG_STEP_SIZE_g, error, cost);
    
    // if error robot object auto rollbacks - else we roll it back ourself
    if (error) {
        return false;
    } else {
        cpy_environment_state.robObj_m.state_rollback();
        return true;
    }
}

bool isTowardsGoal(int action_num, const environment& environment_state) {
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

void multi_thread_closerhistory_rolloutpolicy(const environment &planning_environment, int particle_number,  int total_num_actions, RandomStreams& streams, History& history, double &state_value, ACT_TYPE &first_action) {
    /*
    * This carries out a rollout policy for the specified environment in a seperate thread. 
    * This function uses the specified random streams until they end to complete the rollout. 
    * The probability of the random action is exponentially annealed with the base of 0.95 - from every step. 
    * args: 
    *   - planning_environment
    *   - particle_number: the particle that this environment belongs to 
    *   - streams: the random streams to use
    *   - history: history of actions and observations
    * returns:  
    *   - state_value: by pass by reference - the value of the state as found by doing the rollout
    *   - first_action: by pass by reference - the first action taken during the rollout
    */ 
    int starting_stream_position = streams.position();
    int max_actions = 1e4;
    int action_counter = 0; 
    environment tempEnvironment = planning_environment; 

    ACT_TYPE last_action = thetaDown;
    if (history.Size() > 0) {
        last_action = history.LastAction();
    } 
    double exp_base = 0.95; // for the random probability

    // continue until the state terminates or you hit some max number of actions
    double total_cost = 0;
    while (!tempEnvironment.at_goal() && action_counter < max_actions) {
        double random_action_prob = pow(exp_base, action_counter);
        int stream_position = starting_stream_position + action_counter; 

        double random_number = 0;
        if (streams.Length() - 1 < stream_position) {
            // at the end of stream select action randomly
            random_number = streams.Entry(particle_number, stream_position);
        } else {
            // use the stream
            random_number = Random::RANDOM.NextDouble(0, 1);
        }

        // decide whether to do a random action 
        bool take_completely_random_action = false;
        if (random_number < random_action_prob) {
            take_completely_random_action = true;
        } else {
            take_completely_random_action = false;
        }

        // get the list of all the actions to choose from 
        std::vector<ACT_TYPE> feasible_actions; 
        std::vector<ACT_TYPE> towards_goal_feasible_actions;
        for (int action_num = 0; action_num < total_num_actions; action_num++) {
            if (isFeasibleAction(action_num, tempEnvironment) && !isReverseAction_g(action_num, last_action)) {
                feasible_actions.push_back(action_num);
                if (isTowardsGoal(action_num, tempEnvironment)) {
                    towards_goal_feasible_actions.push_back(action_num);
                }
            }
        }

        // choose the action
        ACT_TYPE chosen_action;
        if (!take_completely_random_action && towards_goal_feasible_actions.size() > 0) {
            chosen_action = towards_goal_feasible_actions[Random::RANDOM.NextInt(towards_goal_feasible_actions.size())];
        } else {
            chosen_action = feasible_actions[Random::RANDOM.NextInt(feasible_actions.size())];
        }

        if (action_counter == 0) {
            // set the first action taken 
            first_action = chosen_action;
        }
        
        // step the environment with the chosen action and move state variables forward
        robotArmActions action_array[NUM_ROBOT_ARMS_g];
        int_to_action_array_g(chosen_action, action_array, NUM_ROBOT_ARMS_g);
        bool error;
        float step_cost; 
        tempEnvironment.step(action_array, error, step_cost);

        total_cost += Globals::Discount(action_counter)*step_cost;
        action_counter ++;
        last_action = chosen_action;
    } 

    // add the terminal reward to te accumulated goal state 
    if (tempEnvironment.at_goal()) {
        state_value = -total_cost + Globals::Discount(action_counter)*TERMINAL_REWARD_g;
    } else {
        state_value = -total_cost;
    }
    return;
}

class SurgicalDespotCloserHistoryPolicy_multiThread: public ScenarioLowerBound {
/*
* This calculates the value of the state using a history based rollout policy
* Policy methodology: 
*   - If there is no history take a random action that does not run into walls. If there is a history
*   take actions that do not double back on the last action. 
*   - When selecting the random action: select actions that lead you closer to the  goal coordinate
*   in the environment with some probability and select actions completely at random with some other probability
*
* Note: rather than just being a default policy this uses a multi threaded approach to do each of the rollouts
* and compute the value in individual threads to allow efficient compute
*/

private:
    const SurgicalDespot *surgicalDespot_m;
public: 
    SurgicalDespotCloserHistoryPolicy_multiThread(const DSPOMDP* model): ScenarioLowerBound(model) {
        /*
        * Default constructor for the Astar based scenario lower bound
        */ 
        surgicalDespot_m = static_cast<const SurgicalDespot *>(model);
    }

    ValuedAction Value(const vector<State*>& particles, RandomStreams& streams, History& history) const {
        /*
        * Find the value of the belief using a multithreading approach for each particle. 
        */
        std::vector<std::thread> all_particle_threads;
        double default_init_value; 

        double all_particle_values[particles.size()] = {0}; // store all the values that will be returned by the multithreaded functions
        ACT_TYPE all_particle_first_actions[particles.size()] = {0}; // stores all the first actions that will be returned by the multithreaded functions
        // loop to spawn all the threads
        int total_num_actions = surgicalDespot_m->NumActions();
        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            const environment *environment_state = static_cast<const environment *>(particles[particle_num]);
            all_particle_threads.push_back(std::thread(multi_thread_closerhistory_rolloutpolicy, std::ref(*environment_state), particle_num, 
                total_num_actions, std::ref(streams), std::ref(history), std::ref(all_particle_values[particle_num]), 
                std::ref(all_particle_first_actions[particle_num])));

            //const environment &planning_environment, int particle_number,  int total_num_actions, RandomStreams& streams, History& history, double &state_value, ACT_TYPE &first_action
        }

        // wait for all the threads
        int total_threads = all_particle_threads.size();
        for (int thread_num = 0; thread_num < total_threads; thread_num++) {
            all_particle_threads[thread_num].join();
        }

        // come up with the total value and action to return
        double action_weight_array[surgicalDespot_m->NumActions()] = {0};
        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            const environment *environment_state = static_cast<const environment *>(particles[particle_num]);
            
            ACT_TYPE particle_action = all_particle_first_actions[particle_num]; 
            double particle_value = all_particle_values[particle_num];

            action_weight_array[particle_action] += (environment_state->weight*particle_value);
        }

        ACT_TYPE best_action = static_cast<ACT_TYPE>(std::distance(action_weight_array, std::max_element(action_weight_array, action_weight_array + surgicalDespot_m->NumActions())));
        double best_value = action_weight_array[best_action];

        // TODO: REMOVE THIS
        cout << "Action weight array: ";
        for (int i = 0; i < surgicalDespot_m->NumActions(); i++) {
            cout << action_weight_array[i] << ", "; 
        }
        cout << endl << endl;

        // TODO: END REMOVE THIS

        ValuedAction retValuedAction = ValuedAction(best_action, best_value);
        return retValuedAction;
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
        double action_weight_array[surgicalDespot_m->NumActions()] = {0};
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
        double action_weight_array[surgicalDespot_m->NumActions()] = {0};
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
        //cout << "Assigning A star per particle number: " << endl;
        for (int i = 0; i < particles.size(); i++) {
            const environment * environment_state = static_cast<const environment *>(particles[i]);
            action_weight_array[astar_best_action_map[*environment_state]] += environment_state->weight;
        }

        ACT_TYPE policy_action = std::distance(action_weight_array, std::max_element(action_weight_array, action_weight_array + surgicalDespot_m->NumActions()));
        //cout << "Found Astar policy action: " << policy_action;
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

static void multi_thread_Astar_getbestvalue_w_initaction(const environment &planning_environment, ACT_TYPE init_action, double &best_discounted_value) {
    /*
    * Function to enable multi thread planning with A star
    * args:
    *   - planning_environment: environment object on which to do A star
    *   - init_action: the initial action that the environment must take before planning with A star. 
    * returns: by pass by reference
    *   - best_action: the best action to take as the first action in that planning environment
    *   - best_value: the discounted value of the Astar algorithm - including the terminal reward
    */ 
    environment init_action_stepped_environment = environment(planning_environment);
    float init_cost = 0;
    bool error = false;
    robotArmActions init_action_array[NUM_ROBOT_ARMS_g];
    int_to_action_array_g(init_action, init_action_array, NUM_ROBOT_ARMS_g);

    init_action_stepped_environment.step(init_action_array, error, init_cost);
    if (init_action_stepped_environment.at_goal()) {
        best_discounted_value = TERMINAL_REWARD_g + (-init_cost);
        return;
    } else {
        ACT_TYPE stepped_best_action = 0;
        double stepped_best_discounted_value = 0;
        multi_thread_Astar_getbestactionvalue(init_action_stepped_environment, stepped_best_action, stepped_best_discounted_value);
        best_discounted_value = (-init_cost) + Globals::Discount()*stepped_best_discounted_value;
        return;
    }
}

// create a global map to to be used for the AstarScenarioLowerBound - used to globally store the values of the environment. 
static std::map<environment, std::map<ACT_TYPE, double>>AstarScenario_env_value_map_g; 

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
        * Methodology: Do A star on all the environments after taking every possible action. This gives the value
        * for each of these actions given the belief. Once all the values for every action with every particle are
        * computed select the best action and return the action and its corresponding vlaue as a valuedAction pair
        * - if two environments are the same - don't redo A star and just use the last A star's value. 
        * 
        * NOTE: USES GLOBAL MAP
        */ 

        //cout << endl << endl << "IN THE NEWLY MADE VALUE MAPPED FUNCTION, MAP SIZE: " << AstarScenario_env_value_map_g.size() << endl;

        astar_planner planner;

        // create a list of threads
        std::vector<std::thread> all_particle_threads;
        double default_init_value = 0;

        // loop to spawn all threads
        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            const environment *environment_state = static_cast<const environment *>(particles[particle_num]);

            // check global map
            if (AstarScenario_env_value_map_g.find(*environment_state) == AstarScenario_env_value_map_g.cend()) {
                // environment has not been encountered yet - not in global map - spawn threads for a star
                for (ACT_TYPE curr_action = 0; curr_action < surgicalDespot_m->NumActions(); curr_action++) {
                    // initialize action to action_array map
                    AstarScenario_env_value_map_g[*environment_state][curr_action] = default_init_value;
                    all_particle_threads.push_back(std::thread(multi_thread_Astar_getbestvalue_w_initaction, std::ref(*environment_state), curr_action, std::ref(AstarScenario_env_value_map_g[*environment_state][curr_action])));
                }
            }
            
        }

        // wait for all the threads to join
        for (int thread_num = 0; thread_num < all_particle_threads.size(); thread_num ++) {
            all_particle_threads[thread_num].join();
        }

        // find the action with the best value
        double action_weight_array[surgicalDespot_m->NumActions()] = {0};
        
        //TODO: REMOVE
        cout << "action weight  array: ";
        for (int i = 0; i < surgicalDespot_m->NumActions(); i++) {
            cout << action_weight_array[i] << ", ";
        }
        cout << endl;
        std::vector<ValuedAction>nonweighted_action_value_vector;
        std::map<double, int>numtimes_seen_astar_value;
        //TODO: END REMOVE

        double double_back_penalty = DOUBLE_BACK_PENALTY_g;//0;//-1e4;

        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            const environment *environment_state = static_cast<const environment *>(particles[particle_num]);
            for (ACT_TYPE action_num = 0; action_num < surgicalDespot_m->NumActions(); action_num++) {

                if (history.Size() > 0 && isReverseAction_g(action_num, history.LastAction())) {
                    // penalize for double backing
                    action_weight_array[action_num] += environment_state->weight*(AstarScenario_env_value_map_g[*environment_state][action_num] + double_back_penalty);
                } else {
                    action_weight_array[action_num] += environment_state->weight*AstarScenario_env_value_map_g[*environment_state][action_num];
                }
                

                // TODO: REMOVE FOR DEBUGGING
                if (nonweighted_action_value_vector.size() < 13) {
                    nonweighted_action_value_vector.push_back(ValuedAction(action_num, AstarScenario_env_value_map_g[*environment_state][action_num]));
                }
                double value_key = AstarScenario_env_value_map_g[*environment_state][action_num];
                if (numtimes_seen_astar_value.find(value_key) == numtimes_seen_astar_value.cend()) {
                    numtimes_seen_astar_value[value_key] = 0;
                } else {
                    numtimes_seen_astar_value[value_key] += 1;
                }
                // TODO: END REMOVE FOR DEBUGGING
            }
        }


        // TODO: REMOVE THIS
        cout << "numtimes seen astar value: " ;
        int numtimes_seen_astar_counter = 0;
        int total_things = 0;
        for (auto it = numtimes_seen_astar_value.cbegin(); it != numtimes_seen_astar_value.cend(); it++) {
            cout << "(" << it->first << " : " << it->second << "), ";
            numtimes_seen_astar_counter += 1;
            total_things += it->second;
        }
        cout << endl << "The number of different kind of values are: " << numtimes_seen_astar_counter << endl;
        cout << "Total things seen: " << total_things << endl;

        cout << "nonweighted action value vector: ";
        for (int i = 0; i < nonweighted_action_value_vector.size(); i++) {
            cout << std::setprecision(17) << nonweighted_action_value_vector[i] << ", ";
        }
        cout << endl << endl;


        cout << "The number of particles is: " << particles.size() << endl;
        cout << "THE SIZE OF THE NONWEIGHTED_ACTION_VALUE_VECTOR IS: " << nonweighted_action_value_vector.size() << endl;

        const environment *environment_state = static_cast<const environment *>(particles[0]);
        cout << endl << endl;
        cout << "printing the weights"; 
        for (int action = 0; action < 6; action ++) {
            if (action == 0) {
                cout << "xRight: ";
            } else if (action == 1) {
                cout << "xLeft: ";
            } else if (action == 2) {
                cout << "yUp: ";
            } else if (action == 3) {
                cout << "yDown: ";
            } else if (action == 4) {
                cout << "thetaUp: ";
            } else if (action == 5) {
                cout << "thetaDown: ";
            }
            cout << std::setprecision(17) << action_weight_array[action] << ", ";
        }
        cout << endl << "Global discount factor: " << Globals::Discount();
        cout << endl;
        // TODO: REMOVE THIS END

        ACT_TYPE best_action = std::distance(action_weight_array, std::max_element(action_weight_array, action_weight_array + surgicalDespot_m->NumActions()));
        double best_discounted_value = action_weight_array[best_action];

        ValuedAction retValuedAction(best_action, best_discounted_value);

        // TODO: REMOVE THIS 
        // printing this since for some reason in the debugger the value does not seem correct. 
        cout << "best action: " << best_action << ", best discounted value: " << best_discounted_value << endl;
        cout << "ret valued action: " << retValuedAction.action << ", ret valued action value: " << retValuedAction.value << endl << endl;
        // TODO: END REMOVE THIS

        return retValuedAction; 
    }
};


/*
* ***********************************************************************************
* Single Astar average parameter based SCENARIO LOWER BOUND
* ***********************************************************************************
*/
class SurgicalDespotAstar_ParamAvgClass_LowerBound: public ScenarioLowerBound{
/*
* This class uses the belief to generate one deterministic environment by averaging the class parameters.
* Then it does Astar on that environment
* and using that A star determines the best action to take. 
*/
private:
    const SurgicalDespot *surgicalDespot_m;
public: 
    SurgicalDespotAstar_ParamAvgClass_LowerBound(const DSPOMDP* model): ScenarioLowerBound(model) {
        /*
        * Default constructor for the Astar based scenario lower bound
        */ 
        surgicalDespot_m = static_cast<const SurgicalDespot *>(model);
    }

    ValuedAction Value(const vector<State*>& particles, RandomStreams& streams, History& history) const {
        /*
        * Calculate the value of the scenario from the belief
        */ 
        float average_obstacle_ks[NUM_OBSTACLES_g] = {0};
        
        for (int particle_num = 0; particle_num < particles.size(); particle_num ++) {
            const environment* environment_state = static_cast<const environment*>(particles[particle_num]);
            float current_environment_obsks[NUM_OBSTACLES_g] = {0};
            environment_state->get_obstacle_ks(current_environment_obsks, NUM_OBSTACLES_g);

            for (int obstacle_num = 0; obstacle_num < NUM_OBSTACLES_g; obstacle_num ++) {
                average_obstacle_ks[obstacle_num] += current_environment_obsks[obstacle_num] / particles.size();
            }
        }
        
        // initialize deterministic environment with the average of the parameters. 
        environment average_environment;
        average_environment.set_obstacle_ks(average_obstacle_ks);        
        
        // now compute the optimal action value pair. 
        astar_planner planner; 
        vector<ACT_TYPE> all_path_actions;

        planner.plan_a_star(average_environment);
        float best_value = -(planner.get_goal_cost()) + TERMINAL_REWARD_g; 
        planner.get_path(all_path_actions);
        ACT_TYPE best_action = all_path_actions[0];

        // get the valued action to return 
        ValuedAction retValuedAction; 
        retValuedAction.action = best_action; 
        retValuedAction.value = best_value;
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

        double min_dist = Globals::POS_INFTY;
        double arm_dist = 0; 
        float x = 0;
        float y = 0;
        env_state_goal_coord = environment_state.get_goal_coord();

        for (int arm_num = 0; arm_num < NUM_ROBOT_ARMS_g; arm_num++) {
            x = static_cast<float>(env_state_robot_coords[arm_num].x);
            y = static_cast<float>(env_state_robot_coords[arm_num].y);

            arm_dist = static_cast<double>(sqrt(pow(x - env_state_goal_coord.x, 2) + pow(y - env_state_goal_coord.y, 2))) 
                - static_cast<double>(environment_state.get_goal_radius());
            arm_dist = std::min(std::max(arm_dist, static_cast<double>(0)), min_dist);
            if (arm_dist < min_dist) {
                min_dist = arm_dist;
            }
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

            if (Globals::Discount() == 1) {
                discountedValue = -(steps_to_goal * CONSTANT_MOVEMENT_COST_g) + TERMINAL_REWARD_g;
            } else {
                // discount factor 1 creates divide by 0 issue here. 
                discountedValue = (-(1 - Globals::Discount(steps_to_goal)) / (1 - Globals::Discount()) * CONSTANT_MOVEMENT_COST_g ) + 
                    TERMINAL_REWARD_g * Globals::Discount(steps_to_goal);
            }

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
            totalValue += environment_state->weight*discountedValue;
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
    // NOTE: this currently computes the non discounted value from a star
    best_value = -(planner.get_goal_cost()) + TERMINAL_REWARD_g;
    return;
}
// map to store A star calculations for upper bounds
static std::map<environment, double>AstarUpperBound_env_value_map_g; 

class SurgicalDespotAstarMultiThreadUpperBound: public ScenarioUpperBound{ 
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

    //using BeliefUpperBound::Value;
    double Value(const std::vector<State*>& particles,
		RandomStreams& streams, History& history) const {
        /*
        * Compute the nondiscounted value of each particle using A star and average them 
        * to compute the belief upper bound value. 
        */ 
        astar_planner planner;

        double totalValue = 0;

        const environment *environment_state;
        double currentValue;

        std::thread all_particle_threads[particles.size()];
        int num_created_threads = 0;
        double default_init_value = 0;

        // spawn threads to calculate upper bound values with A star
        for (int particle_num = 0; particle_num < particles.size(); particle_num++) {
            const environment *environment_state = static_cast<const environment *>(particles[particle_num]);
            //std::map<environment, double>::iterator it = astar_best_value_map.find(*environment_state);
            auto it = AstarUpperBound_env_value_map_g.find(*environment_state);

            if (it == AstarUpperBound_env_value_map_g.cend()) {
                // spawn a thread to run a star 
                AstarUpperBound_env_value_map_g[*environment_state] = default_init_value; 
                all_particle_threads[num_created_threads] = std::thread(multi_thread_Astar_getbestvalue, std::ref(*environment_state), std::ref(AstarUpperBound_env_value_map_g[*environment_state]));
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
            totalValue += (AstarUpperBound_env_value_map_g[*environment_state]*environment_state->weight);
        }

        return totalValue;
    }
};
}