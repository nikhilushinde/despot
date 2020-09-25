#include "tag_state.h"

// Define the TAG class for the tag environment 
using namespace std;

namespace despot {

TagStateNikhil::TagStateNikhil() {
    /*
    * Constructor for the tag environment - initializes the robot and opponent to a random position in the environment
    */
    //randomized the robot position  
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
    this->envState.robY = rand()%this->height;
    if (this->envState.robY <= 1){
        this->envState.robX = rand()%this->length;
    } else {
        // a random number either 5,6,7
        this->envState.robX = rand()%(8-5) + 5;
    }

    // randomize the opponent position 
    this->envState.oppY = rand()%this->height;
    if (this->envState.oppY <= 1){
        this->envState.oppX = rand()%this->length;
    } else {
        // a random number either 5,6,7
        this->envState.oppX = rand()%(8-5) + 5;
    }

    //initialize the map of the environment - where the index in the string array is the y axis height and the index in the string is the x axis length 
    strcpy(this->map[4], "xxxxx___xx");
    strcpy(this->map[3], "xxxxx___xx");
    strcpy(this->map[2], "xxxxx___xx");    
    strcpy(this->map[1], "__________");
    strcpy(this->map[0], "__________");

    // initialize the reward constants
    this->movementReward = -1;
    this->tagReward = 10;
    this->noTagReward = -10;

    array <ACT_TYPE, 5> allActionsTemp = {NORTH, SOUTH, EAST, WEST, TAG};
    // copies the array to the attribute
    this->allActions = allActionsTemp;
}

TagStateNikhil::TagStateNikhil(int robX, int robY, int oppX, int oppY) {
    /*
    * Constructor that takes in the arguments for the robot and opponent position
    * args:
    *   - robX, robY: robot x and robot y positions - must be valid x y positions in the map
    *   - oppX, oppY: opponent x and y positions - must be valid positions in the map
    */

    
    this->envState.robX = robX;
    this->envState.robY = robY;
    this->envState.oppX = oppX;
    this->envState.oppY = oppY;

    // check validity of the positions
    bool isInEnvironment = inEnv(this->envState);
    if (!isInEnvironment) {
        throw OutOfBoundsException();
    }

    strcpy(this->map[4], "xxxxx___xx");
    strcpy(this->map[3], "xxxxx___xx");
    strcpy(this->map[2], "xxxxx___xx");    
    strcpy(this->map[1], "__________");
    strcpy(this->map[0], "__________");

    // initialize the reward constants
    this->movementReward = -1;
    this->tagReward = 10;
    this->noTagReward = -10;

    array <ACT_TYPE, 5> allActionsTemp = {NORTH, SOUTH, EAST, WEST, TAG};
    // copies the array to the attribute
    this->allActions = allActionsTemp;
}

void TagStateNikhil::Render() {
    /* look through the envState and change change the string to render the state appropriately 
        - R: represents robot 
        - Z: represents the opponent 
        - Q: represents when the robot is on top of the opponent 
        - 0: represents a blank space in the environment 
        - x: non occupiable space in the environment 
    */
    // first change the string to reflect the environment 
    if (this->envState.robX == this->envState.oppX && this->envState.robY == this->envState.oppY) {
        this-> map[this->envState.robY][this->envState.robX] = 'Q';
    } else {
        this->map[this->envState.robY][this->envState.robX] = 'R';
        this->map[this->envState.oppY][this->envState.oppX] = 'Z';
    }

    // print the map 
    for (int i = height - 1; i >= 0; i--) {
       cout << this->map[i] << endl;
    }

    // return map back to base configuration without the robot and opponent in it 
    this->map[this->envState.robY][this->envState.robX] = '_';
    this->map[this->envState.oppY][this->envState.oppX] = '_';
}

bool TagStateNikhil::inEnv(tagStateStruct state) {
    /* 
    * Boolean function that returns if the specified state is in the valid environment 
    * args:
    *   - state: a state struct representing the environment that you want to test
    */

    if ((state.robX < 0 || state.robX >= this->length || state.oppX < 0 || state.oppX >= this->length) ||
        (state.robY < 0 || state.robY >= this->height || state.oppY < 0 || state.oppY >= this->height)) {
        return false;
        }

    bool isInEnvironment = true;
    if (state.robY <= 1) {
        isInEnvironment = (isInEnvironment && (state.robX >= 0 && state.robX < this->length));
    } else {
        isInEnvironment = (isInEnvironment && (state.robX == 5 || state.robX == 6 || state.robX == 7));
    }

    if (state.oppY <= 1) {
        isInEnvironment = (isInEnvironment && (state.oppX >= 0 && state.oppX < this->length));
    } else {
        isInEnvironment = (isInEnvironment && (state.oppX == 5 || state.oppX == 6 || state.oppX == 7));
    }

    return isInEnvironment;
}

bool TagStateNikhil::robStep(ACT_TYPE act, double &reward) {
    /*
    * This function steps the robot in the environment with ACTION act. 
    * it populates the pointers reward and observation to "return" the reward and observation to the calling
    * function 
    * args:
    *   - act: of type ACTION that specifies the action that must be taken 
    *   - reward: pointer to a double that will be populated with the reward from taking the step 
    * Returns:
    *   - boolean value that determines if the game of tag has ended - true if ended else false
    */

    cout << "CALLED ROBSTEP" << endl;
    tagStateStruct tempState;
    if (act == TAG) {
        // tag the robot 
        if (this->envState.robX == this->envState.oppX && this->envState.robY == this->envState.oppY) {
            // if they are  in the same position terminate the game
            reward = tagReward;
            return true;
        } else {
            reward = noTagReward;
            return false;
        }
    } else if (act == NORTH) {
        // move one up
        tempState = this->envState;
        tempState.robY = tempState.robY + 1;
        if (inEnv(tempState)) {
            this->envState = tempState; 
        }
    } else if (act == SOUTH) {
        // move one down 
        tempState = this->envState;
        tempState.robY = tempState.robY - 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } else if (act == EAST) {
        // move one left 
        tempState = this->envState;
        tempState.robX = tempState.robX + 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } else if (act == WEST) {
        // move one right
        tempState = this->envState;
        tempState.robX = tempState.robX - 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    }

    // set the reward and return that the game has not ended
    reward = movementReward;
    return false;
}

void TagStateNikhil::oppPolicyDistribution(float retOppActionProbs[]) {
    /*
    * Returns the probability distribution as an array for the action that the opponent will take
    * The policy is that with probability 0.4 it will move away from the robot in the y direction
    * with probability 0.4 it will move away from the robot in the x direction
    * with probability 0.2 it will stay where it is. 
    * NOTE: The opponent will still try to move into squares that are not occupialbe and then just stays put
    */

    // probability array the value corresponds to the probability of the opponent taking that action
    // NOTE: the TAG action for the opponent is just staying stationary 
    array <float, 5> oppActionProbs;
    oppActionProbs.fill(0);

    // handle movement in the y direction
    if (this->envState.robY == this->envState.oppY) {
        // moving up or down moves further from the robot
        oppActionProbs[NORTH] = 0.2;
        oppActionProbs[SOUTH] = 0.2;
    } else if (this->envState.robY > this->envState.oppY) {
        // robot is up so move down 
        oppActionProbs[SOUTH] = 0.4;
    } else {
        // robot is down so move up
        oppActionProbs[NORTH] = 0.4;
    }
    
    // handle movement in the x direction
    if (this->envState.robX == this->envState.oppX) {
        // moving either east or west moves further from the robot
        oppActionProbs[EAST] = 0.2;
        oppActionProbs[WEST] = 0.2;
    } else if (this->envState.robX > this->envState.oppY) {
        // robot is east you want to move further west
        oppActionProbs[WEST] = 0.4;
    } else {
        // robot is west you want to move further east
        oppActionProbs[EAST] = 0.4;
    }

    // set the stationary probability - TAG is the stationary action for the opponent 
    oppActionProbs[TAG] = 0.2;

    // populate the array to return with the correct probabilities
    for (int i = 0; i < (int) oppActionProbs.size(); i++) {
        retOppActionProbs[i] = oppActionProbs[i];
    }
}

int TagStateNikhil::randomNumToInt(float actionProbs[], int actionProbsSize, float randomNum) {
    /*
    * Used for deterministic actions. returns the index of the action to take deterministically 
    * based on the action probability distribution and the random number given 
    * args:
    *   - actionProbs: the action probability distribution - the index in the array corresponds to the action 
    *                  and the value in the array corresponds to the probability of that action occuring
    *   - actionProbsSize: the size of the array - to prevent memory errors 
    *   - randomNum: random number to use to generate the deterministic actions. - NOTE: float (0, 1)
    * returns:
    *   - index of the action that should be taken
    */ 
    float trackedProb = 0;
    int ret_action_index = -1;
    for (int action_num = 0; action_num < actionProbsSize; action_num++) {
        if (action_num == 0) {
            trackedProb = actionProbs[0];
        } else {
            trackedProb += actionProbs[action_num];
        }

        if (randomNum <= trackedProb) {
            // this is the index to return 
            ret_action_index = action_num;
        }
    }

    return ret_action_index;
}

void TagStateNikhil::oppStep(float randomNum) {
    /*
    * This function takes a step in the environment using the opponent policy from oppPolicyDistribution
    * It gets the transition distribution from oppPolicyDistribution and then randomly selects which action
    * to take using that. 
    * 
    * It then takes the action which mutates the state of the environment.
    * args:
    *   - randomNum: random number to deterministically decide the opponents actions
    */

    // get the proabilities for each action
    array <float, NUM_ACTIONS_TAG_NIKHIL_STATE> oppActionProbs;
    oppPolicyDistribution(oppActionProbs.data());

    // deterministically select opponent action to use. 
    int oppActionIndex = this->randomNumToInt(oppActionProbs.data(), NUM_ACTIONS_TAG_NIKHIL_STATE, randomNum);
    ACT_TYPE oppAction = this->allActions[oppActionIndex];


    // step the opponent using the best action
    tagStateStruct tempState;
    if (oppAction == NORTH) {
        // move one up
        tempState = this->envState;
        tempState.oppY = tempState.oppY + 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } else if (oppAction == SOUTH) {
        // move one up
        tempState = this->envState;
        tempState.oppY = tempState.oppY - 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } else if (oppAction == EAST) {
        // move one up
        tempState = this->envState;
        tempState.oppX = tempState.oppX + 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } else if (oppAction == WEST) {
        // move one up
        tempState = this->envState;
        tempState.oppX = tempState.oppX - 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } 
    // NOTE: the TAG action means to stay stationary for the opponent so we do nothing

    // TODO: REMOVE FOR DEBUGGING PRINTING WHICH ACTION WAS TAKEN
    if (oppAction == NORTH) {
        cout << "opp took action: NORTH" << endl << endl;
    } else if (oppAction == SOUTH) {
        cout << "opp took action: SOUTH" << endl << endl;
    } else if (oppAction == EAST) {
        cout << "opp took action: EAST" << endl << endl;
    } else if (oppAction == WEST) {
        cout << "opp took action: WEST" << endl << endl;
    } else if (oppAction == TAG) {
        cout << "opp took action: STATIONARY" << endl << endl;
    }
}

void TagStateNikhil::oppStepRandom() {
    /*
    * This function takes a step in the environment using the opponent policy from oppPolicyDistribution
    * It gets the transition distribution from oppPolicyDistribution and then randomly selects which action
    * to take using that. 
    * 
    * It then takes the action which mutates the state of the environment.
    */

    // get the proabilities for each action
    array <float, NUM_ACTIONS_TAG_NIKHIL_STATE> oppActionProbs;
    oppPolicyDistribution(oppActionProbs.data());

    // create a discreete distribution generator to select the action
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator (seed);
    discrete_distribution<int> oppDistribution (oppActionProbs.begin(), oppActionProbs.end());

    int oppActionIndex = oppDistribution(generator);
    ACT_TYPE oppAction = this->allActions[oppActionIndex];

    // step the opponent using the best action
    tagStateStruct tempState;
    if (oppAction == NORTH) {
        // move one up
        tempState = this->envState;
        tempState.oppY = tempState.oppY + 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } else if (oppAction == SOUTH) {
        // move one up
        tempState = this->envState;
        tempState.oppY = tempState.oppY - 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } else if (oppAction == EAST) {
        // move one up
        tempState = this->envState;
        tempState.oppX = tempState.oppX + 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } else if (oppAction == WEST) {
        // move one up
        tempState = this->envState;
        tempState.oppX = tempState.oppX - 1;
        if (inEnv(tempState)) {
            this->envState = tempState;
        }
    } 
    // NOTE: the TAG action means to stay stationary for the opponent so we do nothing

    // TODO: REMOVE FOR DEBUGGING PRINTING WHICH ACTION WAS TAKEN
    if (oppAction == NORTH) {
        cout << "opp took action: NORTH" << endl << endl;
    } else if (oppAction == SOUTH) {
        cout << "opp took action: SOUTH" << endl << endl;
    } else if (oppAction == EAST) {
        cout << "opp took action: EAST" << endl << endl;
    } else if (oppAction == WEST) {
        cout << "opp took action: WEST" << endl << endl;
    } else if (oppAction == TAG) {
        cout << "opp took action: STATIONARY" << endl << endl;
    }
}

OBS_TYPE TagStateNikhil::observe() {
    /*
    * Returns a OBS_TYPE observation - just an integer
    * returns:
    *   - observation - 1: if the robot and opponent are in the same position 
    *                 - 0: otherwise
    * */

    if (this->envState.robX == this->envState.oppX && this->envState.robY == this->envState.oppY) {
        return 1; 
    } else {
        return 0;
    }
}

bool TagStateNikhil::Step(ACT_TYPE act, float randomNum, double &reward, OBS_TYPE &observation) {
    /*
    * Takes a step in the environment: - the return is true if the game has ended and the opponent is tagged:
    * 
    *   1. If act == TAG then the robot takes the action first and if successfull returns true to signify the end of the game
    *   2. The opponent uses its policy to move in the environment
    *   3. If the robot is moving then it takes it movement step in the environment
    *   4. The robot observes the environment 
    * 
    * args:
    *   - act: the action for the robot to take
    *   - randomNum: to allow the step to be deterministically taken 
    *   - *reward: pointer with which to return the reward
    *   - *observation: pointer with which to return the observation
    * returns:
    *   - bool: true if the game has ended and the opponent is tagged, else false 
    */    
    bool gameOver = false;

    // FIRST: tag
    if (act == TAG) {
        gameOver = robStep(act, reward);
        if (gameOver) {
            return gameOver;
        } else {
            observation = observe();
            return gameOver;
        }
    }

    // SECOND: opponent moves 
    oppStep(randomNum);

    // THIRD: move the robot
    gameOver = robStep(act, reward);
    if (gameOver) {
        return gameOver;
    }

    // FOURTH: observe the environment
    observation = observe();

    return gameOver;
}

} // end namespace despot