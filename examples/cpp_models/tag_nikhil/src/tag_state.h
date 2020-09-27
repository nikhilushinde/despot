#ifndef TAG_STATE_NIKHIL_H
#define TAG_STATE_NIKHIL_H

#include <iostream>
#include <stdlib.h> // use this for the random number generator
#include <ctime> // use this to seed the random number generator 
#include <chrono> // for the random seed as well 

#include <vector> 
#include <array>
#include <random>

// the overarching despot related includes
#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>


namespace despot {

#define NUM_OBSERVATIONS_TAG_NIKHIL_STATE 2
#define NUM_ACTIONS_TAG_NIKHIL_STATE 5
#define LENGTH_TAG_NIKHIL_STATE 10
#define HEIGHT_TAG_NIKHIL_STATE 5
#define NUM_XY_POS_TAG_NIKHIL_STATE 29
#define TAG_REWARD_SUCCESS_TAG_NIKHIL_STATE 10
#define TAG_REWARD_FAIL_TAG_NIKHIL_STATE -10
#define TAG_REWARD_MOVEMENT_TAG_NIKHIL_STATE -1

// structure to hold the state of the environment
struct tagStateStruct {
    int robX;
    int robY;
    int oppX;
    int oppY;

    // override the equal operator
    bool operator==(const tagStateStruct &o) const{ 
        return ((robX == o.robX) && (robY == o.robY) && (oppX == o.oppX) && (oppY == o.oppY));
    }

    // override the compare operator
    bool operator<(const tagStateStruct &o) {
        return ((robX > o.robX) || (robX == o.robX && robY > o.robY) || ((robX == o.robX && robY == o.robY) && ((oppX > o.oppX) || (oppX == o.oppX && oppY > o.oppY))) );
    }
};

// structure to hold one position in the environment
struct tagCoordsStruct {
    int x;
    int y;

    // override the equal operator
    bool operator==(const tagCoordsStruct &o) const {
        return (x == o.x && y == o.y);
    }

    // override the compare operator
    bool operator<(const tagCoordsStruct &o) const {
        return ((x > o.x) || (x == o.x && y > o.y));
    }
};

struct OutOfBoundsException : public std::exception {
   const char * what () const throw () {
      return "Out of bounds exception! - the specified state was out of bounds of the environment";
   }
};

class TagStateNikhil: public State {
public:
    TagStateNikhil(); // no args constructor
    TagStateNikhil(int robPosIntIndex, int oppPosIntIndex); // constructor where you specify a number from 0 to 29 exclusive for the robot and opponent position
    TagStateNikhil(int robX, int robY, int oppX, int oppY); // constructor where you specify the robot and opp positions
    
    void IntToPosCoords(int robPosInt, int oppPosInt, int &retRobX, int &retRobY, int &retOppX, int &retOppY); // 
    void InitState(int robX, int robY, int oppX, int oppY); // initialize the state of the environment with the robot and opponent positions
    void InitStateFromInts(int robPosInt, int oppPosInt); // Initialize the state of the environment from integers that specify the robot and opponent positions
    
    void Render(); // print the environment out to terminal
    void RenderState(std::ostream& out) const; // prints the environment out to the ostream - required for the PrintState function in DESPOT

    bool Step(ACT_TYPE act, float randomNum, double &reward, OBS_TYPE &observation); //steps in the environment and returns reward and observation - mutates the environment state
    bool robStep(ACT_TYPE act, double &reward); // steps the robot in the environment with action sees reward
    void oppStep(float randomNum); // Steps the opponent in the environment according to the opponent policy - takes in random number to deterministically select action
    void oppStepRandom(); // Steps the opponent in the environment according to the opponent policy - generate random number to use in deciding the random action of the opponent
    
    OBS_TYPE observe(); // returns the boolean observation - true if robot and opponent on the same grid position
    double ObsProb(OBS_TYPE obs, ACT_TYPE action) const;

    tagStateStruct get_envState() const; // get the state - the robot and opponent
    ACT_TYPE getOppositeAction(ACT_TYPE action) const; // returns the action that double backs on the action that is fed in
    bool isInvalidStep(ACT_TYPE action, int testRobX, int testRobY) const; // check if taking the specified action from the specified robot position is valid/doesnt run into walls - returns true if it is invalid 
    

    // define the possible actions 
    enum {
        NORTH = 0, 
        SOUTH = 1, 
        EAST = 2, 
        WEST = 3, 
        TAG = 4
    };
private:
    bool inEnv(tagStateStruct state) const; // checks if a given state is in the environment - if in env return true else false
    void oppPolicyDistribution(float retOppActionProbs[]); // using the state of the environment populates the probability distribution of opponent actions
    int randomNumToInt(float actionProbs[], int actionProbsSize, float randomNum); // takes a probability distribution ActionProbs and deterministically returns the index that corresponds to having having used the randomNum

    tagStateStruct envState;
    static const int length = LENGTH_TAG_NIKHIL_STATE;
    static const int height = HEIGHT_TAG_NIKHIL_STATE;
    char map[height][length + 1];

    std::array <ACT_TYPE, 5> allActions;

    double noTagReward;
    double tagReward;
    double movementReward;
};


}

#endif