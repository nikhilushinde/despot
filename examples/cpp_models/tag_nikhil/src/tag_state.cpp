#include "tag_state.h"

// Define the TAG class for the tag environment 
using namespace std;

// the things you are going to use from namespace cv
using cv::Mat;
using cv::Point;
using cv::line;
using cv::circle;
using cv::Rect;
using cv::rectangle;
using cv::Scalar;
using cv::namedWindow;
using cv::WINDOW_AUTOSIZE;
using cv::imshow;
using cv::flip;
using cv::waitKey;

namespace despot {

TagStateNikhil::TagStateNikhil() {
    /*
    * Constructor for the tag environment - initializes the robot and opponent to a random position in the environment
    */
    int robPosRandInt, oppPosRandInt;
    robPosRandInt = Random::RANDOM.NextInt(NUM_XY_POS_TAG_NIKHIL_STATE);
    oppPosRandInt = Random::RANDOM.NextInt(NUM_XY_POS_TAG_NIKHIL_STATE);

    this->InitStateFromInts(robPosRandInt, oppPosRandInt);
}

TagStateNikhil::TagStateNikhil(int robPosIntIndex, int oppPosIntIndex) {
    /*
    * Constructor for the tag environment - initializes the robot and opponent to a random position in the environment
    * robotPosIntIndex and oppPosIntIndex are integers between 0 and 29 exclusive to indicate the position that the robot
    * and the opponent should be initialized in. 
    * 
    * args:
    *   - robPosIntIndex: int [0,29) indicating robot position
    *   - oppPosIntIndex: int [0,29) indicating opponent position
    */
    // select the robot and opponent positions uniformly at random. 
    /* Methodology: there are 29 * 29 possible states in total. Get the (x,y) positions that
    correspond to each of the 29 possible (x,y) states. Then select 2 random numbers between 0 and 29
    (x,y):rob_rand_num corresponds to the robot position and (x,y):opp_rand_num corresponds to the opp pos*/
    this->InitStateFromInts(robPosIntIndex, oppPosIntIndex);
}

TagStateNikhil::TagStateNikhil(int robX, int robY, int oppX, int oppY) {
    /*
    * Constructor that takes in the arguments for the robot and opponent position
    * args:
    *   - robX, robY: robot x and robot y positions - must be valid x y positions in the map
    *   - oppX, oppY: opponent x and y positions - must be valid positions in the map
    */
    this->InitState(robX, robY, oppX, oppY);
}

TagStateNikhil::TagStateNikhil(const TagStateNikhil &stateToCopy) {
    /*
    * Copy constructor for TagStateNikhil. Initializes an object to be a copy in terms of all attributes
    * of the stateToCopy
    */ 
    tagStateStruct envStateToCopy = stateToCopy.get_envState();
    this->InitState(envStateToCopy.robX, envStateToCopy.robY, envStateToCopy.oppX, envStateToCopy.oppY);
}

void TagStateNikhil::IntToPosCoords(int robPosInt, int oppPosInt, int &retRobX, int &retRobY, int &retOppX, int &retOppY) {
    /*
    * Converts the robPosInt and the oppPosInt to (x,y) positions for the robot and the opponent
    * args:
    *   - robPosInt: integer between [0,29) indicating the position for the robot 
    *   - oppPosInt: integer between [0,29) indicating the position for the opponent
    * returns: by pass by reference:
    *   - retRobX, retRobY, retOppX, retOppY 
    */  
    /* Methodology: there are 29 * 29 possible states in total. Get the (x,y) positions that
    correspond to each of the 29 possible (x,y) states. Then select 2 random numbers between 0 and 29
    (x,y):rob_rand_num corresponds to the robot position and (x,y):opp_rand_num corresponds to the opp pos*/
    if (robPosInt < 0 || robPosInt >= NUM_XY_POS_TAG_NIKHIL_STATE || oppPosInt < 0 || oppPosInt >= NUM_XY_POS_TAG_NIKHIL_STATE) {
        // throw an error if the integers specifying the robot pos or opponent pos are incorrect
        cerr << "Integers used to specify robot position and opponent position are out of bounds: " << robPosInt << ", " << oppPosInt << endl;
        exit(1);
    }

    int x_positions[NUM_XY_POS_TAG_NIKHIL_STATE];
    int y_positions[NUM_XY_POS_TAG_NIKHIL_STATE];
    int count = 0;
    for (int x = 0; x < LENGTH_TAG_NIKHIL_STATE; x++) {
        for (int y = 0; y < HEIGHT_TAG_NIKHIL_STATE; y++) {
            if (x >= 5 && x <= 7) {
                x_positions[count] = x;
                y_positions[count] = y;
                count ++;
            } else {
                if (y <= 1) {
                    x_positions[count] = x;
                    y_positions[count] = y;
                    count ++;
                }
            }
        }
    }

    retRobX = x_positions[robPosInt];
    retRobY = y_positions[robPosInt];
    retOppX = x_positions[oppPosInt];
    retOppY = y_positions[oppPosInt];
    
}

void TagStateNikhil::InitStateFromInts(int robPosInt, int oppPosInt) {
    /*
    * Initialize the states from integers that specify the robot and opponent positions
    * args:
    *   - robPosInt: integer between [0,29) indicating the position for the robot 
    *   - oppPosInt: integer between [0,29) indicating the position for the opponen
    */ 

    int robPosRandInt, oppPosRandInt;
    robPosRandInt = Random::RANDOM.NextInt(NUM_XY_POS_TAG_NIKHIL_STATE);
    oppPosRandInt = Random::RANDOM.NextInt(NUM_XY_POS_TAG_NIKHIL_STATE);

    int setRobX, setRobY, setOppX, setOppY;
    this->IntToPosCoords(robPosRandInt, oppPosRandInt, setRobX, setRobY, setOppX, setOppY);
    this->InitState(setRobX, setRobY, setOppX, setOppY);
}

void TagStateNikhil::InitState(int robX, int robY, int oppX, int oppY) {
    /*
    * State initializer that takes in the arguments for the robot and opponent position
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
        std::cout << "robot x: " << robX << ", robot Y:" << robY << ", opponent X:" << oppX << ", opponent Y:" << oppY << endl;
        throw OutOfBoundsException();
    }

    strcpy(this->map[4], "xxxxx___xx");
    strcpy(this->map[3], "xxxxx___xx");
    strcpy(this->map[2], "xxxxx___xx");    
    strcpy(this->map[1], "__________");
    strcpy(this->map[0], "__________");

    // initialize the reward constants
    this->movementReward = TAG_REWARD_MOVEMENT_TAG_NIKHIL_STATE;
    this->tagReward = TAG_REWARD_SUCCESS_TAG_NIKHIL_STATE;
    this->noTagReward = TAG_REWARD_FAIL_TAG_NIKHIL_STATE;

    array <ACT_TYPE, 5> allActionsTemp = {NORTH, SOUTH, EAST, WEST, TAG};
    // copies the array to the attribute
    this->allActions = allActionsTemp;
}

void TagStateNikhil::Render(){
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

void TagStateNikhil::RenderState(ostream &out) const{
    /* look through the envState and change change the string to render the state appropriately 
        - R: represents robot 
        - Z: represents the opponent 
        - Q: represents when the robot is on top of the opponent 
        - 0: represents a blank space in the environment 
        - x: non occupiable space in the environment 
    */
    for (int y = HEIGHT_TAG_NIKHIL_STATE - 1; y >= 0; y--) {
        for (int x = 0; x < LENGTH_TAG_NIKHIL_STATE; x++) {
            if (x == envState.robX && y == envState.robY && x == envState.oppX && y == envState.oppY) {
                out << "Q";
            } else if (x == envState.robX && y == envState.robY) {
                out << "R";
            } else if (x == envState.oppX && y == envState.oppY) {
                out << "Z";
            } else {
                out << this->map[y][x];
            }
        }
        out << endl;
    }

    // render the state using opencv
    int unit_pixel_multiplier = 100;
    int image_height = HEIGHT_TAG_NIKHIL_STATE * unit_pixel_multiplier;
    int image_length = LENGTH_TAG_NIKHIL_STATE * unit_pixel_multiplier;
    Mat image(image_height, image_length, CV_8UC3, Scalar(255, 255, 255));
    Mat flippedImage(image_height, image_length, CV_8UC3, Scalar(255, 255, 255));

    // draw all the grid lines that are used for tag
    Point gridEndPt;
    Point gridStartPt;
    Scalar colorLine(0,0,0);
    int thicknessLine = 2;
    for (int i = 0; i < LENGTH_TAG_NIKHIL_STATE + 1; i++) {
        gridEndPt = Point(i*unit_pixel_multiplier, 0);
        gridStartPt = Point(i*unit_pixel_multiplier, HEIGHT_TAG_NIKHIL_STATE * unit_pixel_multiplier);
        line(image, gridStartPt, gridEndPt, colorLine, thicknessLine);
    }

    for (int i = 0; i < HEIGHT_TAG_NIKHIL_STATE + 1; i++) {
        gridEndPt = Point(0, i*unit_pixel_multiplier);
        gridStartPt = Point(LENGTH_TAG_NIKHIL_STATE * unit_pixel_multiplier, i*unit_pixel_multiplier);
        line(image, gridStartPt, gridEndPt, colorLine, thicknessLine);
    }

    // draw the opponent 
    Scalar opponentColor(255, 0, 0);
    Scalar robotColor(0,0,0);
    Point robotPoint;
    Point opponentPoint;

    int robotXOffset = (int)unit_pixel_multiplier/2;
    int robotYOffset = (int)unit_pixel_multiplier/2;

    int oppXOffset = (int)unit_pixel_multiplier/2;
    int oppYOffset = (int)unit_pixel_multiplier/2;

    int robotRadius = 50;
    int opponentRadius = 30;

    Point startBox;
    Point endBox;
    Rect rectangleToDraw;

    // draw the robot as a large circle and the opponent as smaller circle - first draw the robot and then draw the circle
    for (int y = 0; y < HEIGHT_TAG_NIKHIL_STATE; y++) {
        for (int x = 0; x < LENGTH_TAG_NIKHIL_STATE; x++) {
            if (x == envState.robX && y == envState.robY && x == envState.oppX && y == envState.oppY) {
                robotPoint = Point(x*unit_pixel_multiplier + robotXOffset, y*unit_pixel_multiplier + robotYOffset);
                circle(image, robotPoint, robotRadius, robotColor, -1);
                opponentPoint = Point(x*unit_pixel_multiplier + oppXOffset, y*unit_pixel_multiplier + oppYOffset);
                circle(image, opponentPoint, opponentRadius, opponentColor, -1);
            } else if (x == envState.robX && y == envState.robY) {
                robotPoint = Point(x*unit_pixel_multiplier + robotXOffset, y*unit_pixel_multiplier + robotYOffset);
                circle(image, robotPoint, robotRadius, robotColor, -1);
            } else if (x == envState.oppX && y == envState.oppY) {
                opponentPoint = Point(x*unit_pixel_multiplier + oppXOffset, y*unit_pixel_multiplier + oppYOffset);
                circle(image, opponentPoint, opponentRadius, opponentColor, -1);
            } else if (this->map[y][x] == 'x') {
                //Rect rectangleToDraw(x*unit_pixel_multiplier, y*unit_pixel_multiplier, (x+1)*unit_pixel_multiplier), (y+1)*unit_pixel_multiplier);
                startBox = Point(x*unit_pixel_multiplier, y*unit_pixel_multiplier);
                endBox = Point((x+1)*unit_pixel_multiplier, (y+1)*unit_pixel_multiplier);
                rectangleToDraw = Rect(startBox, endBox);
                //Rect rectangleToDraw(10, 10, 100, 100);
                rectangle(image, rectangleToDraw, Scalar(0,0,0), -1);
            }
        }
    }

    // get the flipped image
    int flipDirection = 0; // 0 - means flip vertically
    flip(image, flippedImage, flipDirection);

    // display the window
    namedWindow("Tag Print State window", WINDOW_AUTOSIZE);
    imshow("Tag Print State window", flippedImage);
    waitKey(10);
    cout << "JUST RENDERED THE STATE" << endl;

}

bool TagStateNikhil::inEnv(tagStateStruct state) const{
    /* 
    * Boolean function that returns if the specified state is in the valid environment 
    * args:
    *   - state: a state struct representing the environment that you want to test
    * returns:
    *   - True if the state is in the Environment, Else returns False
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
    } else if (this->envState.robX > this->envState.oppX) {
        // robot is east you want to move further west
        oppActionProbs[WEST] = 0.4;
    } else {
        // robot is west you want to move further east
        oppActionProbs[EAST] = 0.4;
    }

    // set the stationary probability - TAG is the stationary action for the opponent 
    oppActionProbs[TAG] = 0.2;

    //cout << "In OppProbDistrib: current robot state: " << envState.robX << "," << envState.robY << " and current opponent state: " << envState.oppX << "," << envState.oppY << endl;
    //cout << "Opponent probability distribution is: ";
    // populate the array to return with the correct probabilities
    for (int i = 0; i < (int) oppActionProbs.size(); i++) {
        retOppActionProbs[i] = oppActionProbs[i];
        // TODO: REMOVE FOR DEBUG
        //cout << oppActionProbs[i] << ", ";
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
    // TODO: REMOVE THIS - TEST FOR DEBUGGING - SEED RANDOM GENERATOR WITH DETERMINISTIC SEED
    unsigned int deterministic_seed_val = (unsigned) ((double)randomNum * (double)UINT_MAX); 
    srand(deterministic_seed_val);
    //cout << randomNum << endl;
    //cout << "Deterministic seed val: " << deterministic_seed_val << endl;
    //cout << "Deterministic seed val: " << (unsigned) ((double)randomNum * (double)UINT_MAX) << endl;
    randomNum = (double)rand()/(double)RAND_MAX;
    //cout << rand() << endl; 
    //cout << "NEW RANDOM:" << randomNum << endl;
    

    float trackedProb = 0;
    int ret_action_index = -1;
    for (int action_num = 0; action_num < actionProbsSize; action_num++) {
        if (action_num == 0) {
            trackedProb = actionProbs[0];
        } else {
            trackedProb += actionProbs[action_num];
        }
        // ensure that an action with zero probability id not being selected
        if (randomNum <= trackedProb && actionProbs[action_num] != 0) {
            // this is the index to return 
            ret_action_index = action_num;
            return ret_action_index;
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
    // NOTE: for the opponent the TAG action means to stay put
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
}

OBS_TYPE TagStateNikhil::observe() {
    /*
    * Returns a OBS_TYPE observation - just an integer - accounts for both the position 
    * of the robot and whether or not the robot sees the opponent
    * returns:
    *   - observation: 3 digit number - robY*100 + robX*10 + last_digit
    *       - last digit:
    *           - 1: if the robot and opponent are in the same position 
    *           - 0: otherwise
    * */

    double observation = 0;
    observation += envState.robY*100 + envState.robX*10;
    if (this->envState.robX == this->envState.oppX && this->envState.robY == this->envState.oppY) {
        observation += 1;
    } else {
        observation += 0;
    }
    return observation;
}

double TagStateNikhil::ObsProb(OBS_TYPE obs, ACT_TYPE action) const{
    /*
    * This function returns the probability of seeing the observation from state s 
    * given that you got to state s by taking action a. This is a constant function 
    * meaning that it does not alter the that of the environment
    * args:
    *   - obs: the observation that we are trying to find the probability of seeing from the current state
    *   - a: the action taken to get to the current state of the environment. 
    * returns:
    *   - double: representing the probability of seeing the given obs from the current state gotten to by action a
    */ 
    int observed_yx, true_yx, obs_opp; 
    // obs_opp is 1 if the robot sees the opponent and 0 if not
    // observed_yx is the y*10 + x
    observed_yx = static_cast<int>(obs/10);
    true_yx = envState.robY*10 + envState.robX;
    obs_opp = static_cast<int>(obs%10);

    if (observed_yx != true_yx) {
        // robot position should be known so must match observation
        return 0.0;
    }

    else {
        if (obs_opp == 1) {
            if (this->envState.robX == this->envState.oppX && this->envState.robY == this->envState.oppY) {
                return 1.0; // can only observe 1 when the robot and the opponet are on the same spot
            } else {
                return 0.0; // cannot observe 1 if the robot and the opponent are not on the same spot
            }
        } else if (obs_opp == 0) {
            if (this->envState.robX == this->envState.oppX && this->envState.robY == this->envState.oppY) {
                return 0.0; // can only observe 1 when the robot and the oppoonent are in the ssame spot
            } else {
                return 1.0; // only observe 0 if the robot and the opponent are not on the same spot
            }
        } else {
            cerr << "Error: OBSERVATION:" << obs << "Is not a valid observation for the TagStateNikhil class" << endl;
            exit(1);
        }
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

tagStateStruct TagStateNikhil::get_envState() const{
    /*
    * Get the environment state 
    */ 
    return envState;
}

ACT_TYPE TagStateNikhil::getOppositeAction(ACT_TYPE action) const {
    /*
    * Get the action that double backs on the specified action
    * args:
    *   - action
    * returns:
    *   - action that doubles back on action
    */ 
    if (action == TAG) {
        return TAG;
    } else if (action == EAST) {
        return WEST;
    } else if (action == WEST) {
        return EAST;
    } else if (action == NORTH) {
        return SOUTH;
    } else if (action == SOUTH) {
        return NORTH;
    } else {
        cerr << "Error: Action " << action << " is not a valid action for TagStateNikhil class" << endl;
        exit(1);
    }
}

bool TagStateNikhil::isInvalidStep(ACT_TYPE action, int testRobX, int testRobY) const{
    /*
    * Check if taking the action from the given robot position results in a valid robot position. 
    * args:
    *   - testRobX: the x value to start from 
    *   - testRobY: the y value to start from
    * returns:
    *   - boolean value: True if the step is invalid - runs into walls/ outside the environment else False
    */ 
    // step just the robot
    double reward;
    tagStateStruct tempState;
    tempState = this->envState;
    tempState.robX = testRobX;
    tempState.robY = testRobY;

    if (action == NORTH) {
        // move one up
        tempState.robY = tempState.robY + 1;
    } else if (action == SOUTH) {
        // move one down 
        tempState.robY = tempState.robY - 1;
    } else if (action == EAST) {
        // move one left 
        tempState.robX = tempState.robX + 1;
    } else if (action == WEST) {
        // move one right
        tempState.robX = tempState.robX - 1;
    }

    if (inEnv(tempState)) {
        // false indicates that the environment from the step was valid
        return false;
    } else {
        // true indicates that the resultant step was invalid as the robot position was not in the environment
        return true;
    }
}

} // end namespace despot