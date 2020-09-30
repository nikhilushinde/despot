#include <iostream>
#include <string.h>
#include <stdio.h>

#include "tag_state.h"
using namespace std;

using namespace despot;

int main(int argc, char * argv[]) {
    TagStateNikhil tagEnv(0, 0, 0, 0); 
    //Tag tagEnv;
    tagEnv.RenderState(cout);

    bool gameOver = false;
    ACT_TYPE act;
    double totalReward = 0;
    double reward = 0;
    OBS_TYPE observation = 0;

    int count = 0;

    char c;
    string demarkationLine = "###########################################################################";

    while (not gameOver) {
        cout << "Enter character: ";
        cin >> c;
        cout << endl;

        if (c == 'w') {
            act = tagEnv.NORTH;
        } else if (c == 's') {
            act = tagEnv.SOUTH;
        } else if (c == 'a') {
            act = tagEnv.WEST;
        } else if (c == 'd') {
            act = tagEnv.EAST;
        } else if (c == 't') {
            act = tagEnv.TAG;
        } else {
            cout << "INVALID ENTRY!!" << endl << endl << endl;
        }

        gameOver = tagEnv.Step(act, Random::RANDOM.NextDouble(), reward, observation);
        tagEnv.RenderState(cout);
        totalReward += reward;

        // TODO: REMOVE FOR DEBUGGING PRINTING WHICH ACTION WAS TAKEN
        if (act == tagEnv.NORTH) {
            cout << "act: NORTH" << endl << endl;
        } else if (act == tagEnv.SOUTH) {
            cout << "act: SOUTH" << endl << endl;
        } else if (act == tagEnv.EAST) {
            cout << "act: EAST" << endl << endl;
        } else if (act == tagEnv.WEST) {
            cout << "act: WEST" << endl << endl;
        } else if (act == tagEnv.TAG) {
            cout << "act: STATIONARY" << endl << endl;
        }
        cout << "Total Reward: " << totalReward << endl;
        cout << "Observation: " << observation << endl;
        cout << "Step: " << count << endl;
        count ++;
        cout << demarkationLine << endl << demarkationLine << endl << endl;
    }

    cout << "THE TOTAL FINAL REWARD WAS: " << totalReward << endl;


    // cout << endl;

    // double reward = 0;  
    // double totalReward = 0;

    // tagEnv.robStep(NORTH, &reward);
    // tagEnv.oppStep();
    // totalReward += reward;
    // tagEnv.Render();

    // cout << "The total reward is: " << totalReward << endl;
    // cout << endl;

    // tagEnv.robStep(SOUTH, &reward);
    // tagEnv.oppStep();
    // totalReward += reward;
    // tagEnv.Render();

    // cout << "The total reward is: " << totalReward << endl;
    // cout << endl;

    // tagEnv.robStep(EAST, &reward);
    // tagEnv.oppStep();
    // totalReward += reward;
    // tagEnv.Render();

    // cout << "The total reward is: " << totalReward << endl;
    // cout << endl;

    // tagEnv.robStep(WEST, &reward);
    // tagEnv.oppStep();
    // totalReward += reward;
    // tagEnv.Render();    

    // cout << "The total reward is: " << totalReward << endl;
    // cout << endl;
    // return 0;
    return 0;
}
