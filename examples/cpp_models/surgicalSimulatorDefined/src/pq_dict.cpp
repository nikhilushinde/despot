#include "pq_dict.h"

/*
* Includes the functions for the class defined in pq_dict.h
*/
using std::map;
using std::cout;
using std::cerr;
using std::endl;

namespace despot {
    
pqdict_env_double::pqdict_env_double(bool is_max_pqdict) {
    // default constructor - default initializes to a min pq dict unless specified. 
    is_max_pqdict_m = is_max_pqdict;
}

void pqdict_env_double::update_top() {
    // update the max value 
    if (empty()) {
        return;
    } else {
        int count = 0;
        for (auto it = pqdict_map_m.begin(); it != pqdict_map_m.end(); it++) {

            if (it == pqdict_map_m.begin()) {
                top_val_m = it->second;
                top_key_m = it->first;        
            } else { 
                if (is_max_pqdict_m) { // max pq dict 
                    if (it->second >= top_val_m) {
                        top_val_m = it->second;
                        top_key_m = it->first;
                    }
                } else { // min pq dict
                    if (it->second <= top_val_m) {
                        top_val_m = it->second;
                        top_key_m = it->first;
                    }
                }
            }
            count ++;
        }
        
    }
}

void pqdict_env_double::erase(const environment environment_key) {
    // this erases the key from the underlying map and updates the value 
    if (pqdict_map_m.empty()) {
        cerr << "Tried to erase something while empty" << endl;
        exit(1);
    }

    if (environment_key == top_key_m) { // erase the key and update top
        if (pqdict_map_m.erase(environment_key) != 1) {
            cerr << "Error: erase  in pqdict did not properly erase" << endl;
            exit(1);
        }
        update_top();

    } else { // erase without updating top
        if (pqdict_map_m.erase(environment_key) != 1) {
            cerr << "Error: error in pqdict: did not properly erase key" << endl;
            exit(1);
        }
    }

    return;
}


void pqdict_env_double::set(const environment environment_key, double value) {
    // sets a value in the underlying pqdict
    if (empty()) {
        // set the top key and value automatically 
        top_val_m = value;
        top_key_m = environment_key;
    }

    pqdict_map_m[environment_key] = value;

    if (is_max_pqdict_m) { // max pq dict
        if (value >= top_val_m) {
            top_val_m = value;
            top_key_m = environment_key;
        }
    } else { // min pq dict
        if (value <= top_val_m) {
            top_val_m = value;
            top_key_m = environment_key;
        }
    }
    return;
}

void pqdict_env_double::clear() {
    // clears the entire underlying map
    pqdict_map_m.clear();
    return;
}

bool pqdict_env_double::empty() {
    // returns true if the underlying map is empty else returns false
    return pqdict_map_m.empty();
}

/*
* *******************************************************************************************
* ****************************** Checking functions *****************************************
* *******************************************************************************************
*/

void pqdict_env_double::checker() {
    int count = 0;
    for (auto it = pqdict_map_m.cbegin(); it != pqdict_map_m.cend(); ++it) {
        count ++;
    }

    if (count != pqdict_map_m.size()) {
        cerr << "the sizes dont match failed the checker" << endl;
        exit(1);
    } 

}

void pqdict_env_double::printChecker() {
    int count = 0;
    for (auto it = pqdict_map_m.cbegin(); it != pqdict_map_m.cend(); ++it) {
        count ++;
    }
    cout << "iteration size: " << count << endl;
    cout << "map size: " << pqdict_map_m.size() << endl;
    if (count != pqdict_map_m.size()) {
        cerr << "the sizes dont match failed the checker" << endl;
        exit(1);
    } 

}


}// end namespace despot