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
            //cout << "In update top: START of for loop" << endl;
            if (it == pqdict_map_m.begin()) {
                top_val_m = it->second;
                top_key_m = it->first;        
            } else { // max pq dict 
                if (is_max_pqdict_m) {
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

            //cout << "In update top: END of for loop" << endl;
            count ++;
        }
        
        //cout << endl << "Total number of iterations done in the update top function: " << count << endl;
        //cout << "The size of the pqdict_map_m: " << pqdict_map_m.size() << endl << endl;
    }
}

void pqdict_env_double::erase(const environment environment_key) {
    // this erases the key from the underlying map and updates the value 
    if (pqdict_map_m.empty()) {
        cerr << "Tried to erase something while empty" << endl;
        exit(1);
    }
    /*
    if (pqdict_map_m.find(environment_key) == pqdict_map_m.end()) {
        environment_key.printState();
        top_key_m.printState();
        
        cerr << "INVALID KEY WAS ERASED!!!!!!!!!" << endl;
        exit(1);
    }
    */
    //cout << "Count of the erased key is: " << pqdict_map_m.count(environment_key) << endl;
    if (environment_key == top_key_m) {
        if (pqdict_map_m.erase(environment_key) != 1) {
            cerr << "Error: erase  in pqdict did not properly erase" << endl;
            exit(1);
        }
        //pqdict_map_m[environment_key] = -10;
        //cout << "erased output: " << pqdict_map_m.erase(environment_key) << endl;
        //cout << "ABOUT TO UPDATE TOP" << endl;
        update_top();
        //cout << "FINISHED UPDATING TOP" << endl;
    } else {
        //cout << "non topped erased output: " << pqdict_map_m.erase(environment_key) << endl;
    }

    checker();
    return;
}

void pqdict_env_double::checker() {
    int count = 0;
    for (auto it = pqdict_map_m.cbegin(); it != pqdict_map_m.cend(); ++it) {
        count ++;
    }
    //cout << "iteration size: " << count << endl;
    //cout << "map size: " << pqdict_map_m.size() << endl;
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

void pqdict_env_double::set(const environment environment_key, double value) {
    // sets a value in the underlying pqdict
    if (empty()) {
        // set the top key and value automatically 
        top_val_m = value;
        top_key_m = environment_key;
    }

    //cout << "BEFORE SETTING size: " << pqdict_map_m.size() << " value: " <<  pqdict_map_m[environment_key] <<  endl;
    pqdict_map_m[environment_key] = value;
    /*
    cout << "AFTER SETTING size: " << pqdict_map_m.size() << " value: " <<  pqdict_map_m[environment_key] <<  endl;
    
    cout << pqdict_map_m[environment_key];
    cout << pqdict_map_m[environment_key];
    cout << pqdict_map_m[environment_key];
    cout << pqdict_map_m[environment_key];
    cout << pqdict_map_m[environment_key];
    cout << pqdict_map_m[environment_key];
    cout << pqdict_map_m[environment_key];
    cout << pqdict_map_m[environment_key];
    cout << pqdict_map_m[environment_key];
    cout << pqdict_map_m[environment_key] << endl;

    cout << "contains: " << (pqdict_map_m.find(environment_key) != pqdict_map_m.cend()) << endl;
    cout << (environment_key == environment_key) << endl;
    cout << "true is: " << true << endl << endl << endl;


    
    cout << "test map size: " << testmap.size() << endl;
    cout << testmap[environment_key] << endl; 
    cout << testmap[environment_key] << endl; 
    cout << testmap[environment_key] << endl; 
    cout << testmap[environment_key] << endl; 
    cout << testmap[environment_key] << endl; 
    cout << testmap[environment_key] << endl; 
    cout << testmap[environment_key] << endl; 
    cout << testmap[environment_key] << endl; 
    cout << testmap[environment_key] << endl; 
    cout << "test map size: " << testmap.size() << endl;

    environment_key.printState();
    cout << endl << endl << endl;
    */
    

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
    checker();
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


}// end namespace despot