#ifndef PQ_DICT_H
#define PQ_DICT_H

#include "environment.h"
#include <map>

/*
* This file defines a class that is a priority queue map. it stores the max_key and max_value and updates it at any given time.
* key: environment object
* value: double
*/

namespace despot {

class pqdict_env_double {
public:
    pqdict_env_double(bool is_max_pqdict = false); // default constructor
    void erase(const environment environment_key);//erases the item with the same key
    void set(const environment environment_key, double value); // add a key value pair or set value if key exists
    void clear(); // clears the whole underlying map
    bool empty(); // checks if the map is empty
    void update_top(); // update either the max min 

    void checker();
    void printChecker();

    // the variable
    std::map<environment, double> testmap;
    std::map<environment, double> pqdict_map_m;
    environment top_key_m; //key with the maximum value or minimum value
    double top_val_m; // value of the maximum key or minimum key

    bool is_max_pqdict_m; // boolean indicator that if true indicates that it is a max_pqdict - defaults to false
};


} // end namespace despot

#endif 