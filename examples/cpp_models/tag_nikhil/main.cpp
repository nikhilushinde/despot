#include <despot/planner.h>
#include "tag_nikhil.h"

using namespace despot;

class MyPlanner: public Planner {
public:

    MyPlanner() {}

    MyPlanner(string lower_bound_str,
                string base_lower_bound_str,
                string upper_bound_str,
                string base_upper_bound_str) : Planner(lower_bound_str,
                    base_lower_bound_str,
                    upper_bound_str,
                    base_lower_bound_str) {
    }

    DSPOMDP* InitializeModel(option::Option* options) {
        DSPOMDP* model = new TagNikhil();
        return model;
    }

    World* InitializeWorld(std::string&  world_type, DSPOMDP* model, option::Option* options)
    {
        return InitializePOMDPWorld(world_type, model, options);
    }

    void InitializeDefaultParameters() {
        Globals::config.pruning_constant = 0.01;
    }

    std::string ChooseSolver(){
        return "DESPOT";
    }
};

using namespace std;
int main (int argc, char * argv[]) {
    return MyPlanner("DEFAULT", "DEFAULT", "DEFAULT", "DEFAULT").RunEvaluation(argc, argv);
}