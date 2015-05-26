#ifndef NAVIGATIONTASK_H
#define NAVIGATIONTASK_H

#include <vector>
#include <PredictiveModel.h>

// state class for predictive model
class NavState {
public:
    NavState(); 
    NavState(int r, int c, int r_num, int c_num) : 
        row(r), col(c), row_num(r_num), col_num(c_num) {}; 

    int row; 
    int col; 
    int row_num; 
    int col_num; 
    int index; 
};

// action class for predictive model
class NavAction {
public:
    NavAction(); 
    NavAction(int ind, std::string s) : index(ind), name(s) {}; 

    int index;  // 0, 1, 2, 3: up, right, down, left
    std::string name; 
}; 

// world class for predictive model
class NavWorld : public PredictiveModel <NavState, NavAction> {
public:
    NavWorld();
    NavWorld(bool sunny) {}; 

    std::vector<std::vector<int>> map, map_blocked, map_near_window;
    int row_num, col_num; 

    bool sunny_outside;
    std::vector<NavState> states; 
    std::vector<NavAction> actions; 

    std::vector<std::vector<int>> parseMap(std::string path_to_file); 
    std::vector<NavState> constructStateSet(std::vector<std::vector<int>> map); 
    std::vector<NavAction> constructActionSet(); 
    std::vector<std::vector<float>> constructRewardFunc(); 
    std::vector<std::vector<float>> constructTransitionFunc(); 
};

#endif /* end of include guard:  NavigationTask.h */
