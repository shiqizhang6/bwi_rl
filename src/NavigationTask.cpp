
#include "NavigationTask.h"
#include <fstream>

#define BONUS_FOR_SUCCESS 10.0
#define COST_FOR_ACTION -1.0
#define PENALTY_FOR_FAILURE -10.0
#define PROB_STUCK_BLOCKED 0.5
#define PROB_STUCK_NOT_BLOCKED 0.05
#define PROB_LOST_SUNNY 0.8
#define PROB_LOST_NOT_SUNNY 0.05

NavState::NavState(int r, int c, int r_num, int c_num) {
    index = row * col_num + col; 
}

NavWorld::NavWorld(bool sunny) {
    sunny_outside = sunny; 

    map = NavWorld::parseMap("/home/szhang/catkin_ws/src/bwi_rl/maps/map.txt"); 
    map_blocked = NavWorld::parseMap("/home/szhang/catkin_ws/src/bwi_rl/maps/map_blocked.txt"); 
    map_near_window = NavWorld::parseMap("/home/szhang/catkin_ws/src/bwi_rl/maps/map_near_window.txt"); 

    row_num = map.size();
    col_num = map[0].size(); 

    states = NavWorld::constrctStateSet(map); 
    actions = NavWorld::constrctActionSet(); 
}

std::vector<std::vector<int>> NavWorld::parseMap(std::string path_to_file) {
    std::vector<std::vector<int>> ret;

    ifstream input_file(path_to_file);
    if (input_file) {
        std::string str; 
        vector<int> vec;
        while ( inputFile >> str ) {
            if (str == "\n") {
                ret.push_back(vec); 
                vec.clear();  
            } else {
                vec.push_back(std::stoi(str));  
            }
        }
    }
    return ret; 
}

std::vector<NavState> NavWorld::constructStateSet(std::vector<std::vector<int>> map) {
    std::vector<NavState> ret;
    for (int i=0; i<row_num; i++) {
        for (int j=0; j<col_num; j++) {
            NavState s = NavState(i, j, row_num, col_num); 
            ret.push_back(s); 
        }
    }
    ret.push_back(NavState(-1, -1, row_num, col_num)); 
    return ret; 
}

std::vector<NavAction> NavWorld::constructActionSet() {
    std::vector<NavAction> ret;
    ret.push_back(NavAction(0, std::string("up"))); 
    ret.push_back(NavAction(1, std::string("right"))); 
    ret.push_back(NavAction(2, std::string("down"))); 
    ret.push_back(NavAction(3, std::string("left"))); 
    return ret; 
}

/* 
    to fill up the interface methods 
*/

bool NavWorld::isTerminalState(NavState s) {
    // the top-right conner is the terminal state. 
    return (s.row == 0 && s.col = 4) || (s.row == -1 && s.col == -1); 
}

void NavWorld::getActionsAtState(NavState s, std::vector<NavAction> &actions) {
    int row = s.row, col = s.col; 
    actions.clear(); 
    if (row == -1 && col == -1) return; 
    
    for (int i=0; i<this->actions.size(); i++) {
        if (this->actions[i].name == "up") {
            if (row == 0) continue;
            if (map_blocked[row-1][col] == 1) continue;
            actions.push_back(this->actions[i]); 
        }
        else if (actions[i].name == "right") {
            if (col == col_num-1) continue; 
            if (map_blocked[row][col+1] == 1) continue; 
            actions.push_back(this->actions[i]);
        }
        else if (actions[i].name == "down") {
            if (row = row_num-1) continue; 
            if (map_blocked[row+1][col] == 1) continue; 
            actions.push_back(this->actions[i]); 
        }
        else if (actions[i].name == "left") {
            if (col == 0) continue;
            if (map_blocked[row][col-1] == 1) continue; 
            actions.push_back(this->actions[i]); 
        }
    }
}

void NavWorld::getStateVector(std::vector<NavState> &states) {
    states = this->states; 
}

void NavWorld::getTransitionDynamics(NavState &state, NavAction &action, 
    std::vector<NavState> &next_states, 
    std::vector<float> &rewards, 
    std::vector<float> &probabilities) {

    int row = state.row, col = state.col; 
    std::vector<NavAction> possible_actions; 
    NavWorld::getActionsAtState(state, possible_actions); 

    if (std::find(possible_actions.begin(), possible_actions.end(), action) == 
        action == possible_actions.end())
        std::cerr << "action " << action.name << " not valid" << std::endl; 

    next_states.clear();
    next_states.push_back(state); 
    if (action.name == "up")
        next_states.push_back(NavState(state.row-1, state.col, state.row_num,
            state.col_num));
    else if (action.name == "right")
        next_states.push_back(NavState(state.row, state.col+1, state.row_num,
            state.col_num));
    else if (action.name == "down")
        next_states.push_back(NavState(state.row+1, state.col, state.row_num,
            state.col_num));
    else if (action.name == "left")
        next_states.push_back(NavState(state.row, state.col-1, state.row_num,
            state.col_num));

    next_states.push_back(NavState(-1, -1)); 

    rewards.clear();
    for (int i=0; i<next_states.size(); i++) {
        if (isTerminalState(next_states[i]) && (next_states[i].row == -1))
            rewards.push_back(PENALTY_FOR_FAILURE); 
        else if (isTerminalState(next_states[i]))
            rewards.push_back(BONUS_FOR_SUCCESS); 
        else
            rewards.push_back(COST_FOR_ACTION); 
    }

    // initialized with a uniform distribution
    probabilities = vector<float> (next_states.size(), 1.0/next_states.size()); 

    int next_goal_row = next_states[1].row;
    int next_goal_col = next_states[1].col;

    bool next_goal_blocked = (1 == map_blocked[next_goal_row][next_goal_col]);

    if (NavWorld.sunny_outside && map_near_window[state.row][state.col] == 1) {
        probabilities[0] *= 1.0 - PROB_LOST_SUNNY;
        probabilities[1] *= 1.0 - PROB_LOST_SUNNY;
        probabilities[2] *= PROB_LOST_SUNNY; 
    } else {
        probabilities[0] *= 1.0 - PROB_LOST_NOT_SUNNY;
        probabilities[1] *= 1.0 - PROB_LOST_NOT_SUNNY;
        probabilities[2] *= PROB_LOST_NOT_SUNNY;
    }

    if (next_goal_blocked) {
        probabilities[0] *= PROB_STUCK_BLOCKED; // not moving, stuck in current
        probabilities[1] *= 1.0 - PROB_STUCK_BLOCKED; // moved to "blocked" one
    } else {
        probabilities[0] *= PROB_STUCK_NOT_BLOCKED; 
        probabilities[1] *= 1.0 - PROB_STUCK_NOT_BLOCKED; 
    }

    float sum = 0; 
    for (int i=0; i<probabilities.size(); ++i, sum += probabilities[i]) {}
    probabilities = probabilities / sum; 
}

std::string NavWorld::generateDescription(unsigned int indent) {
    std::string str(""); 
    return str;
}


