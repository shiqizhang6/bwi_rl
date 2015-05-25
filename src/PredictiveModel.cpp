
#include "bwi_rl/planning/PredictiveModel.h"
#include "bwi_rl/planning/NavigationTask.h"

template<>
class PredictiveModel <NavState, NavAction> {
public: 
    bool isTerminalState(NavState state) {
        if (state.roboPos == 1) return true;
        else return false;
    }
    
    void getActionAtState(NavState state, std::vector<NavAction> actions) {

        NavAction up(0); 
        NavAction right(1);
        NavActoin down(2); 
        NavAction left(3); 

        actions.clear();

        switch (state.roboPos) {
            case 1: 
                actions.push_back(down); 
                break;
            case 2:  case 10: case 13: case 16: case 19: case 11: case 14:  
            case 17: case 20: case 12: case 15: case 18: case 21: case 31: 
                actions.push_back(up);
                actions.push_back(down); 
                break;
            case 4: case 5: case 7: case 8: case 23: case 24: case 26: case 27: 
            case 34: case 35: 
                actions.push_back(left); 
                actions.push_back(right); 
                break;
            case 3: case 29: 
                actions.push_back(right); 
                actions.push_back(down); 
                break;
            case 6: case 25:
                actions.push_back(up);
                actions.push_back(right);
                actions.push_back(down);
                actions.push_back(left);
                break; 
            case 36: case 28: 
                actions.push_back(up); 
                actions.push_back(left); 
                break;
            case 9:
                actions.push_back(left); 
                actions.push_back(down); 
                break; 
            case 30:
                actions.push_back(up); 
                actions.push_back(left); 
                actions.push_back(down); 
                break; 
            case 32:
                actions.push_back(up); 
                actions.push_back(right); 
                break; 
            case 33:
                actions.push_back(up); 
                actions.push_back(left); 
                actions.push_back(right); 
                break; 
            default:
                break; 
        }
        
    }
    
}

