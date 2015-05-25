#include <vector>

// state class for predictive model
class NavState {
public:
    int roboPos; 
    bool sunny;
    bool blocked; 
    int index; 
    
    NavState(); 
    NavState(int roboPos, bool sunny, bool blocked); 
    int getIndex(); 
};

// action class for predictive model
class NavAction {
public:
    NavAction(); 
    NavAction(int ind) : index(ind); 
    int index;  // 0, 1, 2, 3: up, right, down, left
}; 
