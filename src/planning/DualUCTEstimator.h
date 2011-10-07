#ifndef DUALUCTESTIMATOR_F3MKQ7CG
#define DUALUCTESTIMATOR_F3MKQ7CG

/*
File: DualUCTEstimator.h
Author: Samuel Barrett
Description: combines 2 value estimators
Created:  2011-10-01
Modified: 2011-10-03
*/

#include <boost/shared_ptr.hpp>

#include "UCTEstimator.h"

template<class State, class Action>
class DualUCTEstimator: public ValueEstimator<State,Action> {
public:
  DualUCTEstimator(boost::shared_ptr<RNG> rng, boost::shared_ptr<UCTEstimator<State,Action> > mainValueEstimator, boost::shared_ptr<UCTEstimator<State,Action> > generalValueEstimator, float b, const StateConverter &stateConverter);

  Action selectWorldAction(const State &state);
  Action selectPlanningAction(const State &state);
  void startRollout();
  void finishRollout(const State &state, bool terminal);
  void visit(const State &state, const Action &action, float reward);
  void restart();
  std::string generateDescription(unsigned int indentation = 0);
  
  Action selectAction(const State &state, bool useBounds);
  float calcActionValue(const State &state, const Action &action, bool useBounds);

private:
  boost::shared_ptr<RNG> rng;
  boost::shared_ptr<UCTEstimator<State,Action> > mainValueEstimator;
  boost::shared_ptr<UCTEstimator<State,Action> > generalValueEstimator;
  float b;
  StateConverter stateConverter;
};

template<class State, class Action>
DualUCTEstimator<State,Action>::DualUCTEstimator(boost::shared_ptr<RNG> rng, boost::shared_ptr<UCTEstimator<State,Action> > mainValueEstimator, boost::shared_ptr<UCTEstimator<State,Action> > generalValueEstimator, float b, const StateConverter &stateConverter):
  rng(rng),
  mainValueEstimator(mainValueEstimator),
  generalValueEstimator(generalValueEstimator),
  b(b),
  stateConverter(stateConverter)
{
}

template<class State, class Action>
Action DualUCTEstimator<State,Action>::selectWorldAction(const State &state) {
  return selectAction(state,false);
}

template<class State, class Action>
Action DualUCTEstimator<State,Action>::selectPlanningAction(const State &state) {
  return selectAction(state,true);
}

template<class State, class Action>
void DualUCTEstimator<State,Action>::startRollout() {
  mainValueEstimator->startRollout();
  generalValueEstimator->startRollout();
}

template<class State, class Action>
void DualUCTEstimator<State,Action>::finishRollout(const State &state, bool terminal) {
  State generalState = stateConverter.convertBeliefStateToGeneralState(state);
  mainValueEstimator->finishRollout(state,terminal);
  generalValueEstimator->finishRollout(generalState,terminal);
}

template<class State, class Action>
void DualUCTEstimator<State,Action>::visit(const State &state, const Action &action, float reward) {
  State generalState = stateConverter.convertBeliefStateToGeneralState(state);
  mainValueEstimator->visit(state,action,reward);
  generalValueEstimator->visit(generalState,action,reward);
}

template<class State, class Action>
void DualUCTEstimator<State,Action>::restart() {
  mainValueEstimator->restart();
  generalValueEstimator->restart();
}

template<class State, class Action>
std::string DualUCTEstimator<State,Action>::generateDescription(unsigned int indentation) {
  return indent(indentation) + "DualUCTEstimator: combining the value functions:\n" + mainValueEstimator->generateDescription(indentation+1) + "\n" + generalValueEstimator->generateDescription(indentation+1);
}

template<class State, class Action>
float DualUCTEstimator<State,Action>::calcActionValue(const State &state, const Action &action, bool useBounds) {
  State generalState = stateConverter.convertBeliefStateToGeneralState(state);
/*
  unsigned int n = mainValueEstimator->getNumVisits(state,action);
  unsigned int nh = generalValueEstimator->getNumVisits(generalState,action);
  float mu = 0.5;
  float mainVal = mainValueEstimator->calcActionValue(state,action,false);
  float useBoundsVal = mainValueEstimator->calcActionValue(state,action,useBounds);
  float bound = useBoundsVal - mainVal;
  float generalVal = generalValueEstimator->calcActionValue(generalState,action,false);
  if ((n == 0) && (nh == 0))
    return useBoundsVal;
  float beta = nh / (n + nh + n * nh * b * b / (mu * (1.0 - mu)));
  float val = (1 - beta) * mainVal + generalVal + bound;
  //std::cout << "vals: " << mainVal << " " << generalVal << " " << bound << " " << beta << std::endl;
  return val;
*/
  return generalValueEstimator->calcActionValue(generalState,action,useBounds);
}

template<class State, class Action>
Action DualUCTEstimator<State,Action>::selectAction(const State &state, bool useBounds) {
  //std::cout << "TOP SELECT ACTION" << std::endl << std::flush;
  std::vector<Action> maxActions;
  float maxVal = -BIGNUM;
  float val;

  for (Action a = (Action)0; a < mainValueEstimator->getNumActions(); a = Action(a+1)) {
    std::pair<State,Action> key(state,a);
    //std::cout << "before calc" << std::endl;
    val = calcActionValue(state,a,useBounds);
    //std::cout << "after calc" << std::endl;

    //std::cerr << val << " " << maxVal << std::endl;
    if (fabs(val - maxVal) < EPS)
      maxActions.push_back(a);
    else if (val > maxVal) {
      maxVal = val;
      maxActions.clear();
      maxActions.push_back(a);
    }
  }
  
  //std::cout << "BOTTOM SELECT ACTION " << maxActions.size() << std::endl << std::flush;
  return maxActions[rng->randomInt(maxActions.size())];
}

#endif /* end of include guard: DUALUCTESTIMATOR_F3MKQ7CG */