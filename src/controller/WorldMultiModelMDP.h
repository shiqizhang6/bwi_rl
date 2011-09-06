#ifndef WORLDMULTIMODELMDP_V3OI0GS4
#define WORLDMULTIMODELMDP_V3OI0GS4

#include <controller/WorldMDP.h>

enum ModelUpdateType {
  BAYESIAN_UPDATES,
  POLYNOMIAL_WEIGHTS
};

class WorldMultiModelMDP: public WorldMDP {
public:
  WorldMultiModelMDP(boost::shared_ptr<RNG> rng, boost::shared_ptr<WorldModel> model, boost::shared_ptr<World> controller, boost::shared_ptr<AgentDummy> adhocAgent,const std::vector<std::vector<boost::shared_ptr<Agent> > > &agentModelList, const std::vector<double> &agentModelProbs, ModelUpdateType modelUpdateType);
  virtual void setState(const State_t &state);

  virtual void updateModels(const Observation &prevObs, Action::Type lastAction, const Observation &currentObs);

protected:
  virtual void selectModel();
  virtual void normalizeModelProbs();
  // model updates
  virtual void getNewModelProbs(const Observation &prevObs, Action::Type lastAction, const Observation &currentObs, std::vector<double> &newModelProbs);
  virtual double calculateModelProb(unsigned int modelInd, const Observation &prevObs, Action::Type lastAction, const Observation &currentObs);
  virtual bool allProbsTooLow(const std::vector<double> &newModelProbs);
  virtual void removeLowProbabilityModels();

protected:
  std::vector<std::vector<boost::shared_ptr<Agent> > > agentModelList;
  std::vector<double> agentModelProbs;
  ModelUpdateType modelUpdateType;

  const static double MIN_MODEL_PROB;
};

#endif /* end of include guard: WORLDMULTIMODELMDP_V3OI0GS4 */
