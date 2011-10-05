#ifndef PLANNINGFACTORY_4TYHDV2K
#define PLANNINGFACTORY_4TYHDV2K

/*
File: PlanningFactory.h
Author: Samuel Barrett
Description: generates objects for planning
Created:  2011-08-24
Modified: 2011-10-02
*/

#include <boost/shared_ptr.hpp>
#include <common/RNG.h>
#include <controller/WorldMDP.h>
#include <controller/ModelUpdater.h>
#include <controller/ModelUpdaterBayes.h>
#include <planning/MCTS.h>
#include <planning/ValueEstimator.h>
#include <planning/UCTEstimator.h>

// model updater
boost::shared_ptr<ModelUpdaterBayes> createModelUpdaterBayes(boost::shared_ptr<RNG> rng, boost::shared_ptr<WorldMDP> mdp, const std::vector<std::vector<boost::shared_ptr<Agent> > > &modelList, const std::vector<double> &modelProbs, const std::vector<std::string> &modelDescriptions, ModelUpdateType updateType);

boost::shared_ptr<ModelUpdater> createModelUpdater(boost::shared_ptr<RNG> rng, boost::shared_ptr<WorldMDP> mdp, boost::shared_ptr<Agent> adhocAgent, const Point2D &dims, int replacementInd, const Json::Value &options);

// WORLD MDPs
boost::shared_ptr<WorldMDP> createWorldMDP(boost::shared_ptr<RNG> rng, const Point2D &dims, bool beliefMDP, unsigned int numBeliefs = 1, unsigned int numBins = 1);
boost::shared_ptr<WorldMDP> createWorldMDP(boost::shared_ptr<RNG> rng, const Point2D &dims, const Json::Value &options);

// VALUE ESTIMATORS

boost::shared_ptr<UCTEstimator<State_t,Action::Type> > createUCTEstimator(boost::shared_ptr<RNG> rng, Action::Type numActions, float lambda, float gamma, float rewardBound, float rewardRangePerStep, float initialValue, unsigned int initialStateVisits, unsigned int initalStateActionVisits, float unseenValue,bool teheoreticallyCorrectLambda);

boost::shared_ptr<ValueEstimator<State_t,Action::Type> > createValueEstimator(boost::shared_ptr<RNG> rng, Action::Type numActions, const Json::Value &options);

boost::shared_ptr<ValueEstimator<State_t,Action::Type> > createValueEstimator(unsigned int randomSeed, Action::Type numActions, const Json::Value &options);

// MCTS
boost::shared_ptr<MCTS<State_t,Action::Type> > createMCTS(boost::shared_ptr<Model<State_t,Action::Type> > model, boost::shared_ptr<ValueEstimator<State_t,Action::Type> > valueEstimator,boost::shared_ptr<ModelUpdater> modelUpdater,unsigned int numPlayouts, double maxPlanningTime, unsigned int maxDepth);

boost::shared_ptr<MCTS<State_t,Action::Type> > createMCTS(boost::shared_ptr<Model<State_t,Action::Type> > model, boost::shared_ptr<ValueEstimator<State_t,Action::Type> > valueEstimator,boost::shared_ptr<ModelUpdater> modelUpdater,const Json::Value &options);

#endif /* end of include guard: PLANNINGFACTORY_4TYHDV2K */
