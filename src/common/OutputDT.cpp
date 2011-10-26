/*
File: OutputDT.cpp
Author: Samuel Barrett
Description: outputs information for the decision tree
Created:  2011-10-26
Modified: 2011-10-26
*/

#include "OutputDT.h"
#include <boost/lexical_cast.hpp>
#include <factory/AgentFactory.h>

OutputDT::FeatureType::FeatureType(const std::string &name, int numCategories):
  name(name),
  numCategories(numCategories)
{
}

OutputDT::OutputDT(const std::string &filename, const Point2D &dims, unsigned int numPredators, const std::vector<std::string> &modelNames, bool outputArff):
  out(filename.c_str()),
  dims(dims),
  numPredators(numPredators),
  modelNames(modelNames),
  outputArff(outputArff)
{
  for (unsigned int i = 0; i < modelNames.size(); i++)
    models.push_back(createAgent(0,dims,modelNames[i],0,1,Json::Value(),Json::Value()));

  features.push_back(FeatureType("Step",0));
  features.push_back(FeatureType("PredInd",numPredators));
  // relative positions of the agents
  features.push_back(FeatureType("Prey.dx",0));
  features.push_back(FeatureType("Prey.dy",0));
  for (unsigned int i = 0; i < numPredators; i++){
    std::string pred = "Pred" + boost::lexical_cast<std::string>(i);
    features.push_back(FeatureType(pred + ".dx",0));
    features.push_back(FeatureType(pred + ".dy",0));
  }
  // some derived features
  for (unsigned int a = 0; a < Action::NUM_NEIGHBORS; a++) {
    std::string name = "Occupied." + boost::lexical_cast<std::string>(a);
    features.push_back(FeatureType(name,2));
  }
  features.push_back(FeatureType("NextToPrey",2));
  // most likely actions predicted by the models
  for (unsigned int i = 0; i < modelNames.size(); i++)
    features.push_back(FeatureType(modelNames[i]+".des",Action::NUM_ACTIONS));
  // the true action
  features.push_back(FeatureType("Pred.act",Action::NUM_ACTIONS));

  if (outputArff)
    outputArffHeader();
  else
    outputCSVHeader();
}

void OutputDT::outputStep(unsigned int numSteps, const Observation &obs) {
  assert(obs.preyInd == 0);
  if (numSteps > 1) {
    for (unsigned int predInd = 0; predInd < numPredators; predInd++) {
      Point2D origin = prevObs.positions[predInd+1];
      out << numSteps - 1;
      out << "," << predInd;
      // agents' positions relative to the current predator
      for (unsigned int i = 0; i < obs.positions.size(); i++) {
        Point2D diff = getDifferenceToPoint(dims,origin,prevObs.positions[i]);
        out << "," << diff.x << "," << diff.y;
      }
      // some derived features
      bool next2prey = false;
      for (unsigned int a = 0; a < Action::NUM_NEIGHBORS; a++) {
        Point2D pos = movePosition(dims,origin,(Action::Type)a);
        bool occupied = false;
        for (unsigned int i = 0; i < obs.positions.size(); i++) {
          if (i == predInd + 1)
            continue;
          if (obs.positions[i] == pos) {
            occupied = true;
            if (i == 0)
              next2prey = true;
            break;
          }
        }
        out << "," << occupied;
      }
      out << "," << next2prey;
      // actions predicted by models
      prevObs.myInd = predInd;
      for (unsigned int i = 0; i < models.size(); i++) {
        ActionProbs ap = models[i]->step(prevObs);
        Action::Type action = ap.maxAction();
        out << "," << action;
      }
      // the true action taken
      Point2D diff = getDifferenceToPoint(dims,prevObs.positions[predInd],obs.positions[predInd]);
      Action::Type action = getAction(diff);
      out << "," << action;

      out << std::endl;
    }
  }
  prevObs = obs;
}

void OutputDT::outputArffHeader() {
  out << "@relation 'Generated by OutputDT.cpp for a " << dims.x << "x" << dims.y << " world'" << std::endl;
  out << std::endl;
  for (unsigned int i = 0; i < features.size(); i++) {
    out << "@attribute " << features[i].name << " ";
    if (features[i].numCategories == 0)
      out << "numeric";
    else {
      out << "{";
      for (int j = 0; j < features[i].numCategories; j++) {
        out << j;
        if (j != features[i].numCategories-1)
          out << ",";
      }
      out << "}";
    }
    out << std::endl;
  }
  out << std::endl;
  out << "@data" << std::endl;
}

void OutputDT::outputCSVHeader() {
  for (unsigned int i = 0; i < features.size(); i++)
    out << features[i].name << ",";
  out << std::endl;
}
