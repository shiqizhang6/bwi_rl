#include <iostream>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <learning/Common.h>
#include <learning/ArffReader.h>
#include <learning/TwoStageTrAdaBoost.h>
#include <factory/ClassifierFactory.h>
#include <factory/AgentFactory.h>
#include <common/Util.h>

std::vector<Feature> getFeatures() {
  std::ifstream in("data/dt/blank.arff");
  ArffReader arff(in);
  in.close();
  return arff.getFeatureTypes();
}

void readArff(const std::string &filename, InstanceSet &instances, int numSourceSamples = -1) {
  std::cout << "Reading " << filename << std::endl;
  std::ifstream in(filename.c_str());
  ArffReader arff(in);
  while ((!arff.isDone()) && ((numSourceSamples < 0) || ((int)instances.size() < numSourceSamples))) {
    InstancePtr instance = arff.next();
    instances.add(instance);
  }
  in.close();
}

std::string getArffName(const std::string &student, const std::string &baseDir) {
  return baseDir + "/train/" + student + ".arff";
}

std::string getDTName(const std::string &student, const std::string &baseDir) {
  return baseDir + "/weighted/only-" + student + ".weka";
}
 
int main(int argc, const char *argv[]) {
  if (argc != 6) {
    std::cerr << "Expected 5 arguments" << std::endl;
    std::cerr << "Usage: boostGivenOrder targetStudent givenOrderFile targetDir sourceDir numSourceSamples" << std::endl;
    return 1;
  }
  std::string targetStudent = argv[1];
  std::string givenOrderFile = argv[2];
  std::string targetDir = argv[3];
  std::string sourceDir = argv[4];
  int numSourceSamples = boost::lexical_cast<int>(argv[5]);
  
  std::vector<std::string> orderedStudents;
  std::vector<double> orderedEvals;
  std::ifstream in(givenOrderFile.c_str());
  assert(in.good());
  while (in.good()) {
    std::string str;
    float val;
    in >> val;
    if (in.eof())
      break;
    in >> str;
    if (str.size() == 0)
      break;
    orderedEvals.push_back(val);
    orderedStudents.push_back(str);
  }
  in.close();
  
  for (unsigned int i = 0; i < orderedEvals.size(); i++) {
    std::cout << orderedEvals[i] << " " << orderedStudents[i] << std::endl;
  }

  InstanceSet targetData(5);
  readArff(getArffName(targetStudent,targetDir),targetData);
  
  Json::Value baseLearnerOptions;
  baseLearnerOptions["type"] = "weka";
  baseLearnerOptions["caching"] = false;
  baseLearnerOptions["options"] = "weka.classifiers.trees.REPTree";
  ClassifierPtr (*baseLearner)(const std::vector<Feature>&,const Json::Value&) = &createClassifier;
  std::vector<Feature> features = getFeatures();

  std::cout << "Creating classifier" << std::endl << std::flush;
  TwoStageTrAdaBoost classifier(features,false,baseLearner,baseLearnerOptions,10,2,-1);
  std::cout << "done creating classifier" << std::endl << std::flush;
  for (unsigned int i = 0; i < targetData.size(); i++)
    classifier.addData(targetData[i]);
 
  std::vector<float> studentWeights;

  BOOST_FOREACH(std::string &student, orderedStudents) {
    InstanceSet sourceData(5);
    readArff(getArffName(student,sourceDir),sourceData,numSourceSamples);
    for (unsigned int i = 0; i < sourceData.size(); i++)
      classifier.addSourceData(sourceData[i]);
    classifier.train();
    std::cout << "done training classifier" << std::endl << std::flush;
    //std::cout << classifier << std::endl;
    classifier.clearSourceData();
    float sourceInstanceWeight = classifier.getBestSourceInstanceWeight();
    studentWeights.push_back(sourceInstanceWeight);
    std::cout << "STUDENT WEIGHT: " << sourceInstanceWeight << " " << student << std::endl;
    if (sourceInstanceWeight > 1e-10) {
      for (unsigned int i = 0; i < sourceData.size(); i++) {
        sourceData[i]->weight = sourceInstanceWeight;
        classifier.addFixedData(sourceData[i]);
      }
    }
  }
  
  for (unsigned int i = 0; i < orderedStudents.size(); i++) {
    std::cout << studentWeights[i] << " " << orderedStudents[i] << std::endl;
  }

  std::cout << classifier << std::endl;

  return 0;
}