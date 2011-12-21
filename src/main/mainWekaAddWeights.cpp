#include <learning/WekaParser.h>
#include <learning/DecisionTree.h>
#include <learning/ArffReader.h>
#include <fstream>
#include <string>
#include <iostream>

void addDataToTree(boost::shared_ptr<DecisionTree> dt, ArffReader &arff);

int main(int argc, const char *argv[]) {
  // handle the command line arguments
  std::string usage = "Usage: wekaAddWeights wekaFile arffFile\n  wekaFile is a tree generated by weka\n  arffFile is a file containing the training data in weka's format";
  if (argc != 3) {
    std::cout << usage << std::endl;
    return 1;
  }
  std::string wekaFile = argv[1];
  const char *arffFile = argv[2];
  
  // read in the original tree
  WekaParser parser(wekaFile,5);
  boost::shared_ptr<DecisionTree> dt = parser.makeDecisionTree();
  std::cerr << "Parsed original tree" << std::endl;

  // open the arffFile and set up some variables
  std::ifstream arffIn(arffFile);
  ArffReader arff(arffIn);
  std::cerr << "Parsed arff header" << std::endl;
  // read in the header
  // add data to tree
  addDataToTree(dt,arff);
  std::cerr << "Added data to tree" << std::endl;
  arffIn.close();

  //dt->randomizeUnseenLeaves();
  //std::cerr << "Randomized unseen leaves" << std::endl;
  
  //dt->generalizeUnseenLeaves();
  //std::cerr << "Generalized unseen leaves" << std::endl;
  
  std::cout << *dt;
  
  return 0;
}

void addDataToTree(boost::shared_ptr<DecisionTree> dt, ArffReader &arff) {
  int count = 0;
  std::string classFeature = arff.getClassFeature();
  while (!arff.isDone()) {
    count++;
    if (count % 10000 == 0)
      std::cerr << count << std::endl;
    
    InstancePtr instance = arff.next();
    dt->addData(instance);
  }
}
