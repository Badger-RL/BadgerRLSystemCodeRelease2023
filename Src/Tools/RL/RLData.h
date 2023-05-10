#ifndef RLData_h
#define RLData_h

#include <thread>
#include <chrono>

#include "Tools/json.h"
#include "Tools/RLConfig.h"

#define DEBUG_MODE false

extern int RLConfig::batch_size;

class DataTransfer {
private:
  json::object trajectories =json::object{};
  bool collect_new_policy;
  
public:
  DataTransfer(const bool collect_new_policy) :
  collect_new_policy(collect_new_policy) {}
  
  void saveTrajectories(const int batch_index, int robotNumber);
  bool doesFileExist(const std::string &name);
  std::string getCurrentDirectory();
  void waitForNewPolicy();
  void newTrajectoriesJSON(int robotNumber);
  json::array vectToJSON(std::vector<float> input_vector);
  std::vector<float> JSON_to_vect(json::array input);

  
  bool getCollectNewPolicy() { return collect_new_policy; }
  void setCollectNewPolicy(bool value) { collect_new_policy = value; }
  

  void appendTrajectoryValue(std::string index, bool value, int robotNumber) {
    trajectories[std::to_string(robotNumber)][index].push_back(value);
  }

  void appendTrajectoryValue(std::string index, int value, int robotNumber) {
    trajectories[std::to_string(robotNumber)][index].push_back(value);
  }
  
  void appendTrajectoryValue(std::string index, float value, int robotNumber) {
    trajectories[std::to_string(robotNumber)][index].push_back(value);
  }
  
  void appendTrajectoryValue(std::string index, std::vector<float> value, int robotNumber) {
    trajectories[std::to_string(robotNumber)][index].push_back(vectToJSON(value));
  }



};    // class DataTransfer

#endif /* RLData_h */
