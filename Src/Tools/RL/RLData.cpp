#include "RLData.h"

void DataTransfer::saveTrajectories(const int batch_index) {
  std::string output_string = stringify(trajectories, json::PrettyPrint);
  
  std::ofstream output_file(getCurrentDirectory() + "/../trajectories_" + std::to_string(batch_index) + ".json");
  output_file << output_string;
  output_file.close();

  std::ofstream output_file2(getCurrentDirectory() + "/../complete_" + std::to_string(batch_index) + ".txt");
  output_file2 << "complete";
  output_file2.close();
}

// derived from
// https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-14-17-c
bool DataTransfer::doesFileExist(const std::string &name) {
  std::ifstream f(name.c_str());
  return f.good();
}

std::string DataTransfer::getCurrentDirectory() {
  char buff[FILENAME_MAX]; // create string buffer to hold path
  getcwd(buff, FILENAME_MAX);
  std::string currentWorkingDir(buff);
  return currentWorkingDir;
}


#ifndef BUILD_NAO_FLAG
void DataTransfer::waitForNewPolicy() {
  while (true) {
    
    if (!doesFileExist(getCurrentDirectory() + "/../shared_policy.h5")) {
#ifdef DEBUG_MODE
      std::cout << "waiting for shared policy \n";
#endif
      continue;
    }
    
    if (!doesFileExist(getCurrentDirectory() + "/../action_policy.h5")) {
#ifdef DEBUG_MODE
      std::cout << "waiting for action policy \n";
#endif
      continue;
    }
    
    if (!doesFileExist(getCurrentDirectory() + "/../value_policy.h5")) {
#ifdef DEBUG_MODE
      std::cout << "waiting for value policy \n";
#endif
      continue;
    }
    
    if (!doesFileExist(getCurrentDirectory() + "/../metadata.json")) {
#ifdef DEBUG_MODE
      std::cout << "waiting for meta data \n";
#endif
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    break;
  }
}
#endif

#ifdef BUILD_NAO_FLAG
void DataTransfer::waitForNewPolicy() {
  while (true) {
    
    if (!doesFileExist(getCurrentDirectory() + "/Config/shared_policy.h5")) {
#ifdef DEBUG_MODE
      std::cout << "waiting for shared policy \n";
#endif
      continue;
    }
    
    if (!doesFileExist(getCurrentDirectory() + "/Config/action_policy.h5")) {
#ifdef DEBUG_MODE
      std::cout << "waiting for action policy \n";
#endif
      continue;
    }
    
    if (!doesFileExist(getCurrentDirectory() + "/Config/value_policy.h5")) {
#ifdef DEBUG_MODE
      std::cout << "waiting for value policy \n";
#endif
      continue;
    }
    
    if (!doesFileExist(getCurrentDirectory() + "/Config/metadata.json")) {
#ifdef DEBUG_MODE
      std::cout << "waiting for meta data \n";
#endif
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    break;
  }
}
#endif



void DataTransfer::newTrajectoriesJSON() {
  trajectories.insert("last_values", json::array{});
  trajectories.insert("length", RLConfig::batch_size);
  trajectories.insert("observations", json::array{});
  trajectories.insert("episode_starts", json::array{});
  trajectories.insert("values", json::array{});
  trajectories.insert("actions", json::array{});
  trajectories.insert("action_means", json::array{});
  trajectories.insert("log_probs", json::array{});
  trajectories.insert("abstract_states", json::array{});
}

json::array DataTransfer::vectToJSON(std::vector<float> inputVector) {
  json::array output = json::array{};
  for (float i : inputVector) {
    output.push_back(i);
  }
  return output;
}
