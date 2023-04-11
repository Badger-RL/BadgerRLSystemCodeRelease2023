#include "RLData.h"





void DataTransfer::saveTrajectories(const int batch_index, int robotNumber) {

  std::string robotNumberString = std::to_string(robotNumber);

  std::string output_string = stringify(trajectories[robotNumberString], json::PrettyPrint);
  

#ifndef BUILD_NAO_FLAG
  std::ofstream output_file(getCurrentDirectory() + "/../trajectories_" +robotNumberString+ "_" + std::to_string(batch_index) + ".json");
#endif

#ifdef BUILD_NAO_FLAG
  std::ofstream output_file("/home/nao/logs/trajectories_" + robotNumberString  +"_"+ std::to_string(batch_index) + ".log");
#endif


  output_file << output_string;
  output_file.close();

/*
turned off for now, a complete file will be useful for training loops, but is unnecesarry for just logging

#ifndef BUILD_NAO_FLAG
  std::ofstream output_file2(getCurrentDirectory() + "/../complete_" + std::to_string(batch_index) + ".txt");
#endif

#ifdef BUILD_NAO_FLAG

#endif

  output_file2 << "complete";
  output_file2.close();

*/

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


void DataTransfer::newTrajectoriesJSON(int robotNumber) {
  std::string robotNumberString = std::to_string(robotNumber);
  if(!(json::has_key(trajectories, robotNumberString))){
    trajectories.insert(robotNumberString,json::object{});
  }
  else{
    trajectories[robotNumberString] = json::object{};
  }
  trajectories[robotNumberString].insert("observations", json::array{});
  trajectories[robotNumberString].insert("actions", json::array{});
  trajectories[robotNumberString].insert("next_observations", json::array{});
}


json::array DataTransfer::vectToJSON(std::vector<float> inputVector) {
  json::array output = json::array{};
  for (float i : inputVector) {
    output.push_back(i);
  }
  return output;
}



std::vector<float> DataTransfer::JSON_to_vect(json::array input) {
  std::vector<float> output;
  for (json::value i : input) {
    output.push_back(std::stof(to_string(i)));
  }
  return output;
}
