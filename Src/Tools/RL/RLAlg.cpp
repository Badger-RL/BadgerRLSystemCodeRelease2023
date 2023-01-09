#include "RLAlg.h"

Algorithm::Algorithm(const int action_length, const std::string shared_policy_path,
                      const std::string action_policy_path, const std::string value_policy_path) :
std_dev(action_length, 1),
covariance_matrix (action_length, action_length),
shared_policy_path(shared_policy_path),
action_policy_path(action_policy_path),
value_policy_path(value_policy_path) {}

// derived from https://stackoverflow.com/a/36527160
float Algorithm::uniformRandom() {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  static std::default_random_engine randomEngine(seed);
  static std::uniform_real_distribution<> dis(0, 1);
  return dis(randomEngine);
}

void Algorithm::deleteModels() {
  delete shared_model;
  delete action_model;
  delete value_model;
}

#ifndef BUILD_NAO_FLAG
void Algorithm::updateModels(DataTransfer data_transfer) {
  shared_model = new NeuralNetwork::Model(shared_policy_path);
  action_model = new NeuralNetwork::Model(action_policy_path);
  value_model = new NeuralNetwork::Model(value_policy_path);
}
#endif

#ifdef BUILD_NAO_FLAG
void Algorithm::updateModels(DataTransfer data_transfer) {


  shared_model = new NeuralNetwork::Model( "/home/nao/Config/" +  shared_policy_path);
  action_model = new NeuralNetwork::Model("/home/nao/Config/"  + action_policy_path);
  value_model = new NeuralNetwork::Model( "/home/nao/Config/"  + value_policy_path);
}

#endif



void Algorithm::deletePolicyFiles(DataTransfer data_transfer) {
  std::remove((shared_policy_path).c_str());
  std::remove((action_policy_path).c_str());
  std::remove((value_policy_path).c_str());
}

void Algorithm::processStdDevAndCov(json::value metadata) {
  auto logStdArray = metadata["log_stds"];
  
  const json::array &stdArray = as_array(logStdArray);
  int index = 0;
  
  for (auto i = stdArray.begin(); i != stdArray.end(); i++) {
    const json::value &logStd = *i;
    
    double logStdDouble = (std::stod(to_string(logStd)));
    
    std_dev(index) = exp(logStdDouble);
    index += 1;
  }
  
  covariance_matrix = std_dev.array().matrix().asDiagonal();
}

std::vector<NeuralNetwork::TensorXf>
  Algorithm::applyModel(NeuralNetwork::Model *model, std::vector<NeuralNetwork::TensorXf> input) {
    std::vector<NeuralNetwork::TensorXf> output(model->getOutputs().size());
  
    NeuralNetwork::CompilationSettings settings;
  
    NeuralNetwork::SimpleNN::apply(input, output, *model,
                                   [&settings](
                                               const NeuralNetwork::Node &node,
                                               const std::vector<const NeuralNetwork::TensorXf *>&inputs,
                                               const std::vector<NeuralNetwork::TensorXf *> &outputs) {}
                                   );
    return output;
}

std::vector<float> Algorithm::computeCurrentAction(std::vector<NeuralNetwork::TensorXf> action_output,
                                     const int action_length) {
  action_means = action_output[0];
  Eigen::MatrixXd action_eigen(action_length, 1);

  // this vector is created for compatibility with vectortojson
  //std::vector<float> action_mean_vector = std::vector<float>();


  std::vector<float> newActionMeans;

  for (unsigned int i = 0; i < action_means.size(); i++) {

    action_eigen(i) = action_means[i];

    newActionMeans.push_back(action_means[i]);
  }
  action_mean_vector = newActionMeans;

  std::cout << action_eigen.cols() << std::endl;
    std::cout << action_eigen.rows() << std::endl;
  std::cout << covariance_matrix.cols() << std::endl;
    std::cout << covariance_matrix.rows() << std::endl;


  action_choice = stats::rmvnorm(action_eigen, covariance_matrix, true);
  current_log_prob = stats::dmvnorm(action_choice, action_eigen, covariance_matrix, true);

  std::vector<float> current_action = std::vector<float>();
  for (unsigned int i = 0; i < action_choice.size(); i++) {
    current_action.push_back(action_choice(i));
  }
  return current_action;
}
