#ifndef RL_CONTROL
#define RL_CONTROL

#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>
#include <CompiledNN/SimpleNN.h>
#include <CompiledNN/Tensor.h>

#include "Tools/json.h"
#include "Tools/RLConfig.h"
#include "Tools/RL/RLData.h"

#define STATS_GO_INLINE
#define STATS_DONT_USE_OPENMP
#define STATS_ENABLE_EIGEN_WRAPPERS
#define STATS_ENABLE_STDVEC_WRAPPERS
#include "Tools/NeuralNetwork/stats.hpp"

#define PI 3.14159265

class Algorithm {
private:
  Eigen::MatrixXd std_dev;
  Eigen::MatrixXd covariance_matrix;

  bool collect_new_policy = true;

  int actionLength;
  
  float current_value;
  float prev_value;
  float prev_log_prob;
  float current_log_prob;
  
  NeuralNetwork::Model *shared_model;
  NeuralNetwork::Model *action_model;
  NeuralNetwork::Model *value_model;
  
  std::string shared_policy_path;
  std::string action_policy_path;
  std::string value_policy_path;
  std::string metadata_path;
  
  float normalization_clip;
  float normalization_epsilon;
  std::vector<float> normalization_mean;
  std::vector<float> normalization_var;

  Eigen::MatrixXd action_choice;
  NeuralNetwork::TensorXf action_means;
  std::vector<float> action_mean_vector;
  
public:
  Algorithm (const std::string policyPath, const std::string policyName);
  
  float uniformRandom();
  
  std::vector<float> getFloatVectorFromJSONArray(const json::value &json_value);
  void waitForNewPolicy();

  void deleteModels();
  void updateModels();
  void deletePolicyFiles();
  
  int getActionLength(){return actionLength;}

  void processStdDevAndCov(json::value metadata);

  bool getCollectNewPolicy() { return collect_new_policy; }
  void setCollectNewPolicy(bool value) { collect_new_policy = value; }


  std::vector<float> normalizeObservation(std::vector<float>);
  bool doesFileExist(const std::string &name);
  
  std::vector<NeuralNetwork::TensorXf>
      applyModel(NeuralNetwork::Model *model, const std::vector<NeuralNetwork::TensorXf> input);
  std::vector<NeuralNetwork::TensorXf>
      inference(std::vector<NeuralNetwork::TensorXf> observation_input);

  NeuralNetwork::Model* getSharedModel() { return shared_model; }
  NeuralNetwork::Model* getActionModel() { return action_model; }
  NeuralNetwork::Model* getValueModel() { return value_model; }
  
  void setCurrentValue(float value) { current_value = value; }
  void setPreviousValue(float value) { prev_value = value; }
  void setCurrentLogProb(float value) { current_log_prob = value; }
  void setPreviousLogProb(float value) { prev_log_prob = value; }
  
  float getCurrentValue() { return current_value; }
  float getPreviousValue() { return prev_value; }
  float getCurrentLogProb() { return current_log_prob; }
  float getPreviousLogProb() { return prev_log_prob; }
  
  Eigen::MatrixXd getActionChoice() { return action_choice; }
  NeuralNetwork::TensorXf getActionMeans() { return action_means; }
  std::vector<float> getActionMeanVector() {return action_mean_vector;}

  
  std::vector<float> computeCurrentAction(std::vector<NeuralNetwork::TensorXf> action_output,
                            const int action_length);
  
};    // class Algorithm

#endif /* RL_CONTROL */
