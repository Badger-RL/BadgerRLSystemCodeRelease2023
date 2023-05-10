#include "RLAlg.h"
#include <iostream>
#include <cassert>

Algorithm::Algorithm(const std::string policyPath, const std::string policyName)/* :
std_dev(action_length, 1),
covariance_matrix (action_length, action_length),
shared_policy_path(shared_policy_path),
action_policy_path(action_policy_path),
value_policy_path(value_policy_path)*/ {




metadata_path = policyPath + policyName + "/metadata.json";

std::cout << "Metadata path: "+ metadata_path << std::endl;

std::ifstream metadataFile(metadata_path);

metadata = json::parse(metadataFile);

actionLength = std::stoi(to_string(metadata["action_length"]));

std_dev = Eigen::MatrixXd(actionLength,1);
covariance_matrix = Eigen::MatrixXd(actionLength,actionLength);

if (to_string(metadata["policy_type"]) == "CORL_CQL")
{
shared_policy_path = policyPath + policyName +  "/policy.h5";
}
else{ //default case is a PPO policy
shared_policy_path = policyPath + policyName + "/shared_policy.h5";
action_policy_path = policyPath + policyName + "/action_policy.h5";
value_policy_path = policyPath + policyName + "/value_policy.h5";
}


normalization_clip = std::stof(to_string(metadata["clip"]));
normalization_epsilon = std::stof(to_string(metadata["epsilon"]));
normalization_mean = getFloatVectorFromJSONArray(metadata["mean"]);
normalization_var = getFloatVectorFromJSONArray(metadata["var"]); 


}

std::vector<float> Algorithm::normalizeObservation(std::vector<float> observation_vector) {


  if (to_string(metadata["policy_type"]) == "CORL_CQL")
  {
    assert(observation_vector.size() == 12);
    for(int i = 0; i < 8; i++)// this removes the dummy information then truncates the empty slots out of the vector
    {
      observation_vector[i] = observation_vector[i+4];
    }
    observation_vector.resize(8);
    assert(observation_vector.size() == 8);

  }




  assert(observation_vector.size() == metadata["observation_length"]);

  std::vector<float> normalized_vector = std::vector<float>(observation_vector.size());
  for (int i = 0; i < observation_vector.size(); i++) {
    float normalized = (observation_vector[i] - normalization_mean[i]) / sqrt(normalization_var[i] + normalization_epsilon);
    
    if (normalized > normalization_clip) {
      normalized = normalization_clip;
    }

    if (normalized < -normalization_clip) {
      normalized = -normalization_clip;
    }

    normalized_vector[i] =  normalized;
  }
  
  return normalized_vector;
}


// derived from https://stackoverflow.com/a/36527160
float Algorithm::uniformRandom() {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  static std::default_random_engine randomEngine(seed);
  static std::uniform_real_distribution<> dis(0, 1);
  return dis(randomEngine);
}

void Algorithm::deleteModels() {
if (to_string(metadata["policy_type"]) == "CORL_CQL")
  {
    delete shared_model;
  }
  else{
    delete shared_model;
    delete action_model;
    delete value_model;
  }
}

void Algorithm::updateModels() {
  if (to_string(metadata["policy_type"]) == "CORL_CQL")
  {
    shared_model = new NeuralNetwork::Model(shared_policy_path);
  }
  else{
    shared_model = new NeuralNetwork::Model(shared_policy_path);
    action_model = new NeuralNetwork::Model(action_policy_path);
    value_model = new NeuralNetwork::Model(value_policy_path);
  }
}


std::vector<float> Algorithm::getFloatVectorFromJSONArray(const json::value &json_value) {
  std::vector<float> result;
  const json::array &json_array = as_array(json_value);
  for(auto entry = json_array.begin(); entry != json_array.end(); ++entry) {
     const json::value &entry_string = *entry;
     result.push_back(std::stof(to_string(entry_string)));
  }
  return result;
}

void Algorithm::deletePolicyFiles() {
  if (to_string(metadata["policy_type"]) == "CORL_CQL")
  {
    std::remove((shared_policy_path).c_str());  
  }
  else{
    std::remove((shared_policy_path).c_str());
    std::remove((action_policy_path).c_str());
    std::remove((value_policy_path).c_str());
  }
}

void Algorithm::waitForNewPolicy() {
  while (true) {
    
    if (to_string(metadata["policy_type"]) == "CORL_CQL")
    {

      if (!doesFileExist(shared_policy_path)) {
        #ifdef DEBUG_MODE
              std::cout << "waiting for shared policy \n";
        #endif
              continue;
            }
    }
    else
    {
      if (!doesFileExist(shared_policy_path)) {
  #ifdef DEBUG_MODE
        std::cout << "waiting for shared policy \n";
  #endif
        continue;
      }
      
      if (!doesFileExist(action_policy_path)) {
  #ifdef DEBUG_MODE
        std::cout << "waiting for action policy \n";
  #endif
        continue;
      }
      
      if (!doesFileExist(value_policy_path)) {
  #ifdef DEBUG_MODE
        std::cout << "waiting for value policy \n";
  #endif
        continue;
      }
    }

    if (!doesFileExist(metadata_path)) {
#ifdef DEBUG_MODE
      std::cout << "waiting for meta data \n";
#endif
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    break;
  }
}

bool Algorithm::doesFileExist(const std::string &name) {
  std::ifstream f(name.c_str());
  return f.good();
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

std::vector<NeuralNetwork::TensorXf> Algorithm::inference(std::vector<NeuralNetwork::TensorXf> observation_input)
{
   
    if (to_string(metadata["policy_type"]) == "CORL_CQL")
    {



    std::vector<NeuralNetwork::TensorXf> shared_output =
        applyModel(getSharedModel(), observation_input);
    
    NeuralNetwork::TensorXf action = shared_output[0];
   // std::cout << "REACHED" << std::endl;
    return shared_output;
    }
    else
    {


    std::vector<NeuralNetwork::TensorXf> shared_output =
        applyModel(getSharedModel(), observation_input);
    
    NeuralNetwork::TensorXf latent_action = shared_output[0];
    NeuralNetwork::TensorXf latent_value = shared_output[1];

    std::vector<NeuralNetwork::TensorXf> value_input(getValueModel()->getInputs().size());
    value_input[0] = latent_value;

    std::vector<NeuralNetwork::TensorXf> value_output =
        applyModel(getValueModel(), value_input);

    NeuralNetwork::TensorXf value_estimate = value_output[0];
    setCurrentValue(value_estimate(0));
    
    std::vector<NeuralNetwork::TensorXf> action_input(getActionModel()->getInputs().size());
    action_input[0] = latent_action;
    

    std::vector<NeuralNetwork::TensorXf> action_output =
        applyModel(getActionModel(), action_input);
    return action_output;
    }
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

  for (unsigned int i = 0; i < action_length; i++) {

    action_eigen(i) = action_means[i];
    newActionMeans.push_back(action_means[i]);
  }
  action_mean_vector = newActionMeans;

  action_choice = stats::rmvnorm(action_eigen, covariance_matrix, true);
  current_log_prob = stats::dmvnorm(action_choice, action_eigen, covariance_matrix, true);

  std::vector<float> current_action = std::vector<float>();
  for (unsigned int i = 0; i < action_choice.size(); i++) {
    current_action.push_back(action_choice(i));
  }
  return current_action;
}
