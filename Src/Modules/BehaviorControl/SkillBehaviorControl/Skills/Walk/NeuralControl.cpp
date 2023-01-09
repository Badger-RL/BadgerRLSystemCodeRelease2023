/**
 * @file NeuralControl.cpp
 *
 * This file implements an implementation for the NeuralControl skill.
 *
 * @author Arne Hasselbring (the actual behavior is older)
 */



#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>
#include <CompiledNN/SimpleNN.h>
#include <CompiledNN/Tensor.h>

#include "Tools/RLConfig.h"
#include "Tools/RL/RLAlg.h"
#include "Tools/RL/RLData.h"
#include "Tools/RL/RLEnv.h"

#include "Tools/json.h"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FootBumperState.h"
#include <cmath>
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

#include <stdio.h>
#include <iostream>


#define PI 3.14159265

int observation_size = 12;


std::mutex cognitionLock;


#ifndef BUILD_NAO_FLAG
std::string shared_policy_path = "../shared_policy.h5";
std::string action_policy_path = "../action_policy.h5";
std::string value_policy_path = "../value_policy.h5";
std::string metadata_path = "../metadata.json";
#endif

#ifdef BUILD_NAO_FLAG
std::string shared_policy_path = "./shared_policy.h5";
std::string action_policy_path = "./action_policy.h5";
std::string value_policy_path = "./value_policy.h5";
std::string metadata_path = "./Config/metadata.json";
#endif


std::ifstream metadataFile(metadata_path);

json::value metadata = json::parse(metadataFile);


DataTransfer data_transfer(true);

FieldPositions field_positions(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
Environment environment(metadata, field_positions, observation_size);


Algorithm algorithm(3, shared_policy_path, action_policy_path, value_policy_path);



SKILL_IMPLEMENTATION(NeuralControlImpl,
{,
  IMPLEMENTS(NeuralControl),
  REQUIRES(ArmContactModel),
  REQUIRES(FootBumperState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(LibWalk),
  REQUIRES(MotionInfo),
  REQUIRES(PathPlanner),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(WalkingEngineOutput),
  MODIFIES(BehaviorStatus),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(PublishMotion),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToPose),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
    (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
    (float)(175.f) targetForwardWalkingSpeed, /**< Reduce walking speed to reach this forward speed (in mm/s). */
  }),
});



std::vector<float> getObservation(RobotPose theRobotPose, FieldBall theFieldBall) {

  const float goal_x = -4500.0;
  const float goal_y = 0;

  float x = theRobotPose.translation.x();
  float y = theRobotPose.translation.y();
  float angle = theRobotPose.rotation;
  
  if (angle < 0) {
    angle += 2 * PI;
  }

  float target_x = theFieldBall.positionOnField.x();
  float target_y = theFieldBall.positionOnField.y();
  
  float relative_angle;
  float delta_x = target_x - x;
  float delta_y = target_y - y;
  float theta_radians = atan2(delta_y, delta_x);

  if (theta_radians >= 0) {
    relative_angle = theta_radians;
  } else {
    relative_angle = theta_radians + (2*PI);
  }
  
  float goal_delta_x = goal_x - x;
  float goal_delta_y = goal_y - y;
  float goal_theta_radians = atan2(goal_delta_y, goal_delta_x);
  float goal_relative_angle;

  if (goal_theta_radians >= 0) {
    goal_relative_angle = goal_theta_radians;
  } else {
    goal_relative_angle = goal_theta_radians + (2*PI);
  }

  if (RLConfig::debug_print == true) {
    std::cout << "position" << std::endl;
    std::cout << x << std::endl;
    std::cout << y << std::endl;
    std::cout << "target position" << std::endl;
    std::cout << target_x << std::endl;
    std::cout << target_y << std::endl;
    std::cout << "ball relative angle" << std::endl;
    std::cout << relative_angle << std::endl;
    std::cout << "goal relative angle" << std::endl;
    std::cout << goal_relative_angle << std::endl;
    std::cout << "robot angle" << std::endl;
    std::cout << angle << std::endl;
  }

  std::vector<float> observation_vector(12);
  
  observation_vector[0] = (0 - x) / 9000.0; // dummy defender positions
  observation_vector[1] = (0 - y) / 6000.0;
  observation_vector[2] = (0 - x) / 9000.0;
  observation_vector[3] = (0 - y) / 6000.0;
  observation_vector[4] = (target_x - x) / 9000.0; //ball position
  observation_vector[5] = (target_y - y) / 6000.0;
  observation_vector[6] = (goal_x - target_x) / 9000.0; //goal position
  observation_vector[7] = (goal_y - target_y) / 6000.0;
  observation_vector[8] = sin(relative_angle - angle);
  observation_vector[9] = cos(relative_angle - angle);
  observation_vector[10] = sin(goal_relative_angle - angle);
  observation_vector[11] = cos(goal_relative_angle - angle);
  return observation_vector;
}


class NeuralControlImpl : public NeuralControlImplBase
{
  option(NeuralControl)
  {

     cognitionLock.lock();
     if (data_transfer.getCollectNewPolicy()) {
              data_transfer.newTrajectoriesJSON();
              data_transfer.waitForNewPolicy();

              algorithm.deleteModels();
              algorithm.updateModels(data_transfer);

              if (RLConfig::train_mode) {
                algorithm.deletePolicyFiles(data_transfer);
              }

              data_transfer.setCollectNewPolicy(false);
    }

        
    const std::vector<NeuralNetwork::TensorLocation> &shared_input = algorithm.getSharedModel()->getInputs();
    std::vector<NeuralNetwork::TensorXf> observation_input(algorithm.getSharedModel()->getInputs().size());
    for (std::size_t i = 0; i < observation_input.size(); ++i) {
      observation_input[i].reshape(
                                    shared_input[i].layer->nodes[shared_input[i].nodeIndex].outputDimensions[shared_input[i].tensorIndex]);
    }
    
    std::vector<float> current_observation = environment.getObservation(theRobotPose, theFieldBall);
    if (RLConfig::normalization) {
      current_observation = environment.normalizeObservation();
    }

    for (int i = 0; i < observation_size; i++) {
      observation_input[0][i] = current_observation[i];
    }
    

    std::cout << "reached 1" << std::endl;

    std::vector<NeuralNetwork::TensorXf> shared_output =
        algorithm.applyModel(algorithm.getSharedModel(), observation_input);
    
    NeuralNetwork::TensorXf latent_action = shared_output[0];
    NeuralNetwork::TensorXf latent_value = shared_output[1];
                std::cout << "reached 2" << std::endl;

    std::vector<NeuralNetwork::TensorXf> value_input(algorithm.getValueModel()->getInputs().size());
    value_input[0] = latent_value;
                std::cout << "reached 3" << std::endl;

    std::vector<NeuralNetwork::TensorXf> value_output =
        algorithm.applyModel(algorithm.getValueModel(), value_input);
                            std::cout << "reached 4" << std::endl;

    NeuralNetwork::TensorXf value_estimate = value_output[0];
    algorithm.setCurrentValue(value_estimate(0));
    
    std::vector<NeuralNetwork::TensorXf> action_input(algorithm.getActionModel()->getInputs().size());
    action_input[0] = latent_action;
    
                            std::cout << "reached 5" << std::endl;

    std::vector<NeuralNetwork::TensorXf> action_output =
        algorithm.applyModel(algorithm.getActionModel(), action_input);

                std::cout << "reached 6" << std::endl;
                    std::cout << environment.getActionLength()<< std::endl;

    std::vector<float> tempCurrentAction = std::vector<float>(algorithm.computeCurrentAction(action_output, environment.getActionLength()));
                    std::cout << "reached 7" << std::endl;


    for (auto i: tempCurrentAction)
      std::cout << i << ' ';

    
    theLookForwardSkill();
    if (theFieldBall.timeSinceBallWasSeen > 2000)
    {
      theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                                        0.0f,
                                        0.0f}});
    }
    else{
      theWalkAtRelativeSpeedSkill({.speed = {(float)(algorithm.getActionMeans()[0]) * 0.7f, (float)(algorithm.getActionMeans()[1]) > 1.0f ? 1.0f : (float)(algorithm.getActionMeans()[1]), (float)(algorithm.getActionMeans()[2])}});
    }
    cognitionLock.unlock();

  }
};

MAKE_SKILL_IMPLEMENTATION(NeuralControlImpl);
