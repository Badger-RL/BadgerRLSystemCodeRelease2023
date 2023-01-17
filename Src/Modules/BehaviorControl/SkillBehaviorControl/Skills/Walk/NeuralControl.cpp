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
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FootBumperState.h"
#include <cmath>
#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

#include <stdio.h>
#include <iostream>


#define PI 3.14159265

int observation_size = 12;
int action_size = 3;


std::mutex cognitionLock;


#ifndef BUILD_NAO_FLAG
std::string policy_path = "../Policies/";
#endif
#ifdef BUILD_NAO_FLAG
std::string policy_path = "./Policies/";
#endif





DataTransfer data_transfer(true);

FieldPositions field_positions(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
Environment environment(field_positions, observation_size, action_size);


Algorithm attackerAlgorithm(policy_path, "AttackerPolicy");
Algorithm goalKeeperAlgorithm(policy_path, "GoalKeeperPolicy");
Algorithm * algorithm;


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
  REQUIRES(StrategyStatus),
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




class NeuralControlImpl : public NeuralControlImplBase
{
  option(NeuralControl)
  {

     cognitionLock.lock();

    
     if (theStrategyStatus.role == PositionRole::toRole(PositionRole::goalkeeper))
     {
     algorithm = & goalKeeperAlgorithm;
     }
     else{
     algorithm = & goalKeeperAlgorithm;
    }

     if (algorithm->getCollectNewPolicy()) {
              data_transfer.newTrajectoriesJSON();
              algorithm->waitForNewPolicy();

              algorithm->deleteModels();
              algorithm->updateModels();

              if (RLConfig::train_mode) {
                algorithm->deletePolicyFiles();
              }

              algorithm->setCollectNewPolicy(false);
    }

        
    const std::vector<NeuralNetwork::TensorLocation> &shared_input = algorithm->getSharedModel()->getInputs();
    std::vector<NeuralNetwork::TensorXf> observation_input(algorithm->getSharedModel()->getInputs().size());
    for (std::size_t i = 0; i < observation_input.size(); ++i) {
      observation_input[i].reshape(
                                    algorithm->getSharedModel()->getInputs()[i].layer->nodes[algorithm->getSharedModel()->getInputs()[i].nodeIndex].outputDimensions[algorithm->getSharedModel()->getInputs()[i].tensorIndex]);
    }
    
    std::vector<float> current_observation = environment.getObservation(theRobotPose, theFieldBall);
    if (RLConfig::normalization) {
      current_observation = algorithm->normalizeObservation(current_observation);
    }

    for (int i = 0; i < observation_size; i++) {
      observation_input[0][i] = current_observation[i];
    }
    


    std::vector<NeuralNetwork::TensorXf> shared_output =
        algorithm->applyModel(algorithm->getSharedModel(), observation_input);
    
    NeuralNetwork::TensorXf latent_action = shared_output[0];
    NeuralNetwork::TensorXf latent_value = shared_output[1];

    std::vector<NeuralNetwork::TensorXf> value_input(algorithm->getValueModel()->getInputs().size());
    value_input[0] = latent_value;

    std::vector<NeuralNetwork::TensorXf> value_output =
        algorithm->applyModel(algorithm->getValueModel(), value_input);

    NeuralNetwork::TensorXf value_estimate = value_output[0];
    algorithm->setCurrentValue(value_estimate(0));
    
    std::vector<NeuralNetwork::TensorXf> action_input(algorithm->getActionModel()->getInputs().size());
    action_input[0] = latent_action;
    

    std::vector<NeuralNetwork::TensorXf> action_output =
        algorithm->applyModel(algorithm->getActionModel(), action_input);


    std::vector<float> tempCurrentAction = std::vector<float>(algorithm->computeCurrentAction(action_output, environment.getActionLength()));

    /*
    for (auto i: tempCurrentAction)
      std::cout << i << ' ';
    */
    
    theLookForwardSkill();
    if (theFieldBall.timeSinceBallWasSeen > 15000)
    {
      theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                                        0.0f,
                                        0.0f}});
    }
    else{
      theWalkAtRelativeSpeedSkill({.speed = {(float)(algorithm->getActionMeans()[0]) * 0.4f, (float)(algorithm->getActionMeans()[1]) > 1.0f ? 1.0f : (float)(algorithm->getActionMeans()[1]), (float)(algorithm->getActionMeans()[2])}});
    }
    cognitionLock.unlock();

  }
};

MAKE_SKILL_IMPLEMENTATION(NeuralControlImpl);
