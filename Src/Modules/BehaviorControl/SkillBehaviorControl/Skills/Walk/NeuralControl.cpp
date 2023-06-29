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
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/ObstacleModel.h"
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


std::vector<float> obstacleXVector;
std::vector<float> obstacleYVector;

//we keep track of timesteps separately for each robot, using this json object
json::object timeData = json::object{};

//we keep the previousObservation seperately for each robot, using this json object
json::object prevObservationData = json::object{};





std::mutex cognitionLock;


#ifndef BUILD_NAO_FLAG
std::string policy_path = "../Policies/";
#endif
#ifdef BUILD_NAO_FLAG
std::string policy_path = "/home/nao/Config/Policies/";
#endif





DataTransfer data_transfer(true);

FieldPositions field_positions(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
Environment environment(field_positions, observation_size, action_size);

//std::vector<float> prevObservation;


Algorithm attackerAlgorithm(policy_path, "AttackerPolicy");
Algorithm goalKeeperAlgorithm(policy_path, "GoalKeeperPolicy");
Algorithm attackerKickAlgorithm(policy_path, "AttackerKickPolicy");
Algorithm defenderKickAlgorithm(policy_path, "DefenderKickPolicy");
Algorithm defenderAlgorithm(policy_path, "DefenderPolicy");
Algorithm CQLAttackerAlgorithm(policy_path, "CQLAttackerPolicy");

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
  REQUIRES(ObstacleModel),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(StrategyStatus),
  REQUIRES(WalkingEngineOutput),
  MODIFIES(BehaviorStatus),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(PublishMotion),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToPose),
  CALLS(WalkToBallAndKick),
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

     if (!(json::has_key(timeData,std::to_string(theGameState.playerNumber))))
     {
        timeData.insert(std::to_string(theGameState.playerNumber), 0);
     }

     int timestep = std::stoi(to_string(timeData[std::to_string(theGameState.playerNumber)]));
     
    
     if (theGameState.playerNumber == 1)
     {
     algorithm = & goalKeeperAlgorithm;
     }
     else if (theGameState.playerNumber == 2 || theGameState.playerNumber == 3)
     {
      algorithm = & defenderKickAlgorithm;

     }
     else{
      algorithm = & attackerAlgorithm;
     }

    
     if (algorithm->getCollectNewPolicy()) {
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
                                    shared_input[i].layer->nodes[shared_input[i].nodeIndex].outputDimensions[shared_input[i].tensorIndex]);
    }
    
    std::vector<float> rawObservation = environment.getObservation(theRobotPose, theFieldBall);

    std::vector<float> inputObservation;

    if (RLConfig::normalization) {
      inputObservation = algorithm->normalizeObservation(rawObservation);
    }
    else{
      inputObservation = rawObservation;
    }

    for (int i = 0; i < observation_size; i++) {
      observation_input[0][i] = inputObservation[i];
    }
    
    std::vector<NeuralNetwork::TensorXf> action_output = algorithm->inference(observation_input);

    std::vector<float> tempCurrentAction = std::vector<float>(algorithm->computeCurrentAction(action_output, algorithm->getActionLength()));
    
    if (RLConfig::logging)
    {
    if (timestep == 0){
        data_transfer.newTrajectoriesJSON(theGameState.playerNumber);
    }
    else  {
      data_transfer.appendTrajectoryValue("observations", data_transfer.JSON_to_vect(as_array(prevObservationData[std::to_string(theGameState.playerNumber)])),theGameState.playerNumber);
      data_transfer.appendTrajectoryValue("actions", algorithm->getActionMeanVector(),theGameState.playerNumber);
      data_transfer.appendTrajectoryValue("next_observations", rawObservation,theGameState.playerNumber);

      if ((timestep) % RLConfig::batch_size  == 0 ){
        data_transfer.saveTrajectories(timestep/RLConfig::batch_size,theGameState.playerNumber);
        data_transfer.newTrajectoriesJSON(theGameState.playerNumber);
      }

    }
    }

   
    theLookForwardSkill();

    float minObstacleDistance = std::numeric_limits<float>::max();
    float minTeammateDistance =  std::numeric_limits<float>::max();
    float angleToClosestObstacle = PI;
    float angleToClosesTeammate = PI; 


    std::vector<float> predictedPosition = environment.getPredictedPosition(theRobotPose, algorithm->getActionMeanVector());


    //std::cout << "predicted position" << std::endl;
    //for (float i :predictedPosition )
    //{
    //  std::cout << i << std::endl;
    //}

  
    bool shield = false;

    if (RLConfig::shieldEnabled)
    {

    if (predictedPosition[0] < -4600 || predictedPosition[0] > 4600 || predictedPosition[1] > 3100 || predictedPosition[1] < -3100){
      shield = true;
    }
    if (predictedPosition[0] > 4300 && predictedPosition[1] > 600 && predictedPosition[1] < 800)
    {
      shield = true;
    }
     if (predictedPosition[0] > 4300 && predictedPosition[1] < -600 && predictedPosition[1] > -800)
    {
      shield = true;
    }



    for (auto & obstacle : theObstacleModel.obstacles)
    {
      obstacleXVector.push_back(obstacle.center.x());
      obstacleYVector.push_back(obstacle.center.y());

      /*
      float distance = (obstacle.center - theRobotPose.translation).norm();
      if (distance < minObstacleDistance)
      {
        minObstacleDistance = distance;
        angleToClosestObstacle = std::abs(obstacle.center.angle());
      }
      */
    }
    for (auto & teammate : theGlobalTeammatesModel.teammates)
    {
      obstacleXVector.push_back(teammate.pose.translation.x());
      obstacleYVector.push_back(teammate.pose.translation.y());

      /*
      float distance = (teammate.pose.translation - theRobotPose.translation).norm();
      if (distance < minTeammateDistance)
      {
        minTeammateDistance = distance;
        float relativeAngle; 

        float x = theRobotPose.translation.x();
        float y = theRobotPose.translation.y();
        float angle = theRobotPose.rotation;
        
        if (angle < 0) {
          angle += 2 * PI;
        }

        float target_x = teammate.pose.translation.x();
        float target_y = teammate.pose.translation.y();
        
        float delta_x = target_x - x;
        float delta_y = target_y - y;
        float theta_radians = atan2(delta_y, delta_x);

        if (theta_radians >= 0) {
          relativeAngle = theta_radians;
        } else {
          relativeAngle = theta_radians + (2*PI);
        }
        angleToClosesTeammate = std::abs(relativeAngle - angle);

        if(angleToClosesTeammate > PI)
        {
          angleToClosesTeammate -= PI;
        }
      
      }
      */

    }
    
    while (obstacleXVector.size() > 15)
      {
        obstacleXVector.erase(obstacleXVector.begin());
        obstacleYVector.erase(obstacleYVector.begin());
        assert(obstacleXVector.size() == obstacleYVector.size());
       // std::cout << obstacleXVector.size() << std::endl;
      }


      
    }



    

    if (theFieldBall.timeSinceBallWasSeen > 4000)
    {
      theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                                        0.0f,
                                        0.0f}});
    }
    else if(RLConfig::shieldEnabled && shield)
    {
      //std::cout << "HEURISTIC ACTIVATED" << std::endl;

       theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                                        0.0f,
                                        0.0f}});
    }
    else{
  
      if (algorithm->getActionLength() == 3){
      theWalkAtRelativeSpeedSkill({.speed = {(float)(algorithm->getActionMeans()[0]) * 0.4f, (float)(algorithm->getActionMeans()[1]) > 1.0f ? 1.0f : (float)(algorithm->getActionMeans()[1]), (float)(algorithm->getActionMeans()[2])}});
      }
      else if(algorithm->getActionLength() == 4)
      {
        if (algorithm->getActionMeans()[3] > 0.0 && (theFieldBall.positionOnField - theRobotPose.translation).norm() < 200.0)
        {
        theWalkToBallAndKickSkill({
        .targetDirection = 0_deg,
        .kickType = KickInfo::walkForwardsRightLong,
        .kickLength = 1000.f,

        });
        }
        else{
          theWalkAtRelativeSpeedSkill({.speed = {(float)(algorithm->getActionMeans()[0]) * 0.4f, (float)(algorithm->getActionMeans()[1]) > 1.0f ? 1.0f : (float)(algorithm->getActionMeans()[1]), (float)(algorithm->getActionMeans()[2])}});

        }

      }
      else
      {
        std::cout << "unsupported action space" << std::endl;
      }
  
    }


  if (!(json::has_key(prevObservationData,std::to_string(theGameState.playerNumber)))){
    prevObservationData.insert(std::to_string(theGameState.playerNumber), data_transfer.vectToJSON(rawObservation));

  }
  else{
    prevObservationData[std::to_string(theGameState.playerNumber)] = data_transfer.vectToJSON(rawObservation);
  }
    

    timeData[std::to_string(theGameState.playerNumber)] = timestep + 1;
    

    cognitionLock.unlock();

  }
};

MAKE_SKILL_IMPLEMENTATION(NeuralControlImpl);
