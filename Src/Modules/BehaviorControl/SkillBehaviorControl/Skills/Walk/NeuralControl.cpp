/**
 * @file NeuralControl.cpp
 *
 * This file implements an implementation for the NeuralControl skill.
 *
 * @author Arne Hasselbring (the actual behavior is older), John Balis, Chen Li
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
#include "Representations/Communication/TeamData.h"
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

#include "Representations/Configuration/FieldDimensions.h"
#include "Platform/Time.h"
#include <algorithm>

#define PI 3.14159265

int observation_size = 12;
int action_size = 3;

std::vector<float> obstacleXVector;
std::vector<float> obstacleYVector;

//we keep track of timesteps separately for each robot, using this json object
json::object timeData = json::object{};

//we keep the previousObservation seperately for each robot, using this json object
json::object prevObservationData = json::object{};

json::object preRole = json::object{}; // This is used to record the role assigned by role assignment method.

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
Algorithm defenderAlgorithm(policy_path, "DefenderPolicy");
Algorithm CQLAttackerAlgorithm(policy_path, "CQLAttackerPolicy");

Algorithm * algorithm = NULL; // This is the current Algorithm.
const int DECISION_INTERVAL = 15; // decision interval in seconds
const int DECISION_TIME = 15; // the time used for decision making in time steps

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
  REQUIRES(TeamData),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(StrategyStatus),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(FieldDimensions),
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

// This function serve to get the minimum distance between a point and a line segment. Inspired by https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment.
double getMinimumDis(double endPointAX, double endPointAY, double endPointBX, double endPointBY, double pointX, double pointY) {
    const double LINE = pow(sqrt(pow((endPointBX - endPointAX), 2) + pow((endPointBY - endPointAY), 2)), 2); // the representation of the line segment
    
    // if the distance of the line is 0, which means it is a point, then return the distance between two points
    if(LINE == 0.0) {
        return sqrt(pow((pointX - endPointAX), 2) + pow((pointY - endPointAY), 2));
    }
    
    const double PVX = pointX - endPointAX;
    const double PVY = pointY - endPointAY;
    const double WVX = endPointBX - endPointAX;
    const double WVY = endPointBY - endPointAY;
    const double T = std::max(0.0, std::min(1.0, (PVX * WVX + PVY * WVY) / LINE));
    const double PX = endPointAX + T * (endPointBX - endPointAX);
    const double PY = endPointAY + T * (endPointBY - endPointAY);
    return sqrt(pow((PX - pointX), 2) + pow((PY - pointY), 2));
}

// Get the dynamic assigned Role of this Robot.
float getOwnDistance(RobotPose theRobotPose, FieldBall theFieldBall, FieldDimensions theField)
{
    float ownDistance = getMinimumDis(theField.xPosOwnGoal, theField.yPosLeftGoal, theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y(), theRobotPose.translation.x(), theRobotPose.translation.y()) - getMinimumDis(theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y(), theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y(), theRobotPose.translation.x(), theRobotPose.translation.y()); // the distance between the robot to the line segment connect the ball and our goal - the distance between the robot to the ball.
    
    return ownDistance;
}


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
     
    
      int role; // the role our robot takes
      int count = 0; // the count of other robots have a greater measure
      theBehaviorStatus.distance = getOwnDistance(theRobotPose, theFieldBall, theFieldDimensions);
      float ownDistance = theBehaviorStatus.distance; // our own distance
      
      // Iterate through all teammates and find out their distance measure
      for(auto & teammate : theTeamData.teammates) {
          // Exclude the goal keeper and ourselves
          if(teammate.number != theGameState.playerNumber && teammate.number != 1) {
              float distance = teammate.theBehaviorStatus.distance; // the distance of one teammate
              
              // if the teammate has a greater distance, update the count
              if(ownDistance < distance) {
                  count++;
              }
          }
      }
      
      // assign role based on num of robots closer
      if(count >= 2) {
          role = 3;
      } else {
          role = 2;
      }
      
      
      
      // Let there be 3 second interval in between changes of roles
      if(algorithm == NULL || Time::getCurrentSystemTime() % (DECISION_INTERVAL * 1000) < DECISION_TIME) {
          // Make sure Goal keeper keeps its role and assign new roles
          if(theGameState.playerNumber == 1) {
              std::cout << "Goalkeeper: Robot - " << theGameState.playerNumber << std::endl;
              algorithm = & goalKeeperAlgorithm;
              
              // store previous role separately for each robot.
              if(!(json::has_key(preRole, std::to_string(theGameState.playerNumber)))) {
                  preRole.insert(std::to_string(theGameState.playerNumber), 1);
              } else {
                  preRole[std::to_string(theGameState.playerNumber)] = 1;
              }
          } else if(role == 2) {
              std::cout << "Attacker: Robot - " << theGameState.playerNumber << std::endl;
              algorithm = & attackerAlgorithm;

              // store previous role separately for each robot.
              if(!(json::has_key(preRole, std::to_string(theGameState.playerNumber)))) {
                  preRole.insert(std::to_string(theGameState.playerNumber), 2);
              } else {
                  preRole[std::to_string(theGameState.playerNumber)] = 2;
              }
          } else {
              std::cout << "Defender: Robot - " << theGameState.playerNumber << std::endl;
              algorithm = & defenderAlgorithm;
              
              // store previous role separately for each robot.
              if(!(json::has_key(preRole, std::to_string(theGameState.playerNumber)))) {
                  preRole.insert(std::to_string(theGameState.playerNumber), 3);
              } else {
                  preRole[std::to_string(theGameState.playerNumber)] = 3;
              }
          }
      }
      
      // Make sure everything is updated every time step, default is attacker
      if(json::has_key(preRole, std::to_string(theGameState.playerNumber))) {
          // Assign role based on previous roles
          if(preRole[std::to_string(theGameState.playerNumber)] == 1) {
              algorithm = & goalKeeperAlgorithm;
          } else if(preRole[std::to_string(theGameState.playerNumber)] == 2) {
              algorithm = & attackerAlgorithm;
          } else if(preRole[std::to_string(theGameState.playerNumber)] == 3) {
              algorithm = & defenderAlgorithm;
          }
      } else {
          algorithm = & attackerAlgorithm;
      }
      
      /*
      
      if(algorithm == & goalKeeperAlgorithm) {
          theBehaviorStatus.roles = 1;
      } else if(algorithm == & defenderAlgorithm) {
          theBehaviorStatus.roles = 3;
          defenderCount++;
      } else {
          theBehaviorStatus.roles = 2;
          attackerCount++;
      }
      
      for(auto & teammate : theTeamData.teammates) {
          if(teammate.number != theGameState.playerNumber) {
              if(teammate.theBehaviorStatus.roles == 3) {
                  defenderCount++;
              } else if(teammate.theBehaviorStatus.roles == 2) {
                  attackerCount++;
              }
          }
      }
      if(attackerCount >= 3 && algorithm == & attackerAlgorithm) {
          algorithm = & defenderAlgorithm;
          theBehaviorStatus.roles = 3;
          
          if(!(json::has_key(preRole, std::to_string(theGameState.playerNumber)))) {
              preRole.insert(std::to_string(theGameState.playerNumber), 3);
          } else {
              preRole[std::to_string(theGameState.playerNumber)] = 3;
          }
      } else if ((defenderCount >= 3 && algorithm == & defenderAlgorithm) || attackerCount == 0) {
          algorithm = & attackerAlgorithm;
          theBehaviorStatus.roles = 2;
          
          if(!(json::has_key(preRole, std::to_string(theGameState.playerNumber)))) {
              preRole.insert(std::to_string(theGameState.playerNumber), 2);
          } else {
              preRole[std::to_string(theGameState.playerNumber)] = 2;
          }
      } */
      
      

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
        theWalkToBallAndKickSkill({
        .targetDirection = 0_deg,
        .kickType = KickInfo::walkForwardsRightLong,
        .kickLength = 1000.f,
      });
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
