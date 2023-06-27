/**
 * @file NeuralControl.cpp
 *
 * This file implements an implementation for the NeuralControl skill.
 *
 * @author Arne Hasselbring (the actual behavior is older)
 * @author John Balis
 * @author Chen Li
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
#include "Representations/Communication/TeamData.h"

#include "Tools/Modeling/Obstacle.h"
#include "Debugging/DebugDrawings.h"
#include "Libs/Debugging/ColorRGBA.h"

#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"

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




//we keep track of timesteps separately for each robot, using this json object
json::object timeData = json::object{};

//we keep the previousObservation seperately for each robot, using this json object
json::object prevObservationData = json::object{};
std::vector<int> robotNum;

bool isSimRobot = false;
bool robotPreCollision = false;

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
const int DECISION_INTERVAL = 2; // decision interval in seconds
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
=======
                     
                     
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
>>>>>>> 92d4ff955b38180a156499ef36f9af03088b9500
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

struct ObstacleVector{
    float x;
    float y;
    bool isteammate;
};

std::vector<ObstacleVector> physicalRobot;
std::vector<ObstacleVector> simRobot1;
std::vector<ObstacleVector> simRobot2;
std::vector<ObstacleVector> simRobot3;
std::vector<ObstacleVector> simRobot4;
std::vector<ObstacleVector> simRobot5;

class NeuralControlImpl : public NeuralControlImplBase
{
public:
    virtual bool onSegment(Vector2f p, Vector2f q, Vector2f r);
    virtual int orientation(Vector2f p, Vector2f q, Vector2f r);
    virtual bool doIntersect(Vector2f p1, Vector2f q1, Vector2f p2, Vector2f q2);
    virtual bool preCollision(std::vector<ObstacleVector>& Obstacle, float predictedPosX, float predictedPosY, bool obstacle[]);
    virtual void addObstaclesSimRobot(std::vector<ObstacleVector>& Obstacle);
    virtual std::pair<int, int> startIndexOfLongestConsecutive0s(const bool data[], int length);
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
          
          int attackerCount = 0;
          int defenderCount = 0;
          
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
          std::cout << "robot" << theGameState.playerNumber << "is a goal keeper" << std::endl;
      } else if(algorithm == & defenderAlgorithm) {
          std::cout << "robot" << theGameState.playerNumber << "is a defender" << std::endl;
      } else {
          std::cout << "robot" << theGameState.playerNumber << "is a attacker" << std::endl;
      }
      */
       /*
      
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
=======
    virtual Vector2f rotate_point(float cx,float cy,float angle, Vector2f p);
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
         std::cout << "robot" << theGameState.playerNumber << "is a goal keeper" << std::endl;
         } else if(algorithm == & defenderAlgorithm) {
         std::cout << "robot" << theGameState.playerNumber << "is a defender" << std::endl;
         } else {
         std::cout << "robot" << theGameState.playerNumber << "is a attacker" << std::endl;
         }
         */
        /*
         
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
>>>>>>> 92d4ff955b38180a156499ef36f9af03088b9500
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
        
        //float minObstacleDistance = std::numeric_limits<float>::max();
        //float minTeammateDistance =  std::numeric_limits<float>::max();
        //float minTeammateDistance =  300.f;
        
        //double angleToClosestObstacle = PI;
        //double angleToClosesTeammate = PI;
        
        
        std::vector<float> predictedPosition = environment.getPredictedPosition(theRobotPose, algorithm->getActionMeanVector());
        
        
        //std::cout << "predicted position" << std::endl;
        //for (float i :predictedPosition )
        //{
        //  std::cout << i << std::endl;
        //}
        
        
        bool shield = false;
        bool obstacles[8] = {false, false, false, false,false, false, false,false};
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
            
            
            std::cout << "Robot Number: " << theGameState.playerNumber << std::endl;
            std::cout << "Robot Position: " << theRobotPose.translation.x() << ", " << theRobotPose.translation.y() << std::endl;
            std::cout << "Predicted Position: " << predictedPosition[0] << ", " << predictedPosition[1] << std::endl;
            if(isSimRobot){
                switch(theGameState.playerNumber){
                    case 1:
                        addObstaclesSimRobot(simRobot1);
                        robotPreCollision = preCollision(simRobot1, predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                    case 2:
                        addObstaclesSimRobot(simRobot2);
                        robotPreCollision = preCollision(simRobot2, predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                    case 3:
                        addObstaclesSimRobot(simRobot3);
                        robotPreCollision = preCollision(simRobot3,  predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                    case 4:
                        addObstaclesSimRobot(simRobot4);
                        robotPreCollision = preCollision(simRobot4,  predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                    case 5:
                        addObstaclesSimRobot(simRobot5);
                        robotPreCollision = preCollision(simRobot5,  predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                }
                
                
            }
            else{
                addObstaclesSimRobot(physicalRobot);
                robotPreCollision = preCollision(physicalRobot, predictedPosition[0], predictedPosition[1], obstacles);
            }
            
            std::cout << "\n";
        }
        
        if (theGameState.isFreeKick() && robotPreCollision) {
            std::cout << "Collision Avoidance activated during free kick" << std::endl;
            std::pair<int, int> index = startIndexOfLongestConsecutive0s(obstacles, sizeof(obstacles)/sizeof(obstacles[0]));
            double angle = ((index.first + index.second)/2 + 1) * (PI/4) - PI/8;
            float x = 300.f * cos(angle);
            float y = 300.f * sin(angle);
            std::cout << "x: " << x << ", y: " << y << std::endl;
            theWalkAtRelativeSpeedSkill({.speed = {0.0f,x,y}});
            
        } else {
            
            if (theFieldBall.timeSinceBallWasSeen > 4000)
            {
                theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                    0.0f,
                    0.0f}});
                //std::cout << "Looking for ball" << std::endl;
            }
            else if(RLConfig::shieldEnabled && shield)
            {
                //std::cout << "HEURISTIC ACTIVATED" << std::endl;
                
                theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                    0.0f,
                    0.0f}});
                std::cout << "Shielding activated" << std::endl;
                
            }
            else if(robotPreCollision){
                std::cout << "Collision Avoidance activated" << std::endl;
                std::pair<int, int> index = startIndexOfLongestConsecutive0s(obstacles, sizeof(obstacles)/sizeof(obstacles[0]));
                double angle = ((index.first + index.second)/2 + 1) * (PI/4) - PI/8;
                float x = 300.f * cos(angle);
                float y = 300.f * sin(angle);
                std::cout << "x: " << x << ", y: " << y << std::endl;
                theWalkAtRelativeSpeedSkill({.speed = {0.0f,x,y}});
                
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


// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool NeuralControlImpl::onSegment(Vector2f p, Vector2f q, Vector2f r)
{
    if (q.x() <= fmax(p.x(), r.x()) && q.x() >= fmin(p.x(), r.x()) &&
        q.y() <= fmax(p.y(), r.y()) && q.y() >= fmin(p.y(), r.y()))
        return true;
    
    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int NeuralControlImpl::orientation(Vector2f p, Vector2f q, Vector2f r)
{
    // for details of below formula.
    float val = (q.y() - p.y()) * (r.x() - q.x()) -
    (q.x() - p.x()) * (r.y() - q.y());
    
    if (val == 0) return 0;  // collinear
    
    return (val > 0)? 1: 2; // clock or counterclock wise
}


// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool NeuralControlImpl::doIntersect(Vector2f p1, Vector2f q1, Vector2f p2, Vector2f q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
    
    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    
    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    
    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    
    return false; // Doesn't fall in any of the above cases
}

bool NeuralControlImpl::preCollision(std::vector<ObstacleVector>& Obstacle, float predictedPosX, float predictedPosY, bool obstacles[8]){
    bool intersect = false;
    bool intersect2 = false;
    bool intersect3 = false;
    bool withInRobotSquare = false;
    for(unsigned int i = 0; i < Obstacle.size(); i ++){
        Vector2f stl(Obstacle[i].x + 250.f, Obstacle[i].y + 250.f);
        Vector2f sbl(Obstacle[i].x - 250.f, Obstacle[i].y + 250.f);
        Vector2f str(Obstacle[i].x + 250.f, Obstacle[i].y - 250.f);
        Vector2f sbr(Obstacle[i].x - 250.f, Obstacle[i].y - 250.f);
        Vector2f PredictedPoseVector(predictedPosX, predictedPosY);
        Vector2f obstaclePose(Obstacle[i].x, Obstacle[i].y);
        
        double dist = (theRobotPose.translation - PredictedPoseVector).norm();
        Vector2f unitVector = Vector2f((PredictedPoseVector - theRobotPose.translation).x()/dist,(PredictedPoseVector - theRobotPose.translation).y()/dist);
        Vector2f newVector = Vector2f(theRobotPose.translation.x() + unitVector.x()*100.f, theRobotPose.translation.y() + unitVector.y()*100.f);
        Vector2f rotateCounterClockwise = rotate_point(theRobotPose.translation.x(), theRobotPose.translation.y(), 10, newVector);
        Vector2f rotateClockwise = rotate_point(theRobotPose.translation.x(), theRobotPose.translation.y(), -10, newVector);
        
        intersect = (doIntersect(theRobotPose.translation, newVector, sbl, stl) || doIntersect(theRobotPose.translation, newVector, sbl, sbr) || doIntersect(theRobotPose.translation, newVector, sbr, str) || doIntersect(theRobotPose.translation, newVector, stl, str));
        intersect2 = (doIntersect(theRobotPose.translation, rotateCounterClockwise, sbl, stl) || doIntersect(theRobotPose.translation, rotateCounterClockwise, sbl, sbr) || doIntersect(theRobotPose.translation, rotateCounterClockwise, sbr, str) || doIntersect(theRobotPose.translation, rotateCounterClockwise, stl, str));
        intersect3 = (doIntersect(theRobotPose.translation, rotateClockwise, sbl, stl) || doIntersect(theRobotPose.translation, rotateClockwise, sbl, sbr) || doIntersect(theRobotPose.translation, rotateClockwise, sbr, str) || doIntersect(theRobotPose.translation, rotateClockwise, stl, str));
        std::cout << "Distance between obstacle and Robot: " <<(theRobotPose.translation - obstaclePose).norm() << std::endl;
        
        double angleRelativeToRobot = atan2( Obstacle[i].y - theRobotPose.translation.y(), Obstacle[i].x - theRobotPose.translation.x());
        if(angleRelativeToRobot<0){
            angleRelativeToRobot += 2*PI;
        }
        if((theRobotPose.translation - obstaclePose).norm() < 500.f){
            obstacles[(int)(angleRelativeToRobot / (PI/4))] = true;
        }
        
        withInRobotSquare = theRobotPose.translation.x() <= stl.x() && theRobotPose.translation.x() >= sbl.x() && theRobotPose.translation.y() <= stl.y() && theRobotPose.translation.y() >= str.y();
        if(intersect || withInRobotSquare ){
            break;
        }
        while (Obstacle.size() > 15)
        {
            Obstacle.erase(Obstacle.begin());
        }
    }
    return intersect || intersect2 || intersect3 || withInRobotSquare;
}

void NeuralControlImpl::addObstaclesSimRobot(std::vector<ObstacleVector>& Obstacle){

    for (auto & obstacle : theObstacleModel.obstacles)
    {
        if(!obstacle.isTeammate()){
            ObstacleVector o{obstacle.center.x() + theRobotPose.translation.x(), obstacle.center.y() + theRobotPose.translation.y(), false};
            Obstacle.push_back(o);
        }
        
    }
    for(auto& teammate: theTeamData.teammates){
        std::cout << "teammate number: " << teammate.number << std::endl;
        std::cout << "teammate Position: " << teammate.theRobotPose.translation.x() << ", "<<teammate.theRobotPose.translation.y() << std::endl;
        ObstacleVector o{teammate.theRobotPose.translation.x(), teammate.theRobotPose.translation.y(), true};
        Obstacle.push_back(o);
    }
}


// Calculate longest sub array with all 1's index
std::pair<int, int> NeuralControlImpl::startIndexOfLongestConsecutive0s(const bool data[], int length) {
    int startIndex{}, counter{}, previousValue{},maxCounter{}, startIndexMax{}, endIndex{}, endIndexMax{};
    std::pair<int, int> result;
    // Go through all elements of the array
    for (int i{}; i < length; ++i) {
        
        // Get the current element. Special Handling for last element
        const bool value = data[i];
        // If we see a 1, then we are in a open sliding window
        if (value == false) {
            
            // If the window was just opened, then remember the start index
            if (previousValue == true) startIndex = i;
            if(data[i+1] == true || data[i] == false){ endIndex = i;}
            // Count the number of 1's in the open window. This will be the width at the end
            ++counter;
        }
        else {
            // If the last window size is bigger than whatever we had before
            // Now, the value is 0. So the window is closed.
            if ((previousValue == false) && (counter > maxCounter)) {
                // Store new max window size and new start index for max.                endIndex =
                maxCounter = counter;
                startIndexMax = startIndex;
                endIndexMax = endIndex;
            }
            // Optimization: If the maximum size of a found window is bigger
            // than the rest of the remaining value, we can stop all activities
            if (maxCounter >= length / 2) break;
            
            // Counter is 0 again
            counter = 0;
        }
        previousValue = value;
        
        
    }
    // Now, the value is 0. So the window is closed.
    if ((data[length-1] == false) && (counter > maxCounter)) {
        // Store new max window size and new start index for max.                endIndex =
        maxCounter = counter;
        startIndexMax = startIndex;
        endIndexMax = endIndex;
    }
    
    result.first = startIndexMax;
    result.second = endIndexMax;
    return result;
}

Vector2f NeuralControlImpl::rotate_point(float cx,float cy,float angleDegree, Vector2f p)
{
    float angle = angleDegree * PI / 180.0;
    float s = sin(angle);
    float c = cos(angle);
    
    float px = p.x();
    float py = p.y();
    // translate point back to origin:
    px -= cx;
    py -= cy;
    
    // rotate point
    float xnew = px * c - py * s;
    float ynew = px * s + py * c;
    
    // translate point back:
    px = xnew + cx;
    py = ynew + cy;
    return Vector2f(px, py);
}

MAKE_SKILL_IMPLEMENTATION(NeuralControlImpl);
