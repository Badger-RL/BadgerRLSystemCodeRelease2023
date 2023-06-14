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


#define PI 3.14159265

int observation_size = 12;
int action_size = 3;


std::vector<float> obstacleXVector;
std::vector<float> obstacleYVector;



std::vector<float> obstacleXVector1;
std::vector<float> obstacleYVector1;

std::vector<float> obstacleXVector2;
std::vector<float> obstacleYVector2;

std::vector<float> obstacleXVector3;
std::vector<float> obstacleYVector3;

std::vector<float> obstacleXVector4;
std::vector<float> obstacleYVector4;

std::vector<float> obstacleXVector5;
std::vector<float> obstacleYVector5;

//we keep track of timesteps separately for each robot, using this json object
json::object timeData = json::object{};

//we keep the previousObservation seperately for each robot, using this json object
json::object prevObservationData = json::object{};
std::vector<int> robotNum;

bool isSimRobot = true;
bool robotPreCollision = false;


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
    REQUIRES(TeamData),
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
public:
    virtual bool onSegment(Vector2f p, Vector2f q, Vector2f r);
    virtual int orientation(Vector2f p, Vector2f q, Vector2f r);
    virtual bool doIntersect(Vector2f p1, Vector2f q1, Vector2f p2, Vector2f q2);
    virtual bool preCollision(std::vector<float> &obstacleXVector, std::vector<float> &obstacleYVector, float predictedPosX, float predictedPosY, bool obstacle[]);
    virtual void addObstaclesSimRobot(std::vector<float> &obstacleXVector, std::vector<float> &obstacleYVector);
    virtual std::pair<int, int> startIndexOfLongestConsecutive0s(const bool data[], int length);
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
            algorithm = & defenderAlgorithm;
            
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
        //    float minTeammateDistance =  std::numeric_limits<float>::max();
        float minTeammateDistance =  300.f;
        
        double angleToClosestObstacle = PI;
        double angleToClosesTeammate = PI;
        
        
        std::vector<float> predictedPosition = environment.getPredictedPosition(theRobotPose, algorithm->getActionMeanVector());
        
        
        //std::cout << "predicted position" << std::endl;
        //for (float i :predictedPosition )
        //{
        //  std::cout << i << std::endl;
        //}
        
        
        bool shield = false;
        bool moveRight = false;
        bool moveLeft = false;
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
            std::cout << "Angle of Robot: " << theRobotPose.rotation << std::endl;
            if(isSimRobot){
                switch(theGameState.playerNumber){
                    case 1:
                        addObstaclesSimRobot(obstacleXVector1, obstacleYVector1);
                        robotPreCollision = preCollision(obstacleXVector1, obstacleYVector1, predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                    case 2:
                        addObstaclesSimRobot(obstacleXVector2, obstacleYVector2);
                        robotPreCollision = preCollision(obstacleXVector2, obstacleYVector2, predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                    case 3:
                        addObstaclesSimRobot(obstacleXVector3, obstacleYVector3);
                        robotPreCollision = preCollision(obstacleXVector3, obstacleYVector3, predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                    case 4:
                        addObstaclesSimRobot(obstacleXVector4, obstacleYVector4);
                        for(unsigned int i = 0; i < obstacleXVector4.size(); i++){
                            std::cout << "Robot 4 Perceived Obstacle Position: " << obstacleXVector4[i] << ", " <<obstacleYVector4[i] << std::endl;
                        }
                        robotPreCollision = preCollision(obstacleXVector4, obstacleYVector4, predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                    case 5:
                        addObstaclesSimRobot(obstacleXVector5, obstacleYVector5);
                        robotPreCollision = preCollision(obstacleXVector5, obstacleYVector5, predictedPosition[0], predictedPosition[1], obstacles);
                        break;
                }
                for(int i = 0; i < 8; i++){
                    std::cout << "obstacle region " << i+1 <<  ": " << obstacles[i] << std::endl;
                }


            }
            else{
                addObstaclesSimRobot(obstacleXVector, obstacleYVector);
                robotPreCollision = preCollision(obstacleXVector, obstacleYVector, predictedPosition[0], predictedPosition[1], obstacles);
                std::cout << obstacleXVector.size() << std::endl;
            }
            
            
            
            
//            for (auto & teammate : theTeamData.teammates)
//            {
//                Vector2f stl(teammate.theRobotPose.translation.x() + 200.f, teammate.theRobotPose.translation.y() + 200.f);
//                Vector2f sbl(teammate.theRobotPose.translation.x() - 200.f, teammate.theRobotPose.translation.y() + 200.f);
//                Vector2f str(teammate.theRobotPose.translation.x() + 200.f, teammate.theRobotPose.translation.y() - 200.f);
//                Vector2f sbr(teammate.theRobotPose.translation.x() - 200.f, teammate.theRobotPose.translation.y() - 200.f);
//
//                Vector2f PredictedPoseVector(predictedPosition[0], predictedPosition[1]);
//
//                float distBetweenPredictedVectorAndPos = (theRobotPose.translation - PredictedPoseVector).norm();
//                Vector2f unitPredictedVector(PredictedPoseVector.x()/distBetweenPredictedVectorAndPos, PredictedPoseVector.y()/distBetweenPredictedVectorAndPos);
//                Vector2f resizedPredictedPoseVector(PredictedPoseVector.x()/distBetweenPredictedVectorAndPos * 300, PredictedPoseVector.y()/distBetweenPredictedVectorAndPos * 300);
//                std::cout << "distBetweenPredictedVectorAndPos: " << distBetweenPredictedVectorAndPos << std::endl;
//                std::cout << "resizedPredictedPoseVector: " << resizedPredictedPoseVector << std::endl;
//                std::cout << "unitPredictedVector: " << unitPredictedVector << std::endl;

//                std::cout << "teammate number: " << teammate.number << std::endl;
//                std::cout << "teammate Position x: " << teammate.theRobotPose.translation.x() << std::endl;
//                std::cout << "teammate Position y: " << teammate.theRobotPose.translation.y() << std::endl;
//                std::cout << "Distance betwen Robot and its teammates: " << (teammate.theRobotPose.translation - theRobotPose.translation).norm() << std::endl;

//                bool intersect = doIntersect(theRobotPose.translation, PredictedPoseVector, sbl, stl) || doIntersect(theRobotPose.translation, PredictedPoseVector, sbl, sbr) || doIntersect(theRobotPose.translation, PredictedPoseVector, sbr, str) || doIntersect(theRobotPose.translation, PredictedPoseVector, stl, str);
//
//                if((teammate.theRobotPose.translation - theRobotPose.translation).norm() < minTeammateDistance && intersect){
//                    std::cout << "Teammate Position that is in the way: " << teammate.theRobotPose.translation << std::endl;
//                    robotPreCollision = true;
//                }
//            }


            
            
            
        }
        
        
        if (theFieldBall.timeSinceBallWasSeen > 4000)
        {
            theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                0.0f,
                0.0f}});
            std::cout << "Looking for ball" << std::endl;
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
            std::cout << "Angle to go: " << angle * 180/PI << std::endl;
            float x = 1000.f * cos(angle);
            float y = 1000.f * sin(angle);
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
bool NeuralControlImpl::preCollision(std::vector<float>& ObstacleX, std::vector<float>& ObstacleY, float predictedPosX, float predictedPosY, bool obstacles[8]){
    bool intersect = false;
    bool withInRobotSquare = false;
    for(unsigned int i = 0; i < ObstacleX.size(); i ++){
          Vector2f stl(ObstacleX[i] + 250.f, ObstacleY[i] + 250.f);
          Vector2f sbl(ObstacleX[i] - 250.f, ObstacleY[i] + 250.f);
          Vector2f str(ObstacleX[i] + 250.f, ObstacleY[i] - 250.f);
          Vector2f sbr(ObstacleX[i] - 250.f, ObstacleY[i] - 250.f);
        Vector2f PredictedPoseVector(predictedPosX, predictedPosY);
        Vector2f obstaclePose(ObstacleX[i], ObstacleY[i]);

        std::cout << (theRobotPose.translation - PredictedPoseVector).norm() << std::endl;
        std::cout << "Obstacle Position in global coordinates: " << ObstacleX[i] << ", " << ObstacleY[i] << std::endl;
        bool intersect = (doIntersect(theRobotPose.translation, PredictedPoseVector, sbl, stl) || doIntersect(theRobotPose.translation, PredictedPoseVector, sbl, sbr) || doIntersect(theRobotPose.translation, PredictedPoseVector, sbr, str) || doIntersect(theRobotPose.translation, PredictedPoseVector, stl, str));
        double angleRelativeToRobot = atan2( ObstacleY[i] - theRobotPose.translation.y(), ObstacleX[i] - theRobotPose.translation.x());
        if(angleRelativeToRobot<0){
            angleRelativeToRobot += 2*PI;
        }
        std::cout << "Angle in degree: " << angleRelativeToRobot * 180 / PI << std::endl;
        if((theRobotPose.translation - obstaclePose).norm() < 500.f){
            std::cout << "Obstacle within distance: " << ObstacleX[i] << ", " << ObstacleY[i] << std::endl;
            obstacles[(int)(angleRelativeToRobot / (PI/4))] = true;
        }
           
        
        
        withInRobotSquare = theRobotPose.translation.x() <= stl.x() && theRobotPose.translation.x() >= sbl.x() && theRobotPose.translation.y() <= stl.y() && theRobotPose.translation.y() >= str.y();
        if(intersect || withInRobotSquare ){
            break;
        }
        while (ObstacleX.size() > 15)
        {
            ObstacleX.erase(ObstacleX.begin());
            ObstacleY.erase(ObstacleY.begin());
            assert(ObstacleX.size() == ObstacleY.size());
        }
    }
    return intersect || withInRobotSquare;
}
void NeuralControlImpl::addObstaclesSimRobot(std::vector<float>& ObstacleX, std::vector<float>& ObstacleY){
    for (auto & obstacle : theObstacleModel.obstacles)
    {
        if(!obstacle.isTeammate()){
            ObstacleX.push_back(obstacle.center.x() + theRobotPose.translation.x());
            ObstacleY.push_back(obstacle.center.y() + theRobotPose.translation.y());
        }
        
    }
    for(auto& teammate: theTeamData.teammates){
        std::cout << "teammate number: " << teammate.number << std::endl;
        std::cout << "teammate Position: " << teammate.theRobotPose.translation.x() << ", "<<teammate.theRobotPose.translation.y() << std::endl;
        ObstacleX.push_back(teammate.theRobotPose.translation.x());
        ObstacleY.push_back(teammate.theRobotPose.translation.y());
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
            std::cout << value << std::endl;
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







MAKE_SKILL_IMPLEMENTATION(NeuralControlImpl);
