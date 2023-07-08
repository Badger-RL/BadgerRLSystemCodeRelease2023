/**
 * @file NeuralControl.cpp
 *
 * This file implements an implementation for the NeuralControl skill.
 *
 * @author Arne Hasselbring (the actual behavior is older)
 * @author John Balis
 * @author Chen Li
 * @author Benjamin Hong
 */

#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>
#include <CompiledNN/SimpleNN.h>
#include <CompiledNN/Tensor.h>

#include "Tools/RLConfig.h"
#include "Tools/RL/RLAlg.h"
#include "Tools/RL/RLData.h"
#include "Tools/RL/RLEnv.h"
#include "Tools/RL/ObsTools.h"

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
#include "Representations/Modeling/TeammatesBallModel.h"
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

json::object returningFlags = json::object{};

bool isSimRobot = true;
bool robotPreCollision = false;
bool ballLost = false;
int standingTime = 0;
bool spinning = false;
int spinningTime = 0;

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
Algorithm goalKeeperKickAlgorithm(policy_path, "GoalKeeperKickPolicy");
Algorithm attackerKickAlgorithm(policy_path, "AttackerKickPolicy");
Algorithm defenderKickAlgorithm(policy_path, "DefenderKickPolicy");
Algorithm defenderAlgorithm(policy_path, "DefenderPolicy");
Algorithm CQLAttackerAlgorithm(policy_path, "CQLAttackerPolicy");

Algorithm * algorithm = NULL; // This is the current Algorithm.
const int DECISION_INTERVAL = 5; // decision interval in seconds
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
    CALLS(LookAtGlobalBall),
    CALLS(Stand),
    CALLS(PublishMotion),
    CALLS(WalkAtRelativeSpeed),
    CALLS(WalkToPose),
    CALLS(WalkToBallAndKick),
    REQUIRES(TeammatesBallModel),
    DEFINES_PARAMETERS(
                       {,
                           (float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
                           (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
                           (float)(175.f) targetForwardWalkingSpeed, /**< Reduce walking speed to reach this forward speed (in mm/s). */
                       }),
});

// This function serve to get the minimum distance between a point and a line segment. Inspired by https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment.
double getMinimumDis(double endPointAX, double endPointAY, double endPointBX, double endPointBY, double pointX, double pointY) {
    const double LINE = sqrt(pow((endPointBX - endPointAX), 2) + pow((endPointBY - endPointAY), 2)); // the representation of the line segment
    
    // if the distance of the line is 0, which means it is a point, then return the distance between two points
    if(LINE == 0.0) {
        return sqrt(pow((pointX - endPointAX), 2) + pow((pointY - endPointAY), 2));
    }
    
    const double PVX = pointX - endPointAX;
    const double PVY = pointY - endPointAY;
    const double WVX = endPointBX - endPointAX;
    const double WVY = endPointBY - endPointAY;
    const double T = std::max(0.0, std::min(1.0, (PVX * WVX + PVY * WVY) / (LINE * LINE))); // Adjust here as well
    const double PX = endPointAX + T * (endPointBX - endPointAX);
    const double PY = endPointAY + T * (endPointBY - endPointAY);
    return sqrt(pow((PX - pointX), 2) + pow((PY - pointY), 2));
}


float getOwnDistance(RobotPose theRobotPose, FieldBall theFieldBall, FieldDimensions theField)
{
    float ownDistance = getMinimumDis(theField.xPosOwnGoal, theField.yPosLeftGoal, theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y(), theRobotPose.translation.x(), theRobotPose.translation.y()) - sqrt(pow((theFieldBall.positionOnField.x() - theRobotPose.translation.x()), 2) + pow((theFieldBall.positionOnField.y() - theRobotPose.translation.y()), 2)); // the distance between the robot to the line segment connect the ball and our goal - the distance between the robot to the ball.
    
    return abs(ownDistance);
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
std::vector<Vector2f> simRobotDeadSpot[5];


class NeuralControlImpl : public NeuralControlImplBase
{
public:
    virtual bool onSegment(Vector2f p, Vector2f q, Vector2f r);
    virtual int orientation(Vector2f p, Vector2f q, Vector2f r);
    virtual bool doIntersect(Vector2f p1, Vector2f q1, Vector2f p2, Vector2f q2);
    virtual bool preCollision(std::vector<ObstacleVector>& Obstacle, float predictedPosX, float predictedPosY, bool obstacle[]);
    virtual void addObstaclesSimRobot(std::vector<ObstacleVector>& Obstacle);
    virtual std::pair<int, int> startIndexOfLongestConsecutive0s(const bool data[], int length);
    virtual Vector2f rotate_point(float cx,float cy,float angle, Vector2f p);
    
    option(NeuralControl)
    {
        
        cognitionLock.lock();
        if (!(json::has_key(timeData,std::to_string(theGameState.playerNumber))))
        {
            timeData.insert(std::to_string(theGameState.playerNumber), 0);
        }
        
        int timestep = std::stoi(to_string(timeData[std::to_string(theGameState.playerNumber)]));
        // double test_distance = getMinimumDis(0.0, 0.0, 10.0, 0.0, 5.0, 5.0);
        
        int role; // the role our robot takes
        int count = 0; // the count of other robots have a greater measure
        theBehaviorStatus.distance = getOwnDistance(theRobotPose, theFieldBall, theFieldDimensions);
        float ownDistance = theBehaviorStatus.distance; // our own distance
        
        for(auto & teammate : theTeamData.teammates) {
            // Exclude the goal keeper and ourselves
            if(teammate.number > theGameState.playerNumber && teammate.number != 1) {
                count+=1;
            }
        }
        
        
        // // assign role based on num of robots closer
        // if(count >= 2) {
        //     // Defender
        //     role = 3;
        // } else {
        //     // Attacker
        //     role = 2;
        // }
        
        if (theGameState.playerNumber == 1){
            role = 1;
            algorithm = & goalKeeperKickAlgorithm;
        }
        else if (theGameState.playerNumber == 2){
            role = 3;
            algorithm = & defenderKickAlgorithm;
        }
        else if (theGameState.playerNumber == 4 || theGameState.playerNumber == 5 || theGameState.playerNumber == 3) {
            role = 2;
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
        
        if (RLConfig::normalization && (role == 1 || role == 3)) {
            inputObservation = algorithm->normalizeObservation(rawObservation);
        }
        else if (role == 2) {
            inputObservation = {};
            
            // origin,
            // goal,
            // other robots,
            // ball
            
            // agent location array
            std::vector<float> agent_loc = {theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation};
            // ball location array
            std::vector<float> ball_loc = {theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y()};
            
            std::vector<float> origin_ref = get_relative_observation(agent_loc, {0, 0});
            std::vector<float> goal_ref = get_relative_observation(agent_loc, {4800, 0});
            
            std::vector<float> ball_ref = get_relative_observation(agent_loc, ball_loc);
            
            
            inputObservation.insert(inputObservation.end(), origin_ref.begin(), origin_ref.end());
            inputObservation.insert(inputObservation.end(), goal_ref.begin(), goal_ref.end());
            inputObservation.insert(inputObservation.end(), ball_ref.begin(), ball_ref.end());
            
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
        std::vector<float> predictedPosition = environment.getPredictedPosition(theRobotPose, algorithm->getActionMeanVector());
        
        
        bool shield = false;
        bool obstacles[8] = {false, false, false, false,false, false, false,false};
        
        if (RLConfig::shieldEnabled)
        {
            
            
            if (!(json::has_key(returningFlags,std::to_string(theGameState.playerNumber))))
            {
                returningFlags.insert(std::to_string(theGameState.playerNumber), false);
            }
            

            
            if ((predictedPosition[0] < -4670 || predictedPosition[0] > 4670 || predictedPosition[1] > 3100 || predictedPosition[1] < -3100) && theGameState.playerNumber!=1){
                shield = true;
            }
//            if (predictedPosition[0] > 4300 && predictedPosition[1] > 600 && predictedPosition[1] < 800)
//            {
//                shield = true;
//            }
//            if (predictedPosition[0] > 4300 && predictedPosition[1] < -600 && predictedPosition[1] > -800)
//            {
//                shield = true;
//            }
            if(predictedPosition[0] > -3900 || predictedPosition[0] < -4750  || predictedPosition[1] > 640 || predictedPosition[1] < -640){
                shield = true;
            }
            if((predictedPosition[0] > -1500 || predictedPosition[0] < -4600  || predictedPosition[1] > 3000 || predictedPosition[1] < -3000 ) && theGameState.playerNumber==2){
                std::cout << "Shielding" << std::endl;
                shield = true;
            }
            
            
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
            
        }
        
        bool deadSpot = false;
        
        if (theFieldBall.timeSinceBallWasSeen > 4000 && role != 2 && !shield)
        {
            // Find angle and x and y to input into walkAtRelativeSpeed (max speed is 1.0)
            // float angle_pos = atan2(position[1] - theRobotPose.translation.y(), position[0] - theRobotPose.translation.x());
            
            // Get angle to origin
            float angle_pos = atan2(0 - theRobotPose.translation.y(), 0 - theRobotPose.translation.x());
            
            // Turn until facing the origin (within 0.2 rads)
            if (theRobotPose.rotation < angle_pos - 0.2 && !spinning)
            {
                theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                    0.0f,
                    0.0f}});
            }
            else if (theRobotPose.rotation > angle_pos + 0.2 && !spinning)
            {
                theWalkAtRelativeSpeedSkill({.speed = {-0.8f,
                    0.0f,
                    0.0f}});
            }
            else if (spinning) {
                theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                    0.0f,
                    0.0f}});
                
                if (spinningTime > 400) {
                    spinning = false;
                    standingTime = 0;
                    spinningTime = 0;
                }
                spinningTime += 1;
            }
            else
            {
                // stand still
                theStandSkill();
                if (standingTime > 300)
                {
                    spinning = true;
                    standingTime = 0;
                    spinningTime = 0;
                }
                standingTime += 1;
            }
            
        }
        else  if (theFieldBall.timeSinceBallWasSeen > 4000 && role == 2){
            theWalkAtRelativeSpeedSkill({.speed = {0.8f,
                0.0f,
                0.0f}});
        }
        else if(RLConfig::shieldEnabled && shield && role!= 2){
            if(theGameState.playerNumber == 1) {
                std::vector<float> agent_loc = {theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation};
                std::vector<float> ball_loc = {-4400, 0};
                std::vector<float> result= get_relative_observation(agent_loc, ball_loc);
                theWalkAtRelativeSpeedSkill({.speed = {0.0f,
                    result[0]*5200 ,
                    result[1] * 3500}});
            }else{
                std::vector<float> agent_loc = {theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation};
                std::vector<float> ball_loc = {-2700, 0};
                std::vector<float> result= get_relative_observation(agent_loc, ball_loc);
                theWalkAtRelativeSpeedSkill({.speed = {0.0f,
                    result[0]*5200/375 ,
                    result[1] * 3500 /250}});
            }
        }
        else if(robotPreCollision){
            std::pair<int, int> index = startIndexOfLongestConsecutive0s(obstacles, sizeof(obstacles)/sizeof(obstacles[0]));
            double angle = ((index.first + index.second)/2 + 1) * (PI/4) - PI/8;
            float x = 300.f * cos(angle);
            float y = 300.f * sin(angle);
            theWalkAtRelativeSpeedSkill({.speed = {0.0f,x,y}});
            
        }
        else{
            if(deadSpot){
                theWalkAtRelativeSpeedSkill({.speed = {0.0f,
                    theFieldBall.recentBallPositionOnField().x(),
                    theFieldBall.recentBallPositionOnField().y()}});
            }else{
                // Check if we are close enough to the ball to kick it and close to the goal
                // Also check if opponents are in a 30 degree cone in front of us
                // If we are in box [4500 - 1300 to 4500] x [-1100 to 1100]
                bool kicking = false;
                if (theRobotPose.translation.x() > 2850 && theRobotPose.translation.x() < 4500 && theRobotPose.translation.y() > -1100 && theRobotPose.translation.y() < 1100 && role == 2)
                {
                    // Check if we are close enough to the ball to kick it
                    // Calculate distance to ball
                    float distanceToBall = (theFieldBall.positionOnField - theRobotPose.translation).norm();
                    // print distance to ball
                    if (distanceToBall < 700.0f)
                    {
                        
                        bool opponentsInCone = false;
                        for (auto & obstacle : theObstacleModel.obstacles)
                        {
                            if(!obstacle.isTeammate()){
                                if (isFacingPoint(obstacle.center.x() - theRobotPose.translation.x(), obstacle.center.y() - theRobotPose.translation.y(), theRobotPose.rotation))
                                {
                                    opponentsInCone = true;
                                }
                            }
                        }
                        // Check facing goal (center within 30 degrees of center of goal (4500, 0))
                        bool isFacingGoal = isFacingPoint(4700 - theRobotPose.translation.x(), 0 - theRobotPose.translation.y(), theRobotPose.rotation);
                        
                        if (!opponentsInCone && isFacingGoal)
                        {
                            // Kick the ball
                            theWalkToBallAndKickSkill({
                                .targetDirection = 0_deg,
                                .kickType = KickInfo::walkForwardsRightLong,
                                .kickLength = 1000.f,
                                
                            });
                            kicking = true;
                        }
                    }
                }
                if (algorithm->getActionLength() == 3 && !kicking){
                    theWalkAtRelativeSpeedSkill({.speed = {(float)(algorithm->getActionMeans()[0]) * 0.4f, (float)(algorithm->getActionMeans()[1]) > 1.0f ? 1.0f : (float)(algorithm->getActionMeans()[1]), (float)(algorithm->getActionMeans()[2])}});
                }
                else if(algorithm->getActionLength() == 4 && !kicking)
                {
                    if (algorithm->getActionMeans()[3] > 0.0 && (theFieldBall.positionOnField - theRobotPose.translation).norm() < 400.0 && role != 2)
                    {
                        theWalkToBallAndKickSkill({
                            .targetDirection = 0_deg,
                            .kickType = KickInfo::walkForwardsRightLong,
                            .kickLength = 1000.f,
                            
                        });
                    }
                    else if (role == 2){
                        float action_0 = std::max(std::min((float)(algorithm->getActionMeans()[0]), 1.0f), -1.0f) * 0.6f;
                        float action_1 = std::max(std::min((float)(algorithm->getActionMeans()[1]), 1.0f), -1.0f) * 0.5f;
                        if (action_1 > 0){
                            action_1 = action_1 * 1.6 + 0.2;
                        }
                        float action_2 = std::max(std::min((float)(algorithm->getActionMeans()[2]), 1.0f), -1.0f);
                        theWalkAtRelativeSpeedSkill({.speed = {action_0, action_1, action_2}});
                    }
                    else{
                        theWalkAtRelativeSpeedSkill({.speed = {(float)(algorithm->getActionMeans()[0]) * 0.4f, (float)(algorithm->getActionMeans()[1]) > 1.0f ? 1.0f : (float)(algorithm->getActionMeans()[1]), (float)(algorithm->getActionMeans()[2])}});
                    }
                    
                }
                else if (!kicking)
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
        Vector2f stl(Obstacle[i].x + 275.f, Obstacle[i].y + 275.f);
        Vector2f sbl(Obstacle[i].x - 275.f, Obstacle[i].y + 275.f);
        Vector2f str(Obstacle[i].x + 275.f, Obstacle[i].y - 275.f);
        Vector2f sbr(Obstacle[i].x - 275.f, Obstacle[i].y - 275.f);
        Vector2f PredictedPoseVector(predictedPosX, predictedPosY);
        Vector2f obstaclePose(Obstacle[i].x, Obstacle[i].y);
        
        double dist = (theRobotPose.translation - PredictedPoseVector).norm();
        Vector2f unitVector = Vector2f((PredictedPoseVector - theRobotPose.translation).x()/dist,(PredictedPoseVector - theRobotPose.translation).y()/dist);
        Vector2f newVector = Vector2f(theRobotPose.translation.x() + unitVector.x()*110.f, theRobotPose.translation.y() + unitVector.y()*110.f);
        Vector2f rotateCounterClockwise = rotate_point(theRobotPose.translation.x(), theRobotPose.translation.y(), 15, newVector);
        Vector2f rotateClockwise = rotate_point(theRobotPose.translation.x(), theRobotPose.translation.y(), -15, newVector);
        
        intersect = (doIntersect(theRobotPose.translation, newVector, sbl, stl) || doIntersect(theRobotPose.translation, newVector, sbl, sbr) || doIntersect(theRobotPose.translation, newVector, sbr, str) || doIntersect(theRobotPose.translation, newVector, stl, str));
        intersect2 = (doIntersect(theRobotPose.translation, rotateCounterClockwise, sbl, stl) || doIntersect(theRobotPose.translation, rotateCounterClockwise, sbl, sbr) || doIntersect(theRobotPose.translation, rotateCounterClockwise, sbr, str) || doIntersect(theRobotPose.translation, rotateCounterClockwise, stl, str));
        intersect3 = (doIntersect(theRobotPose.translation, rotateClockwise, sbl, stl) || doIntersect(theRobotPose.translation, rotateClockwise, sbl, sbr) || doIntersect(theRobotPose.translation, rotateClockwise, sbr, str) || doIntersect(theRobotPose.translation, rotateClockwise, stl, str));
        
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
    
//    for (auto & obstacle : theObstacleModel.obstacles)
//    {
//         if(!obstacle.isTeammate() && !obstacle.isOpponent()){
//             ObstacleVector o{obstacle.center.x() + theRobotPose.translation.x(), obstacle.center.y() + theRobotPose.translation.y(), false};
//             Obstacle.push_back(o);
//         }
//        
//    }
    for(auto& teammate: theTeamData.teammates){
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

