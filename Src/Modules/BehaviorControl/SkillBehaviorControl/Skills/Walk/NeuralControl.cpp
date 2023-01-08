/**
 * @file NeuralControl.cpp
 *
 * This file implements an implementation for the NeuralControl skill.
 *
 * @author Arne Hasselbring (the actual behavior is older)
 */

#include "Tools/RLConfig.h"
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

    std::vector<float> observation = getObservation(theRobotPose, theFieldBall);

    for (auto i: observation)
        std::cout << i << ' ';
    theWalkAtRelativeSpeedSkill({.speed = {1.0, 0.0, 0.0}});
  }
};

MAKE_SKILL_IMPLEMENTATION(NeuralControlImpl);
