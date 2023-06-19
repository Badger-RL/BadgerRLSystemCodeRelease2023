/**
 * @file FreeKickWall.cpp
 *
 * This file implements a behavior to position between the ball and the own penalty mark.
 *
 * @author Arne Hasselbring
 * @author Sina Schreiber
 */

#include "FreeKickWall.h"
#include "Representations/Modeling/RobotPose.h"
#include <iostream>

Vector2f newPosition(float temp, double multiplier, Vector2f ballPosition, Vector2f targetPosition, float normalizeValue) {
  const Vector2f shift =  (multiplier * ballPosition).normalized(normalizeValue);
  targetPosition = ballPosition + shift;
  temp = sqrt(pow(ballPosition.x() - targetPosition.x(), 2)
                    + pow(ballPosition.y() - targetPosition.y(), 2));
  if (temp >= 750.f) {
    return targetPosition;
  } else {
    return newPosition(temp, multiplier - 0.1, ballPosition, targetPosition, normalizeValue);
  }
}

SkillRequest FreeKickWall::execute(const Agent&, const Agents&)
{

//  const Vector2f defenderVector = Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f) - ballPosition; // change of direction
//  const Vector2f newTarget = ballPosition + defenderVector.normalized(normalizeValue); // calculated new target
//  const Angle rotationAngle = defenderVector.angle(); // angle for the rotation of the direction
//  const Vector2f rotatedNewTargetPosition = newTarget.rotated(-rotationAngle); // rotated new target
//  const Vector2f rotatedOldTargetPosition = currentTarget.rotated(-rotationAngle); // rotated old target
    
  const Vector2f ballPosition = theFieldBall.recentBallPositionOnField();
  double multiplier = -1;
  Vector2f targetPosition;
  targetPosition = newPosition(0.f, multiplier, ballPosition, targetPosition, normalizeValue);
  
  /**
   * if the new target is within the threshold keep the position otherwise walk to the new target.
   */
  if(std::abs(targetPosition.x() - ballPosition.x()) > maxXValue ||
     (std::abs(targetPosition.y() - ballPosition.y()) > maxYValue))
    currentTarget = targetPosition;

  //return SkillRequest::Builder::walkTo(Pose2f(0.f, 500.f));
  
  return SkillRequest::Builder::walkTo(Pose2f((ballPosition - currentTarget).angle(), currentTarget));
}
