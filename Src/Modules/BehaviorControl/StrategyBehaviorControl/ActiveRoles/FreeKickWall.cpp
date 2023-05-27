/**
 * @file FreeKickWall.cpp
 *
 * This file implements a behavior to position between the ball and the own penalty mark.
 *
 * @author Arne Hasselbring
 * @author Sina Schreiber
 */

#include "FreeKickWall.h"
#include <iostream>
SkillRequest FreeKickWall::execute(const Agent&, const Agents&)
{
  const Vector2f ballPosition = theFieldBall.recentBallPositionOnField();
  
//  std::cout << defenderVector.normalized(normalizeValue) << std::endl;
  const Vector2f shift =  (-1 * ballPosition).normalized(normalizeValue);
  const Vector2f targetPosition = ballPosition + shift;
//  const Vector2f defenderVector = Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f) - ballPosition; // change of direction
//  const Vector2f newTarget = ballPosition + defenderVector.normalized(normalizeValue); // calculated new target
//  const Angle rotationAngle = defenderVector.angle(); // angle for the rotation of the direction
//  const Vector2f rotatedNewTargetPosition = newTarget.rotated(-rotationAngle); // rotated new target
//  const Vector2f rotatedOldTargetPosition = currentTarget.rotated(-rotationAngle); // rotated old target
//    std::cout << "Ball Position x: " << ballPosition.x() << std::endl;
//    std::cout << "Ball Position y: " << ballPosition.y() << std::endl;
//    std::cout << "shift x: " << shift.x() << std::endl;
//    std::cout << "shift y: " << shift.y() << std::endl;
//        std::cout << "Target Target x: " << targetPosition.x() << std::endl;
//        std::cout << "Target Target y: " << targetPosition.y() << std::endl;
//    std::cout << "Defender Vector y: " << defenderVector.y() << std::endl;
//    std::cout << "New Target x: " << newTarget.x() << std::endl;
//    std::cout << "New Target y: " << newTarget.y() << std::endl;
//    std::cout << "Current Target x: " << currentTarget.x() << std::endl;
//    std::cout << "Current Target y: " << currentTarget.y() << std::endl;
//    std::cout << "Angle: " << rotationAngle.toDegrees() << std::endl;
//    std::cout << "rotated New Target Position x: " << rotatedNewTargetPosition.x() << std::endl;
//    std::cout << "rotated New Target Position y: " << rotatedNewTargetPosition.y() << std::endl;
//    std::cout << "rotated Old Target Position x: " << rotatedOldTargetPosition.x() << std::endl;
//    std::cout << "rotated Old Target Position y: " << rotatedOldTargetPosition.y() << std::endl;
//    std::cout << "Ball Position - current Target angle: " << (ballPosition - currentTarget).angle() << std::endl;

  /**
   * if the new target is within the threshold keep the position otherwise walk to the new target.
   */
  if(std::abs(targetPosition.x() - ballPosition.x()) > maxXValue ||
     (std::abs(targetPosition.y() - ballPosition.y()) > maxYValue))
    currentTarget = targetPosition;
//    return SkillRequest::Builder::walkTo(Pose2f(-1000.f, 0.f));
    
  
  
  return SkillRequest::Builder::walkTo(Pose2f((ballPosition - currentTarget).angle(), currentTarget));
    
    
    
}
