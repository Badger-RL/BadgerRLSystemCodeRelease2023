/**
 * @file WalkToBallAndKickEngine.cpp
 *
 * This file implements a module that provides a walk to ball and kick generator.
 *
 * @author Arne Hasselbring
 */

#include "WalkToBallAndKickEngine.h"
#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Platform/BHAssert.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/KickLengthConverter.h"

MAKE_MODULE(WalkToBallAndKickEngine, motionControl);

void WalkToBallAndKickEngine::update(WalkToBallAndKickGenerator& walkToBallAndKickGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:WalkToBallAndKickEngine:forwardTurnInterpolation");
  DECLARE_DEBUG_RESPONSE("module:WalkToBallAndKickEngine:UseMirroredForwardKick");
  walkToBallAndKickGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    lastPhaseWasKick = lastPhase.type == MotionPhase::kick;

    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(false /* TODO */, lastPhase);
    const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
    const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * theOdometryDataPreview.inverse() * motionRequest.odometryData;

    const Rangef ttrbRange(1.f, 2.f);
    const float timeToReachBall = ttrbRange.limit(std::max(std::abs(motionRequest.ballEstimate.position.x()) / theWalkingEngineOutput.maxSpeed.translation.x(), std::abs(motionRequest.ballEstimate.position.y()) / theWalkingEngineOutput.maxSpeed.translation.y()));

    const Vector2f perceivedBallPosition = scsCognition * motionRequest.ballEstimate.position;
    const float ballVelocityFactor = mapToRange(perceivedBallPosition.norm(), ballVelocityInterpolationRange.min, ballVelocityInterpolationRange.max, 0.f, 1.f);
    const float ballVelocity = motionRequest.ballEstimate.velocity.norm();
    const float useVelocityLength = (1.f - ballVelocityFactor) * std::min(ballVelocity, minBallVelocityCloseRange) + ballVelocityFactor * ballVelocity;
    const Vector2f ballPosition = scsCognition * BallPhysics::propagateBallPosition(motionRequest.ballEstimate.position, motionRequest.ballEstimate.velocity.normalized(useVelocityLength), timeToReachBall, theBallSpecification.friction);

    const Angle targetDirection = Angle::normalize(scsCognition.rotation + motionRequest.targetDirection);
    Rangea precisionRange = motionRequest.directionPrecision;

    Pose2f kickPose = Pose2f(targetDirection, ballPosition);

    // interpolate between forward and turn kick
    if(motionRequest.turnKickAllowed && motionRequest.preStepAllowed && (motionRequest.kickType == KickInfo::walkForwardsLeft || motionRequest.kickType == KickInfo::walkForwardsRight))
    {
      KickInfo::KickType turnKick = motionRequest.kickType == KickInfo::walkForwardsLeft ? KickInfo::walkTurnLeftFootToRight : KickInfo::walkTurnRightFootToLeft;
      const float factor = Rangef::ZeroOneRange().limit(targetDirection / -theKickInfo[turnKick].rotationOffset);
      kickPose.rotate(theKickInfo[motionRequest.kickType].rotationOffset * (1.f - factor) + theKickInfo[turnKick].rotationOffset * factor);
      kickPose.translate(theKickInfo[motionRequest.kickType].ballOffset * (1.f - factor) + theKickInfo[turnKick].ballOffset * factor);
    }
    else
    {
      kickPose.rotate(theKickInfo[motionRequest.kickType].rotationOffset);
      kickPose.translate(theKickInfo[motionRequest.kickType].ballOffset);
    }

    const float kickPoseShiftY = shiftInWalkKickInYDirection(kickPose, motionRequest.kickType, targetDirection);
    kickPose.translation.y() += kickPoseShiftY;

    bool isInPositionForKick = false;
    const Angle directionThreshold = motionRequest.alignPrecisely == KickPrecision::precise ? 2_deg : 3_deg;
    WalkKickVariant walkKickVariant;
    switch(motionRequest.kickType)
    {
      case KickInfo::forwardFastLeft:
      case KickInfo::forwardFastLeftPass:
      case KickInfo::forwardFastLeftLong:
        isInPositionForKick = ballPosition.y() > forwardFastYThreshold.min && ballPosition.y() < forwardFastYThreshold.max && ballPosition.x() < (motionRequest.kickType == KickInfo::forwardFastLeftLong ? forwardFastLongXMaxThreshold : forwardFastXThreshold.max) && ballPosition.x() > forwardFastXThreshold.min;
        isInPositionForKick &= std::abs(targetDirection + theKickInfo[motionRequest.kickType].rotationOffset) < directionThreshold;
        break;
      case KickInfo::forwardFastRight:
      case KickInfo::forwardFastRightPass:
      case KickInfo::forwardFastRightLong:
        isInPositionForKick = -ballPosition.y() > forwardFastYThreshold.min && -ballPosition.y() < forwardFastYThreshold.max && ballPosition.x() < (motionRequest.kickType == KickInfo::forwardFastRightLong ? forwardFastLongXMaxThreshold : forwardFastXThreshold.max) && ballPosition.x() > forwardFastXThreshold.min;
        isInPositionForKick &= std::abs(targetDirection + theKickInfo[motionRequest.kickType].rotationOffset) < directionThreshold;
        break;
      case KickInfo::walkForwardsLeft:
      case KickInfo::walkForwardsLeftLong:
      case KickInfo::walkForwardsRight:
      case KickInfo::walkForwardsRightLong:
      case KickInfo::walkSidewardsRightFootToRight:
      case KickInfo::walkTurnRightFootToLeft:
      case KickInfo::walkSidewardsLeftFootToLeft:
      case KickInfo::walkTurnLeftFootToRight:
      case KickInfo::walkForwardStealBallLeft:
      case KickInfo::walkForwardStealBallRight:
      case KickInfo::walkForwardsRightAlternative:
      case KickInfo::walkForwardsLeftAlternative:
      {
        walkKickVariant = WalkKickVariant(motionRequest.kickType, theKickInfo[motionRequest.kickType].walkKickType, theKickInfo[motionRequest.kickType].kickLeg, motionRequest.alignPrecisely, motionRequest.kickLength, motionRequest.targetDirection);
        isInPositionForKick = theWalkKickGenerator.canStart(walkKickVariant, lastPhase, precisionRange, motionRequest.preStepAllowed, motionRequest.turnKickAllowed, kickPoseShiftY);
        break;
      }
      case KickInfo::newKick:
      {
        isInPositionForKick = true;
        break;
      }
      default:
        FAIL("Unknown kick type.");
    }
    isInPositionForKick &= lastPhase.type == MotionPhase::stand || lastPhase.type == MotionPhase::walk; // prevent kicking from potential unstable phases
    isInPositionForKick &= motionRequest.ballTimeWhenLastSeen >= theMotionInfo.lastKickTimestamp;
    isInPositionForKick &= theFrameInfo.getTimeSince(motionRequest.ballTimeWhenLastSeen) < 3000.f;
    // velocity.norm() does not need to be transformed to another coordinate system.
    if(!(motionRequest.kickType == KickInfo::walkForwardStealBallLeft || motionRequest.kickType == KickInfo::walkForwardStealBallRight)) // we are so close to the ball. The velocity might be a false perception
      isInPositionForKick &= motionRequest.ballEstimate.velocity.norm() < maxBallVelocity;

    if(isInPositionForKick)
    {
      auto kickPhase = createKickPhase(motionRequest, lastPhase, walkKickVariant, kickPoseShiftY);
      if(kickPhase)
      {
        lastPhaseWasKickPossible = isInPositionForKick;
        kickPhase->kickType = motionRequest.kickType;
        return kickPhase;
      }
      else
        OUTPUT_ERROR("Creating Kick Phase of Type " << TypeRegistry::getEnumName(motionRequest.kickType) << " in WalkToBallAndKickEngine returned an empty kick!");
    }

    // force "duck" feet for the forwardSteal kick
    if(motionRequest.kickType == KickInfo::walkForwardStealBallLeft || motionRequest.kickType == KickInfo::walkForwardStealBallRight)   // otherwise get normal kickpose
    {
      const Pose3f supportInTorso3DLeft = theTorsoMatrix * theRobotModel.soleLeft;
      const Pose2f supportInTorsoLeft(supportInTorso3DLeft.rotation.getZAngle(), supportInTorso3DLeft.translation.head<2>());
      const Pose2f scsCognitionLeft = supportInTorsoLeft.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;
      const Vector2f ballLeft = scsCognitionLeft * theMotionRequest.ballEstimate.position;
      const Pose3f supportInTorso3DRight = theTorsoMatrix * theRobotModel.soleRight;
      const Pose2f supportInTorsoRight(supportInTorso3DRight.rotation.getZAngle(), supportInTorso3DRight.translation.head<2>());
      const Pose2f scsCognitionRight = supportInTorsoRight.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;
      const Vector2f ballRight = scsCognitionRight * theMotionRequest.ballEstimate.position;
      if(theMotionRequest.ballEstimate.position.x() > 0.f &&
         std::abs(kickPose.translation.y()) < kickPoseMaxYTranslation &&
         kickPose.translation.x() < forwardStealStepPlanningThreshold.translation.x() &&
         ballLeft.y() < -forwardStealStepPlanningThreshold.translation.y() &&
         ballRight.y() > forwardStealStepPlanningThreshold.translation.y() &&
         std::abs(kickPose.rotation + (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft).rotation.getZAngle()) < forwardStealStepPlanningThreshold.rotation)
      {
        kickPose = theWalkKickGenerator.getVShapeWalkStep(isLeftPhase, motionRequest.targetDirection + theKickInfo[motionRequest.kickType].rotationOffset);
        if(std::abs(kickPose.translation.y()) < kickPoseMaxYTranslation)
          return theWalkGenerator.createPhase(kickPose, lastPhase);
      }
    }

    MotionRequest::ObstacleAvoidance obstacleAvoidanceInSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceInSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceInSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    if(lastPhaseWasKickPossible && !lastPhaseWasKick && std::abs(kickPose.rotation) < kickPoseThresholds.rotation && std::abs(kickPose.translation.x()) < kickPoseThresholds.translation.x() && std::abs(kickPose.translation.y()) < kickPoseThresholds.translation.y())
    {
      const Vector2f offsetToBall = ballPosition + theKickInfo[motionRequest.kickType].ballOffset;
      kickPose.translation.x() = (std::abs(kickPose.translation.x()) > std::abs(offsetToBall.x()) ? kickPose.translation.x() : offsetToBall.x()) * 1.5f;
      kickPose.translation.y() = (std::abs(kickPose.translation.y()) > std::abs(offsetToBall.y()) ? kickPose.translation.y() : offsetToBall.y()) * 1.5f;
    }
    lastPhaseWasKickPossible = isInPositionForKick;

    Vector2f newBallPosition = ballPosition;
    if(calcInterceptionPosition(motionRequest, newBallPosition, perceivedBallPosition, scsCognition))
    {
      kickPose = Pose2f(targetDirection, newBallPosition);

      // interpolate between forward and turn kick
      if(motionRequest.turnKickAllowed && motionRequest.preStepAllowed && (motionRequest.kickType == KickInfo::walkForwardsLeft || motionRequest.kickType == KickInfo::walkForwardsRight))
      {
        KickInfo::KickType turnKick = motionRequest.kickType == KickInfo::walkForwardsLeft ? KickInfo::walkTurnLeftFootToRight : KickInfo::walkTurnRightFootToLeft;
        const float factor = Rangef::ZeroOneRange().limit(targetDirection / -theKickInfo[turnKick].rotationOffset);
        kickPose.rotate(theKickInfo[motionRequest.kickType].rotationOffset * (1.f - factor) + theKickInfo[turnKick].rotationOffset * factor);
        kickPose.translate(theKickInfo[motionRequest.kickType].ballOffset * (1.f - factor) + theKickInfo[turnKick].ballOffset * factor);
      }
      else
      {
        kickPose.rotate(theKickInfo[motionRequest.kickType].rotationOffset);
        kickPose.translate(theKickInfo[motionRequest.kickType].ballOffset);
      }
    }

    return theWalkToBallGenerator.createPhase(kickPose, newBallPosition, theFrameInfo.getTimeSince(motionRequest.ballTimeWhenLastSeen), perceivedBallPosition.norm(), obstacleAvoidanceInSCS, motionRequest.walkSpeed, lastPhase);
  };
}

std::unique_ptr<MotionPhase> WalkToBallAndKickEngine::createKickPhase(const MotionRequest& motionRequest, const MotionPhase& lastPhase, const WalkKickVariant& walkKickVariant, const float kickPoseShiftY)
{
  if(theKickInfo[motionRequest.kickType].motion == MotionPhase::walk)
    return theWalkKickGenerator.createPhase(walkKickVariant, lastPhase, true, kickPoseShiftY);
  else if(theKickInfo[motionRequest.kickType].motion == MotionPhase::kick)
  {
    KickRequest kr;
    kr.kickMotionType = theKickInfo[motionRequest.kickType].kickMotionType;
    kr.mirror = theKickInfo[motionRequest.kickType].mirror;
    kr.armsBackFix = false;
    kr.calcDynPoints = [this, kick = theKickInfo[motionRequest.kickType],
                              power = KickLengthConverter::kickLengthToPower(motionRequest.kickType, motionRequest.kickLength, 0_deg, theKickInfo)]
    (const int phaseNumber) -> std::vector<DynPoint>
    {
      const bool isLeftPhase = kick.kickLeg != Legs::right;
      Vector2f ball = lastStableBall;
      if(phaseNumber <= 2) // after the robot shifts all his weight on the support foot, the torsoMatrix might be off and results in a miss position of the ball by up to 3cm
      {
        const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
        const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
        const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
        const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;

        ball = scsCognition * theMotionRequest.ballEstimate.position;
        lastStableBall = ball;
      }

      std::vector<DynPoint> d;
      switch(kick.kickMotionType)
      {
        case KickRequest::kickForwardFast:
        {
          if(ball.y() > 0.f)
            ball.y() *= -1.f;

          // RoboCup 2021 shift. Should be based on internal sensor data
          //ball.y() -= 5.f + Rangef(0.f, 10.f).limit((ball.x() - 180.f) / 2.f); //this is a hit correction factor
          ball.y() -= 5.f; // this is a hit correction factor

          //clipping
          forwardFastYClipRange.clamp(ball.y());

          const float hitBeforeTheoreticalMaxVelocity = 5.f;
          const float requiredDistanceFromStrikeOutToKick = 50.f + power * (175.f - 50.f);
          const float optimalHitPointX = std::max(0.f, std::min(ball.x() - theBallSpecification.radius - theRobotDimensions.footLength + hitBeforeTheoreticalMaxVelocity, 50.f));

          const Vector2f kickFootX(optimalHitPointX - 0.5f * requiredDistanceFromStrikeOutToKick, optimalHitPointX + 0.5f * requiredDistanceFromStrikeOutToKick);
          const Vector2f kickFootZ(-190.f + power * 20.f, -190.f + power * 10.f);

          const Vector3f strikeOut(kickFootX.x(), ball.y(), kickFootZ.x()), kickTo(kickFootX.y(), ball.y(), kickFootZ.y());
          const DynPoint dynRFoot2(Phase::rightFootTra, 2, strikeOut), dynRFoot3(Phase::rightFootTra, 3, kickTo);

          d.push_back(dynRFoot2);
          d.push_back(dynRFoot3);
          break;
        }
        case KickRequest::kickForwardFastLong:
        {
          if(ball.y() > 0.f)
            ball.y() *= -1.f;

          // RoboCup 2021 shift. Should be based on internal sensor data
          //ball.y() -= Rangef(0.f, 40.f).limit((ball.x() - 200.f)); //this is a hit correction factor
          ball.y() -= 10.f; // this is a hit correction factor

          //clipping
          forwardFastYClipRange.clamp(ball.y());

          const Vector2f kickFootX(-70.f, 70.f);
          const Vector2f kickFootZ(-170.f, -180.f);

          const Vector3f strikeOut(kickFootX.x(), ball.y(), kickFootZ.x()), kickTo(kickFootX.y(), ball.y(), kickFootZ.y());
          const DynPoint dynRFoot2(Phase::rightFootTra, 2, strikeOut), dynRFoot3(Phase::rightFootTra, 3, kickTo);

          d.push_back(dynRFoot2);
          d.push_back(dynRFoot3);
          break;
        }
      }
      return d;
    };
    return theKickGenerator.createPhase(kr, lastPhase);
  }
  FAIL("Unknown motion type.");
  return std::unique_ptr<MotionPhase>();
}

float WalkToBallAndKickEngine::shiftInWalkKickInYDirection(const Pose2f& kickPose, const KickInfo::KickType kickType, const Angle direction)
{
  if(theKickInfo[kickType].walkKickType != WalkKicks::forward)
    return 0.f;
  bool leftBlocked = false;
  bool rightBlocked = false;
  for(const Obstacle& obstacle : theObstacleModel.obstacles)
  {
    const Vector2f oLeftInKickPose = kickPose.inverse() * obstacle.left;
    const Vector2f oRightInKickPose = kickPose.inverse() * obstacle.right;
    leftBlocked |= obstacle.center.y() < 0.f && oLeftInKickPose.squaredNorm() < sqr(350.f);
    rightBlocked |= obstacle.center.y() > 0.f && oRightInKickPose.squaredNorm() < sqr(350.f);
  }
  if(!leftBlocked && rightBlocked && kickType == KickInfo::walkForwardsLeft)
    return Rangef::ZeroOneRange().limit(-direction / 45_deg) * (theKickInfo[kickType].ballOffset.y() - theKickInfo[KickInfo::walkTurnLeftFootToRight].ballOffset.y());
  if(leftBlocked && !rightBlocked && kickType == KickInfo::walkForwardsRight)
    return Rangef::ZeroOneRange().limit(direction / 45_deg) * (theKickInfo[kickType].ballOffset.y() - theKickInfo[KickInfo::walkTurnRightFootToLeft].ballOffset.y());
  return 0.f;
}

bool WalkToBallAndKickEngine::calcInterceptionPosition(const MotionRequest& motionRequest, Vector2f& ballSCS,
                                                       const Vector2f& ballInSCSNow, const Pose2f& scsCognition)
{
  if(std::abs(ballInSCSNow.angle() - ballSCS.angle()) > 45_deg)  // ball will land behind us, but is currently in front of us
  {
    const Angle velocityVector = scsCognition.rotation + motionRequest.ballEstimate.velocity.angle();
    const Vector2f minDistancePoint = ballInSCSNow.rotated(-velocityVector);
    const bool faraway = std::abs(minDistancePoint.x()) > minBallPositionFrontSide;
    const bool endPositionFar = ballSCS.squaredNorm() > sqr(minBallPositionFuture);
    if((faraway && endPositionFar))
    {
      const float distance = (ballSCS - ballInSCSNow).norm();
      ballSCS = ballInSCSNow + (ballSCS - ballInSCSNow).normalized(std::min(distance, minBallPositionFrontSide));
      return true;
    }
  }
  return false;
}
