/**
 * @file Zweikampf.cpp
 *
 * This file implements a card that duels with an opponent.
 *
 * The general procedure is as follows:
 * - Determine some kick angles based on the last kick direction
 * - For each direction, calculate the field ratings for different kick ranges (TODO only calculate ranges that are actually used for the current direction to safe computation)
 * - For each allowed kick type, determine the best field rating and apply some more ratings and hysterese, such as:
 *   - avoid kicks that push us closer to an obstacle
 *   - worse rating for kicks with a different direction
 *   - bonus for the same kick type for goals
 *   - TODO add hysterese for changing goal sectors and passes
 *   - TODO add shifted turn kicks, to better avoid obstacles
 *   - TODO turn kicks do not work great, all kicks in general are a lot shorter than intended
 * - All kicks are sorted based on their priority: goal > steal (forwardSteal kick, that is our V-Form kick) > pass > everything else
 * - Best kick is selected
 * - Afterwards calculate some precision range, to make the execution easier
 *
 * Future TODOs:
 * - If the behavior says, we want to pass, then ignore possible goal shots and soften the forwardSteal condition.
 * @author Philip Reichenberg
 */

#include "Framework/ModuleGraphRunner.h"
#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldRating.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"
#include "MathBase/Approx.h"
#include <cmath>

STREAMABLE(SearchParameters,
{,
  (int) numOfAnglesNearBestDuelPose, /**< Number of direction sample for both sides around the last calculated kick direction.*/
  (Angle) rangeOfBestDuelPose, /**< Range for the sampling around the last calculated kick direction.*/
  (int) numOfOverallSearch, /**< Number of direction samples outside of the rangeOfBestDuelPose.*/
  (Angle) overallSearchRange, /**< Search range for additional kick directions.*/
  (Angle) bonusForStealBallDirectionAdjustmentSmall, /**< The checked directions are expanded by the stealBall range plus this angle offset.*/
  (Angle) bonusForStealBallDirectionAdjustmentBig, /**< The checked directions are expanded by the stealBall range plus this angle offset.*/
  (int) moreSearchAfterDoingNothing, /**< After this much time passed without an InWalkKick was possible, some more goal shot angles are added.*/
  (Angle) goalSectorWidth, /**< A goal kick must have at least this much width in the goal sector.*/
  (Rangea) minMaxAngleAngleRange, /**< The allowed robot rotation adjustment for the kick direction is interpolated between this range.*/
});

STREAMABLE(DuelRatings,
{,
  // rating modifiers
  (float) ratingMinMaxDifference, /**< Add this much malus, the more the robot must rotate for the kick direction.*/
  (float) ratingBallLandsInOwnHalf, /**< Add this malus, if the ball would land in the own half.*/
  (float) ratingPoseBlockedSmallKickAngle, /**< Add this malus, if the kick pose is blocked by an obstacle for forward kicks.*/
  (float) ratingPoseBlocked, /**< Add this malus, if the kick pose is blocked by an obstacle.*/
  (float) ratingOpponentFaster, /**< Add this malus, if the opponent is faster to reach the ball that we are to reach the kick pose.*/

  (float) ratingSameKick, /**< Add this bonus, if the checked kick is the same as the last best kick.*/
  (float) ratingSameKickAngle, /**< Add this bonus, if the checked kick is the same as the last best kick and the direction is similar.*/
  (float) ratingStealBall, /**< removes ratingBallLandsInOwnHalf and adds some bonus too. */
  (float) goalAreaLongKickRating, /**< Near the own goal, we want to kick the ball away. */
  (float) ratingStealBallKick, /**< Add this bonus for steal ball kicks.*/
  (float) ratingStealBallKickBetterSide, /**< Add this bonus for the forwardSteal, to get the better side.*/
  (float) ratingGoalShot, /**< Add this bonus if the kick can reach the goal.*/
});

STREAMABLE(DuelTimings,
{,
  (int) maxTimeDoingNothing, /**< After this much time passed without an InWalkKick was possible, the Zweikampf deactivates itself.*/
  (int) deactiveAfterDoingNothing, /**< After the Zweikampf deactivated itself because of doing nothing, this much time must pass befor going active again.*/
  (int) sidewardRestrictionTime, /**< After the sideward kick was restricted once, so much time must have passed after no restriction is needed anymore, before the restriction is actually disabled again.*/
  (int) noKickMinTime, /**< If the last planned kick was not the forwardSteal, so much time must have passed of no calculated kicks, before the forwardSteal is no longer wanted.*/
  (int) noKickStealMinTime, /**< If the last planned kick was the forwardSteal, so much time must have passed of no calculated kicks, before the forwardSteal is no longer wanted.*/
  (int) ignoreSkillRequestTime, /**< Ignore the pass skill request for this long, if we decided we can not execute it. */
});

STREAMABLE(ObstacleHandling,
{,
  (Vector2f) leftFootEdge, /**< Left foot edge.*/
  (Vector2f) rightFootEdge, /**< Right foot edge.*/
  (Vector2f) leftFootKickEdge, /**< Left foot edge after the InWalkKick.*/
  (Vector2f) rightFootKickEdge, /**< Right foot edge after the InWalkKick.*/
  (float) maxObstacleDistanceForWalkStealBallKick, /**< The obstacle must stand this much more away from the ball than us, to block the forwardSteal InWalkKick.*/
  (float) maxObstacleDistanceForSideStealBallKick, /**< The obstacle must stand this much more away from the ball than us, to block the forwardSteal InWalkKick.*/
  (float) maxObstacleDistancePrediction, /**< If the duel obstacle was not seen for a longer time, assume it moved towards the ball. */
  (float) obstacleDistanceForBlockingPose, /**< Calculate where the obstacle could be, when we execute the kick. This is used to decide, if one kick is worse than the mirror one. */
  (float) obstacleBlockingRadius, /**< Range for the obstacle to check for the foot edges, to add a malus for the InWalkKick.*/
  (float) obstacleBlockingRadiusSidewardsIncrease, /**< Increase the obstacleBlockingRadius by this amount, if the InWalkKick is a sidewards kick.*/
  (float) noKickBlockingPose, /** distance to ball, when no kick was calculated. */
  (float) obstacleMinDistanceForBackPass, /**< The duel obstacle must be this far away to allow for a back pass from the skill request. */
  (Rangef) obstacleShiftRange, /**< Obstacles, that are this near the ball, are shifted behind the ball.*/
  (float) maxObstacleDistanceToBallForRiskyKicks,
  (float) maxObstacleDistanceToForceForwardSteal,
});

STREAMABLE(StealBallParameters,
{,
  (Angle) stealBallTypeTooMuchRotationMinValue, /**< StealBall kick types are allowed to have higher needed rotation. */
  (Angle) bonusForStealBallMinMaxAngle,
  (Rangea) stealBallRange, /**< Field directions get a bonus, if they are in this range.*/ // TODO After Simulation Evaluation set max angle to 90 deg
  (Angle) stealBallMinRobotRotation, /**< The own rotation must be above this value, to give a bonus to directions that kick the ball sideways on the field and not in direction of the goal.*/
  (Angle) stealBallMinPositionRotation, /**< The ball position angle on the field must be above this value, to give a bonus to directions that kick the ball sideways on the field and not in direction of the goal.*/
  (Rangea) forwardStealBallOpponentPositionNormal, /**< The obstacle must have this angle relativ to the ball, to allow the forwardSteal InWalkKick.*/
  (Angle) forwardStealBallOpponentPositionMalus, /**< The obstacle must have this angle relativ to the ball, to allow the forwardSteal InWalkKick, if the last planned InWalkKick was no forwardSteal.*/
});

SKILL_IMPLEMENTATION(ZweikampfImpl,
{,
  IMPLEMENTS(Zweikampf),
  CALLS(DribbleToGoal),
  CALLS(GoToBallAndKick),
  CALLS(LookActive),
  CALLS(WalkToPoint),
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldRating),
  REQUIRES(FrameInfo),
  REQUIRES(KickInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotPose),
  REQUIRES(SkillRequest),
  REQUIRES(TeamData),
  REQUIRES(WalkingEngineOutput),

  LOADS_PARAMETERS(
  {,
    (SearchParameters) searchParameters, /**< Parameters to decide which kick angles are checked. */
    (DuelRatings) duelRatings, /**< All rating modifiers. */
    (DuelTimings) duelTimings, /**< All time stamp parameters. */
    (ObstacleHandling) obstacleHandling, /**< All parameters to handle obstacles. */
    (StealBallParameters) stealBallParameters, /**< All parameters to handle kick angles near obstacles. */

    (Angle) smallerKickAnglePreference, /**< Interpolate ratingLargeKickAngle to 100% over this angle range. */
    (Angle) forwardStealPreferenceRange, /**< If the kick angle for the forwardsteal is bigger or smaller than this value, we can be sure to use the better side. */
    (Angle) replaceNormalToLongKickGoalShot, /**< When shooting a goal, replace straight forward kicks with the forwardLong.*/
    (float) sectorWheelObstacleBallRadiusFactor, /**< Increase obstacle with by this factor times the ball radius.*/
    (Rangea) allowPassAngle, /**< Pass request from the skill request are allowed in this field angle range. */
    (float) fieldBorderSafeDistance, /**< InWalkKick have some uncertainty, so the safe distance to the field border intersection is increased by up to this value.*/
    (float) minForwardTurnRange, /** < While not near the opponent goal, the min range for the forward and turn kicks should be this length. */

    (Angle) sidewardsRotationCorrectionBonus, /**< If last planned kick was the sidewards kick, then add a bonus to the allowed rotation. */

    // look active
    (float) lookActiveMinBallDistance, /**< If the ball is atleast this far away, use lookActive with withBall = true. */
    (int) sampleRateKickRanges, /**< Preprocess the kick ranges by this sample size. */

    (bool) replaceForwardWithLongGoalShot, /** If kicking at the goal, replace this forward kick with the long version. */

    // The allowed InWalkKicks
    // TODO sidewards kick is currently not allowed, but if the SkillRequest planned to do it before ZweiKampf was active, then allow it here too!
    (std::vector<KickInfo::KickType>) allowedKicks,
  }),
});

class ZweikampfImpl : public ZweikampfImplBase
{
  ENUM(TargetType,
  {,
    goalShot,
    stealBall,
    pass,
    other,
  });

  struct DuelPose
  {
    float rating = 0.f; /**< Rating of the kick. Small == good, high == bad. */
    Pose2f pose; /**< Kick pose. */
    bool noKick = true; /**< Is a kick requested? */
    bool preStepAllowed = true; /**< Is a pre step allowed for the kick? */
    bool turnKickAllowed = false; /**< Are turn kicks allowed? */
    KickInfo::KickType kickType = KickInfo::walkForwardsLeft; /**< Type of the kick. */
    Vector2f endPoint; /**< Ball end position after the kick. */
    Rangea precision = Rangea(0_deg, 0_deg); /**< Precision range of the kick direction. */
    Angle kickAngle = 0_deg; /**< Kick direction. */
    float range = -1.f; /**< Kick range. Used to calculate the kick power. */
    TargetType type = TargetType::other; /**< Type of the situation, in which the kick is used. */
  };

  struct SkillRequestPose
  {
    Pose2f target;
    int passTarget = -1;
    SkillRequest::Type skill = SkillRequest::none;
  };

  // Mapping of every kick range (that should be checked) to a rating.
  struct RatingMap
  {
    float rating = 0.f; /**< Rating of the range. */
    float range = 0.f; /**< The kick range. */
    bool isTeammatePass = false;
  };

  void reset(const Zweikampf&) override
  {
    theDuelPose = DuelPose();
    lastRobotPoseRotation = theRobotPose;
    forceForwardSteal = false;

    // Init based on the behavior
    theDuelPose.kickAngle = (theRobotPose.inversePose * theSkillRequest.target).translation.angle();

    if(!calculatedKickDistances)
    {
      // to reduce computation time later, we want to get a list of all kick ranges, we want to precompute later
      // these ranges are the min and max ranges of all allowed kicks, and an interpolation between the forward and turn kick
      // 1. get interpolation ranges for forward and turn kick
      float minRange = std::min(theKickInfo[KickInfo::walkForwardsLeft].range.min, theKickInfo[KickInfo::walkTurnLeftFootToRight].range.min);
      float maxRange = std::max(theKickInfo[KickInfo::walkForwardsLeft].range.max, theKickInfo[KickInfo::walkTurnLeftFootToRight].range.max);
      const float steps = (maxRange - minRange) / sampleRateKickRanges;
      std::vector<float> distances;
      for(float range = minRange + steps; range < maxRange; range += steps)
        distances.emplace_back(range);

      // 2. get ranges for all kicks
      for(KickInfo::KickType kick : allowedKicks)
      {
        if(kick == KickInfo::walkForwardStealBallLeft || kick == KickInfo::walkForwardStealBallRight)
          continue;
        distances.emplace_back(theKickInfo[kick].range.min);
        distances.emplace_back(theKickInfo[kick].range.max);
      }

      distances.emplace_back(minForwardTurnRange);

      // sort them
      std::sort(distances.begin(), distances.end(), [](const float& d1, const float& d2) { return d1 < d2; });

      // reset and save every unique range
      checkKickDistancesForFR = std::vector<float>();
      float lastRange = -1.f;
      for(float dis : distances)
      {
        if(dis != lastRange)
          checkKickDistancesForFR.emplace_back(dis);
        lastRange = dis;
      }
      calculatedKickDistances = true;
    }
  }

  using Skills::ZweikampfSkill::Implementation::preProcess;

  void preProcess() override
  {
    calculateClosestObstacle(theObstacleModel, duelObstacle, duelObstacleMovedNearBall);
    DECLARE_DEBUG_DRAWING("skill:Zweikampf:wheel", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:Zweikampf:kicks", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:Zweikampf:wheelSteal", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:Zweikampf:sidewardRange", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:Zweikampf:sideSteal", "drawingOnField");
  }

  void execute(const Zweikampf&) override
  {
    STOPWATCH("skill:Zweikampf:duel")
    {
      // Localisation might be off (even if it just for a few cm), therefore the ball, that is inside the field might already be perceived outside of the field
      useBallPosition = theFieldBall.positionOnField;

      useBallPosition.x() = ballClipX.limit(useBallPosition.x());
      useBallPosition.y() = ballClipY.limit(useBallPosition.y());
      updateSkillRequest();
      if(forcedDeactive)
      {
        forcedDeactive = theFrameInfo.getTimeSince(timeSinceDoingNothing) < duelTimings.deactiveAfterDoingNothing && Geometry::isPointInsideRectangle(opponentHalf, useBallPosition);
        theDribbleToGoalSkill();
      }
      else
      {
        calculateDuel();
        lastRobotPoseRotation = theRobotPose;
        if(!theDuelPose.noKick)
        {
          float kickRange = theDuelPose.type == TargetType::goalShot ? 10000.f : theDuelPose.range; // Goal shots shall take the strongest version
          theGoToBallAndKickSkill({.targetDirection = Angle::normalize(theDuelPose.kickAngle),
                                   .kickType = theDuelPose.kickType,
                                   .lookActiveWithBall = theFieldBall.positionRelative.squaredNorm() > sqr(lookActiveMinBallDistance),
                                   .alignPrecisely = soonCloseRangeDuel ? KickPrecision::justHitTheBall : KickPrecision::notPrecise,
                                   .length = kickRange,
                                   .turnKickAllowed = theDuelPose.turnKickAllowed,
                                   .directionPrecision = theDuelPose.precision});
        }
        else
        {
          theWalkToPointSkill({.target = theDuelPose.pose,
                               .reduceWalkingSpeed = false,
                               .rough = true,
                               .disableStanding = true});
          theLookActiveSkill({.withBall = true});
        }

        // check of opponent half is needed to prevent stupid stuff near the own goal, gets used in the next computation frame
        forcedDeactive = theFrameInfo.getTimeSince(timeSinceDoingNothing) > duelTimings.maxTimeDoingNothing;
      }
    }
  }

  // TODO Refector Method, updatePassEndPosition() is kinda useless and code is hard to read, because it is unnecessary complicated
  // If skill request changes and it is a pass, we just check if the target exists to verfiy a possible target
  // If dribbling, take the target direction as a base for the search later in the ZweiKampf
  // In general: Just update the last direction based on the odometry
  void updateSkillRequest()
  {
    if((theFrameInfo.getTimeSince(ignoreSkillRequestTimestamp) > duelTimings.ignoreSkillRequestTime || theSkillRequest.skill != SkillRequest::pass) && // Ignore bad to be executed passes
       (theSkillRequest.skill != theSkillRequestPose.skill ||
        theSkillRequest.target != theSkillRequestPose.target ||
        theSkillRequest.passTarget != theSkillRequestPose.passTarget))
    {
      theSkillRequestPose.passTarget = theSkillRequest.passTarget;
      theSkillRequestPose.skill = theSkillRequest.skill;
      theSkillRequestPose.target = theSkillRequest.target;
      switch(theSkillRequest.skill)
      {
        case SkillRequest::pass:
        {
          for(const Teammate& t : theTeamData.teammates)
          {
            if(t.number == theSkillRequestPose.passTarget)
            {
              updatePassEndPosition();
              goto teammateFound;
            }
          }
          theSkillRequestPose.passTarget = -1; // Nothing more needed here. Just use the last duelPose as a base
        teammateFound:
          break;
        }
        case SkillRequest::dribble:
          theDuelPose.kickAngle = theSkillRequestPose.target.rotation - theRobotPose.rotation;

        default:
          break;
      }
    }
    else if(theSkillRequestPose.skill == SkillRequest::pass)
      updatePassEndPosition();
    else
      updateDuelPose();
  }

  void updateDuelPose()
  {
    // Calculate kickAngle based in the planned end point
    theDuelPose.kickAngle = Angle::normalize(theDuelPose.kickAngle + lastRobotPoseRotation.rotation - theRobotPose.rotation);
    if(std::isnan(theDuelPose.kickAngle))
      theDuelPose.kickAngle = 0_deg;
  }

  void updatePassEndPosition()
  {
    const Teammate* teammate = nullptr;
    for(const Teammate& t : theTeamData.teammates)
    {
      if(t.number == theSkillRequestPose.passTarget)
        teammate = &t;
    }
    if(!teammate)
    {
      theSkillRequestPose.passTarget = -1;
      updateDuelPose();
    }
    else
      updateDuelPose();
  }

  void calculateClosestObstacle(const ObstacleModel& model, Obstacle& nearestObstacle, Obstacle& duelObstacleMovedNearBall)
  {
    float distanceToClosestObstacle = std::numeric_limits<float>::max();
    Angle angleToClosestObstacle;
    for(const Obstacle& omo : model.obstacles)
    {
      if(omo.type != Obstacle::Type::goalpost)
      {
        const float sqrNorm = omo.center.squaredNorm();

        if(sqrNorm < distanceToClosestObstacle)
        {
          distanceToClosestObstacle = sqrNorm;
          angleToClosestObstacle = omo.center.angle();
          nearestObstacle = omo;
        }
      }
    }
    distanceToClosestObstacle = std::sqrt(distanceToClosestObstacle);
    this->distanceToClosestObstacle = distanceToClosestObstacle;
    this->angleToClosestObstacle = angleToClosestObstacle;
    duelObstacleMovedNearBall = nearestObstacle;
    shiftObstacleBackward(nearestObstacle.center, duelObstacleMovedNearBall.center);
  }

  void shiftObstacleBackward(Vector2f& obstacleCenter, Vector2f& duelObstacleMovedNearBall)
  {
    // Currently, obstacles that are really close to the ball, but not in front of it, are preceived as being in front of the ball by a few cm.
    // We need to shift them backwards behind the ball, otherwise the sector wheel for the duel calculation will just ignore this obstacle.
    // This would just result in a forward kick -> ball bounces off the opponent and we just walk on top of the opponent feet and fall over.
    const float obstacleDistanceToBall = (obstacleCenter - theFieldBall.positionRelative).norm();
    if(theFieldBall.timeSinceBallWasSeen < 1500.f && obstacleDistanceToBall <= std::abs(obstacleHandling.obstacleShiftRange.min) && obstacleCenter.x() < theFieldBall.positionRelative.x() + obstacleHandling.obstacleShiftRange.max && obstacleCenter.x() > theFieldBall.positionRelative.x() + obstacleHandling.obstacleShiftRange.min)
    {
      const Angle rotationAngle = (Vector2f(theBallSpecification.radius, obstacleCenter.y() - theFieldBall.positionRelative.y())).angle();
      obstacleCenter = Vector2f(std::max(std::abs(obstacleCenter.y() - theFieldBall.positionRelative.y()), theBallSpecification.radius), 0.f).rotated(rotationAngle) + theFieldBall.positionRelative;
    }
    if(duelObstacleMovedNearBall != Vector2f(1.f, 0.f) && (theRobotPose * obstacleCenter - theFieldBall.positionOnField).x() > 0.f)
      duelObstacleMovedNearBall = (duelObstacleMovedNearBall - theFieldBall.positionRelative).normalized(obstacleHandling.obstacleDistanceForBlockingPose) + theFieldBall.positionRelative;
  }

  void handleIgnoreSkillRequest(const Vector2f& opponentOnField)
  {
    if(theSkillRequestPose.skill == SkillRequest::pass)
    {
      const Angle fieldAngle = (theSkillRequestPose.target.translation - useBallPosition).angle();
      if(!allowPassAngle.isInside(fieldAngle) && (opponentOnField - useBallPosition).squaredNorm() < sqr(obstacleHandling.obstacleMinDistanceForBackPass))
      {
        ignoreSkillRequestTimestamp = theFrameInfo.time;
        theSkillRequestPose.skill = SkillRequest::shoot;
        theSkillRequestPose.passTarget = -1;
        if(theDuelPose.type == TargetType::pass)
          theDuelPose = DuelPose();
      }
    }
  }

  void calculateDuel()
  {
    // 1. update some variables
    if(theFrameInfo.getTimeSince(timeSinceDoingNothing) > duelTimings.deactiveAfterDoingNothing)
      timeSinceDoingNothing = theFrameInfo.time;

    DuelPose lastDuelPose = theDuelPose;
    std::vector<Angle> directionPossibilities;
    Angle minMaxAngle;
    std::vector<std::vector<DuelPose>> duelPoses;
    for(std::size_t i = 0; i < TargetType::numOfTargetTypes; i++)
      duelPoses.emplace_back();

    const Vector2f opponentOnField = theRobotPose * duelObstacle.center;
    soonCloseRangeDuel = (duelObstacle.center - theFieldBall.endPositionRelative).squaredNorm() < sqr(obstacleHandling.maxObstacleDistanceToBallForRiskyKicks);

    handleIgnoreSkillRequest(opponentOnField);

    // 2. Calculate distance metric between us and the opponent to the ball
    // fast simulation scenes need in theory a 50 to 100 mm shift
    const float opponentAndSelfDistanceToBallDiff = (opponentOnField - useBallPosition).norm() - std::max(0.f, theFieldBall.positionRelative.norm() - (theRobotDimensions.footLength + theBallSpecification.radius)) // distance to ball for duelObstacle and self
                                                    - ((ModuleGraphRunner::getInstance().getProvider("RobotPose") == "LogDataProvider" || ModuleGraphRunner::getInstance().getProvider("RobotPose") == "OracledWorldModelProvider") && SystemCall::getMode() == SystemCall::simulatedRobot ? 100.f : 0.f); // fast simulation scene robot shift

    // 3. Calculate variables, which disable and enable specific kicks or behavior
    // 3.1. Calculate variables, which disable and enable the side kick
    // get angles to the goal posts of own goal
    const Angle angleToLeftPost = Angle::normalize((leftGoalPostOwn - useBallPosition).angle() - theRobotPose.rotation - 180_deg); // TODO WHY -180deg? WTF
    const Angle angleToRightPost = Angle::normalize((rightGoalPostOwn - useBallPosition).angle() - theRobotPose.rotation + 180_deg);

    // --------------- can the sidewards outer kick be used? -------------- //
    const Angle angleToBall = (useBallPosition - theRobotPose.translation).angle();
    const Angle angleFromGoalToRobot = (theRobotPose.translation - Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f)).angle();
    const Angle angleFromGoalToBall = (useBallPosition - Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f)).angle();

    const Vector2f rotatedVectorRobotToBall = (useBallPosition.rotated(-angleFromGoalToBall) - theRobotPose.translation.rotated(-angleFromGoalToBall));
    bool mustStandBetweenBallAndGoal = (rotatedVectorRobotToBall.x() > 0.f && std::abs(angleFromGoalToRobot - angleToBall) < 90_deg) ||
                                       (Geometry::isPointInsideRectangle(penaltyRect, theRobotPose.translation) && std::abs((useBallPosition - theRobotPose.translation).angle()) < 90_deg);

    if(mustStandBetweenBallAndGoal)
      lastSidewardRestrictionTimestamp = theFrameInfo.time;
    else
      mustStandBetweenBallAndGoal |= theFrameInfo.getTimeSince(lastSidewardRestrictionTimestamp) < duelTimings.sidewardRestrictionTime;

    // side kick must use the following kick direction
    const Rangea forbiddingKickAngle = !mustStandBetweenBallAndGoal ? Rangea(-180_deg, 180_deg) : Rangea(angleToLeftPost, angleToRightPost < angleToLeftPost ? 179.9_deg : angleToRightPost);
    const Range forbiddingKickAngleExtra = !mustStandBetweenBallAndGoal ? Rangea(-180_deg, 180_deg) : (angleToRightPost < angleToLeftPost ? Rangea(-179_deg, angleToRightPost) : forbiddingKickAngle);

    const Angle angleFromBallToOpponent = (opponentOnField - useBallPosition).angle();

    // 3.2. Calculate variables, which disable and enable the forwardsteal kick
    // ------------- get the angle cone for the allowed kick directions of the forwardSteal ------------- //
    // TODO how do I make this less ugly :sob:
    const Angle forbiddingKickAngleMiddle = (Angle::normalize(Angle(std::max(angleToLeftPost, angleToRightPost)) + theRobotPose.rotation) + Angle::normalize(Angle(std::min(angleToLeftPost, angleToRightPost)) + theRobotPose.rotation)) / 2.f;
    const float forwardStealDirectionInterpolation = std::min(1.f, std::max(theFieldBall.endPositionOnField.x() - theFieldDimensions.xPosOwnGoalArea, 0.f) / (theFieldDimensions.xPosOwnGroundLine / 2.f - theFieldDimensions.xPosOwnGoalArea));
    const Angle bestForwardStealAngle = forwardStealDirectionInterpolation * 90_deg + (1.f - forwardStealDirectionInterpolation) * (90_deg + forbiddingKickAngleMiddle);
    const Angle bestForwardStealAngleMirrored = Angle::normalize(bestForwardStealAngle - 180_deg);
    const Rangea bestForwardStealRange(std::min(bestForwardStealAngleMirrored, bestForwardStealAngle), std::max(bestForwardStealAngleMirrored, bestForwardStealAngle));
    const Rangea leftForwardStealRange(bestForwardStealRange.max - 5_deg, bestForwardStealRange.max + 5_deg);
    const Rangea rightForwardStealRange(bestForwardStealRange.min - 5_deg, bestForwardStealRange.min + 5_deg);

    // Angle cone where the duel obstacle must be relative to the ball, to allow the forwardSteal
    const bool lastDuelKickWasForwardSteal = theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight;
    const float forwardStealConeRatio = Rangef::ZeroOneRange().limit((opponentAndSelfDistanceToBallDiff + theBallSpecification.radius) / 100.f);
    const Angle useForwardStealBallOpponentPosition = !lastDuelKickWasForwardSteal ? stealBallParameters.forwardStealBallOpponentPositionMalus : // Normal Case
                                                      Angle(forwardStealConeRatio * stealBallParameters.forwardStealBallOpponentPositionNormal.min + (1.f - forwardStealConeRatio) * stealBallParameters.forwardStealBallOpponentPositionNormal.max); // Use wider cone
    Rangea leftAngleRangeBallToOpponent(std::min(Angle((bestForwardStealRange.max + bestForwardStealRange.min) / 2.f - useForwardStealBallOpponentPosition), 0_deg), std::max(Angle((bestForwardStealRange.max + bestForwardStealRange.min) / 2.f + useForwardStealBallOpponentPosition), 0_deg));

    // 3.3. Calculate if the special behavior can be active, when we are standing orthogonal on the side to the opponent.
    // ----------- bonus for stealing the ball from the opponent, when we do not stand behind the ball / between ball and goal ----------- //
    bool bonusForStealBall = std::abs(theRobotPose.rotation) > stealBallParameters.stealBallMinRobotRotation && std::abs((useBallPosition - theRobotPose.translation).angle()) > stealBallParameters.stealBallMinPositionRotation;

    // set max allowed kick angle adjustment
    if(!bonusForStealBall)
    {
      // base on the distance to the ball, determine the max allowed rotation change
      const float distanceToBall = (theFieldBall.positionRelative - duelObstacle.center).norm();
      if(distanceToBall < 100.f)
        minMaxAngle = searchParameters.minMaxAngleAngleRange.min;
      else if(distanceToBall > 800.f)
        minMaxAngle = searchParameters.minMaxAngleAngleRange.max;
      else
        minMaxAngle = std::max(((distanceToBall - 100.f) / (800.f - 100.f)), 0.f) * (searchParameters.minMaxAngleAngleRange.max - searchParameters.minMaxAngleAngleRange.min) + searchParameters.minMaxAngleAngleRange.min;
    }
    else
      minMaxAngle = stealBallParameters.bonusForStealBallMinMaxAngle;

    bonusForStealBall &= forwardStealDirectionInterpolation > 0.9f && opponentAndSelfDistanceToBallDiff < obstacleHandling.maxObstacleDistanceForSideStealBallKick;
    const Rangea clipRange(-180_deg, 180_deg);
    const Rangea stealBallMax = !bonusForStealBall ? clipRange : Rangea(stealBallParameters.stealBallRange.min - theRobotPose.rotation, stealBallParameters.stealBallRange.max - theRobotPose.rotation);
    const Rangea stealBallMin = !bonusForStealBall ? clipRange : Rangea(-stealBallParameters.stealBallRange.max - theRobotPose.rotation, -stealBallParameters.stealBallRange.min - theRobotPose.rotation);

    // 4. Calculate all kick angles, that shall be checked
    // Search near the current duel pose
    directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle));
    directionPossibilities.push_back(0_deg);
    for(Angle i = searchParameters.rangeOfBestDuelPose / searchParameters.numOfAnglesNearBestDuelPose; i < searchParameters.rangeOfBestDuelPose; i += searchParameters.rangeOfBestDuelPose / searchParameters.numOfAnglesNearBestDuelPose)
    {
      directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle + i));
      directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle - i));
    }

    // Overall search to maybe find something better
    for(Angle i = searchParameters.rangeOfBestDuelPose; i < searchParameters.overallSearchRange; i += (searchParameters.overallSearchRange - searchParameters.rangeOfBestDuelPose) / searchParameters.numOfOverallSearch)
    {
      directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle + i));
      directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle - i));
    }

    // side steal angles
    if(bonusForStealBall)
    {
      directionPossibilities.push_back(Angle::normalize(stealBallMax.min + searchParameters.bonusForStealBallDirectionAdjustmentSmall));
      directionPossibilities.push_back(Angle::normalize(stealBallMin.max - searchParameters.bonusForStealBallDirectionAdjustmentSmall));
      directionPossibilities.push_back(Angle::normalize((stealBallMin.max + stealBallMax.max) / 2.f));
    }

    if(mustStandBetweenBallAndGoal)
      directionPossibilities.push_back(Angle::normalize((forbiddingKickAngle.min + forbiddingKickAngle.max) * 0.5f));
    else
      directionPossibilities.push_back(-theRobotPose.rotation);
    directionPossibilities.push_back(theKickInfo[KickInfo::walkSidewardsLeftFootToLeft].rotationOffset + theRobotPose.rotation);
    directionPossibilities.push_back(theKickInfo[KickInfo::walkSidewardsRightFootToRight].rotationOffset + theRobotPose.rotation);

    // forward steal angles
    directionPossibilities.push_back(Angle::normalize(bestForwardStealRange.max - theRobotPose.rotation));
    directionPossibilities.push_back(Angle::normalize(bestForwardStealRange.min - theRobotPose.rotation));

    // minMaxAngle directions
    directionPossibilities.push_back(Angle::normalize(minMaxAngle + theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset));
    directionPossibilities.push_back(Angle::normalize(-minMaxAngle + theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset));

    // 5. Calculate the sector wheel. This is used to help to rate the different kickangles and kick ranges
    calculateSectorWheel();

    // 6. Based on the sector wheel, we want to make sure we will later check possible goal kick angles
    for(const SectorWheel::Sector& sector : kickAngles)
      if(sector.type == SectorWheel::Sector::goal && sector.angleRange.getSize() > searchParameters.goalSectorWidth)
      {
        // only take the middle, becaue in case the goal kick is the best one, in the next computation cycle most kicks are sampled around this kick direction anyway
        directionPossibilities.push_back(Angle::normalize((sector.angleRange.max + sector.angleRange.min) / 2.f));

        // Check if we could do something, if we just search for more possible angles
        if(theFrameInfo.getTimeSince(timeSinceDoingNothing) > searchParameters.moreSearchAfterDoingNothing || theDuelPose.type != TargetType::goalShot)
        {
          directionPossibilities.push_back(Angle::normalize(sector.angleRange.max + searchParameters.bonusForStealBallDirectionAdjustmentSmall));
          directionPossibilities.push_back(Angle::normalize(sector.angleRange.min + searchParameters.bonusForStealBallDirectionAdjustmentSmall));
          directionPossibilities.push_back(Angle::normalize(sector.angleRange.max + searchParameters.bonusForStealBallDirectionAdjustmentBig));
          directionPossibilities.push_back(Angle::normalize(sector.angleRange.min + searchParameters.bonusForStealBallDirectionAdjustmentBig));
        }
        break;
      }

    // 7. Debug drawings
    // Field drawing for the angle cones for the forwardSteal kick direction and position of the duel obstacle
    COMPLEX_DRAWING("skill:Zweikampf:wheelSteal")
    {
      SectorWheel wheel;
      std::list<SectorWheel::Sector> sectors;
      wheel.begin(theFieldBall.positionOnField);
      wheel.addSector(leftForwardStealRange, std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
      wheel.addSector(rightForwardStealRange, std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
      wheel.addSector(leftAngleRangeBallToOpponent, std::numeric_limits<float>::max(), SectorWheel::Sector::obstacle);
      sectors = wheel.finish();
      DRAW_SECTOR_WHEEL("skill:Zweikampf:wheelSteal", sectors, theFieldBall.positionOnField);
    }

    COMPLEX_DRAWING("skill:Zweikampf:sideSteal")
    {
      if(stealBallMax != clipRange)
      {
        SectorWheel wheel;
        std::list<SectorWheel::Sector> sectors;
        wheel.begin(theFieldBall.positionOnField);
        wheel.addSector(Rangea(stealBallParameters.stealBallRange.min, stealBallParameters.stealBallRange.max), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
        wheel.addSector(Rangea(-stealBallParameters.stealBallRange.max, -stealBallParameters.stealBallRange.min), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
        sectors = wheel.finish();
        DRAW_SECTOR_WHEEL("skill:Zweikampf:sideSteal", sectors, theFieldBall.positionOnField);
      }
    }

    COMPLEX_DRAWING("skill:Zweikampf:sidewardRange")
    {
      SectorWheel wheel;
      std::list<SectorWheel::Sector> sectors;
      wheel.begin(theFieldBall.positionOnField);
      Angle minNormal = std::min(Angle::normalize(forbiddingKickAngle.min + theRobotPose.rotation), Angle::normalize(forbiddingKickAngle.max + theRobotPose.rotation));
      Angle maxNormal = std::max(Angle::normalize(forbiddingKickAngle.min + theRobotPose.rotation), Angle::normalize(forbiddingKickAngle.max + theRobotPose.rotation));
      Angle minExtra = std::min(Angle::normalize(forbiddingKickAngleExtra.min + theRobotPose.rotation), Angle::normalize(forbiddingKickAngleExtra.max + theRobotPose.rotation));
      Angle maxExtra = std::max(Angle::normalize(forbiddingKickAngleExtra.min + theRobotPose.rotation), Angle::normalize(forbiddingKickAngleExtra.max + theRobotPose.rotation));
      wheel.addSector(Rangea(std::min(minNormal, minExtra), std::max(maxNormal, maxExtra)), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
      sectors = wheel.finish();
      DRAW_SECTOR_WHEEL("skill:Zweikampf:sidewardRange", sectors, theFieldBall.positionOnField);
    }

    TargetType highestPriority = TargetType::numOfTargetTypes;
    const float nearOwnGoalScaling = (1.f - Rangef::ZeroOneRange().limit((theFieldBall.endPositionOnField.x() - theFieldDimensions.xPosOwnGoalArea) / (theFieldDimensions.xPosOwnPenaltyArea + 500.f - theFieldDimensions.xPosOwnGoalArea)));

    // 8. Check every kick angle with every kick
    for(const Angle& kickAngle : directionPossibilities)
    {
      // 8.1. Get max allowed kick range, before hitting an obstacle or the goal
      float maxKickRange = std::numeric_limits<float>::max();
      bool isGoalAngle = false;
      for(const SectorWheel::Sector& sector : kickAngles)
      {
        if(sector.angleRange.isInside(kickAngle))
        {
          if(sector.type == SectorWheel::Sector::goal && sector.angleRange.getSize() > searchParameters.goalSectorWidth)
            isGoalAngle = true;
          else if(sector.type == SectorWheel::Sector::obstacle)
            maxKickRange = sector.distance;
          break;
        }
      }

      // 8.2. Calculate distance for out of field, if in opponent half
      float distanceToFieldBorderSquared = std::numeric_limits<float>::max();

      if(!isGoalAngle)
      {
        Vector2f i1(0.f, 0.f);
        Vector2f i2(0.f, 0.f);
        VERIFY(Geometry::getIntersectionPointsOfLineAndRectangle(-opponentHalfFrontLeft, opponentHalfFrontLeft, Geometry::Line(useBallPosition, Vector2f(1.f, 0.f).rotated(kickAngle + theRobotPose.rotation)), i1, i2));
        const Vector2f ballToIntersection1 = i1 - useBallPosition;
        const Vector2f ballToIntersection2 = i2 - useBallPosition;
        // the 5_degs is just a arbitrary number, because the angle to the intersection point is only for a small fraction different to the original kickAngle, resulting from rounding erros.
        if(std::abs(ballToIntersection1.angle() - (kickAngle + theRobotPose.rotation)) < 5_deg)
          distanceToFieldBorderSquared = ballToIntersection1.squaredNorm() - sqr(100.f); // 100.f safe distance
        else if(std::abs(ballToIntersection2.angle() - (kickAngle + theRobotPose.rotation)) < 5_deg)
          distanceToFieldBorderSquared = ballToIntersection2.squaredNorm() - sqr(100.f); // 100.f safe distance
      }

      // 8.2.1 also calculate goal kick range
      // get the range for a goal kick
      // otherwise a long kick might overshoot the goal and gets a really bad rating
      float rangeForGoal = std::numeric_limits<float>::max();
      if(isGoalAngle && rangeForGoal == std::numeric_limits<float>::max())   // only when goal kick and only check once
      {
        Vector2f intersectionPoint;
        VERIFY(Geometry::getIntersectionOfLines(Geometry::Line(useBallPosition, Vector2f::polar(1.f, kickAngle + theRobotPose.rotation)), Geometry::Line(Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoal), Vector2f(0.f, 1.f)), intersectionPoint));
        rangeForGoal = (intersectionPoint - useBallPosition).norm() + 0.01f;
      }

      // Check if the kickAngle is inside the side area, used to modify the rating from the potential field
      const bool isInsideStealBallRange = stealBallMin.contains(kickAngle) || stealBallMax.contains(kickAngle);
      const bool isTypeStealBall = (stealBallMin != stealBallMax &&
                                    (isInsideStealBallRange ||  // normal case, kickAngle is/is not in range. Special case: robot is so much rotated, that the ranges are not inside the -180_deg 180_deg range
                                     (!boundaryCheck.isInside(stealBallMin.min) && (stealBallMin.contains(Angle(kickAngle + 360_deg)) || stealBallMin.contains(Angle(kickAngle - 360_deg)))) ||
                                     (!boundaryCheck.isInside(stealBallMax.max) && (stealBallMax.contains(Angle(kickAngle + 360_deg)) || stealBallMax.contains(Angle(kickAngle - 360_deg))))));

      // 8.3. calculate the fieldRating once before hand to save computation time
      std::vector<RatingMap> rangeRatingMapping;
      const Vector2f direction = Vector2f::polar(1.f, kickAngle + theRobotPose.rotation);
      for(const float range : checkKickDistancesForFR)
      {
        if(range > maxKickRange || sqr(range) > distanceToFieldBorderSquared) // ever range above will not get executed anyway
          break;
        const Vector2f endPoint = useBallPosition + direction * std::min(range, rangeForGoal);
        // normal rating
        bool isTeammatePass = false;
        PotentialValue teammate;
        PotentialValue pvBallNear;
        PotentialValue otherSide;

        // get most ratings, with and without obstacle potential
        PotentialValue ratingField = theFieldRating.potentialFieldOnly(endPoint.x(), endPoint.y(), false);
        if(!isTypeStealBall)
        {
          theFieldRating.getObstaclePotential(ratingField, endPoint.x(), endPoint.y(), false);
          theFieldRating.duelBallNearPotential(pvBallNear, endPoint.x(), endPoint.y(), false);
        }
        theFieldRating.potentialOverall(teammate, endPoint.x(), endPoint.y(), isTeammatePass, false, theSkillRequestPose.passTarget);
        theFieldRating.removeBallNearFromTeammatePotential(teammate, pvBallNear);
        ratingField += teammate;
        ratingField += pvBallNear; // add ball near potentiall after the teammates potential
        ratingField += otherSide;

        rangeRatingMapping.emplace_back();
        rangeRatingMapping.back().range = range;
        rangeRatingMapping.back().rating = ratingField.value;
        rangeRatingMapping.back().isTeammatePass = isTeammatePass;
      }
      if(rangeRatingMapping.size() == 0)
        continue;

      // 8.4. check every InWalkKick
      for(KickInfo::KickType kickType : allowedKicks)
      {
        // side kick is not allowed
        if((kickType == KickInfo::walkSidewardsLeftFootToLeft || kickType == KickInfo::walkSidewardsRightFootToRight) &&
           ((opponentAndSelfDistanceToBallDiff < 100.f) ||
            !(forbiddingKickAngle.isInside(kickAngle) || forbiddingKickAngleExtra.isInside(kickAngle))))
          continue;

        // forwardsteal is not allowed
        if((kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight) &&
           (stealBallMin != stealBallMax || // we are not on front of the opponent, therefore the walkSteal kick would not work in time
            !leftAngleRangeBallToOpponent.isInside(angleFromBallToOpponent) || // only if the opponent is standing behind the ball
            !(rightForwardStealRange.isInside(Angle::normalize(kickAngle + theRobotPose.rotation)) || // only for kicks going to the side
              leftForwardStealRange.isInside(Angle::normalize(kickAngle + theRobotPose.rotation))) || // only for kicks going to the side
            opponentAndSelfDistanceToBallDiff > obstacleHandling.maxObstacleDistanceForWalkStealBallKick)) // only if the opponent is closer to the ball
          continue;
        DuelPose pose;
        Vector2f endpoint;
        const bool possible = getDuelRating(kickType, kickAngle, pose,
                                            endpoint, minMaxAngle, isGoalAngle,
                                            maxKickRange, distanceToFieldBorderSquared,
                                            stealBallMin, stealBallMax,
                                            angleFromBallToOpponent,
                                            rangeRatingMapping, -1.f, rangeForGoal,
                                            angleFromGoalToBall, isTypeStealBall, highestPriority,
                                            nearOwnGoalScaling, forbiddingKickAngle);
        if(!possible)
          continue;
        duelPoses[pose.type].push_back(pose);
        highestPriority = std::min(highestPriority, pose.type);
      }
    }
    // Find duelPose list with highest priority. GoalShots > StealBall > Pass > Others
    std::vector<DuelPose>* toBeCheckDuelPoses = nullptr;
    for(auto& vec : duelPoses)
    {
      if(!vec.empty())
      {
        toBeCheckDuelPoses = &vec;
        break;
      }
    }
    if(toBeCheckDuelPoses)
    {
      DuelPose* bestPose = &(toBeCheckDuelPoses->front());
      // 9. Get the best kick
      // TODO use a std::map, with ENUM(GoalShots, ForwardSteal, Otherwise)
      // Only needs to check for the highest priority
      // Can skip lower ones already when still checking for kick angles! Can safe a lot of computation
      for(DuelPose& pose : *toBeCheckDuelPoses)
      {
        if(pose.rating < bestPose->rating) // Better rating is accepted
          bestPose = &pose;
      }
      theDuelPose = *bestPose;

      COMPLEX_DRAWING("skill:Zweikampf:kicks")
        drawRating(*toBeCheckDuelPoses);

      // in case of a goal kick, always use the forwardLong
      if(replaceForwardWithLongGoalShot && theDuelPose.type == TargetType::goalShot)
      {
        if(std::abs(theDuelPose.kickAngle) < replaceNormalToLongKickGoalShot && (theDuelPose.kickType == KickInfo::walkForwardsLeft || theDuelPose.kickType == KickInfo::walkForwardsRight))
          theDuelPose.kickType = theDuelPose.kickType == KickInfo::walkForwardsLeft ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
        else if((theDuelPose.kickType == KickInfo::walkForwardsLeftAlternative || theDuelPose.kickType == KickInfo::walkForwardsRightAlternative))
          theDuelPose.kickType = theDuelPose.kickType == KickInfo::walkForwardsLeftAlternative ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
      }
      theDuelPose.precision = Rangea(0_deg, 0_deg);

      timeSinceDoingNothing = theFrameInfo.time;
      if(theKickInfo[theDuelPose.kickType].walkKickType == WalkKicks::forwardSteal)
        lastForwardSteal = theFrameInfo.time;

      // 11.2 filter the sector wheel and ignore all obstacles, that are further away than the kick range
      const SectorWheel::Sector* sectorRef = nullptr;
      SectorWheel filteredWheel;
      filteredWheel.begin(theFieldBall.positionOnField);
      std::vector<Rangea> goalSectors;
      for(const SectorWheel::Sector& sector : kickAngles)
      {
        if((sector.type != SectorWheel::Sector::goal  && (sector.type != SectorWheel::Sector::obstacle || sector.distance - 500.f < theDuelPose.range)) ||
           (sector.type == SectorWheel::Sector::goal && theDuelPose.type == TargetType::goalShot)) // only add goal sector if we have a goalShot. Otherwise the goal sector does not matter
          filteredWheel.addSector(sector.angleRange, sector.distance, sector.type);
        if(sector.type == SectorWheel::Sector::goal)
          goalSectors.push_back(sector.angleRange);
      }
      std::list<SectorWheel::Sector> filteredSectors = filteredWheel.finish();

      for(const SectorWheel::Sector& sector : filteredSectors)
      {
        if(sector.angleRange.isInside(theDuelPose.kickAngle))
        {
          sectorRef = &sector;
          break;
        }
      }
      Rangea precision(0_deg, 0_deg);
      // 11.3. Calculate the precision range
      if(sectorRef != nullptr)
      {
        precision = sectorRef->angleRange;
        Rangea borderToMinMax = precision;

        borderToMinMax.min = borderToMinMax.min + 5_deg;
        borderToMinMax.max = borderToMinMax.max - 5_deg;

        if(precision.max < precision.min)
        {
          if(theDuelPose.kickAngle > precision.min)
            borderToMinMax.max = 180_deg;
          else
            borderToMinMax.min = -180_deg;
        }
        borderToMinMax = Rangea(std::min(theDuelPose.kickAngle, borderToMinMax.min),
                                std::max(theDuelPose.kickAngle, borderToMinMax.max));
        const Angle searchOffset = theKickInfo[theDuelPose.kickType].walkKickType != WalkKicks::forwardSteal && soonCloseRangeDuel ? 90_deg : 5_deg;
        precision.max = std::max(0.f, borderToMinMax.limit(theDuelPose.kickAngle + searchOffset) - theDuelPose.kickAngle);
        precision.min = std::min(0.f, borderToMinMax.limit(theDuelPose.kickAngle - searchOffset) - theDuelPose.kickAngle);
      }

      calculateSectorUntilFieldBorder(precision, theDuelPose.kickAngle, theDuelPose.range, theFieldBall.endPositionOnField, goalSectors);
      theDuelPose.precision = precision;
    }
    else
    {
      // 12. no kick was calculated for a longer period of time. Therefore just stand between the ball and the own goal, to make the life for the opponent hard
      if((theKickInfo[theDuelPose.kickType].walkKickType == WalkKicks::forwardSteal && theFrameInfo.getTimeSince(timeSinceDoingNothing) > duelTimings.noKickStealMinTime) ||
         (theKickInfo[theDuelPose.kickType].walkKickType != WalkKicks::forwardSteal && theFrameInfo.getTimeSince(timeSinceDoingNothing) > duelTimings.noKickMinTime))
      {
        DuelPose noPose;
        const Angle rotation = (theFieldBall.positionOnField - Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f)).angle();
        noPose.pose = theRobotPose.inversePose * Pose2f(rotation, Vector2f::polar(-obstacleHandling.noKickBlockingPose, rotation) + theFieldBall.positionOnField);
        noPose.noKick = true;
        theDuelPose = noPose;
        theDuelPose.kickAngle = -theRobotPose.rotation;
      }
    }

    // 13.1. The perception of the obstacle might just be bad, so keep the forwardSteal as long as the new decision is not a goal kick or pass
    if(theKickInfo[lastDuelPose.kickType].walkKickType == WalkKicks::forwardSteal && theKickInfo[theDuelPose.kickType].walkKickType != WalkKicks::forwardSteal &&
       theFrameInfo.getTimeSince(lastForwardSteal) < 500 && theDuelPose.type != TargetType::goalShot && forceForwardSteal)
    {
      const Angle plannedKickAngle = theDuelPose.kickAngle;
      theDuelPose = lastDuelPose;  // use the steal kick that matches the new kick angle the most
      theDuelPose.kickType = plannedKickAngle > 0_deg ? KickInfo::walkForwardStealBallRight : KickInfo::walkForwardStealBallLeft;
      theDuelPose.noKick = false;
      if(theDuelPose.kickType != lastDuelPose.kickType)
        theDuelPose.kickAngle = Angle::normalize(theDuelPose.kickAngle + 180_deg);
      theDuelPose.precision = Rangea(-5_deg, 5_deg);
    }
    if(theKickInfo[theDuelPose.kickType].walkKickType == WalkKicks::forwardSteal && (theFieldBall.endPositionRelative - duelObstacle.center).squaredNorm() < sqr(obstacleHandling.maxObstacleDistanceToForceForwardSteal))
      forceForwardSteal = true;

    if(theKickInfo[theDuelPose.kickType].walkKickType != WalkKicks::forwardSteal)
      forceForwardSteal = false;
  }

  void drawRating(std::vector<DuelPose>& duelPoses)
  {
    minMaxDrawRatings = Rangef(-1.f, 1.f); // Reset
    for(DuelPose& pose : duelPoses)
    {
      minMaxDrawRatings.min = std::min(pose.rating, minMaxDrawRatings.min);
      minMaxDrawRatings.max = std::max(pose.rating, minMaxDrawRatings.max);
    }
    std::vector<float> ranges;
    DuelPose* lastUniqueKick = nullptr;
    const float minShift = (minMaxDrawRatings.min + minMaxDrawRatings.max) / 2.f;
    const float ratingRange = (minMaxDrawRatings.max - minMaxDrawRatings.min) / 2.f;

    for(DuelPose& pose : duelPoses)
    {
      // do not over draw better ratings
      if(!lastUniqueKick || lastUniqueKick->kickAngle != pose.kickAngle)
      {
        lastUniqueKick = &pose;
        ranges.clear();
      }
      if(lastUniqueKick->rating >= pose.rating || !(std::find(ranges.begin(), ranges.end(), pose.endPoint.squaredNorm()) != ranges.end()))
      {
        ranges.emplace_back(pose.endPoint.squaredNorm());
        lastUniqueKick = &pose;

        ColorRGBA color(0, 0, 0, 0);

        const float useRating = (pose.rating - minShift) / ratingRange;
        if(useRating >= 0.f)
        {
          color.r = static_cast<unsigned char>(useRating * 255);
          color.b = static_cast<unsigned char>((1.f - useRating) * 255);
          color.g = 0;
          color.a = 100;
        }
        else
        {
          color.r = 0;
          color.b = static_cast<unsigned char>(useRating * -1.f * 255);
          color.g = static_cast<unsigned char>((1.f - useRating * -1.f) * 255);
          color.a = 100;
        }

        FILLED_RECTANGLE("skill:Zweikampf:kicks",
                         pose.endPoint.x() - 10.f, pose.endPoint.y() - 10.f,
                         pose.endPoint.x() + 10.f, pose.endPoint.y() + 10.f,
                         1, Drawings::noPen, ColorRGBA(), Drawings::solidBrush, color);
      }
    }
  }

  void calculateSectorWheel()
  {
    std::vector<ObstacleSector> obstacleSectors;
    Vector2f dummyOB(1.f, 0.f);
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
    {
      // Do not ignore goal posts, to ensure not kicking into them?
      // if(obstacle.type == Obstacle::goalpost)
      //  continue;

      Vector2f useObstacleCenter = obstacle.center;
      bool filterSector = false;
      shiftObstacleBackward(useObstacleCenter, dummyOB);
      filterSector = useObstacleCenter != obstacle.center;

      if(!filterSector && useObstacleCenter == duelObstacle.center)
      {
        Vector2f disToBall = useObstacleCenter - theFieldBall.positionRelative;
        const float normSquared = disToBall.squaredNorm();
        const float ownDistanceToBallSqaured = theFieldBall.positionRelative.squaredNorm();
        if(normSquared > sqr(300.f))
          useObstacleCenter = theFieldBall.positionRelative + disToBall / (std::sqrt(normSquared)) * std::max(300.f, std::sqrt(normSquared - ownDistanceToBallSqaured));
      }

      // clip obstacle behind the ball
      const Vector2f obstacleOnField = theRobotPose * useObstacleCenter;
      if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGroundLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
        continue;

      const float width = (obstacle.left - obstacle.right).norm() + sectorWheelObstacleBallRadiusFactor * theBallSpecification.radius;
      const float distance = (obstacleOnField - theFieldBall.positionOnField).norm();

      // TODO make it less ugly
      // In case the obstacle is "inside" the ball, the sector needs to be filtered. otherwise the possible kick angles change heavily every frame...
      const float radius = std::atan(width / (2.f * distance));
      const Angle direction = (obstacleOnField - theFieldBall.positionOnField).angle();
      obstacleSectors.emplace_back();
      Rangea obstacleSectorRange = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
      if(filterSector &&
         ((std::abs(obstacleSectorRange.min) > 90_deg && std::abs(obstacleSectorRange.max) < 90_deg) ||
          (std::abs(obstacleSectorRange.max) > 90_deg && std::abs(obstacleSectorRange.min) < 90_deg)))
      {
        const Rangea clippedRange(-90_deg, 90_deg);
        obstacleSectorRange.min = clippedRange.limit(obstacleSectorRange.min) ;
        obstacleSectorRange.max = clippedRange.limit(obstacleSectorRange.max);
      }
      obstacleSectorRange.min -= theRobotPose.rotation;
      obstacleSectorRange.max -= theRobotPose.rotation;
      obstacleSectorRange.max = Angle::normalize(obstacleSectorRange.max);
      obstacleSectorRange.min = Angle::normalize(obstacleSectorRange.min);
      obstacleSectors.back().sector = obstacleSectorRange;
      obstacleSectors.back().distance = distance;
      obstacleSectors.back().x = obstacleOnField.x();
    }

    SectorWheel drawWheel;
    SectorWheel wheel;
    std::list<SectorWheel::Sector> sectors;

    wheel.begin(theFieldBall.positionOnField);
    const float minBallGoalPostOffsetSquared = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
    const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffsetSquared / (leftGoalPost - theFieldBall.positionOnField).norm()));
    const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffsetSquared / (rightGoalPost - theFieldBall.positionOnField).norm()));
    const Angle angleToLeftPost = Angle::normalize((leftGoalPost - theFieldBall.positionOnField).angle() - leftAngleOffset - theRobotPose.rotation - 2_deg);
    const Angle angleToRightPost = Angle::normalize((rightGoalPost - theFieldBall.positionOnField).angle() + rightAngleOffset - theRobotPose.rotation + 2_deg);

    if(angleToLeftPost > angleToRightPost)
      wheel.addSector(Rangea(angleToRightPost, angleToLeftPost), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);

    for(const ObstacleSector& obstacleSector : obstacleSectors)
      wheel.addSector(obstacleSector.sector, obstacleSector.distance, SectorWheel::Sector::obstacle);
    kickAngles = wheel.finish();

    COMPLEX_DRAWING("skill:Zweikampf:wheel")
    {
      drawWheel.begin(theFieldBall.positionOnField);
      for(const SectorWheel::Sector& sector : kickAngles)
        drawWheel.addSector(Rangea(Angle::normalize(sector.angleRange.min + theRobotPose.rotation), Angle::normalize(sector.angleRange.max + theRobotPose.rotation)), sector.distance, sector.type);
      sectors = drawWheel.finish();
      DRAW_SECTOR_WHEEL("skill:Zweikampf:wheel", sectors, theFieldBall.positionOnField);
    }
  }

  void calculateSectorUntilFieldBorder(Rangea& precision, const Angle kickAngle, const float range,
                                       const Vector2f& fieldPoint, std::vector<Rangea>& goalSectors)
  {
    const float rangeWithOffset = range + fieldBorderSafeDistance;
    Vector2f fieldPointClipped = fieldPoint;
    fieldPointClipped.x() = Rangef(-opponentHalfFrontLeft.x() + 1.f, opponentHalfFrontLeft.x() - 1.f).limit(fieldPointClipped.x());
    fieldPointClipped.y() = Rangef(-opponentHalfFrontLeft.y() + 1.f, opponentHalfFrontLeft.y() - 1.f).limit(fieldPointClipped.y());
    Angle min = Angle::normalize(kickAngle + precision.min + theRobotPose.rotation);
    Angle max = Angle::normalize(kickAngle + precision.max + theRobotPose.rotation);
    const Angle kickAngleInField = Angle::normalize(kickAngle + theRobotPose.rotation);

    const std::vector<Vector2f> edges = { {4500.f, 0.f}, {0.f, 3000.f}, {-4500.f, 0.f}, {0.f, -3000.f} };
    Vector2f i1(0.f, 0.f);
    Vector2f i2(0.f, 0.f);
    VERIFY(Geometry::getIntersectionPointsOfLineAndRectangle(-opponentHalfFrontLeft, opponentHalfFrontLeft, Geometry::Line(fieldPointClipped, Vector2f(1.f, 0.f).rotated(kickAngleInField)), i1, i2));
    const Vector2f ballToIntersection2 = i2 - fieldPoint;
    Vector2f& intersection = i1;
    // the 5_degs is just a arbitrary number, because the angle to the intersection point is only for a small fraction different to the original kickAngle, resulting from rounding erros.
    if(std::abs(ballToIntersection2.angle() - (kickAngle + theRobotPose.rotation)) < 5_deg)
      intersection = i2;

    auto it = edges.end();
    for(auto otherIt = edges.begin(); otherIt != edges.end(); otherIt++)
    {
      if(otherIt->x() == intersection.x() || otherIt->y() == intersection.y())
      {
        it = otherIt;
        break;
      }
    }
    if(it == edges.end())
      return;

    auto getIntersectionAngle = [](const Vector2f& fieldPoint, auto & it, Angle& angle, const float length, const Angle originalTarget)
    {
      const float adjacent = std::abs(it->x() != 0.f ? fieldPoint.x() - it->x() : fieldPoint.y() - it->y());
      float sign = 1.f;
      Angle offset = 0_deg;
      if(Approx::isEqual(it->x(), -4500.f, 0.1f))
        offset = -180_deg;
      else if(Approx::isEqual(it->y(), -3000.f, 0.1f))
        offset = -90_deg;
      else if(Approx::isEqual(it->y(), 3000.f, 0.1f))
        offset = 90_deg;

      if(angle > originalTarget)
      {
        if(offset < originalTarget)
          offset += 360_deg;
      }
      else if(angle < originalTarget)
      {
        if(offset > originalTarget)
          offset -= 360_deg;
      }

      if(adjacent > length)
        return !((angle > originalTarget && angle < offset) || (angle < originalTarget && angle > offset));

      if(offset > originalTarget)
        sign = -1.f;

      Angle cosA = offset + sign * std::acos(adjacent / length);
      if(angle >= originalTarget)
        angle = std::min(angle, cosA);
      else
        angle = std::max(angle, cosA);

      angle = Angle::normalize(angle);
      return false;
    };

    if(precision.min < 0)
    {
      bool isGoalAngle = false;
      for(const auto& sec : goalSectors)
      {
        isGoalAngle |= sec.isInside(precision.min + kickAngle);
      }

      if(!isGoalAngle)
      {
        int counter = 0;
        auto itCopy = it;
        bool searching = true;
        while(searching && counter < 4)
        {
          searching = getIntersectionAngle(fieldPointClipped, itCopy, min, rangeWithOffset, kickAngleInField);
          if(itCopy == edges.begin())
            itCopy = --(edges.end());
          else
            itCopy--;
          counter++;
        }
      }
    }
    if(precision.max > 0)
    {
      bool isGoalAngle = false;
      for(const auto& sec : goalSectors)
      {
        isGoalAngle |= sec.isInside(precision.max + kickAngle);
      }
      if(!isGoalAngle)
      {
        int counter = 0;
        auto itCopy = it;
        bool searching = true;
        while(searching && counter < 4)
        {
          searching = getIntersectionAngle(fieldPointClipped, itCopy, max, rangeWithOffset, kickAngleInField);
          itCopy++;
          if(itCopy == edges.end())
            itCopy = edges.begin();
          counter++;
        }
      }
    }

    precision.min = std::min(0.f, Angle::normalize(min - kickAngle - theRobotPose.rotation));
    precision.max = std::max(0.f, Angle::normalize(max - kickAngle - theRobotPose.rotation));
  }

  bool getDuelRating(const KickInfo::KickType kickType, const Angle kickAngle, DuelPose& duelPose, Vector2f& endPoint,
                     const Angle minMaxAngle, const bool isGoalAngle, const float maxKickRange, const float distanceToOutOfFieldSquared,
                     const Rangea& stealBallBonusMin, const Rangea& stealBallBonusMax, const Angle angleFromBallToOpponent,
                     const std::vector<RatingMap>& rangeRatingMapping, const float useRange, const float rangeForGoal,
                     const Angle angleFromGoalToBall, const bool isTypeStealBall, const TargetType highestTargetType,
                     const float nearOwnGoalScaling, const Rangea& forbiddingKickAngle)
  {
    Angle rotationOffset(theKickInfo[kickType].rotationOffset);
    Vector2f ballOffset(theKickInfo[kickType].ballOffset);
    Rangef useKickRange = theKickInfo[kickType].range;

    DuelPose pose;
    // 8.4.1. Interpolate between forward kick and turnOut kick
    if(kickType == KickInfo::KickType::walkForwardsLeft ||
       kickType == KickInfo::KickType::walkForwardsRight)
    {
      const Angle useMaxKickAngle = theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset;
      pose.turnKickAllowed = stealBallBonusMin == stealBallBonusMax;
      const bool isLeft = kickType == KickInfo::KickType::walkForwardsLeft;
      const Rangea angleClip(!isLeft ? 0_deg : -useMaxKickAngle, !isLeft ? useMaxKickAngle : 0_deg);
      const float interpolation = Rangef::ZeroOneRange().limit(angleClip.limit(kickAngle) / -theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].rotationOffset);
      ballOffset = (1 - interpolation) * theKickInfo[kickType].ballOffset + interpolation * theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].ballOffset;
      rotationOffset = -angleClip.limit(kickAngle);
      useKickRange.min = (1 - interpolation) * theKickInfo[kickType].range.min + interpolation * theKickInfo[KickInfo::walkTurnRightFootToLeft].range.min;
      useKickRange.max = (1 - interpolation) * theKickInfo[kickType].range.max + interpolation * theKickInfo[KickInfo::walkTurnRightFootToLeft].range.max;
    }

    // 8.4.2. we need to know already at the start, if the kick is a stealBall type
    if(isTypeStealBall || kickType == KickInfo::KickType::walkSidewardsLeftFootToLeft || kickType == KickInfo::KickType::walkSidewardsRightFootToRight)
      pose.rating += duelRatings.ratingStealBall;

    // 8.4.6. Calculate kick pose
    if(kickType == KickInfo::walkForwardsLeftLong || kickType == KickInfo::walkForwardsRightLong ||
       kickType == KickInfo::walkForwardsLeftAlternative || kickType == KickInfo::walkForwardsRightAlternative)
      rotationOffset = 0_deg; // to ensure the faster one is chosen
    pose.noKick = false;
    pose.kickType = kickType;
    pose.kickAngle = kickAngle;
    pose.pose = (Pose2f(kickAngle + theRobotPose.rotation, theFieldBall.positionOnField) - theRobotPose)
                .rotate(rotationOffset)
                .translate(ballOffset);
    pose.pose.rotation.normalize();

    // 8.4.4. kick range is too long and would hit an opponent
    if(useKickRange.min > maxKickRange)
      return false;

    // 8.4.7. The robot needs to turn too much
    Angle maxAllowedRotation = minMaxAngle;
    if(isTypeStealBall || kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight)
      maxAllowedRotation = std::max(minMaxAngle, stealBallParameters.stealBallTypeTooMuchRotationMinValue);
    if(theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight)
      maxAllowedRotation += sidewardsRotationCorrectionBonus;
    if((kickType == KickInfo::walkForwardsLeftLong || kickType == KickInfo::walkForwardsRightLong) && forbiddingKickAngle.isInside(kickAngle))
      maxAllowedRotation += stealBallParameters.stealBallTypeTooMuchRotationMinValue * nearOwnGoalScaling;
    if(std::abs(pose.pose.rotation) > maxAllowedRotation && // too much rotation needed
       !(isGoalAngle && rangeForGoal < useKickRange.max) // we risk losing the ball to score a goal
       && !(kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight)) // forwardSteal can always be done
      return false;

    // 8.4.8. for the forwardsteal, we are standing on the wrong side
    if(kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight)
    {
      Rangea positionRange(angleFromGoalToBall - 90_deg, angleFromGoalToBall + 90_deg);
      if(!positionRange.isInside((theFieldBall.positionOnField - (theRobotPose + pose.pose.translation).translation).angle())) // calculate in field coordinates. this is wrong
        return false;
    }

    // 8.4.3. Get PotentialField rating, so we know the kick range
    // TODO endpoint is calculated twice in some situations...
    if(kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight || rangeRatingMapping.size() == 0)  // Mostly executed when calculating the precision range
    {
      pose.range = useRange > 0.f ? useKickRange.limit(useRange) : useKickRange.max;
      endPoint = theFieldBall.positionOnField + Vector2f::polar(pose.range, kickAngle + theRobotPose.rotation);
      const bool isForwardBallSteal = kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight || pose.type == TargetType::stealBall;
      bool isTeammatePass = false;
      PotentialValue teammate;
      PotentialValue pvBallNear;
      PotentialValue pv = theFieldRating.potentialFieldOnly(endPoint.x(), endPoint.y(), false);
      if(!isForwardBallSteal)
      {
        theFieldRating.getObstaclePotential(pv, endPoint.x(), endPoint.y(), false);
      }
      // TODO why not use pv instead of pvBallNear/teammate?
      theFieldRating.duelBallNearPotential(pvBallNear, endPoint.x(), endPoint.y(), false);
      theFieldRating.potentialOverall(teammate, endPoint.x(), endPoint.y(), isTeammatePass, false, theSkillRequest.passTarget);
      theFieldRating.removeBallNearFromTeammatePotential(teammate, pvBallNear);
      pv += teammate;
      pv += pvBallNear; // add ball near potentiall after the teammates potential
      pose.rating += pv.value;
      pose.type = TargetType::stealBall;
    }
    else
    {
      size_t bestIndex = static_cast<size_t>(-1);
      float bestRating = std::numeric_limits<float>::max();
      bool useStrongForwardKick = ((kickType == KickInfo::KickType::walkForwardsLeft || kickType == KickInfo::KickType::walkForwardsRight) &&
                                   !Geometry::isPointInsideRectangle(opponentGoalArea, useBallPosition));
      for(size_t index = 0; index < rangeRatingMapping.size(); index++)
      {
        if(sqr(rangeRatingMapping[index].range + fieldBorderSafeDistance) > distanceToOutOfFieldSquared || rangeRatingMapping[index].range > useKickRange.max) // kick range is too long
          break;
        if(rangeRatingMapping[index].rating < bestRating &&   // a "<=" would result in using alway a longer kick than a shorter one. TODO eval what is better
           rangeRatingMapping[index].range >= useKickRange.min && // kick range executable checks
           !(useStrongForwardKick && rangeRatingMapping[index].range < minForwardTurnRange && index + 1 < rangeRatingMapping.size())) // special case checks, to ensure a minimal strong kick

        {
          bestIndex = index;
          bestRating = (rangeRatingMapping[index].rating + (pose.type == TargetType::stealBall || (isGoalAngle && useKickRange.max >= rangeForGoal) ? 0.01f : 0.f)); // + 0.01f makes sure, the strongest kick is used for stealing the ball
        }
      }
      if(bestIndex == static_cast<size_t>(-1))
        return false;
      pose.range = rangeRatingMapping[bestIndex].range;
      pose.rating += bestRating;
      pose.type = rangeRatingMapping[bestIndex].isTeammatePass ? TargetType::pass : pose.type;
    }

    if(isTypeStealBall && pose.type != TargetType::pass)
      return false;

    // 8.4.9. get the end point and check, if we can score a goal
    endPoint = Vector2f(0.f, 0.f);
    Vector2f intersectionPoint(0.f, 0.f);
    if(isGoalAngle && // checks, if kick distance is long enough to reach the goal
       rangeForGoal < useKickRange.max)
    {
      pose.rating += duelRatings.ratingGoalShot;
      // if we can hit the goal with the forwardLong without much rotation adjustment, we always prefer the forwardLong.
      if(std::abs(pose.pose.rotation) < 5_deg && (kickType == KickInfo::walkForwardsLeftLong || kickType == KickInfo::walkForwardsRightLong))
        pose.rating -= 0.1f;
      endPoint = theFieldBall.positionOnField + Vector2f::polar(rangeForGoal, kickAngle + theRobotPose.rotation);
      pose.range = useKickRange.max;
      pose.type = TargetType::goalShot;
    }
    else
      endPoint = theFieldBall.positionOnField + Vector2f::polar(pose.range, kickAngle + theRobotPose.rotation);

    if(highestTargetType == TargetType::goalShot && pose.type > highestTargetType)
      return false;

    // 8.4.10. Own penalty area checks
    // 8.4.10.1. we would kick the ball into our own penalty area, which we do not want
    if(!Geometry::isPointInsideRectangle(penaltyRect, useBallPosition) && Geometry::isPointInsideRectangle(penaltyRect, endPoint))
      return false;

    // 8.4.10.3. we do no want to accidentally kick the ball into a good position for the opponent
    if(kickType != KickInfo::walkForwardStealBallLeft && kickType != KickInfo::walkForwardStealBallRight && endPoint.x() < theFieldDimensions.xPosOwnPenaltyMark && useBallPosition.x() > endPoint.x())
      return false;

    // 8.4.7.2 worse rating for too much needed rotation
    // This is checked here, because after 8.4.7 some values needed to be updated before this calculation
    pose.rating += std::max(0.f, std::abs(pose.pose.rotation) - minMaxAngle) / minMaxAngle * duelRatings.ratingMinMaxDifference * (pose.type == TargetType::stealBall || pose.type == TargetType::goalShot ? 0.3f : 1.f); // too much rotation needed, but worth it for a goal shot

    // 8.4.11. ball will land in own half as a back kick
    if(endPoint.x() < theFieldBall.positionOnField.x() && endPoint.x() < 0.f)
    {
      pose.rating += duelRatings.ratingBallLandsInOwnHalf;
      pose.range = pose.type == TargetType::other ? useKickRange.min : pose.range; // the rating will be off, but the ball is only kicked for a small distance
    }

    // 8.4.12 for the forwarsteal, make sure the better side is chosen, also the ball shall not come too close to the side line
    if(kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight)
    {
      pose.rating += duelRatings.ratingStealBallKick;
      pose.type = TargetType::stealBall;
      float outOfFieldMalus = std::min(0.f, std::abs(std::abs(endPoint.y()) - (theFieldDimensions.yPosLeftSideline - 500.f)) / 500.f);
      if(outOfFieldMalus == 0) // ball does not come close, add a bonus for the kick, that has a highe success chance
      {
        if((angleFromBallToOpponent > forwardStealPreferenceRange && kickType == KickInfo::walkForwardStealBallLeft) ||
           (angleFromBallToOpponent < -forwardStealPreferenceRange && kickType == KickInfo::walkForwardStealBallRight))
          pose.rating += duelRatings.ratingStealBallKickBetterSide;
      }
      else
        pose.rating -= duelRatings.ratingStealBallKickBetterSide * outOfFieldMalus;
    }

    if(pose.type > highestTargetType)
      return false;

    if(kickType == KickInfo::walkForwardsRightLong || kickType == KickInfo::walkForwardsLeftLong)
      pose.rating += nearOwnGoalScaling * duelRatings.goalAreaLongKickRating;

    /////////////////////////////
    // 8.4.13. Fall Prevention //
    // //////////////////////////
    // an obstacle would prevent that the robot can reach this kick pose. To prevent walking on the opponents feet or not reaching the kick pose in time,
    // the kicks rating gets worsens in case an obstacle is only on one side
    switch(kickType)
    {
      case KickInfo::walkForwardsRight:
      case KickInfo::walkForwardsLeft:
      case KickInfo::walkForwardsRightLong:
      case KickInfo::walkForwardsLeftLong:
      case KickInfo::walkForwardsRightAlternative:
      case KickInfo::walkForwardsLeftAlternative:
      {
        if(std::abs(kickAngle) > 25_deg) // otherwise execute default code
          break;
      }
      default:
      {
        // For the other walk kicks, simply check if the duel obstacle is blocking the pose. If so, the rating should be greatly worsen
        // In case this kick is still the only executeable one, the worse rating does not matter
        Vector2f intersection1(0.f, 0.f);
        Vector2f intersection2(0.f, 0.f);
        const float distance = pose.pose.translation.norm();
        const float squaredDistance = sqr(distance + (kickType == KickInfo::walkSidewardsLeftFootToLeft || kickType == KickInfo::walkSidewardsRightFootToRight ? obstacleHandling.obstacleBlockingRadiusSidewardsIncrease : 0.f));
        const int intersections = Geometry::getIntersectionOfLineAndCircle(Geometry::Line(Vector2f(0.f, 0.f), pose.pose.translation / distance),
                                  Geometry::Circle(duelObstacle.center, obstacleHandling.obstacleBlockingRadius), intersection1, intersection2);
        if(intersections > 0 &&
           (intersection1.squaredNorm() < squaredDistance
            || (intersections == 2 && intersection2.squaredNorm() < squaredDistance)))
        {
          pose.preStepAllowed = false;
          pose.rating += duelRatings.ratingPoseBlocked;
        }
      }
    }

    if(pose.preStepAllowed && pose.type != TargetType::goalShot && pose.type != TargetType::pass)
    {
      // feet edge positions at target pose
      const Vector2f leftPoint = obstacleHandling.leftFootEdge.rotated(pose.pose.rotation) + pose.pose.translation;
      const Vector2f rightPoint = obstacleHandling.rightFootEdge.rotated(pose.pose.rotation) + pose.pose.translation;

      // feet edge positions during the kick
      const Vector2f leftKickPoint = obstacleHandling.leftFootKickEdge.rotated(pose.pose.rotation) + pose.pose.translation;
      const Vector2f rightKickPoint = obstacleHandling.rightFootKickEdge.rotated(pose.pose.rotation) + pose.pose.translation;

      // current feet edge positions
      const Vector2f leftPointNear = obstacleHandling.leftFootEdge;
      const Vector2f rightPointNear = obstacleHandling.rightFootEdge;

      bool leftBlocked = false;
      bool rightBlocked = false;

      auto checkBlocked = [](const Obstacle& ob, bool& leftBlocked, bool& rightBlocked,
                             const Pose2f& kickPose, const Vector2f& leftPoint, const Vector2f& rightPoint,
                             const Vector2f& leftPointNear, const Vector2f& rightPointNear,
                             const Vector2f& leftKickPoint, const Vector2f& rightKickPoint,
                             const float obstacleBlockingRadiusSqr, const float obstacleBlockingRadiusSafeSqr)
      {
        const Vector2f obCenter = kickPose.inverse() * ob.center;
        if(obCenter.squaredNorm() > obstacleBlockingRadiusSafeSqr)
          return;
        bool& useBlocked = obCenter.y() > 0.f ? leftBlocked : rightBlocked;
        const Vector2f& offset = obCenter.y() > 0.f ? leftPoint : rightPoint;
        const Vector2f& offsetNear = obCenter.y() > 0.f ? leftPointNear : rightPointNear;
        const Vector2f& offsetKick = obCenter.y() > 0.f ? leftKickPoint : rightKickPoint;
        useBlocked |= (obCenter - offset).squaredNorm() < obstacleBlockingRadiusSqr ||
                      (obCenter - offsetNear).squaredNorm() < obstacleBlockingRadiusSqr ||
                      (obCenter - offsetKick).squaredNorm() < obstacleBlockingRadiusSqr;
      };

      auto addBlockedRating = [this](bool& leftBlocked, bool& rightBlocked,
                                     const bool isLeftForward, const bool isRightForward, DuelPose& pose)
      {
        // this must be higher than the bonus for using the same kick again
        if((rightBlocked && !leftBlocked && isLeftForward) ||
           (leftBlocked && !rightBlocked && isRightForward))
        {
          pose.preStepAllowed = false;
          pose.rating += duelRatings.ratingPoseBlockedSmallKickAngle;
          return true;
        }
        // Reset booleans
        rightBlocked = false;
        leftBlocked = false;
        return false;
      };

      const bool isForwardLeftKick = kickType == KickInfo::walkForwardsLeft || kickType == KickInfo::walkForwardsLeftLong || kickType == KickInfo::walkForwardsLeftAlternative;
      const bool isForwardRightKick = kickType == KickInfo::walkForwardsRight || kickType == KickInfo::walkForwardsRightLong || kickType == KickInfo::walkForwardsRightAlternative;
      const float obstacleBlockingRadiusSqr = sqr(obstacleHandling.obstacleBlockingRadius);
      const float obstacleBlockingRadiusSafeSqr = sqr(obstacleHandling.obstacleBlockingRadius + 300.f); // obstacle is so far away, there will be no collision, 200 = distance to kickPoint + 100 safe distance

      checkBlocked(duelObstacleMovedNearBall, leftBlocked, rightBlocked, pose.pose, leftPoint, rightPoint, leftPointNear, rightPointNear, leftKickPoint, rightKickPoint, obstacleBlockingRadiusSqr, obstacleBlockingRadiusSafeSqr);
      if(!addBlockedRating(leftBlocked, rightBlocked, isForwardLeftKick, isForwardRightKick, pose))
      {
        for(const Obstacle& ob : theObstacleModel.obstacles)
        {
          checkBlocked(ob, leftBlocked, rightBlocked, pose.pose, leftPoint, rightPoint, leftPointNear, rightPointNear, leftKickPoint, rightKickPoint, obstacleBlockingRadiusSqr, obstacleBlockingRadiusSafeSqr);
          if(addBlockedRating(leftBlocked, rightBlocked, isForwardLeftKick, isForwardRightKick, pose))
            break;
        }
      }
    }

    // 8.4.14. Calculate time to reach position and worsen rating accordingly
    const float minTime = std::abs(pose.pose.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x()) +
                          std::abs(pose.pose.translation.y() / theWalkingEngineOutput.maxSpeed.translation.y()) +
                          std::abs(pose.pose.rotation / theWalkingEngineOutput.maxSpeed.rotation * 0.25f);

    pose.rating += std::max(0.f, minTime / 1000.f) * duelRatings.ratingOpponentFaster;

    // 8.4.15. Bonus for same kick, but smaller rotation
    if(theDuelPose.noKick || ((theDuelPose.kickType == kickType) && theDuelPose.type == pose.type &&
                              kickType != KickInfo::walkForwardStealBallLeft && kickType != KickInfo::walkForwardStealBallRight))
    {
      pose.rating += duelRatings.ratingSameKick;
      pose.rating += duelRatings.ratingSameKickAngle * (1.f - std::min((std::abs(Angle::normalize(theDuelPose.kickAngle - kickAngle))) / smallerKickAnglePreference, 1.f));
    }

    pose.endPoint = endPoint;
    duelPose = pose;

    return true;
  }

private:
  Obstacle duelObstacle; // Obstacle, with which we are fighting for the ball
  Obstacle duelObstacleMovedNearBall; // duelObstacle, but shifted based on the ball position
  float distanceToClosestObstacle; // distance to the duelObstacle
  Angle angleToClosestObstacle; // angle to duelObstacle
  DuelPose theDuelPose; // All information about the "to be executed" kick
  SkillRequestPose theSkillRequestPose; // All information about the current behavior request
  unsigned int timeSinceDoingNothing = 0; // timestamp since no kick was calculated
  unsigned int lastForwardSteal = 0; // timestamp since last forwardSteal was the best kick
  unsigned int lastSidewardRestrictionTimestamp = 0; // Last timestamp, when sidewards kick was restricted
  unsigned int ignoreSkillRequestTimestamp = 0; // Time stamp when skill request started to be igored
  Pose2f lastRobotPoseRotation; // robotPose rotation of previous cognition frame
  std::list<SectorWheel::Sector> kickAngles; // SectorWheel
  std::vector<float> checkKickDistancesForFR; // All kick ranges, that shall be checked
  Vector2f useBallPosition = Vector2f(0.f, 0.f); // Clipped Ballposition
  bool forcedDeactive = false; // Deactivate the Zweikampf?
  bool calculatedKickDistances = false; // Were the kick ranges calculated once?
  bool soonCloseRangeDuel = false; // Opponent is dangerous near to the ball. Start using risky kicks
  bool forceForwardSteal = false; // Was the forwardSteal force because the obstacle is actually close to the ball?

  Vector2f lastTeammatePosition; // Pass target robot last known position

  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };

  const Vector2f leftGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal); /**< The position of the left post of the opponent's goal. */
  const Vector2f rightGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal); /**< The position of the right post of the opponent's goal. */
  const Vector2f leftGoalPostOwn = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal); /**< The position of the left post of the own goal. */
  const Vector2f rightGoalPostOwn = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal); /**< The position of the right post of the own goal. */
  const Vector2f penaltyBoxBackRight = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightPenaltyArea); /**< Back right corner of own penalty box. */
  const Vector2f penaltyBoxFrontLeft = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea); /**< Front left corner of own penalty box. */
  const Vector2f opponentHalfBackRight = Vector2f(0.f, theFieldDimensions.yPosRightSideline); /**< Back right corner of opponent half. */
  const Vector2f opponentHalfFrontLeft = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline); /**< Front left corner of opponent half. */
  const Geometry::Rect opponentHalf = Geometry::Rect(opponentHalfBackRight, opponentHalfFrontLeft); /**< Rectangle of opponent half. */
  const Geometry::Rect opponentGoalArea = Geometry::Rect(Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea - 100.f), Vector2f(theFieldDimensions.xPosOpponentGroundLine + 200.f, theFieldDimensions.yPosLeftGoalArea + 100.f)); /**< Rectangle of opponent goal area. */
  const Geometry::Rect penaltyRect = Geometry::Rect(penaltyBoxBackRight, penaltyBoxFrontLeft); /**< Rectangle of own penalty area. */
  const Geometry::Rect playingField = Geometry::Rect(-opponentHalfFrontLeft, opponentHalfFrontLeft);
  const Rangea boundaryCheck = Rangea(-180_deg, 180_deg); /**< Min and max angles. */
  Rangef minMaxDrawRatings = Rangef(-1.f, 1.f);
  const Rangef ballClipX = Rangef(theFieldDimensions.xPosOwnGroundLine + 10.f, theFieldDimensions.xPosOpponentGroundLine - 10.f);
  const Rangef ballClipY = Rangef(theFieldDimensions.yPosRightSideline + 10.f, theFieldDimensions.yPosLeftSideline - 10.f);
};

MAKE_SKILL_IMPLEMENTATION(ZweikampfImpl);
