/**
 * @file TeammatesBallModelProvider.cpp
 *
 * Declares a class that provides a ball model that fuses information
 * from my teammates.
 *
 * @author Tim Laue
 */

#include "TeammatesBallModelProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Framework/Settings.h"

MAKE_MODULE(TeammatesBallModelProvider, modeling);


void TeammatesBallModelProvider::update(TeammatesBallModel& teammatesBallModel)
{
  // Check and adapt size of buffer -----
  if(balls.size() != static_cast<std::size_t>(Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1))
    balls.resize(Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1);
  // Add new observations to buffer (except for phases during which everything is deleted)
  if(!checkForResetByGameSituation())
    updateInternalBallBuffers();
  // Reset lists and variables:
  ballsAvailableForTeamBall.clear();
  teammatesBallModel.isValid = false;
  // Try to compute a team ball
  findAvailableBalls();
  clusterBalls();
  computeModel(teammatesBallModel);
  DECLARE_DEBUG_DRAWING("module:TeammatesBallModelProvider:bufferedBalls", "drawingOnField");
  COMPLEX_DRAWING("module:TeammatesBallModelProvider:bufferedBalls")
  {
    ColorRGBA bufferedBallColor(200, 200, 200, 200);
    for(unsigned int robot = 0; robot < balls.size(); ++robot)
    {
      if(balls[robot].time > 0 && balls[robot].valid)
      {
        const TeammatesBallModelProvider::BufferedBall& ball = balls[robot];
        Vector2f absPos = ball.robotPose * ball.pos;
        CIRCLE("module:TeammatesBallModelProvider:bufferedBalls", absPos.x(), absPos.y(), 150, 20, Drawings::solidPen, bufferedBallColor, Drawings::solidBrush, bufferedBallColor);
      }
    }
  }
}

void TeammatesBallModelProvider::computeModel(TeammatesBallModel& teammatesBallModel)
{
  // No active balls -> no team ball:
  if(ballsAvailableForTeamBall.size() == 0)
  {
    teammatesBallModel.isValid = false;
    return;
  }
  // Find the ball that has a) the biggest cluster and b) the highest weighting
  unsigned int bestBallIdx = 0;
  unsigned int bestBallClusterSize = unsigned(ballsAvailableForTeamBall[0].compatibleBallsIndices.size());
  float bestBallWeighting = ballsAvailableForTeamBall[0].weighting;
  for(unsigned int i=1; i<ballsAvailableForTeamBall.size(); i++)
  {
    if(ballsAvailableForTeamBall[i].compatibleBallsIndices.size() > bestBallClusterSize)
    {
      bestBallClusterSize = unsigned(ballsAvailableForTeamBall[i].compatibleBallsIndices.size());
      bestBallWeighting = ballsAvailableForTeamBall[i].weighting;
      bestBallIdx = i;
    }
    else if(ballsAvailableForTeamBall[i].compatibleBallsIndices.size() == bestBallClusterSize)
    {
      if(ballsAvailableForTeamBall[i].weighting > bestBallWeighting)
      {
        bestBallWeighting = ballsAvailableForTeamBall[i].weighting;
        bestBallIdx = i;
      }
    }
  }

  ActiveBall& bestBall = ballsAvailableForTeamBall[bestBallIdx];
  // Finally, compute position and velocity based on the weighted clustered balls:
  float weightSum = bestBall.weighting;
  Vector2f avgPos = bestBall.position * bestBall.weighting;
  Vector2f avgVel = bestBall.velocity * bestBall.weighting;
  unsigned lastSeen = bestBall.time;
  for(unsigned int i = 0; i < bestBall.compatibleBallsIndices.size(); i++)
  {
    ActiveBall& b = ballsAvailableForTeamBall[bestBall.compatibleBallsIndices[i]];
    weightSum += b.weighting;
    avgPos += b.position * b.weighting;
    avgVel += b.velocity * b.weighting;
    lastSeen = lastSeen < b.time ? b.time : lastSeen;
  }
  teammatesBallModel.newerThanOwnBall = lastSeen > theWorldModelPrediction.timeWhenBallLastSeen;
  if(weightSum == 0.f) // Should not happen
  {
    teammatesBallModel.isValid = false;
  }
  else
  {
    teammatesBallModel.isValid = true;
    teammatesBallModel.position = avgPos / weightSum;
    teammatesBallModel.velocity = avgVel / weightSum;
  }
}

void TeammatesBallModelProvider::updateInternalBallBuffers()
{
  // Observations by teammates:
  for(auto const& teammate : theTeamData.teammates)
  {
    // teammate is participating in the game and has made a new ball observation
    ASSERT(teammate.number >= Settings::lowestValidPlayerNumber);
    ASSERT(teammate.number <= Settings::highestValidPlayerNumber);
    const unsigned n = teammate.number - Settings::lowestValidPlayerNumber;
    if(teammate.isUpright &&
       (balls[n].time != teammate.theBallModel.timeWhenLastSeen ||
        balls[n].vel.norm() < teammate.theBallModel.estimate.velocity.norm()) &&
       teammate.theBallModel.lastPerception != Vector2f::Zero() &&
       teammate.theBallModel.estimate.position != Vector2f::Zero())
    {
      BufferedBall newBall;
      newBall.robotPose              = teammate.theRobotPose;
      newBall.poseQualityModifier    = localizationQualityToModifier(teammate.theRobotPose.quality);
      newBall.pos                    = teammate.theBallModel.estimate.position;
      newBall.vel                    = teammate.theBallModel.estimate.velocity;
      newBall.time                   = teammate.theBallModel.timeWhenLastSeen;
      newBall.valid = true;
      if(newBall.poseQualityModifier > 0.f) // If the value is zero, the final weighting would be 0, too.
        balls[n] = newBall;
    }
  }
  // If any teammate is not playing anymore, invalidate the observations that happened during the time
  // before the status changed:
  for(unsigned index = 0; index < balls.size(); ++index)
  {
    const int robot = Settings::lowestValidPlayerNumber + int(index);
    if(robot == theGameState.playerNumber)
      continue;
    if(!GameState::isPenalized(theGameState.ownTeam.playerStates[index]))
    {
      for(auto const& teammate : theTeamData.teammates)
        if(teammate.number == static_cast<int>(robot) && teammate.isUpright)
        {
          if(teammate.theBallModel.timeWhenDisappeared != teammate.theFrameInfo.time)
            balls[index].valid = false;
          goto teammateValid;
        }
    }

    // Teammate is penalized or fallen
    if(theFrameInfo.getTimeSince(balls[index].time) <= inactivityInvalidationTimeSpan)
      balls[index].valid = false;

  teammateValid:
    ;
  }

  // If any teammate has recently changed its position significantly, all information from this robot might have been false and
  // has to be made invalid:
  for(auto const& teammate : theTeamData.teammates)
  {
    if(theFrameInfo.getTimeSince(teammate.theRobotPose.timestampLastJump) < 1000)
    {
      balls[teammate.number - Settings::lowestValidPlayerNumber].valid = false;
    }
  }
}

void TeammatesBallModelProvider::findAvailableBalls()
{
  for(unsigned int robot = 0; robot < balls.size(); ++robot)
  {
    if(balls[robot].valid)
    {
      const TeammatesBallModelProvider::BufferedBall& ball = balls[robot];
      const float weighting = computeWeighting(ball);
      if(weighting != 0.f)
      {
        Vector2f absPos = ball.robotPose * ball.pos;
        Vector2f absVel = ball.vel;
        absVel.rotate(ball.robotPose.rotation);
        if(ball.time < theFrameInfo.time)
        {
          const float t = (theFrameInfo.time - ball.time) / 1000.f;
          // To be technically correct, for kicked (and not seen) balls, the propagation should start at the
          // point of time of the kick and not at the point of time of the observation.
          // However, as the error is not big and as it would require to send one more timestamp,
          // this formula remains unchanged.
          BallPhysics::propagateBallPositionAndVelocity(absPos, absVel, t, theBallSpecification.friction);
        }
        // Final check to accept the ball:
        if(theFieldDimensions.isInsideCarpet(absPos))
        {
          ActiveBall newActiveBall;
          newActiveBall.position = absPos;
          newActiveBall.velocity = absVel;
          newActiveBall.playerNumber = static_cast<unsigned char>(robot + Settings::lowestValidPlayerNumber);
          newActiveBall.weighting = weighting;
          newActiveBall.time = ball.time;
          ballsAvailableForTeamBall.push_back(newActiveBall);
        }
      }
    }
  }
}

void TeammatesBallModelProvider::clusterBalls()
{
  for(unsigned int i=0; i<ballsAvailableForTeamBall.size(); i++)
  {
    for(unsigned int j=i+1; j<ballsAvailableForTeamBall.size(); j++)
    {
      ActiveBall& a = ballsAvailableForTeamBall[i];
      ActiveBall& b = ballsAvailableForTeamBall[j];
      if((a.position - b.position).norm() < 777.f) // There is probably a solution that is more clever ...
      {
        a.compatibleBallsIndices.push_back(j);
        b.compatibleBallsIndices.push_back(i);
      }
    }
  }
}

float TeammatesBallModelProvider::localizationQualityToModifier(const RobotPose::LocalizationQuality quality) const
{
  if(quality == RobotPose::superb)
    return 1.f;      // If localization appears to be really good, it should not negatively affect the ball weighting
  else if(quality == RobotPose::okay)
    return 0.75f;    // This might be the most common case and the value could be a parameter (but must not)
  else
    return 0.f;      // Do not use any ball from this robot. It assumes that it does not have any clue about its position
}


float TeammatesBallModelProvider::computeWeighting(const TeammatesBallModelProvider::BufferedBall& ball) const
{
  const int timeSinceBallWasSeen = theFrameInfo.getTimeSince(ball.time);
  // Hummm, the ball has not been seen for some time -> weighting is 0
  if(timeSinceBallWasSeen > ballLastSeenTimeout)
    return 0.f;

  // Computing the weighting is based on three parts:
  // 1. the time that has passed since the ball was seen the last time (smaller is better)
  // 2. the distance of the observer to the ball (closer is better)
  // 3. the validity [0,..,1] of the robot's position estimate

  // Part 1: Time *****
  // Compute a value in the interval [0,..,1], which indicates how much of the maximum possible ball age is over:
  float ballAgeRelativeToTimeout = static_cast<float>(timeSinceBallWasSeen) / static_cast<float>(ballLastSeenTimeout);
  // Use hyperbolic tangent for computing the age-based weighting.
  // The input value is multiplied by 2, this has the following effect (given the tanh curve):
  // - the ball weight is about 0.25, if half of the maximum time is over
  // - the ball weight is close to 0, if the maximum time is almost over
  float ballAgeWeight = 1.f - std::tanh(ballAgeRelativeToTimeout * 2.f);

  // Part 2: Angular distance *****
  const float camHeight = 550.f;
  const float angleOfCamera = std::atan(ball.pos.norm() / camHeight); // angle of camera when looking at the ball
  const float ballPositionWithPositiveDeviation = std::tan(angleOfCamera + 1_deg) * camHeight; // ball position when angle of camera is a bit different in position direction
  const float ballPositionWithNegativeDeviation = std::tan(angleOfCamera - 1_deg) * camHeight; // ball position when angle of camera is a bit different in negative direction
  const float ballDeviation = (ballPositionWithPositiveDeviation - ballPositionWithNegativeDeviation) / 2; // averaged deviation of the ball position

  // Part 3: Combination with validity *****
  const float weighting = (ballAgeWeight / ballDeviation) * ball.poseQualityModifier;
  return std::abs(weighting);
}

bool TeammatesBallModelProvider::checkForResetByGameSituation()
{
  bool reset = false;
  // In READY, INITIAL, FINISHED, a team ball model does not make any sense:
  if(!theGameState.isPlaying() && !theGameState.isSet())
    reset = true;
  // Same for any goal kick or corner kick (kick-ins and pushing free kicks are different because the ball stays about where it was):
  if((theGameState.isGoalKick() || theGameState.isCornerKick()) &&
     theGameState.state != theExtendedGameState.stateLastFrame)
    reset = true;
  // If any of the above conditions was true, we delete every ball information that we have stored:
  if(reset)
  {
    for(unsigned int i = 0; i < balls.size(); ++i)
      balls[i].valid = false;
  }
  return reset;
}
