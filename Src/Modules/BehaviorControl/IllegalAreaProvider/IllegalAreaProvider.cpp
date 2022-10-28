/**
 * @file IllegalAreaProvider.cpp
 *
 * This file implements a module that determines which field areas are illegal to enter.
 *
 * @author Arne Hasselbring
 */

#include "IllegalAreaProvider.h"
#include "Math/BHMath.h"
#include <algorithm>

MAKE_MODULE(IllegalAreaProvider, behaviorControl);

void IllegalAreaProvider::update(IllegalAreas& illegalAreas)
{
  DECLARE_DEBUG_DRAWING("module:IllegalAreaProvider:illegalAreas", "drawingOnField");

  illegalAreas.illegal = 0u;
  illegalAreas.anticipatedIllegal = 0u;
  illegalAreas.anticipatedTimestamp = 0u;

  illegalAreas.getIllegalAreas = [this, &illegalAreas](const Vector2f& positionOnField, float margin)
  {
    return getIllegalAreas(illegalAreas.illegal, positionOnField, margin);
  };

  illegalAreas.isPositionIllegal = [this, &illegalAreas](const Vector2f& positionOnField, float margin)
  {
    return isPositionIllegal(illegalAreas.illegal, positionOnField, margin);
  };

  illegalAreas.willPositionBeIllegal = [this, &illegalAreas](const Vector2f& positionOnField, float margin)
  {
    return isPositionIllegal(illegalAreas.anticipatedIllegal, positionOnField, margin);
  };

  illegalAreas.isSameIllegalArea = [this, &illegalAreas](const Vector2f& positionOnField, const Vector2f& targetOnField, float margin)
  {
    return isSameIllegalArea(illegalAreas.illegal, positionOnField, targetOnField, margin);
  };

  if(theGameState.isInitial() || theGameState.isFinished())
    return;

  if(theGameState.isPenaltyShootout())
  {
    if(theGameState.isForOpponentTeam())
      illegalAreas.illegal |= bit(IllegalAreas::notOwnGoalLine);
    return;
  }

  // The rules don't make a difference between the goalkeeper and field players, but we want the goalkeeper to always be able to enter the own penalty area.
  if(!theGameState.isGoalkeeper() && theLibTeammates.nonKeeperTeammatesInOwnPenaltyArea >= 2)
    illegalAreas.illegal |= bit(IllegalAreas::ownPenaltyArea);

  if(theLibTeammates.teammatesInOpponentPenaltyArea >= 3)
    illegalAreas.illegal |= bit(IllegalAreas::opponentPenaltyArea);

  if(theGameState.isReady())
  {
    illegalAreas.anticipatedTimestamp = theGameState.timeWhenStateEnds;
    illegalAreas.anticipatedIllegal |= bit(IllegalAreas::borderStrip);
    if(theGameState.isPenaltyKick())
    {
      if(theGameState.isForOwnTeam())
      {
        if(theLibTeammates.teammatesInOpponentPenaltyArea >= 1)
          illegalAreas.anticipatedIllegal |= bit(IllegalAreas::opponentPenaltyArea);
      }
      else
        illegalAreas.anticipatedIllegal |= bit(theGameState.isGoalkeeper() ? IllegalAreas::notOwnGoalLine : IllegalAreas::ownPenaltyArea);
    }
    else
    {
      illegalAreas.anticipatedIllegal |= bit(IllegalAreas::opponentHalf);
      if(theGameState.isForOpponentTeam())
        illegalAreas.anticipatedIllegal |= bit(IllegalAreas::centerCircle);
    }
  }
  else if(theGameState.isSet())
  {
    illegalAreas.illegal |= bit(IllegalAreas::borderStrip);
    if(theGameState.isPenaltyKick())
    {
      if(theGameState.isForOwnTeam())
      {
        if(theLibTeammates.teammatesInOpponentPenaltyArea >= 1)
          illegalAreas.illegal |= bit(IllegalAreas::opponentPenaltyArea);
      }
      else
        illegalAreas.illegal |= bit(theGameState.isGoalkeeper() ? IllegalAreas::notOwnGoalLine : IllegalAreas::ownPenaltyArea);
    }
    else
    {
      illegalAreas.illegal |= bit(IllegalAreas::opponentHalf);
      if(theGameState.isForOpponentTeam())
        illegalAreas.illegal |= bit(IllegalAreas::centerCircle);
    }
  }
  else if(theGameState.isPlaying())
  {
    if(theGameState.isPenaltyKick())
    {
      // "During the Penalty Kick: [...] 3. All robots must be within the field-of-play. That is, robots may not be outside the field lines, but within the field border."
      // (rule book section 3.8.1)
      illegalAreas.illegal |= bit(IllegalAreas::borderStrip);

      if(theGameState.isForOwnTeam())
      {
        if(theLibTeammates.teammatesInOpponentPenaltyArea >= 1)
          illegalAreas.illegal |= bit(IllegalAreas::opponentPenaltyArea);
      }
      else
        illegalAreas.illegal |= bit(theGameState.isGoalkeeper() ? IllegalAreas::notOwnGoalLine : IllegalAreas::ownPenaltyArea);
    }
    else if(theGameState.isFreeKick())
    {
      if(theGameState.isForOpponentTeam())
        illegalAreas.illegal |= bit(IllegalAreas::ballArea);
    }
    else if(theGameState.isKickOff())
    {
      illegalAreas.illegal |= bit(IllegalAreas::opponentHalf);
      if(theGameState.isForOpponentTeam())
        illegalAreas.illegal |= bit(IllegalAreas::centerCircle);
    }
  }

  draw(illegalAreas.illegal);
}

unsigned IllegalAreaProvider::getIllegalAreas(unsigned illegal, const Vector2f& positionOnField, float margin) const
{
  unsigned illegalAreas = 0u;
  const float halfLineWidth = theFieldDimensions.fieldLinesWidth * 0.5f;
  if((illegal & bit(IllegalAreas::ownPenaltyArea)) &&
     (positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea + halfLineWidth + margin &&
      positionOnField.x() > theFieldDimensions.xPosOwnGroundLine - halfLineWidth - margin &&
      positionOnField.y() > theFieldDimensions.yPosRightPenaltyArea - halfLineWidth - margin &&
      positionOnField.y() < theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth + margin))
    illegalAreas |= bit(IllegalAreas::ownPenaltyArea);
  if((illegal & bit(IllegalAreas::opponentPenaltyArea)) &&
     (positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyArea - halfLineWidth - margin &&
      positionOnField.x() < theFieldDimensions.xPosOpponentGroundLine + halfLineWidth + margin &&
      positionOnField.y() > theFieldDimensions.yPosRightPenaltyArea - halfLineWidth - margin &&
      positionOnField.y() < theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth + margin))
    illegalAreas |= bit(IllegalAreas::opponentPenaltyArea);
  if((illegal & bit(IllegalAreas::borderStrip)) &&
     (positionOnField.x() > theFieldDimensions.xPosOpponentGroundLine + halfLineWidth - margin ||
      positionOnField.x() < theFieldDimensions.xPosOwnGroundLine - halfLineWidth + margin ||
      positionOnField.y() < theFieldDimensions.yPosRightSideline - halfLineWidth + margin ||
      positionOnField.y() > theFieldDimensions.yPosLeftSideline + halfLineWidth - margin))
    illegalAreas |= bit(IllegalAreas::borderStrip);
  if((illegal & bit(IllegalAreas::opponentHalf)) &&
     (positionOnField.x() > theFieldDimensions.xPosHalfWayLine - halfLineWidth - margin &&
      positionOnField.squaredNorm() >= sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + halfLineWidth - margin))))
    illegalAreas |= bit(IllegalAreas::opponentHalf);
  if((illegal & bit(IllegalAreas::centerCircle)) &&
     (positionOnField.squaredNorm() < sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + halfLineWidth + margin))))
    illegalAreas |= bit(IllegalAreas::centerCircle);
  if((illegal & bit(IllegalAreas::ballArea)) &&
     (positionOnField - theFieldBall.recentBallPositionOnField()).squaredNorm() < sqr(std::max(0.f, freeKickClearAreaRadius + margin)))
    illegalAreas |= bit(IllegalAreas::ballArea);
  if((illegal & bit(IllegalAreas::notOwnGoalLine)) &&
     (std::abs(positionOnField.x() - theFieldDimensions.xPosOwnGroundLine) > halfLineWidth - std::min(0.f, margin) ||
      positionOnField.y() > theFieldDimensions.yPosLeftGoal - margin ||
      positionOnField.y() < theFieldDimensions.yPosRightGoal + margin))
    illegalAreas |= bit(IllegalAreas::notOwnGoalLine);
  return illegalAreas;
}

bool IllegalAreaProvider::isPositionIllegal(unsigned illegal, const Vector2f& positionOnField, float margin) const
{
  return getIllegalAreas(illegal, positionOnField, margin);
}

bool IllegalAreaProvider::isSameIllegalArea(unsigned illegal, const Vector2f& positionOnField, const Vector2f& targetOnField, float margin) const
{
  return getIllegalAreas(illegal, positionOnField, margin) & getIllegalAreas(illegal, targetOnField, margin);
}

void IllegalAreaProvider::draw(unsigned illegal) const
{
  COMPLEX_DRAWING("module:IllegalAreaProvider:illegalAreas")
  {
    const float halfLineWidth = theFieldDimensions.fieldLinesWidth * 0.5f;
    const ColorRGBA brushColor(255, 0, 0, 50);

    if(illegal & bit(IllegalAreas::ownPenaltyArea))
    {
      const std::vector<Vector2f> illegalPolygon = {Vector2f(theFieldDimensions.xPosOwnPenaltyArea + halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOwnPenaltyArea + halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOwnGroundLine - halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOwnGroundLine - halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth)};
      POLYGON("module:IllegalAreaProvider:illegalAreas", illegalPolygon.size(), illegalPolygon.data(), 40, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, brushColor);
    }
    if(illegal & bit(IllegalAreas::opponentPenaltyArea))
    {
      const std::vector<Vector2f> illegalPolygon = {Vector2f(theFieldDimensions.xPosOpponentPenaltyArea - halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOpponentPenaltyArea - halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine + halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine + halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth)};
      POLYGON("module:IllegalAreaProvider:illegalAreas", illegalPolygon.size(), illegalPolygon.data(), 40, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, brushColor);
    }
    if(illegal & bit(IllegalAreas::opponentHalf))
    {
      // TODO: Exclude center circle + halfLineWidth
      const std::vector<Vector2f> illegalPolygon = {Vector2f(theFieldDimensions.xPosHalfWayLine - halfLineWidth, theFieldDimensions.yPosRightSideline),
                                                    Vector2f(theFieldDimensions.xPosHalfWayLine - halfLineWidth, theFieldDimensions.yPosLeftSideline),
                                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine + halfLineWidth, theFieldDimensions.yPosLeftSideline),
                                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine + halfLineWidth, theFieldDimensions.yPosRightSideline)};
      POLYGON("module:IllegalAreaProvider:illegalAreas", illegalPolygon.size(), illegalPolygon.data(), 40, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, brushColor);
    }
    if(illegal & bit(IllegalAreas::centerCircle))
    {
      CIRCLE("module:IllegalAreaProvider:illegalAreas", 0.f, 0.f, theFieldDimensions.centerCircleRadius + halfLineWidth, 40, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, brushColor);
    }
    if(illegal & bit(IllegalAreas::ballArea))
    {
      CIRCLE("module:IllegalAreaProvider:illegalAreas", theFieldBall.recentBallPositionOnField().x(), theFieldBall.recentBallPositionOnField().y(), freeKickClearAreaRadius, 40, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, brushColor);
    }
  }
}
