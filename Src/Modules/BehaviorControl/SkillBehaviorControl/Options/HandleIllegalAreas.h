option(HandleIllegalAreas)
{
  const Angle margin = 10_deg;

  auto approximateTarget = [&]
  {
    Vector2f targetPosition = Vector2f::Zero();
    switch(theMotionRequest.motion)
    {
      case MotionRequest::stand:
        break;
      case MotionRequest::walkToPose:
        targetPosition = theMotionRequest.walkTarget.translation;
        break;
      case MotionRequest::walkToBallAndKick:
      case MotionRequest::dribble:
        targetPosition = theMotionRequest.ballEstimate.position;
        break;
      default:
        break;
    }
    return targetPosition;
  };

  auto lookAtBall = [&]
  {
    switch(theMotionRequest.motion)
    {
      case MotionRequest::walkToBallAndKick:
      case MotionRequest::dribble:
        return true;
      default:
        break;
    }
    return false;
  };

  initial_state(notIllegal)
  {
    transition
    {
      if(theIllegalAreas.isPositionIllegal(theRobotPose.translation, 150.f))
        goto illegal;
      else if(theIllegalAreas.isPositionIllegal(theRobotPose.translation, 300.f) &&
              theIllegalAreas.isPositionIllegal(theRobotPose * approximateTarget(), 150.f))
        goto waiting;
    }
  }

  state(illegal)
  {
    transition
    {
      if(!theIllegalAreas.isPositionIllegal(theRobotPose.translation, 300.f))
      {
        if(!theIllegalAreas.isPositionIllegal(theRobotPose * approximateTarget(), 300.f))
          goto notIllegal;
        goto waiting;
      }
    }

    action
    {
      theLibCheck.dec(LibCheck::motionRequest);
      theLibCheck.dec(LibCheck::headMotionRequest);
      theWalkPotentialFieldSkill({.target = theRobotPose.translation,
                                  .playerNumber = -1,
                                  .straight = true});
      theLookActiveSkill({.withBall = lookAtBall()});
    }
  }

  state(waiting)
  {
    transition
    {
      if(!theIllegalAreas.isPositionIllegal(theRobotPose.translation, 300.f) &&
         !theIllegalAreas.isPositionIllegal(theRobotPose * approximateTarget(), 300.f))
        goto notIllegal;
      if(theIllegalAreas.isPositionIllegal(theRobotPose.translation, 150.f))
        goto illegal;
    }

    action
    {
      theLibCheck.dec(LibCheck::motionRequest);
      theLibCheck.dec(LibCheck::headMotionRequest);
      const Vector2f targetRelative = approximateTarget();
      if(Rangea(-margin, margin).isInside(targetRelative.angle()))
        theStandSkill();
      else
        theTurnToPointSkill({.target = targetRelative});
      theLookActiveSkill();
    }
  }
}
