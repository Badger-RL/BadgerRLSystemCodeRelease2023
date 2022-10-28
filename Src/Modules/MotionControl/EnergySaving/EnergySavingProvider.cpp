/*
 * @file FilteredCurrentProvider.cpp
 * @author Philip Reichenberg
 */

#include "EnergySavingProvider.h"
#include <cmath>

MAKE_MODULE(EnergySavingProvider, motionControl);

EnergySavingProvider::EnergySavingProvider()
{
  std::fill(lastOffsets.begin(), lastOffsets.end(), 0_deg);
  std::fill(lastBaseOffset.begin(), lastBaseOffset.end(), 0_deg);
  std::fill(emergencyChangeCounter.begin(), emergencyChangeCounter.end(), stepsBeforeEmergencyStep);
}

void EnergySavingProvider::update(EnergySaving& energySaving)
{
  // reset if no engine is using the energy saving mode
  if(state == State::deactive)
    energySaving.shutDown();
  else
  {
    lastState = state;
    state = State::deactive;
  }

  energySaving.shutDown = [this, &energySaving]
  {
    motionChangeTimestamp = theFrameInfo.time;
    adjustTimestamp = theFrameInfo.time;
    energySaving.offsets.fill(0_deg);
    energySaving.state = EnergyState::off;
    jointBaseOffset.angles.fill(0_deg);
    adjustOnlyOneLegJoint = false;
  };

  energySaving.reset = [&energySaving]
  {
    energySaving.state = EnergyState::resetState;
  };

  energySaving.applyHeatAdjustment = [this, &energySaving](JointRequest& request, const bool adjustLegs, const bool adjustArms, const bool standHigh, const bool accuratePositions)
  {
    usedResetInterpolation = theMotionInfo.isMotion(MotionPhase::stand) ? resetTimeNormal : resetTimeSlow;
    bool adjustLeftArm = adjustArms;
    bool adjustRightArm = adjustArms;
    adjustLeftArm &= theArmMotionInfo.isKeyframeMotion(Arms::left, ArmKeyFrameRequest::back) || theArmMotionInfo.armMotion[Arms::left] == ArmMotionRequest::none;
    adjustRightArm &= theArmMotionInfo.isKeyframeMotion(Arms::right, ArmKeyFrameRequest::back) || theArmMotionInfo.armMotion[Arms::right] == ArmMotionRequest::none;
    auto setInitialOffset = [&]
    {
      if(theMotionInfo.isMotion(MotionPhase::stand) && !standHigh)
      {
        adjustOnlyOneLegJoint = true;
        for(std::size_t joint = Joints::firstLeftLegJoint; joint < Joints::numOfJoints; joint++)
          jointBaseOffset.angles[joint] = theJointAngles.angles[joint] - request.angles[joint];
      }
      else
        jointBaseOffset.angles.fill(0_deg);
    };

    auto applyBaseJointOffset = [&]
    {
      FOREACH_ENUM(Joints::Joint, joint)
      {
        request.angles[joint] += jointBaseOffset.angles[joint];
      }
    };

    auto waitingFunc = [this, &energySaving]
    {
      if(theFrameInfo.getTimeSince(motionChangeTimestamp) > motionChangeWaitTime)
      {
        hasGroundContact = theGroundContactState.contact;
        emergencyChangeCounter.fill(stepsBeforeEmergencyStep);
        energySaving.state = EnergyState::working;
      }
    };

    auto resetFunc = [this, &energySaving]
    {
      //reduce the offsets to 0 over the time of <resetTime>
      if(theFrameInfo.getTimeSince(lastInAdjustmentTimestamp) < usedResetInterpolation)
      {
        float ratio = static_cast<float>(theFrameInfo.getTimeSince(lastInAdjustmentTimestamp) / usedResetInterpolation);
        for(size_t i = 0; i < energySaving.offsets.size(); i++)
        {
          energySaving.offsets[i] = lastOffsets[i] * (1.f - ratio);
          jointBaseOffset.angles[i] = lastBaseOffset[i] * (1.f - ratio);
        }
      }
      //after reduced to 0, check if new offsets need to be calculated, the module needs to wait or needs to be off
      else
      {
        std::fill(emergencyChangeCounter.begin(), emergencyChangeCounter.end(), stepsBeforeEmergencyStep);
        energySaving.offsets.fill(0_deg);
        if(theFrameInfo.getTimeSince(motionChangeTimestamp) > motionChangeWaitTime)
        {
          hasGroundContact = theGroundContactState.contact;
          energySaving.state = EnergyState::working;
        }
        else
          energySaving.state = EnergyState::waiting;
      }
    };

    auto workingPreFunc = [this, &energySaving]
    {
      lastOffsets = energySaving.offsets;
      lastBaseOffset = jointBaseOffset.angles;
      lastInAdjustmentTimestamp = theFrameInfo.time;
      //The offsets shall reset, if we switch from high stand to stand. (Or from stand to other keyframeMotion)
      if(theGroundContactState.contact != hasGroundContact)
      {
        motionChangeTimestamp = theFrameInfo.time;
        energySaving.state = EnergyState::resetState;
        return true;
      }
      return false;
    };

    auto workingPostFunc = [this, &energySaving, &standHigh]
    {
      int offsetCapCounter = 0;
      int offsetSingleCap = 0;
      for(size_t i = Joints::firstLegJoint; i < energySaving.offsets.size(); i++)   //Only reset and cap based on legs
      {
        if(i == Joints::rHipYawPitch)   //skip rHipYawPitch
          continue;
        Angle capOffset = energySaving.offsets[i] > 0 ? std::min(energySaving.offsets[i], maxAngleMultipleJoints) : std::max(energySaving.offsets[i], -maxAngleMultipleJoints);
        if(capOffset != energySaving.offsets[i])
          offsetCapCounter++;
        capOffset = energySaving.offsets[i] > 0 ? std::min(energySaving.offsets[i], maxAngleOneJoint) : std::max(energySaving.offsets[i], -maxAngleOneJoint);
        if(capOffset != energySaving.offsets[i])
          offsetSingleCap++;
        energySaving.offsets[i] = capOffset;
      }
      if((offsetCapCounter >= minNumberForHeatAdjustmentReset || offsetSingleCap != 0) && !isComStable(standHigh))
      {
        lastOffsets = energySaving.offsets;
        lastBaseOffset = jointBaseOffset.angles;
        energySaving.state = EnergyState::resetState;
      }
    };

    // Energy Saving Move is active
    if(!accuratePositions)
    {
      state = State::energySaving;
      if(lastState == State::accuratePosition)
      {
        lastOffsets = energySaving.offsets;
        lastBaseOffset = jointBaseOffset.angles;
        energySaving.state = EnergyState::resetState;
      }

      switch(energySaving.state)
      {
        case EnergyState::off:
        {
          setInitialOffset();
          applyBaseJointOffset();
          energySaving.state = EnergyState::waiting;
          break; // Skip this break to go directly to waiting?
        }
        case EnergyState::waiting:
        {
          //wait a moment to make sure the robot reached it new position
          waitingFunc();
          applyBaseJointOffset();
          break;
        }
        case EnergyState::working:
        {
          applyBaseJointOffset();
          if(workingPreFunc())
            break;
          //Adjust once in a while
          if(theFrameInfo.getTimeSince(adjustTimestamp) > adjustWaitTime)
          {
            adjustTimestamp = theFrameInfo.time;
            std::vector<Angle> jointDiffs(Joints::numOfJoints);
            std::size_t lastJoint = !adjustOnlyOneLegJoint ? Joints::numOfJoints : Joints::firstLeftLegJoint;
            // Only adjust the leg joint with the highest current
            if(adjustOnlyOneLegJoint)
              applyJointEnergySaving(theFilteredCurrent.legJointWithHighestCurrent, energySaving, request, adjustLegs, adjustLeftArm, adjustRightArm, standHigh, jointDiffs);
            for(std::size_t i = 0; i < lastJoint; i++)
            {
              applyJointEnergySaving(i, energySaving, request, adjustLegs, adjustLeftArm, adjustRightArm, standHigh, jointDiffs);
            }
            //cap the offsets and count how many offsets are capped. If too many are capped, reset the offsets
            workingPostFunc();
          }
          break;
        }
        case EnergyState::resetState:
        {
          resetFunc();
          applyBaseJointOffset();
          break;
        }
      }
    }
    // Calibration Mode is active
    else
    {
      state = State::accuratePosition;
      if(lastState == State::energySaving)
      {
        lastOffsets = energySaving.offsets;
        lastBaseOffset = jointBaseOffset.angles;
        energySaving.state = EnergyState::resetState;
      }

      switch(energySaving.state)
      {
        case EnergyState::waiting:
        {
          //wait a moment to make sure the robot reached it new position
          waitingFunc();
          break;
        }
        case EnergyState::working:
        {
          if(workingPreFunc())
            break;
          //Adjust once in a while
          if(theFrameInfo.getTimeSince(adjustTimestamp) > adjustWaitTime)
          {
            adjustTimestamp = theFrameInfo.time;
            std::vector<Angle> jointDiffs(Joints::numOfJoints);
            for(size_t i = 0; i < energySaving.offsets.size(); i++)
            {
              if(i < Joints::firstLegJoint)
                continue;

              jointDiffs[i] = request.angles[i] - theJointAngles.angles[i];
              if(minGearStep < std::abs(jointDiffs[i]))
              {
                //It can happen, that the offset changes around the current measured joint value.
                //In such a case, we adjust by maxGearStep, because if the joint moved a bit more,
                //the current often jumps down to a low value.
                emergencyChangeCounter[i] -= 1;
                if(emergencyChangeCounter[i] <= 0)
                {
                  energySaving.offsets[i] += maxGearStepLegs;
                  emergencyChangeCounter[i] = stepsBeforeEmergencyStep;
                }
                else
                {
                  //adjust step is smaller than the minimal step the gears can do
                  //this help to reach a better optimal offset
                  //check if we can do a big step to adjust the arm as fast as possible
                  Angle dif = std::abs(jointDiffs[i]);
                  Angle step = dif > maxGearStepLegs && i >= Joints::firstLegJoint ? maxGearStepLegs : Angle(minGearStep / 2.f);
                  energySaving.offsets[i] += (jointDiffs[i] > 0 ? 1 : -1) * step;
                }
              }
            }
            //cap the offsets and count how many offsets are capped. If too many are capped, reset the offsets
            workingPostFunc();
          }
          break;
        }
        case EnergyState::resetState:
        {
          resetFunc();
          break;
        }
      }
    }
    FOREACH_ENUM(Joints::Joint, joint)
      request.angles[joint] += energySaving.offsets[joint];
  };
}

void EnergySavingProvider::applyJointEnergySaving(const std::size_t& joint,
                                                  EnergySaving& energySaving, JointRequest& request,
                                                  const bool adjustLegs, const bool adjustLeftArm, const bool adjustRightArm,
                                                  const bool standHigh, std::vector<Angle>& jointDiffs)
{
  //skip joint that can not be effectively adjusted
  bool skip = false;
  for(Joints::Joint sJoint : skipJoints)
    if(sJoint == joint)
      skip = true;
  if(skip)
    return;
  jointDiffs[joint] = theJointAngles.angles[joint] - request.angles[joint];
  const bool adjustThisArm = joint < Joints::firstRightArmJoint ? adjustLeftArm : adjustRightArm;
  const int& useLegCurrentThreshold = std::abs(energySaving.offsets[joint]) < maxAngleOneJoint / 2.f ? currentThresholdLegs : currentThresholdLegsHighOffset;
  if((joint >= Joints::firstLegJoint && theFilteredCurrent.currents[joint] > useLegCurrentThreshold && adjustLegs)
     || (joint < Joints::firstLegJoint && theFilteredCurrent.currents[joint] > currentThresholdArms && adjustThisArm)) //if current is above the threshold, change the offset
  {
    //It can happen, that the offset changes around the current measured joint value.
    //In such a case, we adjust by maxGearStep, because if the joint moved a bit more,
    //the current often jumps down to a low value.
    emergencyChangeCounter[joint] -= 1;
    if(emergencyChangeCounter[joint] <= 0)
    {
      // TODO this stupid stuff needs to be replaced ...
      int sign = !standHigh ? standSigns[joint] : highStandSigns[joint];
      energySaving.offsets[joint] += joint < Joints::firstLegJoint ? (maxGearStepArms * sign) : (maxGearStepLegs * sign);
      emergencyChangeCounter[joint] = stepsBeforeEmergencyStep;
    }
    else
    {
      //adjust step is smaller than the minimal step the gears can do
      //this helps to reach a better optimal offset
      //check if we can do a big step to adjust the arm as fast as possible
      Angle dif = std::abs(jointDiffs[joint]);
      Angle step = joint < Joints::firstLegJoint && dif > maxGearStepArms ? maxGearStepArms :
                   (dif > maxGearStepLegs && joint >= Joints::firstLegJoint ? maxGearStepLegs : minGearStep);
      energySaving.offsets[joint] += (jointDiffs[joint] > 0 ? 1 : -1) * step;
    }
  }
}

bool EnergySavingProvider::isComStable(const bool& isStandHigh)
{
  if(!hasGroundContact || (theMotionInfo.isMotion(MotionPhase::stand) && !isStandHigh))
    return false;

  Vector3f comInTorso = theTorsoMatrix * theRobotModel.centerOfMass;
  return comXDiffRangeBeforeReset.isInside(comInTorso.x()) && comYDiffRangeBeforeReset.isInside(comInTorso.y()); // TODO untested
}
