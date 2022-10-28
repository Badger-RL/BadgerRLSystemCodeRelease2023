/**
 * @file LibLookActiveProvider.h
 * @author Andreas Stolpmann
 * @author Florian Scholz
 */

#include "Framework/Module.h"
#include "Debugging/Debugging.h"
#include "Debugging/DebugDrawings.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibLookActive.h"
#include "Representations/BehaviorControl/Libraries/LibTeam.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Math/RingBufferWithSum.h"

MODULE(LibLookActiveProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  USES(BehaviorStatus),
  USES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(FrameInfo),
  REQUIRES(HeadLimits),
  REQUIRES(GameState),
  USES(HeadMotionRequest),
  REQUIRES(JointAngles),
  REQUIRES(JointRequest),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotPose),
  REQUIRES(TeammatesBallModel),
  REQUIRES(LibTeam),
  REQUIRES(MotionInfo),
  REQUIRES(TorsoMatrix),
  PROVIDES(LibLookActive),
  LOADS_PARAMETERS(
  {,
    (Angle) defaultSpeed,
    (Angle) minSpeed,
    (Angle) maxSpeed,
    (Angle) largeDefaultPan,
    (Angle) smallDefaultPan,
    (Angle) minPanMoving,
    (Angle) defaultTilt,
    (Angle) minTilt,
    (int) ballPositionUnknownTimeout,
    (float) translationSpeedMinValue,
    (float) translationSpeedMaxValue,
    (Angle) rotationSpeedMinValue,
    (Angle) rotationSpeedMaxValue,
    (bool) lookAtCloseObstacleWhenFollowingBall,
    (float) maxObstacleDistanceToBeLookedAt,
    (int) maxObstacleAgeToBeLookedAt,
    (Angle) cameraChoiceHysteresis,
  }),
});

class LibLookActiveProvider : public LibLookActiveProviderBase
{
private:
  Vector2f theBallPositionRelative;
  Vector2f theBallSpeedRelative;

  bool teamBallIsUsed = false;

  float translationSpeedFactor = 1.f;
  float rotationSpeedFactor = 1.f;

  int basePanAngleIndex = 0;

  Angle calculatePan(const bool forceBall);
  Angle calculateTilt(const bool forceBall, const bool onlyOwnBall) const;
  Angle calculateSpeed(const bool forceBall, const float targetPan) const;

  bool shouldLookAtBall() const;

  bool panReached(const Angle pan) const;
  bool ballAnglesReachable() const;
  bool ballInSight() const;
  bool ballPositionUnknown(const bool onlyOwnBall) const;

  Angle clipPanToBall(const Angle pan) const;

  Angle clipPanToNearObstacle(const Angle pan) const;

  float getTranslationOffset(float x) const;

  RingBufferWithSum<float, 180> translationSpeedBuffer;
  RingBufferWithSum<Angle, 120> rotationSpeedBuffer;
  void calculateSpeedFactors();
  HeadTarget calculateHeadTarget(const bool withBall, const bool ignoreBall, const bool withOwnBall, bool fixTilt);
  void update(LibLookActive& libLookActive) override;
};
