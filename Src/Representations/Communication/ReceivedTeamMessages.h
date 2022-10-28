/**
 * @file ReceivedTeamMessages.h
 *
 * This file declares a representation that contains the team messages
 * that have been received in a single frame.
 *
 * @author Arne Hasselbrng
 */

#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Communication/SPLStandardMessageBuffer.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(ReceivedTeamMessage,
{,
  (int)(-1) number,

  (signed char)(0) sequenceNumber,
  (signed char)(0) returnSequenceNumber,

  (bool)(true) isUpright,
  (unsigned)(0) timeWhenLastUpright,

  (RobotPose) theRobotPose,
  (BallModel) theBallModel,
  (FrameInfo) theFrameInfo,
  (ObstacleModel) theObstacleModel,
  (BehaviorStatus) theBehaviorStatus,
  (Whistle) theWhistle,
  (StrategyStatus) theStrategyStatus,
});

STREAMABLE(ReceivedTeamMessages,
{,
  (std::vector<ReceivedTeamMessage>) messages, /**< An unordered(!) list of all team messages that have been received in this frame. */
  (unsigned)(0) unsynchronizedMessages,        /**< The number of received team messages in this frame that were rejected due to lack of known clock offset. */
});
