/**
 * @file TeamMessageHandler.h
 *
 * Declares a module that both sends and receives team messages.
 * It ensures that less messages are sent than are allowed. It also checks whether
 * the data that would be sent is significantly different from the data that was last
 * sent. Otherwise, sending the message is skipped.
 *
 * @author Jesse Richter-Klug
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/SPLMessageHandler.h" // include this first to prevent WinSock2.h/Windows.h conflicts
#include "Representations/Communication/BHumanMessageOutputGenerator.h"
#include "Representations/Communication/GameControllerData.h"
#include "Representations/Communication/ReceivedTeamMessages.h"
#include "Representations/Communication/SentTeamMessage.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Framework/Module.h"
#include "Tools/Communication/GameControllerRBS.h"
#include "Tools/Communication/CompressedTeamCommunicationStreams.h"
#include <map>

MODULE(TeamMessageHandler,
{,
  // for calculations
  REQUIRES(FrameInfo),
  REQUIRES(GameControllerData),
  REQUIRES(MotionInfo),
  USES(GameState),
  USES(ExtendedGameState),
  USES(MotionRequest),
  USES(TeamData),
  USES(TeammatesBallModel),

  // extract data to send
  REQUIRES(FallDownState),
  REQUIRES(GroundContactState),

  // send directly
  USES(BallModel),
  USES(BehaviorStatus),
  REQUIRES(FieldFeatureOverview),
  USES(ObstacleModel),
  USES(RobotHealth),
  USES(RobotPose),
  USES(StrategyStatus),
  USES(Whistle),

  PROVIDES(BHumanMessageOutputGenerator),
  PROVIDES(ReceivedTeamMessages),
  PROVIDES(SentTeamMessage),

  // for communication limitation
  REQUIRES(BallSpecification),

  LOADS_PARAMETERS(
  {,
    (int) minSendInterval, /**<  Minimum time in ms between two messages that are sent to the teammates */
    (int) durationOfHalf, /**< Duration of a half in ms. */
    (unsigned) overallMessageBudget, /**< The message budget available for the whole game. */
    (unsigned) normalMessageReserve, /**< A reserve kept from the budget by normal messages, i.e. at the end of the game, this number of messages should be unused. */
    (unsigned) priorityMessageReserve, /**< A (smaller) reserve kept from the budget by priority messages to allow sending priority messages if normal messages cannot be sent. */
    (int) lookahead, /**< A time in ms in the future in which the message budget has to be kept. */
    (float) positionThreshold, /**< How much must positions have changed at least to be considered as such (in mm)? */
    (Angle) bearingThreshold, /**< How much must angles to positions have changed at least to be considered as such (in radians)? */
    (float) assumedObservationHeight, /**< Height robots are assumed to observe the field (in mm). */
    (int) teamBallThresholdBase, /**< How long without the team seeing the ball before reporting a new ball at the same position (in ms)? */
    (int) teamBallThresholdFactor, /**< How long the wait in addition based on the player number (in ms)? */
    (int) newBallThreshold, /**< How old can a ball be to be still considered "new" (in ms)? */
    (int) disappearedThreshold, /**< Threshold between disappeared and not disappeared (in ms). Crossing it initiates sending. */
    (int) ignoreWhistleBeforeEndOfHalf, /**< Time before the end of a half after which the whistle is not sent anymore (in ms). */
    (int) maxWhistleSendDelay, /**< The time after which whistle messages are not sent anymore (in ms). */
    (int) minTimeBetween2RejectSounds, /**< Time in ms after which another sound output is allowed */
    (bool) sendMirroredRobotPose, /**< Whether to send the robot pose mirrored (useful for one vs one demos such that keeper and striker can share their ball positions). */
    (bool) dropUnsynchronizedMessages, /**< Whether messages in which timestamps cannot be converted should be dropped. */
    (bool) alwaysSend, /**< Send every second. */
  }),
});

class TeamMessageHandler : public TeamMessageHandlerBase
{
public:
  TeamMessageHandler();

private:
  /**
   * Class for creating statistics about value changes in representations
   * potentially sent.
   */
  struct Statistics : public Streamable
  {
    /** Helper for streaming counters. */
    STREAMABLE(Counter,
    {
      Counter() = default;
      Counter(const std::string& name, unsigned changes) : name(name) COMMA changes(changes) {},
      (std::string) name,
      (unsigned) changes,
    });

    /**
     * Increase counter if a value has changed.
     * @param name The name of the counter.
     * @param valueChanged Has the value changed?
     */
    void count(const std::string& name, const bool valueChanged)
    {
      unsigned& counter = counters[name];
      if(valueChanged)
        ++counter;
    }

    /**
     * Streaming support for reading.
     * @param stream The stream from which is read.
     */
    void read(In& stream) override
    {
      std::vector<Counter> counters;
      STREAM(counters);
      this->counters.clear();
      for(const Counter& counter : counters)
        this->counters[counter.name] = counter.changes;
    }

    /**
     * Streaming support for writing.
     * @param stream The stream to which is written.
     */
    void write(Out& stream) const override
    {
      PUBLISH(reg);
      std::vector<Counter> counters;
      counters.reserve(this->counters.size());
      for(auto& counter : this->counters)
        counters.emplace_back(counter.first, counter.second);
      STREAM(counters);
    }

    /** Support for type registration. */
    static void reg()
    {
      REG_CLASS(Statistics);
      REG(std::vector<Counter>, counters);
    }

    std::map<std::string, unsigned> counters; /**< The counters. */
  };

  SPLMessageHandler::Buffer inTeamMessages;
  RoboCup::SPLStandardMessage outTeamMessage;
  SPLMessageHandler theSPLMessageHandler;

  mutable Statistics statistics; /**< The change statistics. */
  SentTeamMessage lastSent; /**< Last team message that was sent. */

  GameControllerRBS theGameControllerRBS;
  mutable RobotStatus theRobotStatus;

  CompressedTeamCommunication::TypeRegistry teamCommunicationTypeRegistry;
  const CompressedTeamCommunication::Type* teamMessageType;

  // output stuff
  mutable unsigned timeWhenLastSent = 0;

  void update(BHumanMessageOutputGenerator& outputGenerator) override;
  void generateMessage(BHumanMessageOutputGenerator& outputGenerator) const;
  void writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m);

  // input stuff
  struct ReceivedBHumanMessage : public BHumanMessage
  {
    const SynchronizationMeasurementsBuffer* bSMB = nullptr;
    unsigned toLocalTimestamp(unsigned remoteTimestamp) const override
    {
      if(bSMB)
        return bSMB->getRemoteTimeInLocalTime(remoteTimestamp);
      else
        return 0u;
    };

    enum ErrorCode
    {
      //add more parsing errors if there is a need of distinguishing
      parsingError,
      magicNumberDidNotMatch,
      myOwnMessage
    } lastErrorCode;
  } receivedMessageContainer;

  static void regTeamMessage();

  void update(ReceivedTeamMessages& receivedTeamMessages) override;
  void update(SentTeamMessage& theSentTeamMessage) override {theSentTeamMessage = lastSent;}

  unsigned timeWhenLastMimimi = 0;
  bool readSPLStandardMessage(const RoboCup::SPLStandardMessage* const m);
  void parseMessage(ReceivedTeamMessage& bMate);

  /** Make a backup of the representations that are sent. */
  void backup();

  /**
   * Checks whether the global coordinates of a relative position have changed.
   * What means "changed" is weighted by the bearings to the relative position.
   * @param origin The current pose of the robot.
   * @param offset The current relative position.
   * @param oldOrigin The previous pose of the robot.
   * @param oldOffset The previous relative position.
   * @param checkZero Check for offsets being zero, which means that they are invalid.
   * @return Was there sufficient change between the previous and the current
   *         global position?
   */
  bool globalBearingsChanged(const RobotPose& origin, const Vector2f& offset,
                             const RobotPose& oldOrigin, const Vector2f& oldOffset,
                             bool checkZero = false) const;

  /**
   * Has a position changed significantly seen from the perspective of a teammate?
   * @param position The current position of the observed object.
   * @param oldPosition The previous position of the observed object.
   * @return Has it changed significantly for any teammate?
   */
  bool teammateBearingsChanged(const Vector2f& position, const Vector2f& oldPosition) const;

  /**
   * Returns whether enough time has passed since the last message was sent.
   * This is determined by the parameter "minSendInterval".
   * @return Has enough time passed?
   */
  bool enoughTimePassed() const;

  /**
   * Is the robot in a "stiff" state?
   * This always returns true in the simulator.
   * @return Is the current motion something else than "play dead"?
   */
  bool notInPlayDead() const;

  /**
   * Checks whether a normal message could be sent without violating
   * the floating message budget. This check targets to keep the
   * "normalMessageReserve" at the end of the game.
   * @return Ok to send a normal message?
   */
  bool withinNormalBudget() const;

  /**
   * Checks whether a priority message could be sent without violating
   * the message budget. This check targets to keep the
   * "priorityMessageReserve" at the end of the game.
   * @return Ok to send a priority message?
   */
  bool withinPriorityBudget() const;

  /** Was a whistle detected since the last message was sent? */
  bool whistleDetected() const;

  /** Did the behavior status change since the last message was sent? */
  bool behaviorStatusChanged() const;

  /** Did the robot status change since the last message was sent? */
  bool robotStatusChanged() const;

  /** Did the strategy status change since the last message was sent? */
  bool strategyStatusChanged() const;

  /** Is the robot pose meaningful? */
  bool robotPoseValid() const;

  /** Did the robot pose change since the last message was sent? */
  bool robotPoseChanged() const;

  /** Did the ball model change since the last message was sent? */
  bool ballModelChanged() const;

  /** Is the team ball old and we know better? */
  bool teamBallOld() const;
};
