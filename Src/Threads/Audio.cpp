/**
 * @file Threads/Audio.cpp
 *
 * This file implements the execution unit for the audio thread.
 *
 * @author Jan Fiedler
 * @author Thomas Röfer
 */

#include "Audio.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Platform/Thread.h"

REGISTER_EXECUTION_UNIT(Audio)

bool Audio::beforeFrame()
{
  return LogDataProvider::isFrameDataComplete();
}

bool Audio::afterFrame()
{
  if(!Blackboard::getInstance().exists("AudioData"))
    Thread::sleep(static_cast<unsigned>(100));

  return FrameExecutionUnit::afterFrame();
}
