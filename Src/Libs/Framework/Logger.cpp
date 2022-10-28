/**
 * @file Logger.cpp
 *
 * This file implements a class that writes a subset of representations into
 * log files. The representations can stem from multiple parallel threads.
 * The class maintains a buffer of message queues that can be claimed by
 * individual threads, filled with data, and given back to the logger for
 * writing them to the log file.
 *
 * @author Thomas Röfer
 */

#include "Logger.h"
#include "Debugging/AnnotationManager.h"
#include "Debugging/Debugging.h"
#include "Debugging/Stopwatch.h"
#include "Framework/Blackboard.h"
#include "Framework/LoggingTools.h"
#include "Framework/Settings.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Streaming/Global.h"
#include "Streaming/TypeInfo.h"
#include <cstdio>
#include <cstring>
#ifdef LINUX
#include <unistd.h>
#endif

#undef PRINT
#ifndef TARGET_ROBOT
#define PRINT(message) OUTPUT_WARNING(message)
#elif defined NDEBUG
#include <cstdlib>
#define PRINT(message) \
  do \
  { \
    OUTPUT_ERROR(message); \
    abort(); \
  } \
  while(false)
#else
#define PRINT(message) FAIL(message)
#endif

Logger::Logger(const Configuration& config) :
  typeInfo(200000),
  settings(200)
{
  TypeInfo::initCurrent();
  InMapFile stream("logger.cfg");
  ASSERT(stream.exists());
  stream >> *this;
  if(!path.empty() && path.back() != '/')
    path += "/";

  // Report wrong logger configuration (even in the simulator).
  // This check is only executed for the initial configuration, because on the robot that is usually the one that is used.
  if(enabled)
    for(const auto& rpt : representationsPerThread)
    {
      for(const auto& thread : config.threads)
        if(thread.name == rpt.thread)
        {
          for(const std::string& loggerRepresentation : rpt.representations)
          {
            for(const std::string& defaultRepresentation : config.defaultRepresentations)
              if(loggerRepresentation == defaultRepresentation)
              {
                PRINT("Logger: Thread " << rpt.thread << " should not log default representation " << defaultRepresentation);
                goto representationFound;
              }

            for(const auto& representationProvider : thread.representationProviders)
              if(loggerRepresentation == representationProvider.representation)
                goto representationFound;
            PRINT("Logger: Thread " << rpt.thread << " does not contain representation " << loggerRepresentation);
          representationFound:;
          }
          goto threadFound;
        }
      PRINT("Logger: Thread " << rpt.thread << " not found");
    threadFound:;
    }

#ifndef TARGET_ROBOT
  enabled = false;
  path = "Logs/";
#endif

  if(enabled)
  {
    typeInfo << *TypeInfo::current;
    LoggingTools::writeSettings(settings, Global::getSettings());

    buffers.resize(numOfBuffers);
    for(MessageQueue& buffer : buffers)
    {
      buffer.setSize(sizeOfBuffer);
      buffersAvailable.push(&buffer);
    }

    writerThread.setPriority(writePriority);
    writerThread.start(this, &Logger::writer);
  }
}

void Logger::update(bool shouldLog, const std::function<std::string()>& getDescription)
{
  if(!enabled)
    return;

  if(shouldLog != logging.load(std::memory_order_relaxed))
  {
    if(shouldLog)
    {
      // Tell the writer thread to start a new logging period.
      SYNC;
      if(!buffersAvailable.empty())
      {
        MessageQueue* buffer = buffersAvailable.top();
        buffersAvailable.pop();
        buffer->out.bin << (path + LoggingTools::createName("", Global::getSettings().headName, Global::getSettings().bodyName,
                                                            Global::getSettings().scenario, Global::getSettings().location,
                                                            getDescription(), Global::getSettings().playerNumber));
        buffer->out.finishMessage(undefined);
        buffersToWrite.push_back(buffer);
      }
      else
        return; // We couldn't start the writer, so \c logging stays false.
    }
    else
    {
      // Tell the writer thread that this logging period has ended.
      SYNC;
      buffersToWrite.push_back(nullptr);
    }
    framesToWrite.post();
    logging.store(shouldLog, std::memory_order_relaxed);
  }
}

bool Logger::execute(const std::string& threadName)
{
  if(!enabled)
    return true; // true because if the logger is not enabled, it doesn't make sense to keep data for later.

  // If not logging, stop here.
  if(!logging.load(std::memory_order_relaxed))
    return false;

  bool bufferAvailabilityChanged = false;
  for(const RepresentationsPerThread& rpt : representationsPerThread)
    if(rpt.thread == threadName && !rpt.representations.empty())
    {
      MessageQueue* buffer = nullptr;
      {
        SYNC;
        if(!buffersAvailable.empty())
        {
          buffer = buffersAvailable.top();
          buffersAvailable.pop();
        }
        const bool bufferIsAvailable = buffer != nullptr;
        bufferAvailabilityChanged = bufferWasAvailable != bufferIsAvailable;
        bufferWasAvailable = bufferIsAvailable;
      }
      if(!buffer)
      {
        if(bufferAvailabilityChanged)
          OUTPUT_WARNING("Logger: No buffer available!");
        return false;
      }
      else if(bufferAvailabilityChanged)
        OUTPUT_WARNING("Logger: Buffer available again!");

      STOPWATCH("Logger")
      {
        buffer->out.bin << threadName;
        buffer->out.finishMessage(idFrameBegin);

        for(const std::string& representation : rpt.representations)
#ifndef NDEBUG
          if(Blackboard::getInstance().exists(representation.c_str()))
#endif
          {
            buffer->out.bin << Blackboard::getInstance()[representation.c_str()];
            if(!buffer->out.finishMessage(static_cast<MessageID>(TypeRegistry::getEnumValue(typeid(MessageID).name(), "id" + representation))))
              OUTPUT_WARNING("Logger: Representation " << representation << " did not fit into buffer!");
          }
#ifndef NDEBUG
          else
            OUTPUT_WARNING("Logger: Representation " << representation << " does not exist!");
#endif

        Global::getAnnotationManager().getOut().copyAllMessages(*buffer);
      }
      Global::getTimingManager().getData().copyAllMessages(*buffer);
      buffer->out.bin << threadName;
      buffer->out.finishMessage(idFrameFinished);
      {
        SYNC;
        buffersToWrite.push_back(buffer);
        bufferWasAvailable = true;
      }
      framesToWrite.post();
      break;
    }

  return true;
}

Logger::~Logger()
{
  writerThread.announceStop();
  framesToWrite.post();
  writerThread.stop();
}

void Logger::writer()
{
  Thread::nameCurrentThread("Logger");
  BH_TRACE_INIT("Logger");

  class FilenameHandler : public MessageHandler
  {
  public:
    FilenameHandler(std::string& filename) :
      filename(filename)
    {}

  private:
    bool handleMessage(InMessage& message) override
    {
      ASSERT(message.getMessageID() == undefined);
      message.bin >> filename;
      return true;
    }

    std::string& filename;
  };

  OutBinaryFile* file = nullptr;
  MessageQueue* buffer = nullptr;
  std::string filename, completeFilename;
  FilenameHandler filenameHandler(filename);

  while(true)
  {
    // Wait for new data to log to arrive.
    framesToWrite.wait();

    // Terminate thread if it is told so or there is no disk space left.
    if(!writerThread.isRunning()
       || (!completeFilename.empty()
           && SystemCall::getFreeDiskSpace(completeFilename.c_str()) < static_cast<unsigned long long>(minFreeDriveSpace) << 20))
      break;

    // Get next buffer to write (there must be one).
    {
      SYNC;
      buffer = buffersToWrite.front();
    }

    if(!buffer)
    {
      // Sync the current file to disk and close it.
      ASSERT(file);
#ifdef LINUX
      ::fsync(::fileno(static_cast<FILE*>(file->getFile()->getNativeFile())));
#endif
      delete file;
      file = nullptr;
      SystemCall::say("Log file written");
    }
    else if(buffer->getNumberOfMessages() == 1)
    {
      // All "real" buffers have at least two messages (idFrameBegin and idFrameFinished).

      // Read filename from message queue.
      buffer->handleAllMessages(filenameHandler);
      buffer->clear();

      // find next free log filename
      for(int i = 0; i < 100; ++i)
      {
        completeFilename = filename + (i ? "_(" + ((i < 10 ? "0" : "") + std::to_string(i)) + ")" : "") + ".log";
        InBinaryFile stream(completeFilename);
        if(!stream.exists())
          break;
      }

      ASSERT(!file);
      file = new OutBinaryFile(completeFilename);
      if(!file->exists())
      {
        OUTPUT_WARNING("Logger: File " << completeFilename << " could not be created!");
        break;
      }

      *file << LoggingTools::logFileSettings;
      file->write(settings.data(), settings.size());
      *file << LoggingTools::logFileMessageIDs;
      buffer->writeMessageIDs(*file);
      *file << LoggingTools::logFileTypeInfo;
      file->write(typeInfo.data(), typeInfo.size());
      *file << LoggingTools::logFileUncompressed;
      buffer->writeAppendableHeader(*file);

      // Turn off userspace buffering.
      std::setvbuf(static_cast<std::FILE*>(file->getFile()->getNativeFile()), nullptr, _IONBF, 0);
    }
    else
    {
      // Write buffered frame to file.
      if(file)
        buffer->append(*file);
      buffer->clear();
    }

    // Return the buffer.
    {
      SYNC;
      buffersToWrite.pop_front();
      if(buffer)
        buffersAvailable.push(buffer);
    }
  }

  // Delete file before thread ends.
  delete file;
}
