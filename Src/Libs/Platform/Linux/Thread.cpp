#include "Platform/BHAssert.h"
#include "Platform/Thread.h"

#include <pthread.h>

#ifdef MACOS
static struct Helper
{
  int basePriority;

  Helper()
  {
    int policy;
    sched_param param;
    VERIFY(!pthread_getschedparam(pthread_self(), &policy, &param));
    basePriority = param.sched_priority;
  }
} helper;
#endif

thread_local Thread* Thread::instance = nullptr;

void Thread::stop()
{
  running = false;
  if(thread && thread->joinable())
  {
    if(!terminated.wait(10000))
      pthread_cancel(thread->native_handle());
    thread->join();
    delete thread;
    thread = nullptr;
  }
}

void Thread::threadStart(const std::function<void()>& lambda)
{
  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, 0);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, 0);
  lambda();
  SYNC;
  running = false;
  terminated.post();
}

const std::string Thread::getCurrentThreadName()
{
  char cname[16];
  VERIFY(!pthread_getname_np(pthread_self(), cname, 16));
  std::string name(cname);
  demangleThreadName(name);
  return name;
}

void Thread::changePriority()
{
  SYNC;
  if(thread && running)
  {
    ASSERT((priority >= -2 && priority <= 0)
           || (priority > 0 && priority <= sched_get_priority_max(SCHED_FIFO)));
    sched_param param;
    param.sched_priority = 0;
    switch(priority)
    {
      case -2:
#ifdef TARGET_ROBOT
        VERIFY(!pthread_setschedparam(thread->native_handle(), SCHED_IDLE, &param));
        break;
#endif
      case -1:
#ifdef TARGET_ROBOT
        VERIFY(!pthread_setschedparam(thread->native_handle(), SCHED_BATCH, &param));
        break;
#endif
      case 0:
        VERIFY(!pthread_setschedparam(thread->native_handle(), SCHED_OTHER, &param));
        break;
      default:
        param.sched_priority = priority
#ifdef MACOS
        + helper.basePriority
#endif
        ;
        VERIFY(!pthread_setschedparam(thread->native_handle(), SCHED_FIFO, &param));
    }
  }
}

void Thread::nameCurrentThread(const std::string& name)
{
#ifdef LINUX
  char cname[16] = "";
#else
  char cname[64] = "";
#endif
  name.copy(cname, sizeof(cname) - 1);
  cname[sizeof(cname) - 1] = '\0';
#ifdef LINUX
  VERIFY(!pthread_setname_np(pthread_self(), cname));
#else
  VERIFY(!pthread_setname_np(cname));
#endif
}
