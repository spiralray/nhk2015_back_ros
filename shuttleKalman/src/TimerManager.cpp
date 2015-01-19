#include <algorithm>
#include <iterator>
#include <sys/time.h>
#include <unistd.h>
#include "TimerManager.h"

extern "C" void *create_pthread(void *data)
{
  TimerManager *thread_timer_manager = static_cast<TimerManager *>(data);
  thread_timer_manager->run();
  return data;
}

TimerManager::TimerManager() :
  m_bRunning(false),
  m_bGo(false),
  m_lMinSleep(0)
{
  int mutex_creation = pthread_mutex_init(&m_tGoLock, NULL);
  if(mutex_creation != 0) {
    std::cerr << "Failed to create mutex" << std::endl;                                   // Use RAII if resource acquisition fails
    return;
  }

  int mutex_cond_creation = pthread_cond_init(&m_tGoLockCondition, NULL);
  if(mutex_cond_creation != 0) {
    std::cerr << "Failed to create condition mutex" << std::endl;                         // Use RAII if resource acquisition fails
    return;
  }

  int thread_creation = pthread_create(&m_tTimerThread, NULL, create_pthread, this);      // On success call create_pthread to start tiemr loop
  if(thread_creation != 0) {
    std::cerr << "Failed to create thread" << std::endl;                                  // Use RAII if resource acquisition fails
    return;
  }
  m_bRunning = true;
}

TimerManager::~TimerManager()
{
    pthread_mutex_lock(&m_tGoLock);                                                       // m_bRunning access on other thread
    m_bRunning = false;
    pthread_mutex_unlock(&m_tGoLock);
    void *result;
    pthread_join(m_tTimerThread, &result);                                                // Do not let calling thread exit before deleting
    pthread_mutex_destroy(&m_tGoLock);                                                    // Now destroy the mutex (release resources)
    pthread_cond_destroy(&m_tGoLockCondition);
}

void TimerManager::run()
{
  pthread_mutex_lock(&m_tGoLock);                                                         // Timers run on seperate thread
  while(m_bRunning) {                                                                     // While timer manager exists
    while (!m_bGo) {                                                                      // While timer manager told to run
      pthread_cond_wait(&m_tGoLockCondition, &m_tGoLock);                                 // Set in the start() member function
    }
    pthread_mutex_unlock(&m_tGoLock);                                                     // Once timer unlocked and mutex released
    if (!m_bRunning) {                                                                    // Make sure timer manager not out of scope
      break;
    }

    struct timeval l_tv;
    usleep(std::max(0l, m_lMinSleep));
    gettimeofday(&l_tv, NULL);                                                            // System call to get time of day
    m_lMinSleep = 0;                                                                      // Used to sleep if no timer to go off soon
    long l_lMin = 0;                                                                      // Used if timer goes off to get actual Min sleep
    for(std::list<Timer>::iterator it = m_cTimers.begin(); it != m_cTimers.end(); ++it) { // Iterate over timers to see which one is going off
      TimerManager::Timer l_oTimer = *it;                                                 // Obtain a copy of the timer
      long elapsed_time = ((l_tv.tv_sec * 1000000 + l_tv.tv_usec) - (l_oTimer.start));    // Calcuate the elapsed time from the start of timer X
      l_lMin = elapsed_time - l_oTimer.duration;                                          // Minimum time you can sleep in loop
      if (elapsed_time >= l_oTimer.duration) {                                            // If time passed is greater than or equal to duration: THEN
        l_lMin = l_oTimer.duration;                                                       // The minimum you can wait is possibly the entire duration of that timer
        l_oTimer.callback(0);                                                             // Call the call back
        gettimeofday(&l_tv, NULL);                                                        // After callback called...
        it->start = (l_tv.tv_sec * 1000000) + l_tv.tv_usec;                               // Start the timer over again
      }
      m_lMinSleep = std::min(m_lMinSleep, l_lMin);                                        // Find the actual minumum time you can sleep to not lock
    }
  }
}

void TimerManager::start()
{
  pthread_mutex_lock(&m_tGoLock);                                                         // Go flag accessed from another thread
  m_bGo = true;
  pthread_cond_signal(&m_tGoLockCondition);
  pthread_mutex_unlock(&m_tGoLock);
}

void TimerManager::stop()
{
  pthread_mutex_lock(&m_tGoLock);                                                         // Go flag accessed from another thread
  m_bGo = false;
  pthread_mutex_unlock(&m_tGoLock);
}

TimerManager::Timer TimerManager::setUpTimer(long micro_duration, void (*callback)(int id))
{
  struct timeval l_tv;
  gettimeofday(&l_tv, NULL);                                                              // System call to get the ms and sec since Epoch0
  Timer l_oTimer(micro_duration, callback);                                               // Create a timer
  l_oTimer.start = (l_tv.tv_sec * 1000000) + l_tv.tv_usec;                                // Tell the timer when to start
  return l_oTimer;                                                                        // Return a copy to addTimer
}

void TimerManager::addTimer(long usec, void (*callback)(int id))
{
  pthread_mutex_lock(&m_tGoLock);                                                         // Tell object to wait till timer inserted
  Timer insert = setUpTimer(usec, callback);
  for (std::list<Timer>::iterator it = m_cTimers.begin(); it != m_cTimers.end(); ++it) {
    if (*it == insert) {                                                                  // Return if timer callback and duration the same
        return;
    }
  }
  m_cTimers.push_back(insert);                                                            // If no duplicate timers found then insert into list
  pthread_mutex_unlock(&m_tGoLock);                                                       // Tell object it is okay to proceed
}
