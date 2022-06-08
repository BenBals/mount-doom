// compatibility layer for POSIX timers on macOS
// Inspired by
// https://gist.githubusercontent.com/lundman/731d0d7d09eca072cd1224adb00d9b9e/raw/4687eb828175245f336d8790c5aae83c4d0f2222/time.h

#include "spdlog/spdlog.h"
#include <csignal>
#include <ctime>
#include <mach/boolean.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/errno.h>

#include <dispatch/dispatch.h>

struct itimerspec {
  struct timespec it_interval; /* timer period */
  struct timespec it_value;    /* timer expiration */
};

struct sigevent;

/* If used a lot, queue should probably be outside of this struct */
struct macos_timer {
  dispatch_queue_t tim_queue;
  dispatch_source_t tim_timer;
  void (*tim_func)(union sigval);
  int tim_signo;
  void *tim_arg;
};

typedef struct macos_timer *timer_t;

static inline void _timer_cancel(void *arg) {
  struct macos_timer *tim = (struct macos_timer *)arg;
  dispatch_release(tim->tim_timer);
  dispatch_release(tim->tim_queue);
  tim->tim_timer = NULL;
  tim->tim_queue = NULL;
  free(tim);
}

static inline void _timer_handler(void *arg) {
  struct macos_timer *tim = (struct macos_timer *)arg;
  union sigval sv;

  sv.sival_ptr = tim->tim_arg;

  if (tim->tim_func != NULL)
    tim->tim_func(sv);
  else if (std::raise(tim->tim_signo) != 0) {
    spdlog::critical("could not dispatch signal handler");
  }
}

static inline int timer_create(clockid_t clockid, struct sigevent *sevp, timer_t *timerid) {
  struct macos_timer *tim;

  *timerid = NULL;

  switch (clockid) {
  case CLOCK_REALTIME:

    /* What is implemented so far */
    if (sevp->sigev_notify != SIGEV_THREAD && sevp->sigev_notify != SIGEV_SIGNAL) {
      errno = ENOTSUP;
      return (-1);
    }

    tim = (struct macos_timer *)malloc(sizeof(struct macos_timer));
    if (tim == NULL) {
      errno = ENOMEM;
      return (-1);
    }

    tim->tim_queue = dispatch_queue_create("org.openzfsonosx.timerqueue", 0);
    tim->tim_timer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, tim->tim_queue);

    tim->tim_func = sevp->sigev_notify_function;
    tim->tim_arg = sevp->sigev_value.sival_ptr;
    tim->tim_signo = sevp->sigev_signo;
    *timerid = tim;

    /* Opting to use pure C instead of Block versions */
    dispatch_set_context(tim->tim_timer, tim);
    dispatch_source_set_event_handler_f(tim->tim_timer, _timer_handler);
    dispatch_source_set_cancel_handler_f(tim->tim_timer, _timer_cancel);

    return (0);
  default:
    break;
  }

  errno = EINVAL;
  return (-1);
}

static inline int timer_settime(timer_t tim, int /* flags */, const struct itimerspec *its,
                                struct itimerspec * /* remainvalue */) {
  if (tim != NULL) {

    /* Both zero, is disarm */
    if (its->it_value.tv_sec == 0 && its->it_value.tv_nsec == 0) {
      /* There's a comment about suspend count in Apple docs */
      dispatch_suspend(tim->tim_timer);
      return (0);
    }

    dispatch_time_t start;
    start =
        dispatch_time(DISPATCH_TIME_NOW, static_cast<int64_t>(NSEC_PER_SEC) * its->it_value.tv_sec +
                                             its->it_value.tv_nsec);
    if (its->it_interval.tv_sec == 0 && its->it_interval.tv_nsec == 0) {
      dispatch_source_set_timer(tim->tim_timer, start, DISPATCH_TIME_FOREVER, 0);
    } else {
      dispatch_source_set_timer(
          tim->tim_timer, start,
          NSEC_PER_SEC * static_cast<uint64_t>(its->it_interval.tv_sec + its->it_interval.tv_nsec),
          0);
    }
    dispatch_activate(tim->tim_timer);
    dispatch_resume(tim->tim_timer);
  }
  return (0);
}

static inline int timer_delete(timer_t tim) {
  /* Calls _timer_cancel() */
  if (tim != NULL)
    dispatch_source_cancel(tim->tim_timer);

  return (0);
}
