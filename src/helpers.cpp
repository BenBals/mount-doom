#include "spdlog/spdlog.h"
#include <optional>
#include <signal.h>
#include <vc-satreduce/include/bnb.hpp>

#ifdef __MACH__
// compatibility layer for macOS
#include "macos_timer.h"
#define SIGRTMIN SIGUSR1
#endif

#define CLOCKID CLOCK_REALTIME
#define TIMER_SIGNAL SIGRTMIN

namespace helpers {
void assert_and_log([[maybe_unused]] bool cond, [[maybe_unused]] const std::string &message) {
#ifdef NDEBUG
  return;
#endif
  if (!cond) {
    spdlog::error(message);
    exit(1);
  }
}

volatile sig_atomic_t timeout_hit = 0, termination_hit = 0;
static std::optional<timer_t> timer_opt;
static void timer_signal_handler(int sig) {
  vc_bnb::catch_sigxcpu(sig);
  timeout_hit = 1;
}

static void termination_signal_handler(int sig) {
  timer_signal_handler(sig);
  termination_hit = 1;
}

bool is_timeout_hit() { return timeout_hit != 0; }

bool is_termination_requested() { return termination_hit != 0; }

void initialize_signal_handlers() {
  // register timer handler
  {
    struct sigaction sa {};
    sa.sa_handler = timer_signal_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    if (sigaction(TIMER_SIGNAL, &sa, nullptr) == -1) {
      spdlog::critical("sigaction TIMER_SIGNAL({})", TIMER_SIGNAL);
      exit(1);
    }
  }

  {
    // register termination handler
    struct sigaction sa {};
    sa.sa_handler = termination_signal_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGTERM, &sa, nullptr) == -1) {
      spdlog::critical("sigaction SIGTERM({})", SIGTERM);
      exit(1);
    }
  }
	
  {
    // create own 10 minute timer 
    timer_t timerid;
    struct sigevent sev {};
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGTERM;
    sev.sigev_value.sival_ptr = &timerid;
    if (timer_create(CLOCKID, &sev, &timerid) == -1) {
      spdlog::critical("timer_create");
      exit(1);
    }

    struct itimerspec its {};
    its.it_value.tv_sec = 10 * 60;
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;

    // flags is 0 as we want the timer to be relative not absolute
    if (timer_settime(timerid, 0, &its, nullptr) == -1) {
      spdlog::critical("timer_settime in set_timeout");
      exit(1);
    }
  }
}

void initialize_timer() {
  // create timer
  timer_t timerid;
  struct sigevent sev {};
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = TIMER_SIGNAL;
  sev.sigev_value.sival_ptr = &timerid;
  if (timer_create(CLOCKID, &sev, &timerid) == -1) {
    spdlog::critical("timer_create");
    exit(1);
  }

  timer_opt = {timerid};
}

void set_timeout(time_t seconds, long nano_seconds) {
  if (!timer_opt) {
    initialize_timer();
  }

  struct itimerspec its {};
  // set timeout
  its.it_value.tv_sec = seconds;
  its.it_value.tv_nsec = nano_seconds;
  // interval is not needed
  its.it_interval.tv_sec = 0;
  its.it_interval.tv_nsec = 0;

  // flags is 0 as we want the timer to be relative not absolute
  if (timer_settime(timer_opt.value(), 0, &its, nullptr) == -1) {
    spdlog::critical("timer_settime in set_timeout");
    exit(1);
  }
}

void reset_timeout() {
  if (!timer_opt) {
    initialize_timer();
  }

  struct itimerspec its {};
  its.it_value.tv_sec = 0;
  its.it_value.tv_nsec = 0;
  its.it_interval.tv_sec = 0;
  its.it_interval.tv_nsec = 0;
  if (timer_settime(timer_opt.value(), 0, &its, nullptr) == -1) {
    spdlog::critical("timer_settime in reset_timeout");
    exit(1);
  }

  // clear timeout flags
  if (!is_termination_requested()) {
    vc_bnb::reset_timeout();
    timeout_hit = 0;
  }
}
} // namespace helpers
