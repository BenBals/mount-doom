#pragma once

#include "spdlog/spdlog.h"
#include <chrono>
#include <filesystem>
#include <string>
#include <ctime>

namespace helpers {
void assert_and_log([[maybe_unused]] bool cond, [[maybe_unused]] const std::string &message);

extern std::optional<std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds>>
    runout_time_point;

void initialize_signal_handlers();

void set_timeout_inner(time_t seconds, long nanoseconds);

template <class Rep, class Period>
static decltype(runout_time_point) set_timeout(std::chrono::duration<Rep, Period> duration) {
  auto nano_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);

  auto old_timeout_point = runout_time_point;
  set_timeout_inner(nano_duration.count() / 1000000000, nano_duration.count() % 1000000000);
  return old_timeout_point;
}

decltype(runout_time_point)
set_timeout_to(std::chrono::time_point<std::chrono::high_resolution_clock> time_point);

std::optional<std::chrono::nanoseconds> remaining_time();

template <class Rep, class Period>
static std::chrono::duration<Rep, Period>
min_with_remaining(std::chrono::duration<Rep, Period> duration) {
  auto remaining_time_opt = remaining_time();
  if (!remaining_time_opt)
    return duration;
  return std::min(duration,
                  std::chrono::duration_cast<decltype(duration)>(remaining_time_opt.value()));
}

void reset_timeout();

bool is_timeout_hit();

bool is_termination_requested();

/**
 * This concept asserts, that we can use F(Args...) to induce a total_order on Args...
 */
template <class F, class... Args>
concept keying_function =
    std::regular_invocable<F, Args...> && std::totally_ordered<std::invoke_result_t<F, Args...>>;

} // namespace helpers