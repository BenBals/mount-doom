#pragma once

#include "spdlog/spdlog.h"
#include <filesystem>
#include <string>
#include <time.h>


namespace helpers {
void assert_and_log([[maybe_unused]] bool cond, [[maybe_unused]] const std::string &message);

void initialize_signal_handlers();

void set_timeout(time_t seconds, long nano_seconds);

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