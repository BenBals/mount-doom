#include "spdlog/cfg/env.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace executable_helpers {

namespace fs = std::filesystem;

fs::path find_vcs_root() {
  auto current_dir = fs::current_path();

  while (!fs::is_directory(current_dir / ".git") && current_dir != current_dir.root_path()) {
    current_dir = current_dir.parent_path();
  }

  if (current_dir == current_dir.root_path()) {
    return fs::current_path();
  }

  return current_dir;
}

bool init_logging_to_file(const std::string &executable_name) {
  try {

    auto current_time = std::time(nullptr);
    auto tm = *std::localtime(&current_time);
    std::ostringstream filename_stream;
    filename_stream << find_vcs_root().c_str() << "/logs/" << executable_name << "-"
                    << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".log";

    auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);

    auto file_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename_stream.str(), true);
    file_sink->set_level(spdlog::level::trace);

    spdlog::logger logger("multi_sink", {console_sink, file_sink});

    logger.set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%@] %v");
    spdlog::set_default_logger(std::make_shared<spdlog::logger>(
        "multi_sink", spdlog::sinks_init_list({console_sink, file_sink})));
    logger.set_level(spdlog::level::debug);

    // Example:
    // export SPDLOG_LEVEL=info
    spdlog::cfg::load_env_levels();
    return true;
  } catch (const spdlog::spdlog_ex &ex) {
    std::clog << "Log init failed: " << ex.what() << std::endl;
    return false;
  }
}
} // namespace executable_helpers
