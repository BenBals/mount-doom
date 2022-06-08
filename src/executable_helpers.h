#pragma once

#include <filesystem>
#include <string>

namespace executable_helpers {
bool init_logging_to_file(const std::string &executable_name);
std::filesystem::path find_vcs_root();
} // namespace executable_helpers