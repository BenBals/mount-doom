#pragma once

#include "graph.h"
#include "fvs_instance.h"

namespace datastructures {
FvsInstance read_instance_from_file(const std::filesystem::path &filename);
FvsInstance read_instance_from_istream(std::istream &stream);
}
