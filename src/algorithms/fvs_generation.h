#pragma once
#include "datastructures.h"

using namespace datastructures;

namespace algorithms {

FvsInstance generate_overlapping_cycles(size_t cycles, size_t cycle_size, size_t edges);
FvsInstance generate_wide_cycle(size_t perimeter, size_t width);
FvsInstance generate_vertex_cover_like(size_t num_nodes, size_t extra_arcs, size_t seed);
FvsInstance generate_random_instance(size_t num_nodes, size_t num_arcs, size_t seed);
FvsInstance gen_random_instance(size_t num_nodes, size_t num_arcs, size_t seed, bool selfloops_allowed);
} // namespace algorithms