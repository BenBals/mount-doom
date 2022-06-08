#include "greedy.h"
namespace algorithms {
MaxByDegreeHeuristicSolver::MaxByDegreeHeuristicSolver()
    : MaxByKeyHeuristicSolver(&degree_for_node_key) {}
MaxByDegreeHeuristicSolver::MaxByDegreeHeuristicSolver(bool use_two_one_swaps, size_t reduce_after_taking_n)
    : MaxByKeyHeuristicSolver(&degree_for_node_key, use_two_one_swaps, reduce_after_taking_n) {}
} // namespace algorithms
