#include "greedy.h"
namespace algorithms {
MaxByDegreeHeuristicSolver::MaxByDegreeHeuristicSolver()
    : MaxByKeyHeuristicSolver(&degree_for_node_key) {}
MaxByDegreeHeuristicSolver::MaxByDegreeHeuristicSolver(size_t reduce_after_taking_n)
    : MaxByKeyHeuristicSolver(&degree_for_node_key, reduce_after_taking_n) {}
} // namespace algorithms
