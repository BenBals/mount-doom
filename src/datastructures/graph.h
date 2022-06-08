#pragma once

#include "helpers.h"
#include "spdlog/spdlog.h"
#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <numeric>
#include <sstream>
#include <stack>
#include <string>
#include <vector>
#include <optional>

namespace datastructures {

class Graph {
public:
  enum class EdgeDirection { UNDIRECTED, OUTGOING, INCOMING };

  class Node;
  std::vector<Node> _nodes;

  Graph() = default;
  explicit Graph(size_t _num_nodes);

  bool is_arc(size_t from, size_t to) const;

  size_t size() const;
  bool empty() const;

  /**
   * Insert's the arc, without checking for multi edges
   */
  void add_arc_unsafe(size_t from, size_t to);
  void add_arc(size_t from, size_t to);
  /**
   * Removes the arc. You must ensure, that the arc exists.
   * If there exist multiple such arcs, only one is removed.
   * If it does not it is undefined behaviour.
   * Removes arc from both the incident nodes lists.
   */
  void remove_arc_unsafe(size_t from, size_t to);

  /**
   * This removes all neighbors of idx, for which f(neighbor, edge_direction) is true.
   * edge_direction gives the role of the edge from the point of view of idx. You have to ensure
   * that an edge is deleted from both ends. **THIS CAN NOT EVEN BE TESTED IN DEBUG MODE**
   */
  template <class F>
    requires std::predicate<F, Graph::Node &, Graph::EdgeDirection>
  void remove_neighbors_of_node_unsafe(size_t idx, F &&f);

  /**
   * Changes the direction of the arc. You must ensure, that the arc exists.
   * If there exist multiple such arcs, only one is swapped.
   * This method assumes that a lot of arcs are being swapped and uses some non-asymptomatic
   * optimization for this. Note that this method explicitly does not maintain any ordering of
   * adjacency lists, if present.
   * @param from previous from node, now to node
   * @param to previous to node, now from node
   */
  void swap_arc_unsafe(size_t from, size_t to);

  void isolate_node(size_t idx);

  /**
   * Adds all outgoing arcs of this node to each of its ingoing adjacent nodes outgoing arcs
   * Does the analogue with ingoing arcs to outgoing adjacent nodes
   * The shortcutted node is not deleted but isolated
   */
  void shortcut_node(size_t idx);

  Node &operator[](size_t idx);
  const Node &operator[](size_t idx) const;

private:
  template <class F>
  std::pair<size_t, std::vector<size_t>>
  strongly_connected_components_inner(F &&edges_of_node) const;

public:
  /**
   * @return the number of SCCs as well as for each node the number (color) of the SCC it's in
   */
  std::pair<size_t, std::vector<size_t>> strongly_connected_components() const;

  /**
   * @return the number of SCCs as well as for each node the number (color) of the SCC it's in
   */
  std::pair<size_t, std::vector<size_t>> strongly_connected_components_ignoring_k2() const;

  /**
   * @return the indices of all weak articulation points
   */
  std::vector<size_t> weak_articulation_points() const;

  /**
   * @return the number of CCs as well as for each node the number (color) of the CC it's in
   */
  std::pair<size_t, std::vector<size_t>> weakly_connected_components() const;

  class CycleFindingData {
    std::vector<size_t> previous;
    size_t iteration = 0;
    std::vector<size_t> last_touched;

    void new_iteration(size_t graph_size);

    void set_previous(size_t node_index, size_t previous_index);

    std::optional<size_t> get_previous(size_t node_index);

    /**
     * Follows previous links until there either is none from this iteration, or the link is >=
     * graph_size for this iteration.
     */
    std::vector<size_t> back_track_from(size_t node_index);

    friend Graph;
  };
  /**
   * Finds a cycle including the given start_index.
   * @param start_index
   * @param data Supply an arbitrary CycleFindingData
   * @param max_cycle_size Maximum number of nodes on the returned cycle
   * @return If there is no cycle an empty option is returned, otherwise a vector with the nodes in
   * cycle order is returned.
   */
  std::optional<std::vector<size_t>> find_limited_cycle_with_node(size_t start_index, CycleFindingData &data,
                                     size_t max_cycle_size) const;

  /// Facade of find_limited_cycle_with_node, s.t. cycles can be arbitrarily large
  std::optional<std::vector<size_t>> find_cycle_with_node(size_t start_index,
                                                          CycleFindingData &data) const;
  /**
   * This method will find disjoint cycles. Additionally there will be no cycle among the nodes, which we
   * do not cover. Thus, if there is a cycle, we finds at least one cycle.
   */
  std::vector<std::vector<size_t>> greedy_find_disjoint_cycles() const;

  size_t greedy_count_disjoint_cycles() const;

  bool is_acyclic() const;

  bool is_acyclic_without_nodes(std::vector<size_t> excluded_nodes) const;

  bool has_selfloop() const;

  void symmetrize();

  void transpose();

  size_t create_node();

  std::string export_as_input_format() const;

  // Returns the permutation applied by the deletion operation
  // Deleted nodes get the permuted index at least size()
  template <class F>
    requires std::predicate<F, Graph::Node &>
  std::vector<size_t> remove_nodes(F &&f);

  class Node {
    size_t _idx;
    std::vector<size_t> _in, _out, _undirected;

    friend Graph::Graph(size_t _num_nodes);
    friend size_t Graph::create_node();
    friend void Graph::add_arc_unsafe(size_t, size_t);
    template <class F>
      requires std::predicate<F, Graph::Node &, Graph::EdgeDirection>
    friend void Graph::remove_neighbors_of_node_unsafe(size_t, F &&);
    friend void Graph::swap_arc_unsafe(size_t, size_t);
    friend void Graph::transpose();
    friend void Graph::symmetrize();
    template <class F>
      requires std::predicate<F, Graph::Node &>
    friend std::vector<size_t> Graph::remove_nodes(F &&);

  public:
    struct EdgeList {
      struct Iterator {
        using iterator_category = std::input_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = const size_t;
        using pointer = const size_t *;
        using reference = const size_t &;

        pointer operator->();
        reference operator*() const;

        Iterator &operator++();
        Iterator operator++(int);

        bool operator==(const Iterator &b) const = default;
        bool operator!=(const Iterator &b) const = default;

      private:
        Iterator(const Node *ptr, bool undirected, bool out_directed, bool in_directed);
        Iterator(const Node *ptr, bool undirected, bool out_directed, bool in_directed,
                 size_t index);
        friend EdgeList;

        const Node *_node;
        bool _undirected, _out_directed, _in_directed;
        size_t _index = 0;

        inline pointer value_ptr() const;
      };
      Iterator begin() const;
      Iterator end() const;
      bool empty() const;
      size_t size() const;

      friend Node;

    private:
      const Node *_node;
      bool _undirected, _out_directed, _in_directed;
      EdgeList(const Node *ptr, bool undirected, bool out_directed, bool in_directed);
    };
    bool operator==(const Node &other) const;

    size_t index() const;

    EdgeList all_edges() const;
    EdgeList out_edges() const;
    EdgeList in_edges() const;
    EdgeList undirected_edges() const;
    EdgeList all_edges_not_undirected() const;
    EdgeList out_edges_not_undirected() const;
    EdgeList in_edges_not_undirected() const;
  };
#ifdef TEST
  bool operator==(const Graph &other) const = default;
#endif
};
} // namespace datastructures

#include "graph_impl.h"
