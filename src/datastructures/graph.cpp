#include "graph.h"
#include "src/helpers.h"
#include <algorithm>
#include <numeric>
#include <queue>
#include <random>

namespace datastructures {

bool Graph::Node::operator==(const Node &other) const {
  return _idx == other._idx && _in.size() == other._in.size() && _out.size() == other._out.size() &&
         std::is_permutation(_in.begin(), _in.end(), other._in.begin()) &&
         std::is_permutation(_out.begin(), _out.end(), other._out.begin()) &&
         std::is_permutation(_undirected.begin(), _undirected.end(), other._undirected.begin());
}

void Graph::transpose() {
  for (auto &node : _nodes)
    std::swap(node._in, node._out);
}

void Graph::symmetrize() {
  for (auto &node : _nodes) {
    std::copy(node.all_edges_not_undirected().begin(), node.all_edges_not_undirected().end(),
              std::back_inserter(node._undirected));
    node._in.clear();
    node._out.clear();
  }
}

std::pair<size_t, std::vector<size_t>> Graph::strongly_connected_components() const {
  return strongly_connected_components_inner([](const auto &node) { return node.out_edges(); });
}

std::pair<size_t, std::vector<size_t>> Graph::strongly_connected_components_ignoring_k2() const {
  return strongly_connected_components_inner(
      [](const auto &node) { return node.out_edges_not_undirected(); });
}
std::vector<size_t> Graph::weak_articulation_points() const {
  // Tarjan's algorithm for biconnected components
  std::vector<size_t> depth(size(), SIZE_MAX), low(size(), SIZE_MAX), result;
  std::vector<bool> cut_vertex(size());

  auto dfs = [&](auto &&rec, size_t current_node_idx, size_t parent) -> void {
    low[current_node_idx] = depth[current_node_idx];
    size_t children = 0;

    for (auto neighbor : _nodes[current_node_idx].all_edges()) {
      if (neighbor == parent)
        continue;

      if (depth[neighbor] == SIZE_MAX) {
        depth[neighbor] = depth[current_node_idx] + 1;
        children++;

        rec(rec, neighbor, current_node_idx);

        // this is an articulation point
        if (low[neighbor] >= depth[current_node_idx] &&
            (depth[current_node_idx] != 0 || children > 1) && !cut_vertex[current_node_idx]) {
          cut_vertex[current_node_idx] = true;
          result.push_back(current_node_idx);
        }
        low[current_node_idx] = std::min(low[current_node_idx], low[neighbor]);
      } else {
        low[current_node_idx] = std::min(low[current_node_idx], depth[neighbor]);
      }
    }
  };

  for (size_t idx = 0; idx < size(); idx++) {
    if (depth[idx] == SIZE_MAX) {
      depth[idx] = 0;
      dfs(dfs, idx, SIZE_MAX);
    }
  }

  return result;
}

std::pair<size_t, std::vector<size_t>> Graph::weakly_connected_components() const {
  std::vector<size_t> components(size(), SIZE_MAX), stack;
  size_t cur_component = 0;

  auto dfs = [&](size_t start) {
    components[start] = cur_component;
    stack.push_back(start);

    while (!stack.empty()) {
      auto cur = stack.back();
      stack.pop_back();

      for (auto neigh : _nodes[cur].all_edges()) {
        if (components[neigh] == SIZE_MAX) {
          components[neigh] = cur_component;
          stack.push_back(neigh);
        }
      }
    }
  };

  for (size_t idx = 0; idx < size(); idx++) {
    if (components[idx] == SIZE_MAX) {
      dfs(idx);
      cur_component++;
    }
  }
  return {cur_component, components};
}

std::vector<std::vector<size_t>> Graph::greedy_find_disjoint_cycles() const {
  std::vector<std::vector<size_t>> cycles;
  std::vector<bool> in_stack(size()), visited(size());

  // node_index, in_edges, end_in_edges
  // we find the cycle nodes in reverse order of discovery, thus we need to use in edges for search
  std::stack<std::tuple<size_t, Graph::Node::EdgeList::Iterator, Graph::Node::EdgeList::Iterator>>
      stack;
  // returns true iff the stack was changed
  auto push_onto_stack = [&](size_t node_index) {
    visited[node_index] = true;
    if (is_arc(node_index, node_index)) {
      cycles.push_back({node_index});
      return false;
    }

    const auto &node = _nodes[node_index];
    // check matching edges
    for (auto in_neighbor : node.undirected_edges()) {
      if (!visited[in_neighbor]) {
        visited[in_neighbor] = true;
        std::vector<size_t> cycle = {node_index, in_neighbor};
        cycles.emplace_back(std::move(cycle));
        return false;
      }
    }

    auto edge_list = _nodes[node_index].in_edges();
    stack.emplace(node_index, edge_list.begin(), edge_list.end());
    in_stack[node_index] = true;

    // check for induced loops
    for (auto neighbor : node.in_edges()) {
      if (in_stack[neighbor]) {
        cycles.emplace_back();
        // pop everything in cycle but neighbor
        while (get<0>(stack.top()) != neighbor) {
          in_stack[get<0>(stack.top())] = false;
          cycles.back().push_back(get<0>(stack.top()));
          stack.pop();
        }
        // pop last one in cycle
        in_stack[neighbor] = false;
        cycles.back().push_back(neighbor);
        stack.pop();
        return true;
      }
    }

    return true;
  };

  for (size_t current_start_node_index = 0; current_start_node_index < size();
       current_start_node_index++) {
    if (visited[current_start_node_index])
      continue;

    push_onto_stack(current_start_node_index);
    while (!stack.empty()) {
      {
        auto &[current_node_index, edge_list_it, edge_list_end] = stack.top();

        while (edge_list_it != edge_list_end) {
          auto neighbor = *edge_list_it;
          ++edge_list_it;

          if (!visited[neighbor]) {
            if (push_onto_stack(neighbor))
              break;
          }
        }
      }

      if (!stack.empty()) {
        // need to reinitialize, size the stack might has changed
        auto &[current_node_index, edge_list_it, edge_list_end] = stack.top();
        if (edge_list_it == edge_list_end) {
          in_stack[current_node_index] = false;
          stack.pop();
        }
      }
    }
  }

  return cycles;
}

size_t Graph::greedy_count_disjoint_cycles() const { return greedy_find_disjoint_cycles().size(); }

bool Graph::is_arc(const size_t from, const size_t to) const {
  helpers::assert_and_log(from < size() && to < size(), "from or to is not a node");
  auto &node = _nodes[from];
  return std::find(node.out_edges().begin(), node.out_edges().end(), to) != node.out_edges().end();
}

size_t Graph::size() const { return _nodes.size(); }

void Graph::add_arc_unsafe(const size_t from, const size_t to) {
  helpers::assert_and_log(from < size() && to < size(), "from or to is not a node");
  helpers::assert_and_log(std::find(_nodes[from].out_edges().begin(),
                                    _nodes[from].out_edges().end(),
                                    to) == _nodes[from].out_edges().end(),
                          "can not add multi edges");

  auto reverse_edge_iterator = std::find(_nodes[to].out_edges_not_undirected().begin(),
                                         _nodes[to].out_edges_not_undirected().end(), from);
  auto exists_reverse_edge = reverse_edge_iterator != _nodes[to].out_edges_not_undirected().end();
  if (exists_reverse_edge) {
    std::erase(_nodes[to]._out, from);
    std::erase(_nodes[from]._in, to);
    _nodes[from]._undirected.push_back(to);
    _nodes[to]._undirected.push_back(from);
  } else {
    _nodes[from]._out.push_back(to);
    _nodes[to]._in.push_back(from);
  }
}
void Graph::add_arc(const size_t from, const size_t to) {
  if (!this->is_arc(from, to)) {
    this->add_arc_unsafe(from, to);
  }
}
Graph::Node &Graph::operator[](size_t idx) { return _nodes[idx]; }
const Graph::Node &Graph::operator[](size_t idx) const { return _nodes[idx]; }

void Graph::remove_arc_unsafe(size_t from, size_t to) {
  remove_neighbors_of_node_unsafe(
      from, [to](const auto &node, [[maybe_unused]] const auto edge_direction) {
        return node.index() == to;
      });
  remove_neighbors_of_node_unsafe(
      to, [from](const auto &node, [[maybe_unused]] const auto edge_direction) {
        return node.index() == from;
      });
}

Graph::Graph(const size_t _num_nodes) : _nodes(_num_nodes) {
  for (size_t i = 0; i < size(); i++) {
    this->_nodes[i]._idx = i;
  }
}

bool Graph::empty() const { return size() == 0; }
bool Graph::is_acyclic() const {
  auto [num_colors, colors] = strongly_connected_components();
  return num_colors == size() && !has_selfloop();
}

size_t Graph::create_node() {
  _nodes.emplace_back();
  _nodes.back()._idx = size() - 1;
  return size() - 1;
}

bool Graph::has_selfloop() const {
  for (size_t idx = 0; idx < size(); idx++) {
    if (is_arc(idx, idx)) {
      return true;
    }
  }

  return false;
}

void Graph::isolate_node(size_t idx) {
  for (auto neighbor_idx : _nodes[idx].all_edges()) {
    if (neighbor_idx != idx)
      remove_neighbors_of_node_unsafe(
          neighbor_idx, [idx](const auto &node, [[maybe_unused]] const auto edge_direction) {
            return node.index() == idx;
          });
  }
  remove_neighbors_of_node_unsafe(idx,
                                  []([[maybe_unused]] const auto &node,
                                     [[maybe_unused]] const auto edge_direction) { return true; });
}
void Graph::shortcut_node(size_t idx) {
  for (size_t in_neighbor_idx : this->_nodes[idx].in_edges()) {
    for (size_t out_neighbor_idx : this->_nodes[idx].out_edges()) {
      add_arc(in_neighbor_idx, out_neighbor_idx);
    }
  }
  isolate_node(idx);
}

void Graph::swap_arc_unsafe(size_t from, size_t to) {
  helpers::assert_and_log(from < size() && to < size(), "from or to is not a node");
  auto &from_node = _nodes[from];
  auto &to_node = _nodes[to];

  if (std::find(from_node.undirected_edges().begin(), from_node.undirected_edges().end(), to) !=
      from_node.undirected_edges().end())
    return;

  helpers::assert_and_log(std::find(from_node.out_edges_not_undirected().begin(),
                                    from_node.out_edges_not_undirected().end(),
                                    to) != from_node.out_edges_not_undirected().end(),
                          "tried to swap non existing arc");
  auto from_out_rev_it = std::find(from_node._out.rbegin(), from_node._out.rend(), to);
  auto from_out_it = (++from_out_rev_it).base();
  from_node._out.erase(from_out_it);

  auto to_in_rev_it = std::find(to_node._in.rbegin(), to_node._in.rend(), from);
  auto to_in_it = (++to_in_rev_it).base();
  to_node._in.erase(to_in_it);

  to_node._out.push_back(from);
  from_node._in.push_back(to);
}
size_t Graph::Node::index() const { return _idx; }
Graph::Node::EdgeList Graph::Node::all_edges() const { return EdgeList(this, true, true, true); }
Graph::Node::EdgeList Graph::Node::out_edges() const { return EdgeList(this, true, true, false); }
Graph::Node::EdgeList Graph::Node::in_edges() const { return EdgeList(this, true, false, true); }
Graph::Node::EdgeList Graph::Node::undirected_edges() const {
  return EdgeList(this, true, false, false);
}
Graph::Node::EdgeList Graph::Node::all_edges_not_undirected() const {
  return EdgeList(this, false, true, true);
}
Graph::Node::EdgeList Graph::Node::out_edges_not_undirected() const {
  return EdgeList(this, false, true, false);
}
Graph::Node::EdgeList Graph::Node::in_edges_not_undirected() const {
  return EdgeList(this, false, false, true);
}
std::string Graph::export_as_input_format() const {
  std::stringstream body;
  std::stringstream header;
  size_t cnt_edges = 0;

  for (const auto &node : _nodes) {
    cnt_edges += node.out_edges().size();

    bool first_edge = true;
    for (const auto out_idx : node.out_edges()) {
      if (!first_edge)
        body << ' ';
      body << out_idx + 1;
      first_edge = false;
    }

    body << '\n';
  }
  header << size() << " " << cnt_edges << " 0\n";
  return header.str() + body.str();
}

void Graph::CycleFindingData::new_iteration(size_t graph_size) {
  iteration++;
  last_touched.resize(graph_size);
  previous.resize(graph_size);
}

void Graph::CycleFindingData::set_previous(size_t node_index, size_t previous_index) {
  last_touched[node_index] = iteration;
  previous[node_index] = previous_index;
}

std::optional<size_t> Graph::CycleFindingData::get_previous(size_t node_index) {
  if (last_touched[node_index] != iteration)
    return {};
  return previous[node_index];
}

std::vector<size_t> Graph::CycleFindingData::back_track_from(size_t node_index) {
  std::vector<size_t> path = {node_index};

  while (get_previous(node_index).value_or(SIZE_MAX) < previous.size()) {
    node_index = get_previous(node_index).value();
    path.push_back(node_index);
  }
  return path;
}

std::optional<std::vector<size_t>>
Graph::find_cycle_with_node(size_t start_index, Graph::CycleFindingData &data) const {
    return find_limited_cycle_with_node(start_index, data, size());
}

std::optional<std::vector<size_t>> Graph::find_limited_cycle_with_node(size_t start_index,
      Graph::CycleFindingData &data,
      size_t max_cycle_size) const {
  // we find cycles using a BFS
  data.new_iteration(size());
  std::queue<int32_t> queue{{static_cast<int32_t>(start_index)}};
  data.set_previous(start_index, INT32_MAX);
  queue.push(-2);
  size_t current_cycle_size = 1; // number of nodes with which we look for cycles

  while (!queue.empty()) {
    auto current_node_index = queue.front();
    queue.pop();

    // note that the distance values are negative to distinguish them from nodes.
    if (current_node_index < 0) {
      current_cycle_size = -current_node_index;
      if (current_cycle_size > max_cycle_size)
        break;
      else{
        queue.push(-(current_cycle_size + 1));
        continue;
      }
    }

    for (auto out_neighbor_index : _nodes[current_node_index].out_edges()) {
      if (out_neighbor_index == start_index) {
        // reconstruct cycle
        auto cycle = data.back_track_from(current_node_index);
        std::reverse(cycle.begin(), cycle.end());
        return {cycle};
      } else if (!data.get_previous(out_neighbor_index)) {
        queue.push(out_neighbor_index);
        data.set_previous(out_neighbor_index, current_node_index);
      }
    }
  }

  return {};
}

bool Graph::is_acyclic_without_nodes(std::vector<size_t> excluded_nodes) const {
  std::vector<size_t> visited(this->size(), 0);
  for (auto ex_node : excluded_nodes)
    visited[ex_node] = SIZE_MAX;

  std::function<bool(size_t)> dfs = [&](size_t current_node) {
    visited[current_node] = 1;
    for (auto out_nei_idx : this->_nodes[current_node].out_edges()) {
      if (visited[out_nei_idx] == 1) {
        // back-edge
        return true;
      } else if (visited[out_nei_idx] == 0) {
        if (dfs(out_nei_idx))
          return true;
      }
    }
    visited[current_node] = 2;
    return false;
  };

  for (size_t idx = 0; idx < this->size(); idx++)
    if (visited[idx] == 0)
      if (dfs(idx))
        return false;

  return true;
}

Graph::Node::EdgeList::Iterator::pointer Graph::Node::EdgeList::Iterator::operator->() {
  return value_ptr();
}
const size_t &Graph::Node::EdgeList::Iterator::operator*() const { return *value_ptr(); }
Graph::Node::EdgeList::Iterator &Graph::Node::EdgeList::Iterator::operator++() {
  _index++;
  return *this;
}
Graph::Node::EdgeList::Iterator Graph::Node::EdgeList::Iterator::operator++(int) {
  auto tmp = *this;
  ++(*this);
  return tmp;
}
Graph::Node::EdgeList::Iterator::Iterator(const Graph::Node *ptr, bool undirected,
                                          bool out_directed, bool in_directed)
    : _node(ptr), _undirected(undirected), _out_directed(out_directed), _in_directed(in_directed) {}
Graph::Node::EdgeList::Iterator::Iterator(const Graph::Node *ptr, bool undirected,
                                          bool out_directed, bool in_directed, size_t index)
    : _node(ptr), _undirected(undirected), _out_directed(out_directed), _in_directed(in_directed),
      _index(index) {}
Graph::Node::EdgeList::Iterator::pointer Graph::Node::EdgeList::Iterator::value_ptr() const {
  auto current_index = _index;
  if (_undirected) {
    if (current_index < _node->_undirected.size())
      return &_node->_undirected[current_index];
    else
      current_index -= _node->_undirected.size();
  }
  if (_out_directed) {
    if (current_index < _node->_out.size())
      return &_node->_out[current_index];
    else
      current_index -= _node->_out.size();
  }
  if (_in_directed) {
    if (current_index < _node->_in.size())
      return &_node->_in[current_index];
  }
  helpers::assert_and_log(false, "dereferencing past end");
  return nullptr;
}
Graph::Node::EdgeList::Iterator Graph::Node::EdgeList::begin() const {
  return Iterator(_node, _undirected, _out_directed, _in_directed);
}
Graph::Node::EdgeList::Iterator Graph::Node::EdgeList::end() const {
  return Iterator(_node, _undirected, _out_directed, _in_directed, size());
}
bool Graph::Node::EdgeList::empty() const {
  if (_undirected && !_node->_undirected.empty())
    return false;
  if (_out_directed && !_node->_out.empty())
    return false;
  if (_in_directed && !_node->_in.empty())
    return false;
  return true;
}
size_t Graph::Node::EdgeList::size() const {
  size_t size = 0;
  size += _undirected * _node->_undirected.size();
  size += _out_directed * _node->_out.size();
  size += _in_directed * _node->_in.size();
  return size;
}
Graph::Node::EdgeList::EdgeList(const Graph::Node *ptr, bool undirected, bool out_directed,
                                bool in_directed)
    : _node(ptr), _undirected(undirected), _out_directed(out_directed), _in_directed(in_directed) {
  helpers::assert_and_log(undirected || out_directed || in_directed,
                          "cannot use the empty edge list");
}
} // namespace datastructures
