#include "graph_io.h"
#include "src/helpers.h"

#include "fmt/core.h"
#include "spdlog/spdlog.h"
#include <filesystem>
#include <optional>
#include <vector>

namespace datastructures {
FvsInstance read_instance_from_istream(std::istream &stream) {
  std::string header_line_string;
  std::getline(stream, header_line_string);
  std::istringstream header_line(header_line_string);
  size_t num_nodes, num_arcs, num_zero;
  std::optional<size_t> known_size;
  header_line >> num_nodes >> num_arcs >> num_zero;
  if (!header_line.eof()) {
    size_t temp;
    header_line >> temp;
    known_size = temp;
  }

  Graph graph(num_nodes);

  size_t read_arcs = 0;
  for (size_t i = 0; i < graph.size(); i++) {
    std::string line_string;
    std::getline(stream, line_string);
    if (!stream.good()) {
      spdlog::error("Reading file failed ({:x})", stream.rdstate());
      exit(1);
    }

    std::istringstream line(line_string);
    size_t neighbor;

    while (line >> neighbor) {
      helpers::assert_and_log(1 <= neighbor && neighbor <= graph.size(),
                              "input neighbor index out of bounds");
      graph.add_arc_unsafe(i, neighbor - 1);
      read_arcs++;
    }
  }

  if (read_arcs != num_arcs) {
    spdlog::warn("we read {} arcs, while {} arcs should have been present", read_arcs, num_arcs);
  }

  FvsInstance instance(graph);
  if (known_size.has_value()) {
    instance.set_solution_size(known_size.value());
  }
  instance._name = "stdin";

  return instance;
}

FvsInstance read_instance_from_file(const std::filesystem::path &filename) {
  spdlog::debug("Reading graph file: {}", filename.c_str());
  std::ifstream file(filename);
  if (!file) {
    spdlog::error("Graph file does not exist: {}", filename.c_str());
    exit(1);
  }

  auto instance = read_instance_from_istream(file);
  instance._name = filename.filename();

  return instance;
}
} // namespace datastructures
