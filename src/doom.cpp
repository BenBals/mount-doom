#include "algorithms.h"
#include "boost/program_options.hpp"
#include "datastructures.h"
#include "executable_helpers.h"
#include "helpers.h"
#include "license.h"
#include "spdlog/spdlog.h"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <regex>
#include <vector>

namespace po = boost::program_options;
namespace ds = datastructures;
namespace alg = algorithms;
using namespace std::chrono;

bool is_optil_mode(const po::variables_map &vm) {
  return vm["optil"].as<bool>() ||
         (!vm["repeat-reduction-rules-exhaustively"].as<bool>() &&
          !vm["every-reduction-rule-once"].as<bool>() && !vm["export-as-graphviz"].as<bool>() &&
          !vm["branching"].as<bool>() && !vm["greedy-heuristic"].as<bool>());
}

po::variables_map argument_parsing(int argc, char *argv[]) {
  spdlog::debug("Parsing Arguments");

  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
      "graph-path,p",
      po::value<std::string>()->default_value(
          std::string(executable_helpers::find_vcs_root() / "instances/exact_public")),
      "path to folder with graph files to process")(
      "repeat-reduction-rules-exhaustively,r", po::bool_switch()->default_value(false),
      "apply all reduction rules exhaustively per graph")(
      "branching,b", po::bool_switch()->default_value(false),
      "perform branch and bound")("greedy-heuristic,g", po::bool_switch()->default_value(false),
                                  "get greedy heuristic solution")(
      "reduce-after", po::value<size_t>()->default_value(SIZE_MAX),
      "when doing greedy heuristic, reduce after taking this many vertices")(
      "optil", po::bool_switch()->default_value(false), "run in optil IO mode")(
      "every-reduction-rule-once,o", po::bool_switch()->default_value(false),
      "apply all reduction rules once for all graphs")(
      "matching,m", po::value<std::string>()->default_value(".*"),
      "only run on graphs whose filenames match the regex")(
      "visualization-path,v",
      po::value<std::string>()->default_value(
          std::string(executable_helpers::find_vcs_root() / "visualizations/exact_public")),
      "path to folder for the visualizations")(
      "export-as-graphviz,e", po::bool_switch()->default_value(false),
      "write dot language representation of the instances")(
      "info", po::bool_switch()->default_value(false), "display about and license");

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
  } catch (po::error &e) {
    spdlog::error("boost program options failed with error: {}", e.what());
    exit(1);
  }
  po::notify(vm);

  if (vm.count("help")) {
    std::clog << desc << "\n";
    exit(0);
  }

  if (vm["repeat-reduction-rules-exhaustively"].as<bool>() +
          vm["every-reduction-rule-once"].as<bool>() + vm["branching"].as<bool>() +
          vm["greedy-heuristic"].as<bool>() >
      1) {
    spdlog::error("at most one of `repeat-reduction-rules-exhaustively`, "
                  "`every-reduction-rule-once`, and `branching` should be true");
    exit(1);
  }
  if (is_optil_mode(vm)) {
    spdlog::info("running in optil mode");
  }

  return vm;
}

int optil_main() {
  auto instance = ds::read_instance_from_istream(std::cin);
  auto instance_copy = instance;

  std::optional<PartialSolution> solution;

  if (instance.is_vc_instance()) {
    instance.light_reductions_exhaustively();

    spdlog::info("VC instance");
    auto vc_solver = algorithms::VertexCoverSolver(nullptr);

    helpers::set_timeout(std::chrono::seconds(5));
    solution = vc_solver.solve(std::move(instance));
    helpers::reset_timeout();

    if (!solution) {
      spdlog::info("vc light phase failed");
      instance_copy.reductions_exhaustively();
      solution = vc_solver.solve(std::move(instance_copy));
    }
  } else {
    helpers::set_timeout(std::chrono::seconds(5));

    auto light_solver = alg::CycleCliqueVertexCoverLightlyReducingChallengeSolver();
    solution = light_solver.solve(std::move(instance));

    helpers::reset_timeout();
    if (!solution) {
      spdlog::info("dfvs light phase failed");
      auto solver = alg::CycleCliqueVertexCoverChallengeSolver();
      solution = solver.solve(std::move(instance_copy));
    }
  }

  for (auto node : solution->solution_nodes()) {
    std::cout << node << '\n';
  }

  return 0;
}

int license_main() {
  std::cout << "=============================== Mount Doom ===============================\n";
  std::cout << "=              Orodruin, where the ring shall find its end.              =\n";
  std::cout << "=                                                                        =\n";
  std::cout << "=                                   By                                   =\n";
  std::cout << "=    Armin Wells, Ben Bals, Davis Issac, Jonas Schmidt, Katrin Casel,    =\n";
  std::cout << "=          Leo Wendt, Niko Hastrich, Otto Kißig, Robin Wersich,          =\n";
  std::cout << "=                Sebastian Angrick, and Theresa Hradilak                 =\n";
  std::cout << "=                                                                        =\n";
  std::cout << "=                 Hasso Plattner Institute / Uni Potsdam                 =\n";
  std::cout << "=                        mount-doom@lists.myhpi.de                       =\n";
  std::cout << "==========================================================================\n";

  std::cout << "\n\n\nThis program is licensed under the GPL v3, provided below.\n";

  std::cout << license;

  return 0;
}

int main(int argc, char *argv[]) {
  helpers::initialize_signal_handlers();
  std::string executable_name = "doom-main";
  if (!executable_helpers::init_logging_to_file(executable_name)) {
    exit(1);
  }

  auto args = argument_parsing(argc, argv);

  if (args["info"].as<bool>()) {
    return license_main();
  }

  if (is_optil_mode(args)) {
    return optil_main();
  }

  spdlog::info("Orodruin, where the ring shall find its end.");
  std::vector<ds::FvsInstance> instances;
  std::string graph_dir_path = args["graph-path"].as<std::string>();
  std::regex graph_regex = std::regex(args["matching"].as<std::string>());

  helpers::assert_and_log(std::filesystem::is_directory(graph_dir_path),
                          "not a valid dir given as graph-path");

  spdlog::info("Reading graphs from directory: {}", graph_dir_path);

  std::set<std::filesystem::path> filenames;
  for (const auto &path : std::filesystem::directory_iterator(graph_dir_path)) {
    if (std::regex_match(path.path().filename().string(), graph_regex)) {
      filenames.insert(path.path());
    }
  }

  for (const auto &graph_file : filenames) {
    instances.emplace_back(ds::read_instance_from_file(graph_file));
  }
  spdlog::info("\tDONE, num_graphs={}", instances.size());

  spdlog::info("Initial Mean Graph Stats\nNote that taking the mean among quartiles behaves "
               "unintuitively.\n{}",
               ds::meanGraphStatsForInstances(instances).to_string());

  if (args["repeat-reduction-rules-exhaustively"].as<bool>()) {
    spdlog::info("Performing all reduction rules exhaustively per graph");
    for (auto &instance : instances) {
      instance.reductions_exhaustively();
      std::clog << "." << std::flush;
    }
    std::clog << std::endl;
  } else if (args["greedy-heuristic"].as<bool>()) {
    spdlog::info("Computing greedy heuristic solution... DONE");
    auto instance_number = 1;

    spdlog::info("\t\tcsv friendly: instance_number; name; duration; solution size;");
    for (auto &instance : instances) {
      auto name = instance._name;
      auto start = high_resolution_clock::now();
      algorithms::MaxByDegreeHeuristicSolver solver(args["reduce-after"].as<size_t>());
      auto solution = solver.solve(std::move(instance));

      if (!solution) {
        spdlog::error("greedy did not find a solution");
        exit(1);
      }

      auto stop = high_resolution_clock::now();
      auto duration = duration_cast<milliseconds>(stop - start);
      spdlog::info("\t\tDone with {} (time {}ms) solution has size {}", name, duration.count(),
                   solution->number_of_final_nodes());
      spdlog::info("\t\tcsv friendly: {}; {}; {}; {};", instance_number, name, duration.count(),
                   solution->number_of_final_nodes());
      instance_number++;
    }

    spdlog::info("Computing greedy heuristic solution... DONE");
  } else if (args["branching"].as<bool>()) {

    spdlog::info("Branching...");

    size_t instance_number = 1;
    spdlog::info("\t\tcsv friendly: instance_number; duration; ");
    for (auto &instance : instances) {
      auto name = instance._name;

      spdlog::info("\tInstance {}", instance_number);

      auto solver = alg::CycleCliqueVertexCoverChallengeSolver();

      auto start = high_resolution_clock::now();
      auto solution = solver.solve(std::move(instance));

      if (!solution) {
        spdlog::info("greedy found optimal solution");
        exit(0);
      }

      for (auto node : solution->solution_nodes()) {
        std::cout << node << '\n';
      }

      auto stop = high_resolution_clock::now();
      auto duration = duration_cast<milliseconds>(stop - start);
      spdlog::info("\t\tDone with {} (time {}ms)", name, duration.count());
      spdlog::info("\t\tcsv friendly: {}; {}; ", instance_number, duration.count());

      instance_number++;
    }
    spdlog::info("Branching... DONE");

    return 0;

  } else if (args["every-reduction-rule-once"].as<bool>()) {

    spdlog::info("Performing Reduction 5: Finding SCCs and deleting crossings arcs");
    auto start = high_resolution_clock::now();

    for (auto &graph : instances) {
      graph.reduction_delete_arcs_between_strongly_connected_components_ignoring_k2();
      std::clog << "." << std::flush;
    }
    std::clog << std::endl;
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    spdlog::info("\tDone (time {}ms)", duration.count());

    std::clog << std::endl;
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    spdlog::info("\tDone (time {}ms)", duration.count());

    spdlog::info("Collecting self-loops into partial FVS");
    start = high_resolution_clock::now();
    for (auto &graph : instances) {
      graph.collect_self_loops_into_partial_fvs();
      std::clog << "." << std::flush;
    }
    std::clog << std::endl;
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    spdlog::info("\tDone (time {}ms)", duration.count());
  }

  spdlog::info("Mean Graph Stats After Reductions\nNote that taking the "
               "mean among quartiles behaves unintuitively.\n{}",
               ds::meanGraphStatsForInstances(instances).to_string());

  if (args["export-as-graphviz"].as<bool>()) {
    auto folder = args["visualization-path"].as<std::string>();
    spdlog::info("Writing graphviz dot files to {}", folder);

    for (const auto &instance : instances) {
      auto output_path = std::filesystem::path(folder) / (instance._name + ".dot");
      spdlog::info("writing to {}", output_path.string());
      auto stream = std::ofstream(output_path);
      stream << instance.export_as_graphviz() << std::endl;
    }
  }

  return 0;
}
