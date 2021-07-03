#include "simulated_annealing.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

namespace vrp {

SimulatedAnnealing::SimulatedAnnealing(const VehicleRoutingProblem& vrp,
                                       const Args& args)
    : vrp_(vrp),
      args_(args),
      random_device_(std::random_device()),
      generator_(std::mt19937(random_device_())) {}

SimulatedAnnealing::~SimulatedAnnealing() {}

const Solution SimulatedAnnealing::Start() {
  auto solution = InitialSolution();

  double temperature = args_.initial_temperature;
  int value_delta = std::numeric_limits<int>::max();

  while (!Cutoff(temperature, value_delta)) {
    for (std::size_t i = 0; i < args_.num_iterations; ++i) {
      auto new_solution = Neighbor(solution);
      value_delta = new_solution.cost - solution.cost;

      auto rand_dis = std::uniform_real_distribution<>(0, 1);
      if (value_delta <= 0 ||
          rand_dis(generator_) <
              AcceptanceProbability(value_delta, temperature)) {
        solution = new_solution;
      }
    }

    std::cout << solution << std::endl << std::endl;
    DecrementTemperature(temperature);
  }

  return solution;
}

const Solution SimulatedAnnealing::InitialSolution() {
  auto solution = Solution();

  auto cities = std::vector<std::size_t>();
  for (std::size_t i = 1; i <= vrp_.dimension(); ++i) {
    if (i != vrp_.depot_id()) {
      cities.push_back(i);
    }
  }

  std::shuffle(cities.begin(), cities.end(),
               std::default_random_engine(std::time(nullptr)));

  auto truck_dis = std::uniform_int_distribution<>(1, args_.num_trucks);
  while (!cities.empty()) {
    solution.routes[truck_dis(generator_)].push_front(cities.back());
    cities.pop_back();
  }

  for (auto& [truck_id, route] : solution.routes) {
    route.push_front(vrp_.depot_id());
    route.push_back(vrp_.depot_id());
  }

  UpdateCost(solution);

  return solution;
}

const Solution SimulatedAnnealing::Neighbor(const Solution& solution) {
  auto neighbor = Solution(solution);

  // auto u_dis = std::uniform_int_distribution<>(0, 1);
  auto dimension_dis = std::uniform_int_distribution<>(0, 5);
  auto truck_dis = std::uniform_int_distribution<>(1, args_.num_trucks);

  int num_swaps = dimension_dis(generator_);
  for (int i = 0; i < num_swaps; ++i) {
    std::size_t truck1_id = truck_dis(generator_);
    std::size_t truck2_id = truck_dis(generator_);

    auto truck1_route_dis = std::uniform_int_distribution<>(
        1, neighbor.routes[truck1_id].size() - 2);
    auto truck2_route_dis = std::uniform_int_distribution<>(
        1, neighbor.routes[truck2_id].size() - 2);

    std::swap(neighbor.routes[truck1_id][truck1_route_dis(generator_)],
              neighbor.routes[truck2_id][truck2_route_dis(generator_)]);
  }

  int num_push_pops = dimension_dis(generator_);
  for (int i = 0; i < num_push_pops; ++i) {
    std::size_t truck1_id = truck_dis(generator_);
    std::size_t truck2_id = truck_dis(generator_);

    auto& truck1_route = neighbor.routes[truck1_id];
    auto& truck2_route = neighbor.routes[truck2_id];

    if (truck1_route.size() > 3) {
      std::swap(truck1_route[truck1_route.size() - 1],
                truck1_route[truck1_route.size() - 2]);
      std::size_t city_id = truck1_route.back();
      truck1_route.pop_back();

      truck2_route.push_back(city_id);
      std::swap(truck2_route[truck2_route.size() - 1],
                truck2_route[truck2_route.size() - 2]);
    }
  }

  UpdateCost(neighbor);

  return neighbor;
}

void SimulatedAnnealing::UpdateCost(Solution& solution) const {
  solution.cost = 0;

  for (const auto& [truck_id, route] : solution.routes) {
    for (std::size_t i = 0; i < route.size() - 1; ++i) {
      solution.cost += vrp_.city_distance(route[i], route[i + 1]);
    }

    for (std::size_t i = 0; i < route.size(); ++i) {
      solution.cost += vrp_.city_demand(route[i]);
    }
  }
}

void SimulatedAnnealing::DecrementTemperature(double& temperature) const {
  if (args_.decrement_rule == DecrementRule::kLinear) {
    temperature -= args_.alpha;
  } else if (args_.decrement_rule == DecrementRule::kGeometric) {
    temperature *= args_.alpha;
  } else if (args_.decrement_rule == DecrementRule::kSlow) {
    temperature = temperature / (1.0 + (args_.beta * temperature));
  } else {
    throw std::logic_error("Unknown temperature decrement rule!");
  }
}

const bool SimulatedAnnealing::Cutoff(const double temperature,
                                      const int value_delta) const {
  return temperature < args_.min_temperature;
  // return temperature < args_.min_temperature ||
  //        value_delta <= args_.value_delta_cutoff;
}

const double SimulatedAnnealing::AcceptanceProbability(
    const int value_delta, const double temperature) {
  return std::exp((double)-value_delta / temperature);
}

}  // namespace vrp
