#ifndef VRP_SIMULATED_ANNEALING_H_
#define VRP_SIMULATED_ANNEALING_H_

#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "vrp/solution.h"
#include "vrp/vehicle_routing_problem.h"

namespace vrp {

class SimulatedAnnealing {
 public:
  enum class DecrementRule {
    kLinear,
    kGeometric,
    kSlow,
  };

  struct Args {
    static constexpr std::size_t kDefaultNumTrucks = 6;
    static constexpr double kDefaultInitialTemperature = 1000.0;
    static constexpr double kDeafultMinTemperature = 0.1;
    static constexpr std::size_t kDefaultNumIterations = 10000;
    static constexpr std::size_t kDefaultValueDeltaCutoff = 1;
    static constexpr double kDefaultAlpha = 0.99;
    static constexpr double kDefaultBeta = 1.0;

    constexpr Args(
        const std::size_t num_trucks = kDefaultNumTrucks,
        const double initial_temperature = kDefaultInitialTemperature,
        const double min_temperature = kDeafultMinTemperature,
        const std::size_t num_iterations = kDefaultNumIterations,
        const DecrementRule decrement_rule = DecrementRule::kGeometric,
        const double alpha = kDefaultAlpha, const double beta = kDefaultBeta,
        const std::size_t value_delta_cutoff = kDefaultValueDeltaCutoff)
        : num_trucks(num_trucks),
          initial_temperature(initial_temperature),
          min_temperature(min_temperature),
          num_iterations(num_iterations),
          decrement_rule(decrement_rule),
          alpha(alpha),
          beta(beta),
          value_delta_cutoff(value_delta_cutoff) {}

    ~Args() = default;

    std::size_t num_trucks;
    double initial_temperature;
    double min_temperature;
    std::size_t num_iterations;
    DecrementRule decrement_rule;
    double alpha;
    double beta;
    std::size_t value_delta_cutoff;
  };

  SimulatedAnnealing(const VehicleRoutingProblem& vrp,
                     const Args& args = Args());
  ~SimulatedAnnealing();

  const Solution Start();

 private:
  const Solution InitialSolution();

  const Solution Neighbor(const Solution& solution);

  void UpdateCost(Solution& solution) const;

  void DecrementTemperature(double& temperature) const;

  const bool Cutoff(const double temperature, const int value_delta) const;

  static const double AcceptanceProbability(const int value_delta,
                                            const double temperature);

  VehicleRoutingProblem vrp_;
  Args args_;

  std::random_device random_device_;
  std::mt19937 generator_;
};

}  // namespace vrp

#endif  // VRP_SIMULATED_ANNEALING_H_