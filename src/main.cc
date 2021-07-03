#include <iostream>

#include "vrp/simulated_annealing.h"
#include "vrp/vehicle_routing_problem.h"

int main(const int argc, const char* const argv[]) {
  auto vrp = vrp::VehicleRoutingProblem("bin/A-n39-k6.vrp");
  auto solver = vrp::SimulatedAnnealing(vrp);
  solver.Start();

  return 0;
}