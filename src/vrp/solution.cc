#include "vrp/solution.h"

#include <limits>
#include <string>

namespace vrp {

Solution::Solution() : cost(std::numeric_limits<int>::max()) {}

Solution::Solution(const Solution& solution)
    : cost(solution.cost), routes(solution.routes) {}

Solution::~Solution() {}

std::ostream& operator<<(std::ostream& os, const Solution& solution) {
  for (const auto& [truck_id, route] : solution.routes) {
    os << truck_id << ": [";
    std::string seperator = "";
    for (const auto& city_id : route) {
      os << seperator << city_id;
      seperator = "->";
    }
    os << "]\n";
  }

  os << "Cost: " << solution.cost;

  return os;
}

}  // namespace vrp
