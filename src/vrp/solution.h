#ifndef VRP_SOLUTION_H_
#define VRP_SOLUTION_H_

#include <cstddef>
#include <deque>
#include <iostream>
#include <unordered_map>

namespace vrp {

struct Solution {
  Solution();
  Solution(const Solution& solution);
  ~Solution();

  int cost;
  std::unordered_map<std::size_t, std::deque<std::size_t>> routes;
};

std::ostream& operator<<(std::ostream& os, const Solution& solution);

}  // namespace vrp

#endif  // VRP_SOLUTION_H_