#include "vehicle_routing_problem.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

namespace vrp {

VehicleRoutingProblem::VehicleRoutingProblem(const std::string& vrp_file_name) {
  auto points = std::unordered_map<std::size_t, Point>();

  auto vrp_file = std::ifstream(vrp_file_name.c_str());

  std::string line;
  while (std::getline(vrp_file, line)) {
    auto iss = std::istringstream(line);
    std::string key;
    std::string seperator;
    std::string value;

    if (!(iss >> key >> seperator >> value)) {
      break;
    }

    if (key == "DIMENSION") {
      dimension_ = std::stol(value);
    }
  }

  while (std::getline(vrp_file, line)) {
    auto iss = std::istringstream(line);
    std::size_t id;
    double x;
    double y;

    if (!(iss >> id >> x >> y)) {
      break;
    }

    points[id] = Point(x, y);
  }

  while (std::getline(vrp_file, line)) {
    auto iss = std::istringstream(line);
    std::size_t id;
    int demand;

    if (!(iss >> id >> demand)) {
      break;
    }

    city_demands_[id] = demand;
  }

  depot_id_ = 1;

  vrp_file.close();

  for (const auto& [id_i, point_i] : points) {
    for (const auto& [id_j, point_j] : points) {
      city_distances_[id_i][id_j] = EuclideanDistance(point_i, point_j);
    }
  }
}

VehicleRoutingProblem::~VehicleRoutingProblem() {}

const std::size_t VehicleRoutingProblem::EuclideanDistance(const Point& p1,
                                                           const Point& p2) {
  double xd = p1.x - p2.x;
  double yd = p1.y - p2.y;
  return std::round(std::sqrt(xd * xd + yd * yd));
}

}  // namespace vrp
