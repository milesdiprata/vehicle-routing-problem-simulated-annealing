#ifndef VPR_VEHICLE_ROUTING_PROBLEM_H_
#define VPR_VEHICLE_ROUTING_PROBLEM_H_

#include <cstddef>
#include <string>
#include <unordered_map>

namespace vrp {

class VehicleRoutingProblem {
 public:
  VehicleRoutingProblem(const std::string& vrp_file_name);
  ~VehicleRoutingProblem();

  inline const std::size_t dimension() const { return dimension_; }

  inline const std::size_t depot_id() const { return depot_id_; }

  inline const std::size_t city_distance(const std::size_t id_i,
                                         const std::size_t id_j) const {
    return city_distances_.at(id_i).at(id_j);
  }

  inline const std::size_t city_demand(const std::size_t id) const {
    return city_demands_.at(id);
  }

  private:
  struct Point {
    constexpr Point(const double x = 0.0, const double y = 0.0) : x(x), y(y) {}
    ~Point() = default;

    double x;
    double y;
  };

  static const std::size_t EuclideanDistance(const Point& p1, const Point& p2);

  std::size_t dimension_;

  std::size_t depot_id_;

  std::unordered_map<std::size_t, std::unordered_map<std::size_t, int>>
      city_distances_;

  std::unordered_map<std::size_t, int> city_demands_;
};

}  // namespace vrp

#endif  // VPR_VEHICLE_ROUTING_PROBLEM_H_
