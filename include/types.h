#pragma once
#include <vector>

namespace route_opt {
struct Point {
  double x, y;
  double distanceTo(const Point &other) const;
};

struct Route {
  std::vector<int> path;
  double totalDistance;
};

}; // namespace route_opt

using PointVector = std::vector<route_opt::Point>;
