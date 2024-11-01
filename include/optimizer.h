#pragma once
#include "types.h"

namespace route_opt {

class RouteOptimizer {
  virtual ~RouteOptimizer() = default;
  virtual Route findOptimalRoute(const PointVector& points) = 0;

  static RouteOptimizer *createOptimizer(bool useGPU = false);
};
}; // namespace route_opt
