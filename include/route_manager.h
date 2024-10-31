#pragma once
#include "optimizer.h"
#include "types.h"
#include <memory>

namespace route_opt {
class RouteManager {
public:
  explicit RouteManager(bool useGPU = false);
  void setPoints(const PointVector &points);
  Route optimize();

private:
  PointVector points_;
  std::unique_ptr<RouteOptimizer> optimizer_;
};

}; // namespace route_opt
