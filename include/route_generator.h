#pragma once
#include "types.h"
#include <random>
#include <string>
#include <vector>

namespace route_opt {

struct GeneratorConfig {
  double minCoord = 0.0;
  double maxCoord = 100.0;
  size_t numPoints = 100;
  unsigned seed = 42;

  double trafficFactor = 0.3;
  double oneWayProbability = 0.2;

  size_t numTimeSlots = 24;
  double peakHourFactor = 2.0;
};

class RouteGenerator {
public:
  explicit RouteGenerator(unsigned seed = 42);

  std::vector<std::vector<double>>
  generateEuclidean(const PointVector& points) const;

  std::pair<PointVector, std::vector<std::vector<double>>>
  generateRandomEuclidean(const GeneratorConfig& config = GeneratorConfig{});

  std::vector<std::vector<double>>
  generateRoadNetwork(const PointVector& points, double trafficFactor = 0.3,
                      double oneWayProbabilty = 0.2);

  std::vector<std::vector<std::vector<double>>> generateTimeDependent(
        const PointVector& points,
        const GeneratorConfig& config = GeneratorConfig{}
    );

  void saveToFile(const std::vector<std::vector<double>>& distances,
                  const std::string& filename) const;

  void savePointsToFile(const PointVector& points,
                        const std::string& filename) const;

  std::vector<std::vector<double>>
  loadFromFile(const std::string& filename) const;

  PointVector loadPointsFromFile(const std::string& filename) const;

#ifdef ENABLE_CUDA
  std::vector<std::vector<double>>
  generateGPUDistances(const PoinPointVector& points);
#endif

private:
  std::mt19937 rng_;
  double calculateDistance(const Point& p1, const Point& p2) const;

  Point generateRandomPoint(double minCoord, double maxCoord) ;

  double calculateTimeFactor(size_t timeSlot,
                             const GeneratorConfig& config) const;
};

namespace utils {
std::vector<std::vector<double>>
convertToMatrix(const std::vector<double>& flatMatrix, size_t size);

bool isValidDistanceMatrix(const std::vector<std::vector<double>>& distances);

bool isSymmetric(const std::vector<std::vector<double>>& distances);
}; // namespace utils

}; // namespace route_opt
