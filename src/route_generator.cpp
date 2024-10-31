#include "route_generator.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace route_opt {
RouteGenerator::RouteGenerator(unsigned seed) : rng_(seed) {}

double RouteGenerator::calculateDistance(const Point& p1, const Point& p2) const {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return sqrt(dx*dx + dy*dy);
}

Point RouteGenerator::generateRandomPoint(double minCoord, double maxCoord)  {
  std::uniform_real_distribution<double> dist(minCoord, maxCoord);
  return Point{dist(rng_), dist(rng_)};
}

std::vector<std::vector<double>> RouteGenerator::generateEuclidean(const PointVector& points) const {
 const size_t numPoints = points.size();
 std::vector<std::vector<double>> distances(numPoints, std::vector<double>(numPoints, 0.0));

 #pragma omp parallel for collapse(2)
 for (size_t i = 0; i < numPoints; ++i) {
   for (size_t j = 0; j < numPoints; ++j) {
     double distance = calculateDistance(points[i], points[j]);
     distances[i][j] = distance;
     distances[j][i] = distance;
   }
 }
 return distances;
}

std::pair<PointVector, std::vector<std::vector<double>>> RouteGenerator::generateRandomEuclidean( const GeneratorConfig &config) {
  PointVector points;
  points.reserve(config.numPoints);
  for (size_t i = 0; i < config.numPoints; ++i) {
    points.push_back(generateRandomPoint(config.minCoord, config.maxCoord));
  }
  return {points, generateEuclidean(points)};
}

std::vector<std::vector<double>> RouteGenerator::generateRoadNetwork(const PointVector& points, double trafficFactor, double oneWayProbability) {
  const size_t n = points.size();
  std::vector<std::vector<double>> distances(n, std::vector<double>(n, 0.0));

  // Distributions for traffic and one-way probability
  std::uniform_real_distribution<double> trafficDist(1.0, 1.0 + trafficFactor);
  std::uniform_real_distribution<double> oneWayDist(0.0, 1.0);

  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      double baseDistance = calculateDistance(points[i], points[j]);

      // Add traffic variation
      double traffic1 = trafficDist(rng_);
      double traffic2 = trafficDist(rng_);

      if (oneWayDist(rng_) < oneWayProbability) {
        // One-way street
        distances[i][j] = baseDistance * traffic1;
        distances[j][i] = baseDistance * traffic2 * 3.0; // Longer return route
      } else {
        // Two-way street with different traffic conditions
        distances[i][j] = baseDistance * traffic1;
        distances[j][i] = baseDistance * traffic2;
      }
    }
  }
  return distances;
}


std::vector<std::vector<std::vector<double>>>
RouteGenerator::generateTimeDependent(
    const PointVector& points,
    const GeneratorConfig& config) {

    const size_t n = points.size();

    // Initialize 3D matrix [time][from][to]
    std::vector<std::vector<std::vector<double>>> timeDistances(
        config.numTimeSlots,
        std::vector<std::vector<double>>(
            n,
            std::vector<double>(n, 0.0)
        )
    );

    // Generate base distances
    auto baseDistances = generateEuclidean(points);

    // Add time-dependent variations
    for (size_t t = 0; t < config.numTimeSlots; ++t) {
        double timeFactor = calculateTimeFactor(t, config);
        std::normal_distribution<double> variation(timeFactor, 0.1 * timeFactor);

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                if (i != j) {
                    timeDistances[t][i][j] = baseDistances[i][j] * variation(rng_);
                }
            }
        }
    }

    return timeDistances;
}

double RouteGenerator::calculateTimeFactor(
    size_t timeSlot,
    const GeneratorConfig& config) const {

    // Morning peak (8-10)
    if (timeSlot >= 8 && timeSlot <= 10) {
        return config.peakHourFactor;
    }
    // Evening peak (16-18)
    else if (timeSlot >= 16 && timeSlot <= 18) {
        return config.peakHourFactor;
    }
    // Normal hours
    return 1.0;
}

void RouteGenerator::saveToFile(
    const std::vector<std::vector<double>>& distances,
    const std::string& filename) const {

    std::ofstream file(filename);
    if (!file) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    for (const auto& row : distances) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) file << ",";
        }
        file << "\n";
    }
}

void RouteGenerator::savePointsToFile(
    const PointVector& points,
    const std::string& filename) const {

    std::ofstream file(filename);
    if (!file) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    for (const auto& point : points) {
        file << point.x << "," << point.y << "\n";
    }
}

std::vector<std::vector<double>> RouteGenerator::loadFromFile(
    const std::string& filename) const {

    std::ifstream file(filename);
    if (!file) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::vector<std::vector<double>> distances;
    std::string line;

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }
        distances.push_back(row);
    }

    if (!utils::isValidDistanceMatrix(distances)) {
        throw std::runtime_error("Invalid distance matrix in file: " + filename);
    }

    return distances;
}

PointVector RouteGenerator::loadPointsFromFile(
    const std::string& filename) const {

    std::ifstream file(filename);
    if (!file) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    PointVector points;
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x, y;

        if (std::getline(ss, x, ',') && std::getline(ss, y, ',')) {
            points.push_back(Point{std::stod(x), std::stod(y)});
        }
    }

    return points;
}

namespace utils {

std::vector<std::vector<double>> convertToMatrix(
    const std::vector<double>& flatMatrix,
    size_t size) {

    if (flatMatrix.size() != size * size) {
        throw std::invalid_argument("Invalid flat matrix size");
    }

    std::vector<std::vector<double>> matrix(
        size, std::vector<double>(size)
    );

    for (size_t i = 0; i < size; ++i) {
        for (size_t j = 0; j < size; ++j) {
            matrix[i][j] = flatMatrix[i * size + j];
        }
    }

    return matrix;
}

std::vector<double> convertToFlat(
    const std::vector<std::vector<double>>& matrix) {

    if (matrix.empty() || matrix.size() != matrix[0].size()) {
        throw std::invalid_argument("Invalid matrix dimensions");
    }

    const size_t size = matrix.size();
    std::vector<double> flat(size * size);

    for (size_t i = 0; i < size; ++i) {
        for (size_t j = 0; j < size; ++j) {
            flat[i * size + j] = matrix[i][j];
        }
    }

    return flat;
}

bool isValidDistanceMatrix(
    const std::vector<std::vector<double>>& distances) {

    if (distances.empty()) return false;

    const size_t n = distances.size();

    // Check square matrix
    for (const auto& row : distances) {
        if (row.size() != n) return false;
    }

    // Check diagonal is zero
    for (size_t i = 0; i < n; ++i) {
        if (distances[i][i] != 0.0) return false;
    }

    // Check non-negative distances
    for (const auto& row : distances) {
        if (std::any_of(row.begin(), row.end(),
            [](double d) { return d < 0.0; })) {
            return false;
        }
    }

    return true;
}

bool isSymmetric(
    const std::vector<std::vector<double>>& distances) {

    if (!isValidDistanceMatrix(distances)) return false;

    const size_t n = distances.size();
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (std::abs(distances[i][j] - distances[j][i]) > 1e-10) {
                return false;
            }
        }
    }

    return true;
}
}

};
