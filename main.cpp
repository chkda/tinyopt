#include "route_generator.h"
#include <iostream>

int main() {
    route_opt::RouteGenerator generator;

    // Configure generator
    route_opt::GeneratorConfig config;
    config.numPoints = 10;
    config.minCoord = 0.0;
    config.maxCoord = 100.0;
    config.trafficFactor = 0.3;
    config.oneWayProbability = 0.2;

    // Generate random points and Euclidean distances
    auto [points, distances] = generator.generateRandomEuclidean(config);

    // Print points
    std::cout << "Generated Points:\n";
    for (const auto& p : points) {
        std::cout << "(" << p.x << ", " << p.y << ")\n";
    }

    // Print distance matrix
    std::cout << "\nEuclidean Distances:\n";
    for (const auto& row : distances) {
        for (double d : row) {
            std::cout << d << " ";
        }
        std::cout << "\n";
    }

    // Generate road network distances
    auto roadDistances = generator.generateRoadNetwork(points);

    // Generate time-dependent distances
    auto timeDistances = generator.generateTimeDependent(points, config);

    // Save to files
    generator.saveToFile(distances, "euclidean_distances.csv");
    generator.savePointsToFile(points, "points.csv");

    return 0;
}
