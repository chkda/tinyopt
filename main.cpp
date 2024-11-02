#include "cuda/optimizer.cuh"
#include "route_generator.h"
#include "visualizer.h"
#include <iostream>
#include <iomanip>
#include <cmath>

// Helper function to calculate route distance
double calculateRouteDistance(const PointVector &points, const std::vector<int> &path) {
    if (points.empty() || path.empty()) {
        return 0.0;
    }

    double total_distance = 0.0;
    for (size_t i = 0; i < path.size(); ++i) {
        size_t current = path[i];
        size_t next = path[(i + 1) % path.size()];

        // Bounds checking
        if (current >= points.size() || next >= points.size()) {
            std::cerr << "Warning: Invalid point index in path\n";
            continue;
        }

        total_distance += points[current].distanceTo(points[next]);
    }
    return total_distance;
}

void printRouteInfo(const std::string &label, const route_opt::Route &route, const PointVector &points) {
    std::cout << "\n" << label << ":\n";

    // Verify route validity
    if (route.path.empty()) {
        std::cout << "Warning: Empty route\n";
        return;
    }

    // Recalculate distance to verify
    double verified_distance = calculateRouteDistance(points, route.path);

    std::cout << "Total Distance (stored): " << std::fixed << std::setprecision(2)
            << route.totalDistance << "\n";
    std::cout << "Total Distance (verified): " << std::fixed << std::setprecision(2)
            << verified_distance << "\n";

    std::cout << "Path length: " << route.path.size() << "\n";
    std::cout << "Route: ";
    for (size_t i = 0; i < route.path.size(); ++i) {
        std::cout << route.path[i];
        if (i < route.path.size() - 1)
            std::cout << " -> ";
    }
    std::cout << "\n";
}

int main() {
    try {
        // Generate random points
        route_opt::GeneratorConfig config;
        config.numPoints = 20;
        config.minCoord = 0.0;
        config.maxCoord = 100.0;
        config.seed = 42;

        route_opt::RouteGenerator generator(config.seed);
        auto [points, distances] = generator.generateRandomEuclidean(config);

        std::cout << "Generated " << points.size() << " random points\n";

        // Verify points
        for (size_t i = 0; i < points.size(); ++i) {
            std::cout << "Point " << i << ": (" << std::fixed << std::setprecision(2)
                    << points[i].x << ", " << points[i].y << ")\n";
        }

        // Create GPU optimizer
        route_opt::RouteOptimizer *optimizer =
                route_opt::cuda::factory::createCUDAOptimizer(
                        route_opt::cuda::factory::RouteAlgorithm::TwoOpt
                        );

        if (!optimizer) {
            throw std::runtime_error("Failed to create optimizer");
        }

        // Configure visualizer
        route_opt::VisualizerConfig visConfig;
        visConfig.width = 800;
        visConfig.height = 800;
        visConfig.outputPath = "route_optimization.mp4";
        visConfig.fps = 30;
        visConfig.showGrid = true;
        visConfig.showProgress = true;

        route_opt::RouteVisualizer visualizer(visConfig);
        visualizer.showPreview(false);
        visualizer.beginRecording();

        // Create and verify initial route
        route_opt::Route initial_route;
        initial_route.path.resize(points.size());
        std::iota(initial_route.path.begin(), initial_route.path.end(), 0);
        initial_route.totalDistance = calculateRouteDistance(points, initial_route.path);

        // Add initial frame
        visualizer.addFrame(points, initial_route);
        printRouteInfo("Initial Route", initial_route, points);

        // Optimize route
        std::cout << "\nOptimizing route...\n";
        route_opt::Route optimized_route = optimizer->findOptimalRoute(points);

        // Verify optimized route
        if (optimized_route.path.empty()) {
            throw std::runtime_error("Optimization returned empty route");
        }

        // Recalculate optimized distance to verify
        optimized_route.totalDistance = calculateRouteDistance(points, optimized_route.path);

        // Add final frame
        visualizer.addFrame(points, optimized_route);
        printRouteInfo("Optimized Route", optimized_route, points);

        // Calculate improvement with checks
        double improvement = 0.0;
        if (initial_route.totalDistance > 0.0) {
            improvement = ((initial_route.totalDistance - optimized_route.totalDistance)
                           / initial_route.totalDistance) * 100.0;

            if (std::isfinite(improvement)) {
                std::cout << "\nImprovement: " << std::fixed << std::setprecision(2)
                        << improvement << "%\n";
            } else {
                std::cout << "\nWarning: Invalid improvement calculation\n";
            }
        } else {
            std::cout << "\nWarning: Invalid initial distance\n";
        }

        visualizer.finalizeVideo();
        delete optimizer;

        std::cout << "\nVisualization saved to: " << visConfig.outputPath << "\n";
        return 0;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
