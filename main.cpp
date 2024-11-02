#include "cuda/optimizer.cuh"
#include "route_generator.h"
#include "visualizer.h"
#include <iostream>
#include <iomanip>

// Helper function to print route details
void printRouteInfo(const std::string &label, const route_opt::Route &route, const PointVector &points) {
    std::cout << "\n" << label << ":\n";
    std::cout << "Total Distance: " << std::fixed << std::setprecision(2) << route.totalDistance << "\n";
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
        config.numPoints = 20; // Start with 20 points for testing
        config.minCoord = 0.0;
        config.maxCoord = 100.0;
        config.seed = 42; // Fixed seed for reproducibility

        route_opt::RouteGenerator generator(config.seed);
        auto [points, distances] = generator.generateRandomEuclidean(config);

        std::cout << "Generated " << points.size() << " random points\n";

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
        visualizer.showPreview(false); // Show real-time preview
        visualizer.beginRecording();

        // Initial route (before optimization)
        route_opt::Route initial_route;
        initial_route.path.resize(points.size());
        std::iota(initial_route.path.begin(), initial_route.path.end(), 0);

        // Calculate initial distance
        double initial_distance = 0.0;
        for (size_t i = 0; i < points.size(); ++i) {
            size_t j = (i + 1) % points.size();
            initial_distance += points[initial_route.path[i]].distanceTo(points[initial_route.path[j]]);
        }
        initial_route.totalDistance = initial_distance;

        // Add initial frame
        visualizer.addFrame(points, initial_route);
        printRouteInfo("Initial Route", initial_route, points);

        // Optimize route
        std::cout << "\nOptimizing route...\n";
        route_opt::Route optimized_route = optimizer->findOptimalRoute(points);

        // Add final frame
        visualizer.addFrame(points, optimized_route);
        printRouteInfo("Optimized Route", optimized_route, points);

        // Calculate improvement
        double improvement = ((initial_route.totalDistance - optimized_route.totalDistance)
                              / initial_route.totalDistance) * 100.0;
        std::cout << "\nImprovement: " << std::fixed << std::setprecision(2)
                << improvement << "%\n";

        visualizer.finalizeVideo();
        delete optimizer;

        std::cout << "\nVisualization saved to: " << visConfig.outputPath << "\n";
        return 0;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
