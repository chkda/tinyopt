#include "visualizer.h"
#include "route_generator.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace route_opt;

// Standalone callback function
RouteVisualizer::ProgressCallback createProgressCallback() {
    return [](const Route &route, double progress) {
        std::cout << "Optimization progress: " << (progress * 100) << "%, "
                << "Current route length: " << route.totalDistance << std::endl;
    };
}

// Simulates an optimization step for demonstration
Route simulateOptimizationStep(const PointVector &points, int iteration) {
    Route route;
    route.path.reserve(points.size());

    // Simple nearest neighbor algorithm for demonstration
    std::vector<bool> visited(points.size(), false);
    route.path.push_back(0); // Start from first point
    visited[0] = true;

    // Limit iterations for demonstration
    size_t maxPoints = std::min(points.size(), size_t(iteration + 2));

    for (size_t i = 1; i < maxPoints; ++i) {
        int current = route.path.back();
        double minDist = std::numeric_limits<double>::max();
        int nextPoint = -1;

        for (size_t j = 0; j < points.size(); ++j) {
            if (!visited[j]) {
                double dist = points[current].distanceTo(points[j]);
                if (dist < minDist) {
                    minDist = dist;
                    nextPoint = j;
                }
            }
        }

        if (nextPoint != -1) {
            route.path.push_back(nextPoint);
            visited[nextPoint] = true;
        }
    }

    // Calculate total distance
    route.totalDistance = 0;
    for (size_t i = 1; i < route.path.size(); ++i) {
        route.totalDistance += points[route.path[i - 1]].distanceTo(points[route.path[i]]);
    }

    return route;
}

int main() {
    try {
        // Create route generator and generate random points
        RouteGenerator generator(42); // Fixed seed for reproducibility
        GeneratorConfig config;
        config.numPoints = 20; // Small number of points for demonstration
        config.minCoord = 0;
        config.maxCoord = 100;

        auto [points, distances] = generator.generateRandomEuclidean(config);

        // Configure visualizer
        VisualizerConfig visConfig;
        visConfig.width = 800;
        visConfig.height = 800;
        visConfig.fps = 30;
        visConfig.outputPath = "route_optimization.mp4";
        visConfig.showGrid = true;
        visConfig.showProgress = true;

        // Create visualizer
        RouteVisualizer visualizer(visConfig);
        visualizer.showPreview(true); // Enable real-time preview

        // Set up progress callback using the standalone function
        RouteVisualizer::ProgressCallback progressCallback = createProgressCallback();
        visualizer.setProgressCallback(progressCallback);

        // Begin recording
        visualizer.beginRecording();

        // Simulate optimization process
        const int numIterations = 30;
        for (int i = 0; i < numIterations; ++i) {
            Route currentRoute = simulateOptimizationStep(points, i);

            // Add frame to video
            visualizer.addFrame(points, currentRoute);

            // Report progress
            double progress = static_cast<double>(i) / numIterations;
            visualizer.addIntermediateRoute(currentRoute, progress);

            // Simulate some processing time
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Finalize video
        visualizer.finalizeVideo();

        std::cout << "Visualization completed. Output saved to: route_optimization.mp4" << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
