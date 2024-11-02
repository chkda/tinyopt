#include "cuda/optimizer.cuh"
#include "cuda/two_opt.cuh"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <random>
#include <numeric>

namespace route_opt {
    namespace cuda {
        thrust::device_vector<double> CUDAOptimizer::prepareDistances(
                const PointVector &points) {
            const int n = points.size();
            thrust::host_vector<double> distances_h(n * n);

#pragma omp parallel for collapse(2)
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    distances_h[i * n + j] = points[i].distanceTo(points[j]);
                }
            }

            return thrust::device_vector<double>(distances_h);
        }

        thrust::device_vector<int> CUDAOptimizer::initializeRoute(int size) {
            thrust::host_vector<int> route_h(size);
            std::iota(route_h.begin(), route_h.end(), 0);

            std::random_device rd;
            std::mt19937 gen(rd());
            std::shuffle(route_h.begin(), route_h.end(), gen);

            return thrust::device_vector<int>(route_h);
        }

        double CUDAOptimizer::computeTotalDistance(
                const thrust::device_vector<double> &distances,
                const thrust::device_vector<int> &route) {

            const int n = route.size();
            thrust::host_vector<int> route_h = route;
            thrust::host_vector<double> distances_h = distances; // Changed to host_vector

            double total = 0.0;

            for (int i = 0; i < n; ++i) {
                int from = route_h[i];
                int to = route_h[(i + 1) % n];
                total += distances_h[from * n + to];
            }
            return total;
        }
    }
}
