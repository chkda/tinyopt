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

        namespace two_opt {
            // Changed from utils to two_opt
            __host__ __device__
            TwoOptSwapFunctor::TwoOptSwapFunctor(const double *d, int *p, int size) :
                distances(d), path(p), n(size) {
            } // Removed extra semicolon

            __host__ __device__
            bool TwoOptSwapFunctor::operator()(const thrust::tuple<int, int> &swap) {
                int i = thrust::get<0>(swap);
                int j = thrust::get<1>(swap);

                if (j <= i + 1) {
                    return false;
                }

                double current_distance =
                        distances[path[i] * n + path[i + 1]] +
                        distances[path[j - 1] * n + path[j]]; // Fixed index calculation
                double new_distance =
                        distances[path[i] * n + path[j - 1]] +
                        distances[path[i + 1] * n + path[j]];

                if (new_distance < current_distance) {
                    int left = i + 1;
                    int right = j - 1;
                    while (left < right) {
                        int temp = path[left];
                        path[left] = path[right];
                        path[right] = temp;
                        left++;
                        right--;
                    }
                    return true;
                }
                return false;
            } // Removed extra semicolon
        }
    }
}
