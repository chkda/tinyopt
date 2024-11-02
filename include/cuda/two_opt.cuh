#include <thrust/tuple.h>

namespace route_opt {
    namespace cuda {
        namespace two_opt {
            struct TwoOptSwapFunctor {
                const double *distances;
                int *path;
                const int n;

                __host__ __device__
                TwoOptSwapFunctor(const double *d, int *p, int size) : distances(d), path(p), n(size) {
                }

                __host__ __device__
                bool operator()(const thrust::tuple<int, int> &swap) {
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
                }
            };
        }

        class TwoOptOptimizer : public CUDAOptimizer {
        public:
            TwoOptOptimizer() = default;

            ~TwoOptOptimizer() override = default;

            Route findOptimalRoute(const PointVector &points) override;
        };
    }
}
