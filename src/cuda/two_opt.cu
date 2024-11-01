#include "cuda/optimizer.cuh"
#include "cuda/two_opt.cuh"
#include <thrust/execution_policy.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/logical.h>

namespace route_opt {
    namespace gpu {
        class TwoOptOptimizer : public route_opt::cuda::CUDAOptimizer {
        public:
            Route findOptimalRoute(const PointVector &points) override {
                const int n = points.size();

                auto distances_d = prepareDistances(points);
                auto route_d = initializeRoute(n);
                thrust::device_vector<bool> improved_d(1);

                const int max_iterations = 100;
                int iteration = 0;

                do {
                    improved_d[0] = false;
                    thrust::counting_iterator<int> begin(0);
                    thrust::counting_iterator<int> end(n - 2);

                    TwoOptSwapFunctor::TwoOptSwapFunctor swap_op(
                            thrust::raw_pointer_cast(distances_d.data()),
                            thrust::raw_pointer_cast(route_d.data()),
                            n
                            );

                    auto pairs_begin = thrust::make_zip_iterator(
                            thrust::make_tuple(
                                    thrust::make_counting_iterator<int>(0),
                                    thrust::make_counting_iterator<int>(2)
                                    )
                            );

                    auto pairs_end = thrust::make_zip_iterator(
                            thrust::make_tuple(
                                    thrust::make_counting_iterator<int>(n - 2),
                                    thrust::make_counting_iterator<int>(n)
                                    )
                            );

                    thrust::transform(
                            thrust::device,
                            pairs_begin,
                            pairs_end,
                            improved_d.begin(),
                            swap_op
                            );

                    iteration++;
                } while (thrust::any_of(improved_d.begin(), improved_d.end(),
                                        thrust::identity<bool>()) && iteration < max_iterations);

                Route result;
                thrust::host_vector<int> route_h = route_d;
                result.path.assign(route_h.begin(), route_h.end());
                result.totalDistance = computeTotalDistance(route_d, distances_d);

                return result;
            }
        };
    }
};
