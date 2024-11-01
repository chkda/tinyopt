#include <thrust/tuple.h>

namespace route_opt {
    namespace cuda {
        namespace two_opt {
            struct TwoOptSwapFunctor {
                const double *distances;
                int *path;
                const int n;

                __host__ __device__
                TwoOptSwapFunctor(const double *d, int *p, int size);

                __host__ __device__
                bool operator()(const thrust::tuple<int, int> &swap);
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
