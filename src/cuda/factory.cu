#include "cuda/optimizer.cuh"
#include "cuda/two_opt.cuh"

namespace route_opt {
    namespace cuda {
        namespace factory {
            RouteOptimizer *createCUDAOptimizer(RouteAlgorithm optimizer) {
                switch (optimizer) {
                    case RouteAlgorithm::TwoOpt:
                        return new TwoOptOptimizer();
                    default:
                        return nullptr;
                }
            };
        }
    }
}
