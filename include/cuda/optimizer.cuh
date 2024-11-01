#pragma once
#include "optimizer.h"
#include <thrust/device_vector.h>

namespace route_opt {
    namespace cuda {
        class CUDAOptimizer : public RouteOptimizer {
        public:
            virtual ~CUDAOptimizer() = default;

        protected:
            thrust::device_vector<double> prepareDistances(const PointVector &points);

            thrust::device_vector<int> initializeRoute(int size);

            double computeTotalDistance(const thrust::device_vector<double> &distances,
                                        const thrust::device_vector<int> &routes);
        };

        namespace factory {
            enum class RouteAlgorithm {
                TwoOpt,
                SimulatedAnnealing,
                AntColony,
            };

            RouteOptimizer *createCUDAOptimizer(RouteAlgorithm algo);
        }
    }
}
