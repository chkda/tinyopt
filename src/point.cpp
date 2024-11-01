#include "types.h"
#include <cmath>

namespace route_opt {

    double Point::distanceTo(const Point& p) const {
        double x = this->x - p.x;
        double y = this->y - p.y;
        return sqrt(pow(x, 2) + pow(y, 2));
    }

};