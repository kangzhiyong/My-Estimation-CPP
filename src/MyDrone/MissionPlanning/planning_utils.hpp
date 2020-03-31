//
//  planning_utils.hpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/16.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#ifndef planning_utils_hpp
#define planning_utils_hpp

#include <cmath>
using namespace std;

inline float norm(float x0, float y0, float z0, float x1, float y1, float z1)
{
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2) + pow(z0 - z1, 2));
}

#endif /* planning_utils_hpp */
