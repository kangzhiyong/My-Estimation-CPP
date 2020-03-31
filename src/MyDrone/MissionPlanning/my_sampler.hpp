//
//  my_sampler.hpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/11.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#ifndef my_sampler_hpp
#define my_sampler_hpp

#include "k_d_tree.hpp"
#include "my_data.hpp"
#include "my_polygon.hpp"

#include <vector>

class MySampler
{
private:
    std::vector<MyPolygon> m_mpPoloygons;
    float m_fXmin{0};
    float m_fXmax{0};
    float m_fYmin{0};
    float m_fYmax{0};
    float m_fZmin{0};
    float m_fZmax{0};
    float m_fMaxPolyXY{0};
    tree2D  m_kdTree2D;
    
public:
    MySampler(MyData datas, float safe_distance);
    void extract_polygons(MyData datas, float safe_distance);
    ~MySampler();
    VFloat uniform(float min, float max, int num);
    std::vector<point3D> sample(int num);
    bool can_connect(point3D p1, point3D p2);
};
#endif /* my_sampler_hpp */
