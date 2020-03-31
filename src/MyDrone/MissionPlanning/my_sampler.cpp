//
//  my_sampler.cpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/11.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#include "my_sampler.hpp"

MySampler::MySampler(MyData datas, float safe_distance)
{
    extract_polygons(datas, safe_distance);
    m_fXmin = datas.getXMin();
    m_fXmax = datas.getXMax();
    m_fYmin = datas.getYMin();
    m_fYmax = datas.getYMax();
    m_fZmin = 0;
    m_fZmax = 20;
    m_fMaxPolyXY = 2 * datas.getMaxPolyXY();
}

void MySampler::extract_polygons(MyData datas, float safe_distance)
{
    float north, east, alt, d_north, d_east, d_alt = 0;
    for (int i = 0; i < datas.dataCount(); i++)
    {
        north = datas.getNorth(i);
        east = datas.getEast(i);
        alt = datas.getAlt(i);
        d_north = datas.getDNorth(i);
        d_east = datas.getDEast(i);
        d_alt = datas.getDAlt(i);
        float x1 = north - d_north - safe_distance;
        float y1 = east - d_east - safe_distance;
        float x2 = north + d_north + safe_distance;
        float y2 = east + d_east + safe_distance;
        m_mpPoloygons.push_back(MyPolygon(point2D({x1, y1}), point2D({x2, y2}), alt + d_alt + safe_distance));
        m_kdTree2D.addNode(tree2D::point_type({north, east}));
    }
    m_kdTree2D.generate_tree();
}

MySampler::~MySampler()
{
    m_mpPoloygons.clear();
}

VFloat MySampler::uniform(float min, float max, int num)
{
    VFloat d;
    std::mt19937 engine_((std::random_device()()));
    std::uniform_real_distribution<float> distribution_(min, max);
    for (int i = 0; i < num; i++)
    {
        d.push_back(distribution_(engine_));
    }
    return d;
}

std::vector<point3D> MySampler::sample(int num)
{
    VFloat xs = uniform(m_fXmin, m_fXmax, num);
    VFloat ys = uniform(m_fYmin, m_fYmax, num);
    VFloat zs = uniform(m_fZmin, m_fZmax, num);
    std::vector<point3D> pts;
    for (int i = 0; i < num; i++)
    {
        bool in_collision = false;
        tree2D::point_type point({xs[i], ys[i]});
        m_kdTree2D.nearest_radius(point, m_fMaxPolyXY);
        for (int j = 0; j < m_kdTree2D.m_vNbrNodes.size(); j++)
        {
            MyPolygon poly = m_mpPoloygons[m_kdTree2D.m_vNbrNodes[j].pos_index];
            if (poly.contains(point2D({xs[i], ys[i]})) && poly.getHeight() >= zs[i])
            {
                in_collision = true;
            }
        }
        if (!in_collision)
        {
            pts.push_back(point3D({xs[i], ys[i], zs[i]}));
        }
    }
    return pts;
}

bool MySampler::can_connect(point3D p1, point3D p2)
{
    MyRect rect(point2D({p1[0], p1[1]}), point2D({p2[0], p2[1]}));
    float h = std::min(p1[2], p2[2]);
    for (int i = 0; i < m_mpPoloygons.size(); i++)
    {
        MyPolygon poly = m_mpPoloygons[i];
        if (poly.crosses(rect) && poly.getHeight() >= h)
        {
            return false;
        }
    }
    return true;
}
