//
//  my_polygon.cpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/11.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#include "my_polygon.hpp"

MyRect::MyRect(const point2D topLeft, const point2D bottomRight): m_pTopLeft(topLeft), m_pBottomRight(bottomRight)
{
    if (topLeft.get(0) > bottomRight.get(0))
    {
        m_pTopLeft = bottomRight;
        m_pBottomRight = topLeft;
    }
    else
    {
        m_pTopLeft = topLeft;
        m_pBottomRight = bottomRight;
    }
}

point2D MyRect::getTopLeft()
{
    return m_pTopLeft;
}

point2D MyRect::getBottomRight()
{
    return m_pBottomRight;
}

void MyRect::setTopLeft(point2D topLeft)
{
    m_pTopLeft = topLeft;
}

void MyRect::setBottomRight(point2D bottomRight)
{
    m_pBottomRight = bottomRight;
}

point2D MyRect::center()
{
    float cx = m_pTopLeft[0] + (m_pBottomRight[0] - m_pTopLeft[0]) / 2.0;
    float cy = m_pTopLeft[1] + (m_pBottomRight[1] - m_pTopLeft[1]) / 2.0;
    return point2D({cx, cy});
}

bool MyRect::contains(point2D p)
{
    if (p[0] >= m_pTopLeft[0] && p[0] <= m_pBottomRight[0]
        && p[1] >= m_pTopLeft[1] && p[1] <= m_pBottomRight[1])
    {
        return true;
    }
    return false;
}

bool MyRect::crosses(MyRect rect)
{
    float minx = std::max(m_pTopLeft[0], rect.getTopLeft()[0]);
    float miny = std::max(m_pTopLeft[1], rect.getTopLeft()[1]);
    float maxx = std::min(m_pBottomRight[0], rect.getBottomRight()[0]);
    float maxy = std::min(m_pBottomRight[1], rect.getBottomRight()[1]);
    if (minx > maxx || miny > maxy)
    {
        return false;
    }
    return true;
}


MyPolygon::MyPolygon(const point2D topLeft, const point2D bottomRight, float height):MyRect(topLeft, bottomRight), m_fHeight(height)
{
}

float MyPolygon::getHeight()
{
    return m_fHeight;
}
void MyPolygon::setHeight(float height)
{
    m_fHeight = height;
}
