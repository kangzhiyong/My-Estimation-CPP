//
//  my_polygon.hpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/11.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#ifndef my_polygon_hpp
#define my_polygon_hpp

#include "my_point.hpp"

class MyRect
{
private:
    point2D m_pTopLeft;
    point2D m_pBottomRight;
public:
    MyRect(const point2D topLeft, const point2D bottomRight);
    point2D getTopLeft();
    point2D getBottomRight();
    void setTopLeft(point2D topLeft);
    void setBottomRight(point2D bottomRight);
    point2D center();
    bool contains(point2D p);
    bool crosses(MyRect rect);
};

class MyPolygon: public MyRect
{
private:
    float m_fHeight;
public:
    MyPolygon(const point2D topLeft, const point2D bottomRight, float height);
    float getHeight();
    void setHeight(float height);
};

#endif /* my_polygon_hpp */
