//
//  my_data.hpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/11.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#ifndef my_data_hpp
#define my_data_hpp

#include <algorithm>
#include <vector>

typedef std::vector<float> VFloat;

class MyData
{
private:
    float m_dLat;
    float m_dLon;
    VFloat m_qvNorths;
    VFloat m_qvEasts;
    VFloat m_qvAlts;
    VFloat m_qvDNorths;
    VFloat m_qvDEasts;
    VFloat m_qvDAlts;
public:
    MyData(std::string fileName, std::string delimiter);
    ~MyData();
    float getXMin();
    float getXMax();
    float getYMin();
    float getYMax();
    float getZMin();
    float getZMax();
    float getMaxPolyXY();
    size_t dataCount();
    float getNorth(int i);
    float getEast(int i);
    float getAlt(int i);
    float getDNorth(int i);
    float getDEast(int i);
    float getDAlt(int i);
    float getLat();
    float getLon();
};
#endif /* my_data_hpp */
