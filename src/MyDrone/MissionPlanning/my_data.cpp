//
//  my_data.cpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/11.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#include "my_data.hpp"
#include <string>
#include <fstream>
using namespace std;

typedef vector<string> VString;
VString split(string str, string delimiter)
{
    VString strList;
    char*  tmp = nullptr;
    tmp = strtok((char *)str.c_str(), delimiter.c_str());
    while (tmp)
    {
        strList.emplace_back(tmp);
        tmp = strtok(nullptr, ",");
    }
    return strList;
}

MyData::MyData(std::string fileName, std::string delimiter)
{
    fstream file;
    file.open(fileName.c_str());
    if (file.is_open())
    {
        int i = 1;
        string str;
        while (getline(file, str))
        {
            VString strList = split(str, delimiter);
            if (i == 1 && strList.size() == 2)
            {
                VString lat0 = split(strList[0], " ");
                VString lon0 = split(strList[1], " ");
                if (lat0.size() == 2)
                {
                    m_dLat =atof(lat0[1].c_str());
                }
                if (lon0.size() == 2)
                {
                    m_dLon = atof(lon0[1].c_str());
                }
            }
            else if (i > 2 && strList.size() == 6)
            {
                m_qvNorths.push_back(atof(strList[0].c_str()));
                m_qvEasts.push_back(atof(strList[1].c_str()));
                m_qvAlts.push_back(atof(strList[2].c_str()));
                m_qvDNorths.push_back(atof(strList[3].c_str()));
                m_qvDEasts.push_back(atof(strList[4].c_str()));
                m_qvDAlts.push_back(atof(strList[5].c_str()));
            }
            i++;
        }
    }
}
MyData::~MyData()
{
    m_qvNorths.clear();
    m_qvEasts.clear();
    m_qvAlts.clear();
    m_qvDNorths.clear();
    m_qvDEasts.clear();
    m_qvDAlts.clear();
}
float MyData::getXMin()
{
    int i = 0;
    VFloat x;
    for (i = 0; i < m_qvNorths.size(); i++)
    {
        x.push_back(m_qvNorths[i] - m_qvDNorths[i]);
    }
    
    return  ( i > 0) ? *(std::min_element(x.begin(), x.end())) : 0;
}
float MyData::getXMax()
{
    int i = 0;
    VFloat x;
    for (i = 0; i < m_qvNorths.size(); i++)
    {
        x.push_back(m_qvNorths[i] + m_qvDNorths[i]);
    }
    return ( i > 0) ? *(std::max_element(x.begin(), x.end())) : 0;
}
float MyData::getYMin()
{
    int i = 0;
    VFloat y;
    for (i = 0; i < m_qvEasts.size(); i++)
    {
        y.push_back(m_qvEasts[i] - m_qvDEasts[i]);
    }
    return ( i > 0) ? *(std::min_element(y.begin(), y.end())) : 0;
}
float MyData::getYMax()
{
    int i = 0;
    VFloat y;
    for (i = 0; i < m_qvEasts.size(); i++)
    {
        y.push_back(m_qvEasts[i] + m_qvDEasts[i]);
    }
    return ( i > 0) ? *(std::max_element(y.begin(), y.end())) : 0;
}
float MyData::getZMin()
{
    int i = 0;
    VFloat z;
    for (i = 0; i < m_qvAlts.size(); i++)
    {
        z.push_back(m_qvAlts[i] - m_qvDAlts[i]);
    }
    return ( i > 0) ? *(std::min_element(z.begin(), z.end())) : 0;
}
float MyData::getZMax()
{
    int i = 0;
    VFloat z;
    for (i = 0; i < m_qvAlts.size(); i++)
    {
        z.push_back(m_qvAlts[i] + m_qvDAlts[i]);
    }
    return ( i > 0) ? *(std::max_element(z.begin(), z.end())) : 0;
}
float MyData::getMaxPolyXY()
{
    float maxN = 0, maxE = 0;
    if (m_qvDNorths.size() > 0)
    {
        maxN = *(std::max_element(m_qvDNorths.begin(), m_qvDNorths.end()));
    }
    if (m_qvDEasts.size() > 0)
    {
        maxE = *(std::max_element(m_qvDEasts.begin(), m_qvDEasts.end()));
    }
    return std::max(maxN, maxE);
}
size_t MyData::dataCount()
{
    return m_qvNorths.size();
}
float MyData::getNorth(int i)
{
    return (i >= 0 && i < dataCount()) ? m_qvNorths[i] : 0;
}
float MyData::getEast(int i)
{
    return (i >= 0 && i < dataCount()) ? m_qvEasts[i] : 0;
}
float MyData::getAlt(int i)
{
    return (i >= 0 && i < dataCount()) ? m_qvAlts[i] : 0;
}
float MyData::getDNorth(int i)
{
    return (i >= 0 && i < dataCount()) ? m_qvDNorths[i] : 0;
}
float MyData::getDEast(int i)
{
    return (i >= 0 && i < dataCount()) ? m_qvDEasts[i] : 0;
}
float MyData::getDAlt(int i)
{
    return (i >= 0 && i < dataCount()) ? m_qvDAlts[i] : 0;
}

float MyData::getLat()
{
    return m_dLat;
}

float MyData::getLon()
{
    return m_dLon;
}
