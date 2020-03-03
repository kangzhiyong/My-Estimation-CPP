//
//  message_types.cpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/25.
//

#include "message_types.hpp"

MessageBase::MessageBase(int64_t t)
{
    _time = t;
}
int64_t MessageBase::getTime()
{
    return _time;
}

StateMessage::StateMessage(int64_t time, bool armed, bool guided, int status):MessageBase(time)
{
    _armed = armed;
    _guided = guided;
    _status = status;
}

bool StateMessage::armed()
{
    //bool: true if the drone is armed and ready to fly
    return _armed;
}

bool StateMessage::guided()
{
    //bool: true if the drone can be commanded from python
    return _guided;
}
int StateMessage::status()
{
    //int: status value from the autopilot not corresponding to anything in particular
    return _status;
}

GlobalFrameMessage::GlobalFrameMessage( int64_t time, float latitude, float longitude, float altitude):MessageBase(time)
{
    _longitude = longitude;
    _latitude = latitude;
    _altitude = altitude;
}

float GlobalFrameMessage::longitude()
{
    //float: longitude in degrees
    return _longitude;
}

float GlobalFrameMessage::latitude()
{
    //float: latitude in degrees
    return _latitude;
}

float GlobalFrameMessage::altitude()
{
    //float: altitude in meters above sea level
    return _altitude;
}

vector<float> GlobalFrameMessage::global_vector()
{
    vector<float> gv;
    gv.push_back(_longitude);
    gv.push_back(_latitude);
    gv.push_back(_altitude);
    return gv;
}

LocalFrameMessage::LocalFrameMessage( int64_t time, float north, float east, float down):MessageBase(time)
{
    _north = north;
    _east = east;
    _down = down;
}

float LocalFrameMessage::north()
{
    //float: north position in meters
    return _north;
}

float LocalFrameMessage::east()
{
    //float: east position in meters
    return _east;
}

float LocalFrameMessage::down()
{
    //float: down position in meters
    return _down;
}

vector<float> LocalFrameMessage::local_vector()
{
    vector<float> lv;
    lv.push_back(_north);
    lv.push_back(_east);
    lv.push_back(_down);
    return lv;
}

BodyFrameMessage::BodyFrameMessage(int64_t time, float x, float y, float z):MessageBase(time)
{
    _x = x;
    _y = y;
    _z = z;
}

float BodyFrameMessage::x()
{
    //float: x value
    return _x;
}

float BodyFrameMessage::y()
{
    //float: y value
    return _y;
}

float BodyFrameMessage::z()
{
    //float: z value
    return _z;
}

vector<float> BodyFrameMessage::body_vector()
{
    vector<float> bv;
    bv.push_back(_x);
    bv.push_back(_y);
    bv.push_back(_z);
    return bv;
}

FrameMessage::FrameMessage(int64_t time, float roll, float pitch, float yaw): MessageBase(time)
{
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;

    float sp = sin(pitch / 2.0);
    float cp = cos(pitch / 2.0);
    float sr = sin(roll / 2.0);
    float cr = cos(roll / 2.0);
    float sy = sin(yaw / 2.0);
    float cy = cos(yaw / 2.0);

    _q0 = cr * cp * cy + sr * sp * sy;
    _q1 = sr * cp * cy - cr * sp * sy;
    _q2 = cr * sp * cy + sr * cp * sy;
    _q3 = cr * cp * sy - sr * sp * cy;
}

FrameMessage::FrameMessage(int64_t time, float q0, float q1, float q2, float q3): MessageBase(time)
{
    
    _q0 = q0;
    _q1 = q1;
    _q2 = q2;
    _q3 = q3;

    _roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (pow(q1, 2) + pow(q2, 2)));
    _pitch = asin(2.0 * (q0 * q2 - q3 * q1));
    _yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (pow(q2, 2) + pow(q3, 2)));
}

float FrameMessage::roll()
{
    //roll in radians
    return _roll;
}

float FrameMessage::pitch()
{
    //pitch in radians
    return _pitch;
}

float FrameMessage::yaw()
{
    //yaw in radians
    return _yaw;
}

float FrameMessage::q0()
{
    //float: 0th element of quaternion
    return _q0;
}

float FrameMessage::q1()
{
    //float: 1st element of quaternion
    return _q1;
}

float FrameMessage::q2()
{
    //float: 2nd element of quaternion
    return _q2;
}

float FrameMessage::q3()
{
    //float: 3rd element of quaternion
    return _q3;
}

vector<float> FrameMessage::euler_angles()
{
    vector<float> eav;
    eav.push_back(_roll);
    eav.push_back(_pitch);
    eav.push_back(_yaw);
    return eav;
}

vector<float> FrameMessage::quaternions()
{
    vector<float> qv;
    qv.push_back(_q0);
    qv.push_back(_q1);
    qv.push_back(_q2);
    qv.push_back(_q3);
    return qv;
}

DistanceSensorMessage::DistanceSensorMessage(int64_t time, float min_distance, float max_distance, float direction, float measurement, float covariance): MessageBase(time)
{
    measuremen.push_back(direction);
    measuremen.push_back(measurement);
    measuremen.push_back(covariance);
    properties.push_back(min_distance);
    properties.push_back(max_distance);
}
