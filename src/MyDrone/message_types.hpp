/*
Message Types
custom set of message types to use between a specific connection type and
the drone class.
this enables abstracting away the protocol specific messages so different
protocols can be used with the same student facing interface and code.
NOTE: besides the state message, all messages are in the frame they are
defined in.
NOTE: to ensure minimal errors due to typos, use the MSG_* constants when
registering for specific messages
Attributes:
    MSG_ALL: flag to be used to register a listener for all messages
    MSG_STATE: name of the state message
    MSG_GLOBAL_POSITION: name of the global position message [StateMessage]
    MSG_LOCAL_POSITION: name of the local position message [LocalFrameMessage]
    MSG_GLOBAL_HOME: name of the global home message [GlobalFrameMessage]
    MSG_VELOCITY: name of the velocity message [LocalFrameMessage]
    MSG_CONNECTION_CLOSED: name of the message sent when the connection is closed (no data)
    MSG_RAW_GYROSCOPE: name of the raw gyro message [BodyFrameMessage]
    MSG_RAW_ACCELEROMETER: name of the raw acceleromater message [BodyFrameMessage]
    MSG_BAROMETER: name of the barometer message [LocalFrameMessage - only down populated]
    MSG_ATTITUDE: name of attitude message [FrameMessage]
*/

#ifndef message_types_hpp
#define message_types_hpp

#include <cmath>
#include <vector>
using namespace std;

class MessageBase
{
    //Message super class
public:
    MessageBase(){_time = 0;}
    MessageBase(int64_t t);
    int64_t getTime();
private:
    int64_t _time;
};

class StateMessage: public MessageBase
{
    /*
     State information message
    message to carry drone state information
    Attributes:
        _armed: whether or not drone is armed
        _guided: whether or not drone can be commanded from script
    */

public:
    StateMessage(){};
    StateMessage(int64_t time, bool armed, bool guided, int status = 0);
    bool armed();
    bool guided();
    int status();
private:
    bool _armed{false};
    bool _guided{false};
    int _status{0};
};


class GlobalFrameMessage: public MessageBase
{
    /*
     Global frame message
    message to carry information in a global frame
    Attributes:
        _latitude: latitude in degrees
        _longitude: longitude in degrees
        _altitude: altitude in meters above mean sea level (AMSL)
    */
public:
    GlobalFrameMessage( int64_t time, float latitude, float longitude, float altitude);
    float longitude();
    float latitude();
    float altitude();
    vector<float> global_vector();
private:
    float _longitude;
    float _latitude;
    float _altitude;
};

class LocalFrameMessage: public MessageBase
{
    /*
     Local frame message
    message to carry information in a local (NED) frame
    Attributes:
        _north: north position in meters
        _east: east position in meters
        _down: down position in meters (above takeoff point, positive down)
    */
public:
    LocalFrameMessage( int64_t time, float north, float east, float down);
    float north();
    float east();
    float down();
    vector<float> local_vector();
private:
    float _north;
    float _east;
    float _down;
};

class BodyFrameMessage: public MessageBase
{
    /*
     Body frame message
    message to carry information in a body frame
    Attributes:
        _x: x value
        _y: y value
        _z: z value
    */
public:
    BodyFrameMessage(int64_t time, float x, float y, float z);
    float x();
    float y();
    float z();
    vector<float> body_vector();
private:
    float _x;
    float _y;
    float _z;
};

class FrameMessage: public MessageBase
{
    /*
        Message representating frame information
        Messages defining the rotation between frames (Euler angles or Quaternions)
        Attributes:
            _roll: drone roll in radians
            _pitch: drone pitch in radians
            _yaw: drone yaw in radians
            _q0: 0th element of quaterion
            _q1: 1th element of quaterion
            _q2: 2th element of quaterion
            _q3: 3th element of quaterion
    */
public:
    FrameMessage(int64_t time, float roll, float pitch, float yaw);
    FrameMessage(int64_t time, float q0, float q1, float q2, float q3);
    float roll();
    float pitch();
    float yaw();
    float q0();
    float q1();
    float q2();
    float q3();
    vector<float> euler_angles();
    vector<float> quaternions();
    
private:
    float _roll;
    float _pitch;
    float _yaw;
    float _q0;
    float _q1;
    float _q2;
    float _q3;
};

class DistanceSensorMessage: public MessageBase
{
    /*
    Message for distance sensor (e.g. Lidar) information
    the properties of and measurement from a given distance sensor onboard
    the drone.
    Attributes:
        _min_distance: minimum detectable distance in meters
        _max_distance: maximum detectable distance in meters
        _direction: the heading of the sensor for this measurement in radians
        _measurement: the distance measured in meters
        _covariance: the covariance of the measurement
    */
public:
    DistanceSensorMessage(int64_t time, float min_distance, float max_distance, float direction, float measurement, float covariance);
private:
    vector<float> measuremen;
    vector<float> properties;
};

#endif /* message_types_hpp */
