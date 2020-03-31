//
//  MyDrone.cpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/25.
//

#include <algorithm>
using namespace std;

#include "my_drone.hpp"

MyDrone::MyDrone(MavlinkConnection *conn)
{
    m_conn = conn;
    
    _update_property[STATE] = &MyDrone::_update_state;
    _update_property[GLOBAL_POSITION] = &MyDrone::_update_global_position;
    _update_property[LOCAL_POSITION] = &MyDrone::_update_local_position;
    _update_property[GLOBAL_HOME] = &MyDrone::_update_global_home;
    _update_property[LOCAL_VELOCITY] = &MyDrone::_update_local_velocity;
    _update_property[RAW_GYROSCOPE] = &MyDrone::_update_gyro_raw;
    _update_property[RAW_ACCELEROMETER] = &MyDrone::_update_acceleration_raw;
    _update_property[BAROMETER] = &MyDrone::_update_barometer;
    _update_property[ATTITUDE] = &MyDrone::_update_attitude;

    // set the internal callbacks list to an empty map
    _callbacks.clear();
    
    if (m_conn != nullptr) {
        m_conn->set_notify_callback(&MyDrone::on_message_receive);
        m_conn->set_drone(this);
    }
}

void MyDrone::on_message_receive(message_ids msg_name, void *msg1)
{
    MessageBase msg = *((MessageBase *)msg1);
    //Sorts incoming messages, updates the drone state variables and runs callbacks
    if (((msg.getTime() - _message_time) > 0.0))
    {
        _message_frequency = 1.0 / (msg.getTime() - _message_time);
        _message_time = msg.getTime();
        _time_bias = msg.getTime() - time(nullptr);
    }

    if (msg_name == CONNECTION_CLOSED)
    {
        stop();
    }
    
    update_property_t::iterator iter;
    iter = _update_property.find(msg_name);
    if(iter != _update_property.end())
    {
        (this->*_update_property[msg_name])(msg1);
    }
    
    notify_callbacks(msg_name);  // pass it along to these listeners
}

point3D MyDrone::global_position()
{
    return {_longitude, _latitude, _altitude};
}

time_t MyDrone::global_position_time()
{
    return _global_position_time;
}

void MyDrone::_update_global_position(void *msg)
{
    GlobalFrameMessage gfm = *(GlobalFrameMessage *)msg;
    _longitude = gfm.longitude();
    _latitude = gfm.latitude();
    _altitude = gfm.altitude();
    if ((gfm.getTime() - _global_position_time) > 0.0)
    {
        _global_position_frequency = 1.0 / (gfm.getTime() - _global_position_time);
    }
    _global_position_time = gfm.getTime();
}

point3D MyDrone::global_home()
{
    return {_home_longitude, _home_latitude, _home_altitude};
}

time_t MyDrone::home_position_time()
{
    return _home_position_time;
}

void MyDrone::_update_global_home(void *msg)
{
    GlobalFrameMessage gfm = *(GlobalFrameMessage *)msg;
    _home_longitude = gfm.longitude();
    _home_latitude = gfm.latitude();
    _home_altitude = gfm.altitude();
    if ((gfm.getTime() - _home_position_time) < 0.0)
    {
        _home_position_frequency = 1.0 / (gfm.getTime() - _home_position_time);
    }
    _home_position_time = gfm.getTime();
}

point3D MyDrone::local_position()
{
    return {_north, _east, _down};
}

time_t MyDrone::local_position_time()
{
    return _local_position_time;
}

void MyDrone::_update_local_position(void *msg)
{
    LocalFrameMessage lfm = *(LocalFrameMessage *)msg;
    _north = lfm.north();
    _east = lfm.east();
    _down = lfm.down();
    if ((lfm.getTime() - _local_position_time) > 0.0)
    {
        _local_position_frequency = 1.0 / (lfm.getTime() - _local_position_time);
    }
    _local_position_time = lfm.getTime();
}

void MyDrone::_update_local_position(point3D p)
{
    _north = p[0];
    _east = p[1];
    _down = p[2];
}

point3D MyDrone::local_velocity()
{
    return {_velocity_north, _velocity_east, _velocity_down};
}

time_t MyDrone::local_velocity_time()
{
    return _local_velocity_time;
}

void MyDrone::_update_local_velocity(void *msg)
{
    LocalFrameMessage lfm = *(LocalFrameMessage *)msg;
    _velocity_north = lfm.north();
    _velocity_east = lfm.east();
    _velocity_down = lfm.down();
    if ((lfm.getTime() - _local_velocity_time) > 0.0)
    {
        _local_velocity_frequency = 1.0 / (lfm.getTime() - _local_velocity_time);
    }
    _local_velocity_time = lfm.getTime();
}

bool MyDrone::armed()
{
    return _armed;
}

bool MyDrone::guided()
{
    return _guided;
}

bool MyDrone::connected()
{
    if (m_conn != nullptr) {
        return m_conn->open();
    }
    return false;
}

time_t MyDrone::state_time()
{
    return _state_time;
}

int MyDrone::status()
{
    return _status;
}

void MyDrone::_update_state(void *msg)
{
    StateMessage sm = *((StateMessage *)msg);
    _armed = sm.armed();
    _guided = sm.guided();
    if ((sm.getTime() - _state_time) > 0.0)
    {
        _state_frequency = 1.0 / (sm.getTime() - _state_time);
    }
    _state_time = sm.getTime();
    _status = sm.status();
}

//Roll, pitch, yaw euler angles in radians
point3D MyDrone::attitude()
{
    return {_roll, _pitch, _yaw};
}

time_t MyDrone::attitude_time()
{
    return _attitude_time;
}

void MyDrone::_update_attitude(void *msg)
{
    FrameMessage fm = *(FrameMessage *)msg;
    _roll = fm.roll();
    _pitch = fm.pitch();
    _yaw = fm.yaw();
    if ((fm.getTime() - _attitude_time) > 0.0)
    {
        _attitude_frequency = 1.0 / (fm.getTime() - _attitude_time);
    }
    _attitude_time = fm.getTime();
}

point3D MyDrone::acceleration_raw()
{
    return {_acceleration_x, _acceleration_y, _acceleration_z};
}

time_t MyDrone::acceleration_time()
{
    return _acceleration_time;
}

void MyDrone::_update_acceleration_raw(void *msg)
{
    BodyFrameMessage bfm = *(BodyFrameMessage *)msg;
    _acceleration_x = bfm.x();
    _acceleration_y = bfm.y();
    _acceleration_z = bfm.z();
    if ((bfm.getTime() - _acceleration_time) > 0.0)
    {
        _acceleration_frequency = 1.0 / (bfm.getTime() - _acceleration_time);
    }
    _acceleration_time = bfm.getTime();
}

//Angular velocites in radians/second
point3D MyDrone::gyro_raw()
{
    return {_gyro_x, _gyro_y, _gyro_z};
}

time_t MyDrone::gyro_time()
{
    return _gyro_time;
}

void MyDrone::_update_gyro_raw(void *msg)
{
    BodyFrameMessage bfm = *(BodyFrameMessage *)msg;
    _gyro_x = bfm.x();
    _gyro_y = bfm.y();
    _gyro_z = bfm.z();
    if ((bfm.getTime() - _gyro_time) > 0.0)
    {
        _gyro_frequency = 1.0 / (bfm.getTime() - _gyro_time);
    }
    _gyro_time = bfm.getTime();
}

float MyDrone::barometer()
{
    return _baro_altitude;
}

time_t MyDrone::barometer_time()
{
    return _baro_time;
}

void MyDrone::_update_barometer(void *msg)
{
    BodyFrameMessage bfm = *(BodyFrameMessage *)msg;
    _baro_altitude = bfm.z();
    _baro_frequency = 1.0 / (bfm.getTime() - _baro_time);
    _baro_time = bfm.getTime();
}

// Handling of internal messages for callbacks

void MyDrone::register_callback(message_ids name, user_callback fn)
{
    /*
    Add the function, `fn`, as a callback for the message type, `name`.

    Args:
        name: describing the message id
        fn: Callback function

    Example:

        add_message_listener(GLOBAL_POSITION, global_msg_listener)

        OR

        add_message_listener(ANY, all_msg_listener)

    These can be added anywhere in the code and are identical to initializing a callback with the decorator
    */
    user_callback_t::iterator iter;
    iter = _callbacks.find(name);
    if(iter != _callbacks.end())
    {
        _callbacks[name].push_back(fn);
    }
    else
    {
        vector<user_callback> fns;
        fns.push_back(fn);
        _callbacks[name] =  fns;
    }
}

void MyDrone::remove_callback(message_ids name, user_callback fn)
{
    /*
    Remove the function, `fn`, as a callback for the message type, `name`

    Args:
        name: describing the message id
        fn: Callback function

    Example:

        remove_message_listener(GLOBAL_POSITION, global_msg_listener)
     */
    user_callback_t::iterator iter;
    iter = _callbacks.find(name);
    if(iter != _callbacks.end())
    {
        vector<user_callback> fns = iter->second;
        vector<user_callback>::iterator ifns = find(fns.begin(), fns.end(), fn);
        if (ifns != fns.end()) {
            iter->second.erase(ifns);
            if (iter->second.size() == 0) {
                _callbacks.erase(name);
            }
        }
    }
}

void MyDrone::notify_callbacks(message_ids name)
{
    //Passes the message to the appropriate listeners
    user_callback_t::iterator iter;
    iter = _callbacks.find(name);
    if(iter != _callbacks.end())
    {
        vector<user_callback> fns = _callbacks[name];
        for (int i = 0; i < fns.size(); i++) {
            (this->*fns[i])();
        }
    }
    
    iter = _callbacks.find(ANY);
    if(iter != _callbacks.end())
    {
        vector<user_callback> fns = _callbacks[ANY];
        for (int i = 0; i < fns.size(); i++) {
            (this->*fns[i])();
        }
    }
}

// Command method wrappers

void MyDrone::arm()
{
    // Send an arm command to the drone
    try {
        if (m_conn != nullptr) {
            m_conn->arm();
        }
    } catch (...) {
        perror("arm failed: ");
    }
}

void MyDrone::disarm()
{
    // Send a disarm command to the drone
    try {
        if (m_conn != nullptr) {
            m_conn->disarm();
        }
    } catch (...) {
        perror("disarm failed: ");
    }
}

void MyDrone::take_control()
{
    /*
    Send a command to the drone to switch to guided (autonomous) mode.

    Essentially control the drone with code.
    */
    try {
        if (m_conn != nullptr) {
            m_conn->take_control();
        }
    } catch (...) {
        perror("take_control failed: ");
    }
}

void MyDrone::release_control()
{
    /*
    Send a command to the drone to switch to manual mode.

    Essentially you control the drone manually via some interface.
     */
    try {
        if (m_conn != nullptr) {
            m_conn->release_control();
        }
    } catch (...) {
        perror("release_control failed: ");
    }
}

void MyDrone::cmd_position( float north, float east, float altitude, float heading)
{
    /*
    Command the local position and drone heading.

    Args:
        north: local north in meters
        east: local east in meters
        altitude: altitude above ground in meters
        heading: drone yaw in radians
    */
    try {
        // connection cmd_position is defined as NED, so need to flip the sign
        // on altitude
        if (m_conn != nullptr) {
            m_conn->cmd_position(north, east, -altitude, heading);
        }
    } catch (...) {
        perror("cmd_position failed: ");
    }
}

void MyDrone::takeoff(float target_altitude)
{
    // Command the drone to takeoff to the target_alt (in meters)
    try {
        if (m_conn != nullptr) {
            m_conn->takeoff(local_position()[0], local_position()[1], target_altitude);
        }
    } catch (...) {
        perror("takeoff failed: ");
    }
}

void MyDrone::land()
{
    // Command the drone to land at its current position
    try {
        if (m_conn != nullptr) {
            m_conn->land(local_position()[0], local_position()[1]);
        }
    } catch (...) {
        perror("land failed: ");
    }
}

void MyDrone::cmd_attitude( float roll, float pitch, float yaw, float thrust)
{
    /*
    Command the drone through attitude command

    Args:
        roll: in radians
        pitch: in randians
        yaw_rate: in radians
        thrust: normalized thrust on [0, 1] (0 being no thrust, 1 being full thrust)
    */
    try {
        if (m_conn != nullptr) {
            m_conn->cmd_attitude(roll, pitch, yaw, thrust);
        }
    } catch (...) {
        perror("cmd_attitude failed: ");
    }
}

void MyDrone::cmd_attitude_rate( float roll_rate, float pitch_rate, float yaw_rate, float thrust)
{
    /*
    Command the drone orientation rates.

    Args:
        roll_rate: in radians/second
        pitch_rate: in radians/second
        yaw_rate: in radians/second
        thrust: upward acceleration in meters/second^2
    */
    try {
        if (m_conn != nullptr) {
            m_conn->cmd_attitude_rate(roll_rate, pitch_rate, yaw_rate, thrust);
        }
    } catch (...) {
        perror("cmd_attitude_rate failed: ");
    }
}

void MyDrone::cmd_moment( float roll_moment, float pitch_moment, float yaw_moment, float thrust)
{
    /*
    Command the drone moments.

    Args:
        roll_moment: in Newton*meter
        pitch_moment: in Newton*meter
        yaw_moment: in Newton*meter
        thrust: upward force in Newtons
    */
    try {
        if (m_conn != nullptr) {
            m_conn->cmd_moment(roll_moment, pitch_moment, yaw_moment, thrust);
        }
    } catch (...) {
        perror("cmd_moment failed: ");
    }
}

void MyDrone::cmd_velocity( float velocity_north, float velocity_east, float velocity_down, float heading)
{
    /*
    Command the drone velocity.

    Args:
        north_velocity: in meters/second
        east_velocity: in meters/second
        down_velocity: in meters/second
        heading: in radians
    */
    try {
        if (m_conn != nullptr) {
            m_conn->cmd_velocity(velocity_north, velocity_east, velocity_down, heading);
        }
    } catch (...) {
        perror("cmd_velocity failed: ");
    }
}

void MyDrone::set_home_position( float longitude, float latitude, float altitude)
{
    // Set the drone's home position to these coordinates
    try {
        if (m_conn != nullptr) {
            m_conn->set_home_position(latitude, longitude, altitude);
        }
    } catch (...) {
        perror("set_home_position failed: ");
    }
}

void MyDrone::set_home_as_current_position()
{
    // Set the drone's home position to its current position
    set_home_position(_longitude, _latitude, _altitude);
}

void MyDrone::start()
{
    // Starts the connection to the drone
    if (m_conn != nullptr) {
        m_conn->start();
    }
}

void MyDrone::stop()
{
    // Stops the connection to the drone
    if (m_conn != nullptr) {
        m_conn->stop();
    }
}
