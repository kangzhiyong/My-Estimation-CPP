//
//  MyDrone.hpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/25.
//

#ifndef MyDrone_hpp
#define MyDrone_hpp

#include <map>
#include <vector>
using namespace std;

#include "mavlink_connection.hpp"

class MyDrone
{
private:
    time_t _message_time{0};
    float _message_frequency{0.0};
    time_t _time_bias{0};
    // Global position in degrees (int)
    // Altitude is in meters
    float _longitude{0.0};
    float _latitude{0.0};
    float _altitude{0.0};
    time_t _global_position_time{0};
    float _global_position_frequency{0.0};

    // Reference home position in degrees (int)
    // Altitude is in meters
    float _home_longitude{0.0};
    float _home_latitude{0.0};
    float _home_altitude{0.0};
    time_t _home_position_time{0};
    float _home_position_frequency{0.0};

    // Local positions in meters from the global home (float)
    // In NED frame
    float _north{0.0};
    float _east{0.0};
    float _down{0.0};
    time_t _local_position_time{0};
    float _local_position_frequency{0.0};

    // Locally oriented velocity in meters/second
    // In NED frame
    float _velocity_north{0.0};
    float _velocity_east{0.0};
    float _velocity_down{0.0};
    time_t _local_velocity_time{0};
    float _local_velocity_frequency{0.0};

    // If the drone is armed the motors are powered and the rotors are spinning.
    bool _armed{false};

    // If the drone is guided it is being autonomously controlled,
    // the other opposite would be manual control.
    bool _guided{false};
    
    // An integer to pass along random status changes specific for different vehicles
    int _status{0};
    time_t _state_time{0};
    float _state_frequency{0.0};

    // Euler angles in radians
    float _roll{0.0};
    float _pitch{0.0};
    float _yaw{0.0};
    time_t _attitude_time{0};
    float _attitude_frequency{0.0};

    // Drone body accelerations
    float _acceleration_x{0.0};
    float _acceleration_y{0.0};
    float _acceleration_z{0.0};
    time_t _acceleration_time{0};
    float _acceleration_frequency{0.0};

    // Drone gyro rates or angular velocities in radians/second
    float _gyro_x{0.0};
    float _gyro_y{0.0};
    float _gyro_z{0.0};
    time_t  _gyro_time{0};
    float _gyro_frequency{0.0};

    // Barometer
    float _baro_altitude{0.0};
    float _baro_time{0.0};
    float _baro_frequency{0.0};

    typedef void (MyDrone::*update)(MessageBase data);
    typedef void (MyDrone::*user_callback)();
    typedef map<message_ids, update> update_property_t;
    typedef map<message_ids,  vector<user_callback>> user_callback_t;
    
    update_property_t _update_property;
    user_callback_t _callbacks;
    MavlinkConnection *m_conn;
public:
    MyDrone(MavlinkConnection *conn);
    void on_message_receive(message_ids msg_name, MessageBase msg);
    vector<float> global_position();
    time_t global_position_time();
    void _update_global_position(MessageBase msg);
    vector<float> global_home();
    time_t home_position_time();
    void _update_global_home(MessageBase msg);
    vector<float> local_position();
    time_t local_position_time();
    void _update_local_position(MessageBase msg);
    vector<float> local_velocity();
    time_t local_velocity_time();
    void _update_local_velocity(MessageBase msg);
    bool armed();
    bool guided();
    bool connected();
    time_t state_time();
    int status();
    void _update_state(MessageBase msg);
    
    // Roll, pitch, yaw euler angles in radians
    vector<float> attitude();
    time_t attitude_time();
    void _update_attitude(MessageBase msg);
    vector<float> acceleration_raw();
    time_t acceleration_time();
    void _update_acceleration_raw(MessageBase msg);
    
    // Angular velocites in radians/second
    vector<float> gyro_raw();
    time_t gyro_time();
    void _update_gyro_raw(MessageBase msg);
    float barometer();
    time_t barometer_time();
    void _update_barometer(MessageBase msg);

    // Handling of internal messages for callbacks
    void register_callback(message_ids name, user_callback fn);
    void remove_callback(message_ids name, user_callback fn);
    void notify_callbacks(message_ids name);
    
    // Command method wrappers
    void arm();
    void disarm();
    void take_control();
    void release_control();
    void cmd_position( float north, float east, float altitude, float heading);
    void takeoff(float target_altitude);
    void land();
    void cmd_attitude( float roll, float pitch, float yaw, float thrust);
    void cmd_attitude_rate( float roll_rate, float pitch_rate, float yaw_rate, float thrust);
    void cmd_moment( float roll_moment, float pitch_moment, float yaw_moment, float thrust);
    void cmd_velocity( float velocity_north, float velocity_east, float velocity_down, float heading);
    void set_home_position( float longitude, float latitude, float altitude);
    void set_home_as_current_position();
    void start();
    void stop();
    MavlinkConnection * getConnection()
    {
        return m_conn;
    }
};
#endif /* MyDrone_hpp */
