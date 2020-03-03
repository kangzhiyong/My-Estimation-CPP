//
//  flyer.cpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/29.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#include "flyer.hpp"

#include <iostream>

Point::Point()
{
    memset(coordinate, 0, sizeof(float) * 3);
}
Point::Point(float x, float y, float z)
{
    coordinate[0] = x;
    coordinate[1] = y;
    coordinate[2] = z;
}

float& Point::operator[](int i)
{
    if( i > 3 )
    {
        cout << "索引超过最大值" << endl;
        // 返回第一个元素
        return coordinate[0];
    }
    return coordinate[i];
}

Point Point::operator+(Point &b)
{
    Point p;
    p[0] = coordinate[0] + b[0];
    p[1] = coordinate[1] + b[1];
    p[2] = coordinate[2] + b[2];
    return p;
}

void Point::operator=(Point &p)
{
    coordinate[0] = p[0];
    coordinate[1] = p[1];
    coordinate[2] = p[2];
}

void Point::print()
{
   cout << "(x, y, z):(" << coordinate[0] << ", " << coordinate[1] << ", " << coordinate[2] << ")" << endl;
}

Flyer::Flyer(MavlinkConnection *conn): MyDrone(conn)
{
    check_state.clear();

    // register all your callbacks here
    register_callback(LOCAL_POSITION, ((void (MyDrone::*)())&Flyer::local_position_callback));
    register_callback(LOCAL_VELOCITY, ((void (MyDrone::*)())&Flyer::velocity_callback));
    register_callback(STATE, ((void (MyDrone::*)())&Flyer::state_callback));
}

void Flyer::local_position_callback()
{
    if (flight_state == TAKEOFF)
    {
        if (-1.0 * local_position()[2] > 0.95 * target_position[2])
        {
            calculate_box();
            waypoint_transition();
        }
    else if (flight_state == WAYPOINT)
    {
        if (SLR::norm(target_position[0], target_position[1], target_position[2],
                      local_position()[0], local_position()[1], local_position()[2]) < 1.0)
        {
            if (all_waypoints.size() > 0)
            {
                waypoint_transition();
            }
            else
            {
                if (SLR::norm(local_velocity()[0], local_velocity()[1], local_velocity()[2], 0, 0, 0) < 1.0)
                {
                    landing_transition();
                }
            }
        }
    }
    }
}

void Flyer::velocity_callback()
{
    
}

void Flyer::state_callback()
{
    if (in_mission)
    {
        if (flight_state == MANUAL)
        {
            if (guided())
            {
                flight_state = ARMING;
            }
        }
        else if (flight_state == ARMING)
        {
            if (armed())
            {
                takeoff_transition();
            }
        }
        else if (flight_state == LANDING)
        {
            if (!armed() && !guided())
            {
                stop();
                in_mission = false;
            }
        }
        else if (flight_state == DISARMING)
        {
            
        }
    }
}

void Flyer::calculate_box()
{
    cout << "Setting Home\r\n" << endl;
    Point cp(local_position()[0], local_position()[1], -local_position()[2]);
    Point p1(10.0, 0.0, 3.0), p2(10.0, 10.0, 3.0), p3(0.0, 10.0, 3.0), p4(0.0, 0.0, 3.0);
    all_waypoints.push(cp + p1);
    all_waypoints.push(cp + p2);
    all_waypoints.push(cp + p3);
    all_waypoints.push(cp + p4);
}

void Flyer::arming_transition()
{
    cout << "arming transition\r\n" << endl;
    take_control();
    arm();
    set_home_position(global_position()[0], global_position()[1], global_position()[2]);  // set the current location to be the home position
    flight_state = ARMING;
}

void Flyer::takeoff_transition()
{
    cout << "takeoff transition\r\n" << endl;
    float target_altitude = 3.0;
    target_position[2] = target_altitude;
    takeoff(target_altitude);
    flight_state = TAKEOFF;
}

void Flyer::waypoint_transition()
{
    cout << "waypoint transition" << endl;
    target_position = all_waypoints.front();
    all_waypoints.pop();
    target_position.print();
    cmd_position(target_position[0], target_position[1], target_position[2], 0.0);
    flight_state = WAYPOINT;
}

void Flyer::landing_transition()
{
    cout << "landing transition" << endl;
    land();
    flight_state = LANDING;
}

void Flyer::disarming_transition()
{
    cout << "disarm transition" << endl;
    disarm();
    release_control();
    flight_state = DISARMING;
}

void Flyer::manual_transition()
{
    cout << "manual transition" << endl;
    stop();
    in_mission = false;
    flight_state = MANUAL;
}

void Flyer::start()
{
    cout << "starting connection" << endl;
    MyDrone::start();
}
