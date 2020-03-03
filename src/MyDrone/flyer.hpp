//
//  flyer.hpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/29.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#ifndef flyer_hpp
#define flyer_hpp

#include <queue>
using namespace std;
#include "my_drone.hpp"
#include "mavlink_connection.hpp"
#include "Utility/StringUtils.h"

enum States
{
    MANUAL = 0,
    ARMING = 1,
    TAKEOFF = 2,
    WAYPOINT = 3,
    LANDING = 4,
    DISARMING = 5
};

class Point
{
public:
    Point();
    Point(float x, float y, float z);
    float& operator[](int i);
    Point operator+(Point &b);
    void operator=(Point &p);
    void print();
private:
    float coordinate[3];
};

class Flyer: public MyDrone
{
private:
    queue<Point> all_waypoints;
    Point target_position;
    bool in_mission{true};
    vector<States> check_state;
    States flight_state{MANUAL};
public:
    Flyer(MavlinkConnection *conn);
    void local_position_callback();
    void velocity_callback();
    void state_callback();
    void calculate_box();
    void arming_transition();
    void takeoff_transition();
    void waypoint_transition();
    void landing_transition();
    void disarming_transition();
    void manual_transition();
    void start();
};
#endif /* flyer_hpp */
