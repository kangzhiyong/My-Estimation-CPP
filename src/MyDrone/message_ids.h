//
//  message_ids.h
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/25.
//

#ifndef message_ids_h
#define message_ids_h

enum message_ids
{
    ANY,
    STATE,
    GLOBAL_POSITION,
    LOCAL_POSITION,
    GLOBAL_HOME,
    LOCAL_VELOCITY,
    CONNECTION_CLOSED,
    RAW_GYROSCOPE,
    RAW_ACCELEROMETER,
    BAROMETER,
    DISTANCE_SENSOR,
    ATTITUDE
};

#endif /* message_ids_h */
