//
//  mavlink_utils.hpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/25.
//

#ifndef mavlink_utils_hpp
#define mavlink_utils_hpp

enum ConnectionType
{
    /*
    Different possible connection types.
    NOTE: Right now the only implemented type is PX4 mavlink
    */
    MAVLINK_PX4 = 1,
    // MAVLINK_APM = 2,
    // PARROT = 3,
    // DJI = 4
};

enum MainMode
{
    //Constant which isn't defined in Mavlink but is useful for PX4
    PX4_MODE_MANUAL = 1,
    PX4_MODE_OFFBOARD = 6
};

enum PlaneMode
{
    /*
     Constant which isn't defined in Mavlink but useful when dealing with
    the airplane simulation
     */
    SUB_MODE_LONGITUDE = 1,
    SUB_MODE_LATERAL = 2,
    SUB_MODE_STABILIZED = 3
};

enum PositionMask
{
    //Useful masks for sending commands used in set_position_target_local_ned
    MASK_IGNORE_POSITION = 0x007,
    MASK_IGNORE_VELOCITY = 0x038,
    MASK_IGNORE_ACCELERATION = 0x1C0,
    MASK_IGNORE_YAW = 0x400,
    MASK_IGNORE_YAW_RATE = 0x800,
    MASK_IS_FORCE = (1 << 9),
    MASK_IS_TAKEOFF = 0x1000,
    MASK_IS_LAND = 0x2000,
    MASK_IS_LOITER = 0x3000
};

enum AttitudeMask
{
    MASK_IGNORE_ATTITUDE = 0b10000000,
    MASK_IGNORE_RATES = 0b00000111
};

#endif /* mavlink_utils_hpp */
