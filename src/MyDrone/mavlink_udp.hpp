//
//  mavlink_udp.hpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/3/1.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#ifndef mavlink_udp_hpp
#define mavlink_udp_hpp

#include <time.h>
#include <string>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/fcntl.h>

#include "mavlink/common/mavlink.h"

#define MAX_UDP_PACKET_SIZE 65467 // UDP protocol max message size
#define LOCAL_UDP_PORT  14550
#define DEST_UDP_PORT 14555
#define DEST_UDP_IP "192.168.4.1"
class mavudp
{
    //a UDP mavlink socket
public:
    mavudp(std::string destAddr, bool input=true, bool broadcast=false, int source_system=255, int source_component=0, bool use_native=true);
    void close();
    void fillAddr(const std::string &address, unsigned short port, sockaddr_in &addr);
    int recv(void *buffer, int bufferLen);
    void write(const void *buffer, int bufferLen);
    bool recv_msg(mavlink_message_t *msg);
    bool recv_match(void *msg, bool blocking=false, int timeout=0);
    int get_socket_fd();
    bool connected()
    {
        return _connected;
    }
private:
    int _socket{-1};
    bool udp_server{false};
    bool _broadcast{false};
    std::string destination_addr{DEST_UDP_IP};
    int destination_port{DEST_UDP_PORT};
    std::string last_addr{""};
    int last_port{-1};
    std::string resolved_destination_addr{""};
    int resolved_destination_port{-1};
    bool _connected{false};
};

#endif /* mavlink_udp_hpp */
