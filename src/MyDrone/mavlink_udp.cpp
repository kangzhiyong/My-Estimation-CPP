//
//  mavlink_udp.cpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/3/1.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#include <sys/errno.h>
#include "mavlink_udp.hpp"
#include "StringUtils.h"

mavudp::mavudp(std::string destAddr, bool input, bool broadcast, int source_system, int source_component, bool use_native)
{
    int val = 1;
    std::vector<std::string> ip_Port = SLR::Split(destAddr, ':');
    if (ip_Port.size() != 2)
    {
        printf("UDP ports must be specified as host:port, now set to %s:%d\r\n", DEST_UDP_IP, DEST_UDP_PORT);
    }
    else
    {
        destination_addr = ip_Port[0];
        destination_port = atoi(ip_Port[1].c_str());
    }
    _socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&val, sizeof(val));
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr("224.0.0.1");
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
    {
        perror("setsockopt MEMBERSHIP");
        close();
        return;
    }
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));  // Zero out address structure
    addr.sin_family = PF_INET;       // Internet address
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(LOCAL_UDP_PORT);     // Assign port in network byte order

    if (::bind(_socket, (sockaddr *) &addr, sizeof(sockaddr_in)) < 0)
    {
        perror("bind:");
        close();
        return;
    }
    printf("succe\r\n");

    int flags = fcntl(_socket, F_GETFD);
    flags |= FD_CLOEXEC;
    fcntl(_socket, F_SETFD, flags);

    flags = fcntl(_socket, F_GETFL);
    flags |= O_NONBLOCK;
    fcntl(_socket, F_SETFL, flags);

    last_addr = "";
    last_port = 0;
    resolved_destination_addr = "";
    resolved_destination_port = 0;
    _connected = true;
}

void mavudp::close()
{
    ::close(_socket);
}

void mavudp::fillAddr(const std::string &address, unsigned short port, sockaddr_in &addr)
{
  memset(&addr, 0, sizeof(addr));  // Zero out address structure
  addr.sin_family = PF_INET;       // Internet address

//  hostent *host;  // Resolve name
//  if ((host = gethostbyname(address.c_str())) == NULL) {
//    // strerror() will not work for gethostbyname() and hstrerror()
//    // is supposedly obsolete
//    perror("Failed to resolve name (gethostbyname())");
//  }
  addr.sin_addr.s_addr = inet_addr(address.c_str());

  addr.sin_port = htons(port);     // Assign port in network byte order
}

int mavudp::recv(void *buffer, int bufferLen)
{
    sockaddr_in clntAddr;
    socklen_t addrLen = sizeof(clntAddr);
    ssize_t rtn;
    if ((rtn = ::recvfrom(_socket, buffer, bufferLen, 0, (struct sockaddr *)&clntAddr, (socklen_t *) &addrLen)) < 0) {
        if (errno != EAGAIN)
        {
            perror("Receive failed (recvfrom()): ");
            return -1;
        }
    }
//    destination_addr = inet_ntoa(clntAddr.sin_addr);
//    destination_port = ntohs(clntAddr.sin_port);
    return (int)rtn;
}

void mavudp::write(const void *buffer, int bufferLen)
{
    sockaddr_in destAddr;

    if (udp_server)
    {
        if (last_addr != "" && last_port != 0) {
            fillAddr(last_addr, last_port, destAddr);
            if (::sendto(_socket, buffer, bufferLen, 0, (struct sockaddr *)&destAddr, sizeof(destAddr)) != bufferLen)
            {
                perror("Send failed (sendto()): ");
            }
        }
    }
    else
    {
        if (last_addr != "" && _broadcast)
        {
            destination_addr = last_addr;
            destination_port = last_port;
            _broadcast = false;
            fillAddr(destination_addr, destination_port, destAddr);
            ::connect(_socket, (struct sockaddr *)&destAddr, sizeof(destAddr));
        }
        if (!destination_addr.empty() && destination_port > 0) {
            fillAddr(destination_addr, destination_port, destAddr);
            // Write out the whole buffer as a single message.
            if (sendto(_socket, buffer, bufferLen, 0, (sockaddr *) &destAddr, sizeof(destAddr)) != bufferLen) {
                perror("Send failed (sendto()): ");
            }
        }
    }
}

bool mavudp::recv_msg(mavlink_message_t *msg)
{
    //message receive routine for UDP link
    unsigned char buf[MAX_UDP_PACKET_SIZE];
    memset(buf, 0, MAX_UDP_PACKET_SIZE);
    int n = recv(buf, MAX_UDP_PACKET_SIZE);
    if (n > 0)
    {
        mavlink_status_t status;
        for (unsigned int i = 0; i < n; ++i)
        {
          if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], msg, &status))
          {
              // Packet received
              //printf("Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
              return true;
          }
        }
    }
    return false;
}

bool mavudp::recv_match(void *msg, bool blocking, int timeout)
{
    /*
    recv the next MAVLink message that matches the given condition
    type can be a string or a list of strings
     */
    time_t start_time = time(nullptr);
    while(1)
    {
//        if (timeout != 0)
//        {
//            time_t now = time(nullptr);
//            if (now < start_time)
//            {
//                start_time = now; // If an external process rolls back system time, we should not spin forever.
//            }
//            if (start_time + timeout < time(nullptr))
//            {
//                return false;
//            }
//        }
        if (!recv_msg((mavlink_message_t *)msg))
        {
            if (blocking)
            {
                sleep((unsigned int)timeout);
                continue;
            }
            return false;
        }
        else
        {
            return true;
        }
    }
    return false;
}

int mavudp::get_socket_fd()
{
    return _socket;
}
