#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <string>
typedef std::string String;

typedef int SOCKET;
typedef int UDPSOCKET;
typedef uint32_t IPADDRESS; // On linux use uint32_t in network byte order (per getpeername)
typedef uint16_t IPPORT; // on linux use network byte order

#define NULLSOCKET 0

inline void tcpsocketclose(SOCKET s)
{
    close(s);
}

#define getRandom() rand()

inline void socketpeeraddr(SOCKET s, IPADDRESS *addr, IPPORT *port)
{

    sockaddr_in r;
    socklen_t len = sizeof(r);

    if (getpeername(s, (struct sockaddr *)&r, &len) < 0)
    {
        ERROR_PRINT("getpeername failed\n");
        *addr = 0;
        *port = 0;
    }
    else
    {
        *port  = r.sin_port;
        *addr = r.sin_addr.s_addr;
    }
}

inline void udpsocketclose(UDPSOCKET s)
{
    close(s);
}

inline UDPSOCKET udpsocketcreate(unsigned short portNum)
{
    sockaddr_in addr;

    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(portNum);

    int s = socket(AF_INET, SOCK_DGRAM, 0);

    if (bind(s, (sockaddr *)&addr, sizeof(addr)) != 0)
    {
        ERROR_PRINT("Error, can't bind\n");
        close(s);
        s = 0;
    }

    return s;
}

// TCP sending
inline ssize_t tcpsocketsend(SOCKET sockfd, const void *buf, size_t len)
{
    DEBUG_PRINT("TCP send\n");
    return send(sockfd, buf, len, 0);
}

inline ssize_t udpsocketsend(UDPSOCKET sockfd, const void *buf, size_t len,
                             IPADDRESS destaddr, uint16_t destport)
{
    sockaddr_in addr;
    int status;

    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = destaddr;
    addr.sin_port = htons(destport);
    DEBUG_PRINT("UDP send to 0x%0x:%0x\n", destaddr, destport);

    status = sendto(sockfd, buf, len, 0, (sockaddr *) &addr, sizeof(addr));

    if (status == -1)
    {
        //might be out of buffers, let lwip send some stuff and try again
        vTaskDelay(1);
        status = sendto(sockfd, buf, len, 0, (sockaddr *) &addr, sizeof(addr));
    }

    return status;
}

/**
   Read from a socket with a timeout.

   Return 0=socket was closed by client, -1=timeout, >0 number of bytes read
 */
inline int tcpsocketread(SOCKET sock, char *buf, size_t buflen, int timeoutmsec)
{
    // Use a timeout on our socket read to instead serve frames
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeoutmsec * 1000; // send a new frame ever

    //original implimentation tried to use SO_RCVTIMEO, which will block on esp-idf if the timeout is zero!

    int retval;
    fd_set read_push_set;
    FD_ZERO(&read_push_set);
    FD_SET(sock, &read_push_set);

    retval = select(FD_SETSIZE, &read_push_set, NULL, NULL, &tv);

    if (retval > 0)
    {
        int res = recv(sock, buf, buflen, 0);
        if (res > 0)
        {
            return res;
        }
        else if (res == 0)
        {
            return 0; // client dropped connection
        }
        else
        {
            if (errno == EWOULDBLOCK || errno == EAGAIN)
                return -1;
            else
                return 0; // unknown error, just claim client dropped it
        }
    }
    else if (retval == -1)
    {
        return 0; //something broke
    }
    else
    {
        //timeout
        return -1;
    }
}
