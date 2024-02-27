#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "json.hpp"
#include <fcntl.h> // Include this header for fcntl
#include <errno.h> // Include this header for errno

using json = nlohmann::json;

class TCPServer {
private:
    int serverSock;
    int clientSock;
    struct sockaddr_in serverAddr;
    struct sockaddr_in clientAddr;

public:
    TCPServer();
    ~TCPServer();
    void connect(int port);
    void acceptConnection();
    json receiveJSON();
    void closeConnection();
};

#endif /* TCP_SERVER_H */

