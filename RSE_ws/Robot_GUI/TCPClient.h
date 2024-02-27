#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

class TCPClient {
private:
    int sockfd;
    struct sockaddr_in serverAddr;

public:
    TCPClient();
    void connectToServer(const char* ip, int port);
    void sendData_Velocity(float linear, float angular);
    void sendData_Navigation(float x, float y, bool nav_state);
    std::string receiveData(int bufferSize = 1024);
    void closeConnection();
};

#endif // TCPCLIENT_H
