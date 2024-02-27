#include "TCPServer.h"

TCPServer::TCPServer() : serverSock(-1), clientSock(-1) {}

TCPServer::~TCPServer() {
    close(serverSock);
    close(clientSock);
}

void TCPServer::connect(int port) {
    // Create socket
    serverSock = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSock == -1) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Set socket option to allow address reuse
    int opt = 1;
    if (setsockopt(serverSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        perror("Setsockopt failed");
        exit(EXIT_FAILURE);
    }

    // Server address setup
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);

    // Bind the socket to the port
    if (bind(serverSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        perror("Binding failed");
        exit(EXIT_FAILURE);
    }

    // Listen for incoming connections
    if (listen(serverSock, 5) == -1) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << port << std::endl;
}

void TCPServer::acceptConnection() {
    socklen_t clientAddrLen = sizeof(clientAddr);
    clientSock = accept(serverSock, (struct sockaddr *)&clientAddr, &clientAddrLen);
    if (clientSock == -1) {
        perror("Accept failed");
        exit(EXIT_FAILURE);
    }
    std::cout << "Connection accepted from " << inet_ntoa(clientAddr.sin_addr) << std::endl;
}

json TCPServer::receiveJSON() {
    constexpr int bufferSize = 1024;
    char buffer[bufferSize] = {0};
    
    // Receive data with MSG_DONTWAIT flag for non-blocking behavior
    int bytesRead = recv(clientSock, buffer, bufferSize, MSG_DONTWAIT);
    
    if (bytesRead == -1) {
        // Check if the error indicates that there's no data available to read
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // No data available to read
            return json(); // Return empty JSON object
        } else {
            // Other error occurred
            perror("Receive failed");
            exit(EXIT_FAILURE);
        }
    } else if (bytesRead == 0) {
        // Client disconnected
        std::cerr << "Client disconnected\n";
        exit(EXIT_FAILURE);
    } else {
        // Data received, parse and return JSON
        return json::parse(buffer);
    }
}


void TCPServer::closeConnection() {
    close(clientSock);
    clientSock = -1;
}

