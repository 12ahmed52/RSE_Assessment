#include "TCPClient.h"
#include "json.hpp"
using json = nlohmann::json;
TCPClient::TCPClient() {
    // Create socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
}

void TCPClient::connectToServer(const char* ip, int port) {
    // Server address setup
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    inet_aton(ip, &serverAddr.sin_addr);

    // Connect to the server
    if (connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        perror("Connection failed");
        exit(EXIT_FAILURE);
    }
}

void TCPClient::sendData_Velocity(float linear, float angular) {
    // Create a JSON object with two floats
    json data;
    data["action"] = "Velocity";
    data["linear"] = linear;
    data["angular"] = angular;

    // Convert the JSON object to a string
    std::string jsonData = data.dump();

    // Send the JSON string to the server
    if (send(sockfd, jsonData.c_str(), jsonData.length(), 0) == -1) {
        perror("Send failed");
        exit(EXIT_FAILURE);
    }
}

void TCPClient::sendData_Navigation(float x, float y, bool nav_state) {
    // Create a JSON object with two floats
    json data;
    data["action"] = "Navigation";
    data["x"] = x;
    data["y"] = y;
    data["nav_state"] = nav_state;

    // Convert the JSON object to a string
    std::string jsonData = data.dump();

    // Send the JSON string to the server
    if (send(sockfd, jsonData.c_str(), jsonData.length(), 0) == -1) {
        perror("Send failed");
        exit(EXIT_FAILURE);
    }
}

std::string TCPClient::receiveData(int bufferSize) {
    char buffer[bufferSize] = {0};
    int bytesRead = recv(sockfd, buffer, bufferSize, 0);
    if (bytesRead == -1) {
        perror("Receive failed");
        exit(EXIT_FAILURE);
    } else if (bytesRead == 0) {
        std::cerr << "Connection closed by server\n";
        exit(EXIT_FAILURE);
    } else {
        return std::string(buffer);
    }
}

void TCPClient::closeConnection() {
    close(sockfd);
}
