#include "cosmos.h"
#include <csignal>
#include <cstdio>
#include <cerrno>
#include <unistd.h>
#include <cstdlib>
#include <string.h>

Cosmos::Cosmos(int portno) {
    port = portno;

    // ignore sigpipe signal - don't stop program when writing to closed socket
    // (it will be handled instead)
    signal(SIGPIPE, SIG_IGN);
}

Cosmos::~Cosmos() {
    cosmosDisconnect();
}

void Cosmos::cosmosConnect() {
    // open socket
    bindSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (bindSocket<0) {
        perror("ERROR opening socket");
        exit(1);
    }

    // clear and set up server address structure
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);

    // bind socket
    if (bind(bindSocket, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        perror("ERROR on binding");
        exit(1);
    }

    // listen on socket
    listen(bindSocket, 1);
    printf("listening on port %d\n", port);

    // accept a connection
    clilen = sizeof(cli_addr);
    connectionSocket = accept(bindSocket, (struct sockaddr *) &cli_addr, &clilen);
    if (connectionSocket<0) {
        perror("ERROR on accept");
        exit(1);
    }

    printf("accepted connection from %s:%d\n", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));
}

void Cosmos::cosmosDisconnect() {
    close(bindSocket);
    close(connectionSocket);
}

void Cosmos::sendPacket(char* buffer, int size) {
    if (send(connectionSocket, buffer, size, 0) < 0) {
        perror("ERROR on send");
        cosmosDisconnect();
        exit(1);
    }
}