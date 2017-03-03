//
// Created by root on 3/2/17.
//

#ifndef JDROBOT_SOCKETCLASS_H
#define JDROBOT_SOCKETCLASS_H
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>

#define MYPORT  6000
#define BUFFER_SIZE 1024

class SocketClass {
private:
    char ip[14];

    ///定义sockfd
    int sock_cli = socket(AF_INET,SOCK_STREAM, 0);

    ///定义sockaddr_in
    struct sockaddr_in servaddr;
public:
//    SocketClass(char* tIp);
    SocketClass(){
//        ip="192.168.1.125";
    }
    bool connetServer(void);

    bool sendMsg(unsigned char* frame);
};


#endif //JDROBOT_SOCKETCLASS_H
