#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "data_sender.h"

#define PORT 8001

int create_socket(const char *ip_addr);

const char IP_ADDR[] = "192.168.1.109";

void send(char *data){
    int sockfd = create_socket(IP_ADDR);
    if(sockfd != -1){
        if( send(sockfd, data, strlen(data), 0) < 0)
        {
            printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
        }

        close(sockfd);
    }
}


int create_socket(const char *ip_addr){
    int    sockfd;
    struct sockaddr_in    servaddr;


    if( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
        exit(0);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    if( inet_pton(AF_INET, ip_addr, &servaddr.sin_addr) <= 0){
        printf("inet_pton error for %s\n",ip_addr);
        return -1;
    }

    if( connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0){
        printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
        return -1;
    }
    return sockfd;
}