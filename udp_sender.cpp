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
#include <signal.h>	
#include "data_sender.h"

#define PORT 8001

int create_socket(const char *ip_addr);

const char IP_ADDR[] = "192.168.1.107";

int udp_sender_status = create_socket(IP_ADDR);

int g_sockfd;
struct sockaddr_in  servaddr;

void send(char *data){
//    printf("%s\n", data);
     if(g_sockfd != -1){
         if( sendto(g_sockfd, data, strlen(data), 0, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
         {
             printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
         }
     }
}

void close_connection(int sig_num){
    close(g_sockfd);
    exit(0);
}
int create_socket(const char *ip_addr){
    
    if( (g_sockfd = socket(AF_INET,SOCK_DGRAM,0)) < 0){
        printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
        exit(0);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=inet_addr(ip_addr);
    servaddr.sin_port = htons(PORT);

    if (servaddr.sin_addr.s_addr == INADDR_NONE)
    {
        printf("Incorrect ip address!");
        close(g_sockfd);
        exit(1);
    }
    //
    signal(SIGINT, close_connection);

    return 0;
}

