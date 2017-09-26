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
#include <wiringPi.h>

#include "tca9548.h"
#include "mpu9250.h"
#include "imu.h"


#define delay_ms(a) usleep(a*1000)
#define PORT 8001
#define N_SENSORS 16


int current_channel;

uint8_t devAddr = 0x68;


SENSOR_DATA Gyrobufs[N_SENSORS];//陀螺仪
SENSOR_DATA Accbufs[N_SENSORS];//加速度
SENSOR_DATA Magbufs[N_SENSORS];//磁力计

IMU_DATA GyroFinals[N_SENSORS];
IMU_DATA AccFinals[N_SENSORS];
IMU_DATA MagFinals[N_SENSORS];


EULER_ANGLE YPRs[N_SENSORS];

////////////////////////////////////////////////////



static int create_socket(char*);
static void start_mpu9250();
static void getYawPitchRoll();

int main() {
     uint8_t sensors_channels[] = {6, 7};
     int n_sensors = 2;
     int curr_sensor = 0;
     int counter = 0;


        int sockfd, lSize;//for socket
        char sensor_data[512], ip_addr[128];//for sensor
        FILE *pFile = fopen("ip_addr.txt","r");
        fseek (pFile , 0 , SEEK_END);
        lSize = ftell (pFile)+1;
        rewind (pFile);
        fgets(ip_addr, lSize, pFile);
        printf("remote ip address %s\n",ip_addr);

    

        if(-1==wiringPiSetup()){
            printf("wiring pi setup error\n");
            exit(-1);
        }
        mpu9250_hello();

        imu_init();

        for(int i=0; i<n_sensors; i++){
            select_channel(sensors_channels[i]);
            start_mpu9250();
        }

   
        do{
            counter ++;
                sockfd = create_socket(ip_addr);
                if(sockfd != -1){
                    current_channel = sensors_channels[curr_sensor];
                    select_channel(current_channel);
                    getYawPitchRoll();
 
                    sprintf(sensor_data,"%d#%.2f,%.2f,%.2f",current_channel, YPRs[current_channel].yaw, YPRs[current_channel].pitch, YPRs[current_channel].roll);
                    if( send(sockfd, sensor_data, strlen(sensor_data), 0) < 0)
                    {
                        printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
                        break;
                    }

                    close(sockfd);
                    if(counter % 40 == 0){
                        printf("channel= %d : %.2f %.2f %.2f\n",current_channel, YPRs[current_channel].yaw, YPRs[current_channel].pitch, YPRs[current_channel].roll);
                        counter = 0;
                    }
                    
                    curr_sensor = (++curr_sensor) % n_sensors;
                    delay_ms(50);
                }else{
                    break;
                }

        }while(1);

        return 0;
}

static void start_mpu9250(){

    Init_MPU9250(devAddr);                     //初始化MPU9250 
    Gyro_Correct();//陀螺仪校正
    Acc_Correct();//加速度计校正
    //Gyro_Correct();//电子罗盘校正
}

static void getYawPitchRoll(){
    AHRS_Dataprepare();//准备数据
    AHRSupdate(GyroFinals[current_channel].X,GyroFinals[current_channel].Y,GyroFinals[current_channel].Z,
                    AccFinals[current_channel].X,AccFinals[current_channel].Y,AccFinals[current_channel].Z,
                    0.0,0.0,0.0);
}


static int create_socket(char *ip_addr){
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