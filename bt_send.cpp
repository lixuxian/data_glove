#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringSerial.h>
#include <string>
#include "bt_send.h"
using namespace std;
// mpu_num-->gear_num: 9-->4, 0-->5, 1-->6, 3-->17, 5-->18, 7-->19
// int mpu_num[6] = {9, 0, 1, 3, 5, 7};
// int gear_num[6] = {4, 5, 6, 17, 18, 19};

int send_bt(char *data){
	char* instruction = data_process(data);
        printf("instruction: %s\n", instruction);
        int fd;
        if ((fd=serialOpen("/dev/rfcomm0", 115200)) < 0) {
                printf("serial doesn't open");
                return 1;
        }
        serialPrintf(fd, instruction);
        // serialClose(fd);
	return 0;
}

char inst[130];

char* data_process(char data[]) {
        // data = "0#1,2,3;1#4,5,6;3#6,3,2;"
        // instruction = "004P1000T300!005P1000T300!"
        char *ptr[10], *ptr2, *ptr3[3];
        char *p, *p2;
        //char instruction[130];

        int angles[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        int gears[6] = { 4, 5, 6, 17, 18, 19 };
        int mpu[6] = { 9, 0, 1, 3, 5, 7 };
        int rotate_angles[6] = { 0, 0, 0, 0, 0, 0 };
        string instruction = "";

        ptr[0] = strtok_r(data, ";", &p);
        ptr2 = strtok_r(ptr[0], "#", &p2);
        for (int i = 0; i < 3; i++) {
                ptr2 = strtok_r(NULL, ",", &p2);
                // printf("angles:%s\n", ptr2);
        }
        for (int i = 1; i < 10; i++) {
                ptr[i] = strtok_r(NULL, ";", &p);
                // printf("ptr=%s\n", ptr[i]);

                ptr2 = strtok_r(ptr[i], "#", &p2);
                // printf("channel num:%s\n", ptr2);
                for (int j = 0; j < 3; j++) {
                        ptr3[j] = strtok_r(NULL, ",", &p2);
                        // printf("angels:%s\n", ptr3[j]);
                }

                if (atoi(ptr2) == 0 || atoi(ptr2) == 9) {
                        // char *instr = single_instrcution(ptr2, ptr3[1]);
                        angles[atoi(ptr2)] = atoi(ptr3[1]);
                }
                else {
                        // char *instr = single_instrcution(ptr2, ptr3[2]);
                        angles[atoi(ptr2)] = atoi(ptr3[2]);
                }
        }

        for (int i = 0; i < 6; i++) {
                if (gears[i] == 4) {
                        rotate_angles[i] = 90 - angles[mpu[i]];
                }
                else if (gears[i] == 5) {
                        rotate_angles[i] = -angles[mpu[i]];
                }
                else if (gears[i] == 6 || gears[i] == 17) {
                        rotate_angles[i] = angles[mpu[i]] - angles[mpu[i] + 1];
                }
                else {
                        rotate_angles[i] = 180 - (angles[mpu[i]] - angles[mpu[i] + 1]);
                }
        }
        int pwm_ms;
        char c[3];
        char pwm[8];
        for (int i = 0; i < 6; i++) {
                // printf("rotate_angles: %d\n", rotate_angles[i]);
                if (rotate_angles[i] >= 0 && rotate_angles[i] <= 180) {
                        if (gears[i] == 18 || gears[i] == 19) {
                                pwm_ms = int(rotate_angles[i] / 180.0 * 2000) + 100;
                        }
                        else {
                                pwm_ms = int(rotate_angles[i] / 180.0 * 2000) + 500;
                        }
                        // cout << "pwm_ms: " << pwm_ms << endl;
                        //itoa(pwm_ms, pwm, 10);
                        sprintf(pwm, "%d", pwm_ms);
                        string str1;
                        string str2;
                        if (gears[i] < 10) {
                                //itoa(gears[i], c, 10);
                                sprintf(c, "%d", gears[i]);
                                // cout << "c<10: " << c << endl;
                                str1 = c;
                                str2 = pwm;
                                instruction += "#00" + str1 + "P" + str2 + "T100!";
                        }
                        else {
                                // itoa(gears[i], c, 10);
                                sprintf(c, "%d", gears[i]);
                                // cout << "c>10: " << c << endl;
                                str1 = c;
                                str2 = pwm;
                                instruction += "#0" + str1 + "P" + str2 + "T100!";
                        }
                }
        }
        // cout << "instruction:" << instruction << endl;
        strcpy(inst, instruction.c_str());
        return inst;
}
