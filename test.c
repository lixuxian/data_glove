#include <stdio.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <stdint.h>
#include "I2Cdev/I2Cdev.h"

#define i2c_write   writeBytes
#define i2c_read(a,b,c,d)    (readBytes(a,b,c,d)!=-1?0:1)

#define CHIP_0_PIN  3
#define CHIP_1_PIN  4
#define CHIP_0_ADDR  0x70
#define CHIP_1_ADDR  0x77

void init_channel(){
    if(-1==wiringPiSetup()){
        printf("wiring pi setup error\n");
        exit(-1);
    }

    pinMode(CHIP_0_PIN, OUTPUT);
    pinMode(CHIP_1_PIN, OUTPUT);

    digitalWrite(CHIP_0_PIN, LOW);
    digitalWrite(CHIP_1_PIN, LOW);
}

static uint8_t tmp = 0;

void select_channel(uint8_t channel){
    int cc = channel & 7;
    printf("cc=%d\n",cc );
    
    int chip = channel >> 3;
    printf("chip=%d\n",chip );
    if(chip == 0){
        digitalWrite(CHIP_0_PIN, HIGH);
        digitalWrite(CHIP_1_PIN, LOW);
        i2c_write(CHIP_0_ADDR, 1<<cc, 0 , &tmp); //select channel
    }else{
        digitalWrite(CHIP_0_PIN, LOW);
        digitalWrite(CHIP_1_PIN, HIGH);
        i2c_write(CHIP_1_ADDR, 1<<cc, 0 , &tmp); //select channel
    }
    
}

int main(int argc, char const *argv[])
{
    init_channel();

    //select_channel(8);
    return 0;
}