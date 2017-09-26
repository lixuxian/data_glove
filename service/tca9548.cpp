#include <stdlib.h>
#include <stdio.h>
#include "tca9548.h"
#include "I2Cdev/I2Cdev.h"

#define i2c_write   writeBytes
#define i2c_read(a,b,c,d)    (readBytes(a,b,c,d)!=-1?0:1)
#define CHIP_0_PIN  3
#define CHIP_1_PIN  4
#define CHIP_0_ADDR  0x70
#define CHIP_1_ADDR  0x77

static uint8_t tmp = 0x00;

void init_channel(){
    disable_all_chanel();
}

void select_channel(uint8_t channel){
    int cc = channel & 7;
    
    int chip = channel >> 3;
    if(chip == 0){
        i2c_write(CHIP_1_ADDR, 0, 0 , &tmp);
        i2c_write(CHIP_0_ADDR, 1<<cc, 0 , &tmp); //select channel
    }else{
        i2c_write(CHIP_0_ADDR, 0, 0 , &tmp);
        i2c_write(CHIP_1_ADDR, 1<<cc, 0 , &tmp); //select channel
    }
    
}
void select_all_channel(){
    i2c_write(CHIP_0_ADDR, 0xff, 0 , &tmp); //select channel
    i2c_write(CHIP_1_ADDR, 0xff, 0 , &tmp); //select channel
}

void disable_all_chanel(){
    i2c_write(CHIP_0_ADDR, 0, 0 , &tmp); //select channel
    i2c_write(CHIP_1_ADDR, 0, 0 , &tmp); //select channel
}