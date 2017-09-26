#ifndef _TCA9548_H_
#define _TCA9548_H_

#include <stdint.h>

void init_channel();
void select_channel(uint8_t channel);
void select_all_channel();
void disable_all_chanel();


#endif