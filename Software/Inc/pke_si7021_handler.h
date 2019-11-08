#ifndef PKE_SI7021_HANDLER_H

#define PKE_SI7021_HANDLER_H

#include "includes.h"

#define SI7021_ADDR 0x40


enum{
	SI7021_READ_RH_HOLD = 0xE5,
	SI7021_READ_RH_NOHOLD = 0xF5,
	SI7021_READ_T_HOLD = 0xE3,
	SI7021_READ_T_NOHOLD = 0xF3,
	SI7021_READ_T_PREV = 0xE0,
	SI7021_RESET = 0xFE,
	SI7021_READ_UR = 0xE6,
	SI7021_WRITE_UR = 0xE7,
	SI7021_WRITE_HC = 0x51,
	SI7021_READ_HC = 0x11
	
};

void pke_si7021_handler_init(I2C_HandleTypeDef * i2c_instance,GPIO_TypeDef * port, uint16_t pin);


#endif