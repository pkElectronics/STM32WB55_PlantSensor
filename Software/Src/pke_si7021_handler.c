#include "pke_si7021_handler.h"

I2C_HandleTypeDef * pke_si7021_i2c_instance;

uint8_t pke_si7021_internal_state = 0;
uint8_t pke_si7021_perform_measurement = 0;

GPIO_TypeDef * sensor_pwr_port;
uint16_t sensor_pwr_pin;

uint32_t timing_helper;

void pke_si7021_handler_init(I2C_HandleTypeDef * i2c_instance,GPIO_TypeDef * port, uint16_t pin){
	pke_si7021_i2c_instance = i2c_instance;
	sensor_pwr_port = port;
	sensor_pwr_pin = pin;
}	

void pke_si7021_cyclic(void){
	uint8_t data[16];
switch(pke_si7021_internal_state){
	case 0:
		if(pke_si7021_perform_measurement == 1){
			pke_si7021_internal_state++;
		}
	break;
		
	case 1:
		HAL_GPIO_WritePin(sensor_pwr_port,sensor_pwr_pin,GPIO_PIN_SET);
		pke_si7021_internal_state++;
		timing_helper = HAL_GetTick();
	break;
	
	case 2:
		if(HAL_GetTick() - timing_helper > 85UL){
			pke_si7021_internal_state++;
		}
	break;
		
	case 3:
		data[0] = SI7021_READ_RH_NOHOLD;
		HAL_I2C_Master_Transmit(pke_si7021_i2c_instance,SI7021_ADDR,data,1,0);
		timing_helper = HAL_GetTick();
		pke_si7021_internal_state++;
	break;
	
	case 4:
		if(HAL_GetTick() - timing_helper > 15UL){
			pke_si7021_internal_state++;
		}
	break;
	case 5:	
			HAL_I2C_Master_Receive(pke_si7021_i2c_instance,SI7021_ADDR,data,2,0);
			//RH parsen
			pke_si7021_internal_state++;
	break;
	
	case 6:
			data[0] = SI7021_READ_T_PREV;

			HAL_I2C_Master_Transmit(pke_si7021_i2c_instance,SI7021_ADDR,data,1,0);

			HAL_I2C_Master_Receive(pke_si7021_i2c_instance,SI7021_ADDR,data,2,0);
	
			//temp parsen
	
			pke_si7021_internal_state++;

	break;
		
	case 7:
		HAL_GPIO_WritePin(sensor_pwr_port,sensor_pwr_pin,GPIO_PIN_RESET);
		pke_si7021_perform_measurement = 0;
		pke_si7021_internal_state = 0;
	break;
	
	default:
		pke_si7021_internal_state = 0;
	break;
}


	
	
}