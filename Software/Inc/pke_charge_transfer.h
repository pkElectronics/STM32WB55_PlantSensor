#ifndef PKE_CHARGE_TRANSFER_H

#define PKE_CHARGE_TRANSFER_H


#include "includes.h"

void pke_ct_hal_tim_period_elapsed_callback(void);
void pke_charge_transfer_init(TIM_HandleTypeDef * timer_instance, GPIO_TypeDef * s1_port, uint16_t s1_pin, GPIO_TypeDef * s2_port, uint16_t s2_pin );
void pke_ct_measure(void (*callback)(int16_t));


#define PKE_CT_CHARGEITERATIONS_THRESHOLD 1000

#endif