#include "pke_charge_transfer.h"

TIM_HandleTypeDef * pke_ct_timer_instance;
volatile uint8_t pke_ct_internalstate = 0;
volatile uint32_t pke_ct_internal_chargeiterations;

GPIO_TypeDef * sense_port,* cap_port;
uint16_t sense_pin, cap_pin;

volatile int16_t pke_ct_last_sense_value = -1;

void (*pke_ct_sense_cplt_callback)(int16_t);

void pke_ct_timer_init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  pke_ct_timer_instance->Instance = TIM17;
  pke_ct_timer_instance->Init.Prescaler = 0;
  pke_ct_timer_instance->Init.CounterMode = TIM_COUNTERMODE_UP;
  pke_ct_timer_instance->Init.Period = 64;
  pke_ct_timer_instance->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  pke_ct_timer_instance->Init.RepetitionCounter = 0;
  pke_ct_timer_instance->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
	HAL_TIM_Base_Init(pke_ct_timer_instance);
  
  HAL_TIM_OC_Init(pke_ct_timer_instance);
    
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
	HAL_TIM_OC_ConfigChannel(pke_ct_timer_instance, &sConfigOC, TIM_CHANNEL_1);
	
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  
	HAL_TIMEx_ConfigBreakDeadTime(pke_ct_timer_instance, &sBreakDeadTimeConfig);
  
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

void pke_ct_configure_io_output(GPIO_TypeDef * port, uint16_t pin){

  /*Configure GPIO pin : LED1_Pin */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void pke_ct_configure_io_input(GPIO_TypeDef * port, uint16_t pin){

	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &GPIO_InitStruct);

}

void pke_ct_hal_tim_period_elapsed_callback(){
	
	switch(pke_ct_internalstate){
		case 0:
			pke_ct_configure_io_output(sense_port,sense_pin);
		  pke_ct_configure_io_output(cap_port,cap_pin);
			HAL_GPIO_WritePin(sense_port,sense_pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(cap_port,cap_pin,GPIO_PIN_RESET);
			pke_ct_internalstate++;
		break;
		
		case 1:
			pke_ct_configure_io_input(sense_port,sense_pin);
			pke_ct_internalstate++;
			pke_ct_internal_chargeiterations = 0;
		break;
		
		case 2:
			HAL_GPIO_WritePin(sense_port,sense_pin,GPIO_PIN_SET);
			pke_ct_internalstate++;
		break;
		
		case 3:
			pke_ct_configure_io_input(sense_port,sense_pin);
			HAL_GPIO_WritePin(cap_port,cap_pin,GPIO_PIN_RESET);
			pke_ct_configure_io_output(cap_port,cap_pin);
			pke_ct_internalstate++;
		break;
			
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			pke_ct_internalstate++;
		break;
			
		case 9:
			if(HAL_GPIO_ReadPin(sense_port,sense_pin)){
				pke_ct_internalstate++;
			}else if (pke_ct_internal_chargeiterations > PKE_CT_CHARGEITERATIONS_THRESHOLD){
				pke_ct_internalstate+=2;
			}else{
				pke_ct_internal_chargeiterations++;
				pke_ct_internalstate = 2;
				pke_ct_configure_io_input(cap_port,cap_pin);
			}
			
		break;
		
		case 10: //finished w. value
			HAL_TIM_Base_Stop_IT(pke_ct_timer_instance);
			pke_ct_internalstate = 0;
			pke_ct_last_sense_value = pke_ct_internal_chargeiterations;
			if(pke_ct_sense_cplt_callback != NULL){
				pke_ct_sense_cplt_callback(pke_ct_last_sense_value);
			}
		break;
			
		case 11: //finished timeout
			HAL_TIM_Base_Stop_IT(pke_ct_timer_instance);
			pke_ct_internalstate = 0;
			pke_ct_last_sense_value = -1;
			if(pke_ct_sense_cplt_callback != NULL){
				pke_ct_sense_cplt_callback(pke_ct_last_sense_value);
			}
		break;
			
		default:
			pke_ct_internalstate = 0;
		break;
			
	}
	
	/*
	pinMode(cap,OUTPUT);
  pinMode(sense,OUTPUT);

  digitalWrite(cap,LOW);
  digitalWrite(sense,LOW);
  pinMode(cap,INPUT); //wichtig no-pullup
  uint16_t touch_cnt;
	
  for( touch_cnt = 0 ; touch_cnt < 1000 ; touch_cnt++){
		case2
    digitalWrite(sense,HIGH);
7
	delayMicroseconds(1);
		case 3

	pinMode(sense,INPUT);
    digitalWrite(cap,LOW);
    pinMode(cap,OUTPUT);
	
    delayMicroseconds(5);

case 4

    if(digitalRead(sense)){
      break;
    }
    pinMode(cap,INPUT);
  }
	*/
	

}

void pke_charge_transfer_init(TIM_HandleTypeDef * timer_instance, GPIO_TypeDef * s1_port, uint16_t s1_pin, GPIO_TypeDef * s2_port, uint16_t s2_pin ){
	
	pke_ct_timer_instance = timer_instance;
	
	pke_ct_timer_init();
	
	sense_port = s1_port;
	cap_port = s2_port;
	
	sense_pin = s1_pin;
	cap_pin = s2_pin;
	
}

void pke_ct_measure(void (*callback)(int16_t)){
	pke_ct_sense_cplt_callback = callback;
	
	HAL_TIM_Base_Start_IT(pke_ct_timer_instance);
}

