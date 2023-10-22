/*
 * gpio.h
 *
 *  Created on: 13-Jul-2021
 *      Author: THE LOGICBOX
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_


/* Digital Status Read Function Prototypes-----------------*/
static int Read_DI_IN1(void);
static int Read_DI_IN2(void);
static int Read_DI_IN3(void);
static int Read_DI_MAINS_STATE(void);
static int Read_DI_ACC_STATE(void);
static char Read_DI_BOX_STATE(void);
static int Read_DI_SOS_STATE(void);
//static int Read_DI_INT1_ACCEL(void);
/* Digital Status Read Function Prototypes-----------------*/

/* Variable for ADC */
uint32_t ADCvalue[4];

float VSENSE=.000805664;
//float VSENSE=.00322266;
/* Variable for ADC */

/* Analog Value Read Function Prototypes-----------------*/
static void Init_ADC(void);
static float Read_ADC1(void);
static float Read_ADC2(void);
static float Read_EXT_B_SENSE(void);
static float Read_INT_B_SENSE(void);
/* Analog Value Read Function Prototypes-----------------*/


/* Digital Output Function Prototypes-----------------*/
static void SET_OUT3_P_LED(int);
static void SET_5V_OUT_EN(int);
static void SET_OUT2(int);
static void SET_OUT1(int);
static void SET_LED_GPS(int);
static void SET_LED_NET(int);
static void SET_LED_PWR(int);
static void SET_GPS_VCC_EN(int);
static void SET_PWRKEY(int);
static void SET_GSM_VCC_EN(int);

/* Digital Output Function Prototypes-----------------*/


/* Digital Status Read Function Definition-----------------*/
static int Read_DI_IN1(){
	int val=HAL_GPIO_ReadPin (DI_IN1_GPIO_Port, DI_IN1_Pin);
	return val;
}
static int Read_DI_IN2(){
	int val=HAL_GPIO_ReadPin (DI_IN2_GPIO_Port, DI_IN2_Pin);
	return val;
}
static int Read_DI_IN3(){
	int val=HAL_GPIO_ReadPin (DI_IN3_GPIO_Port, DI_IN3_Pin);
	return val;
}
static int Read_DI_MAINS_STATE(){
	int val=HAL_GPIO_ReadPin (DI_MAINS_STATE_GPIO_Port, DI_MAINS_STATE_Pin);
	return val;
}
static int Read_DI_ACC_STATE(){
	int val=HAL_GPIO_ReadPin (DI_ACC_STATE_GPIO_Port, DI_ACC_STATE_Pin);
	return val;
}
static char Read_DI_BOX_STATE(){
	int val=HAL_GPIO_ReadPin (DI_BOX_STATE_GPIO_Port, DI_BOX_STATE_Pin);
	char box='O';
	if(val==1)box='C';
		else box='O';

	return box;
}
static int Read_DI_SOS_STATE(){
	int val=HAL_GPIO_ReadPin (DI_SOS_STATE_GPIO_Port, DI_SOS_STATE_Pin);
	return val;
}
//static int Read_DI_INT1_ACCEL(){
//	int val=HAL_GPIO_ReadPin (DI_INT1_ACCEL_GPIO_Port, DI_INT1_ACCEL_Pin);
//	return val;
//}

/* Digital Status Read Function Definition-----------------*/


/* Analog Value Read Function Definition-----------------*/


static void Init_ADC(){
	HAL_ADC_Start_DMA(&hadc1, ADCvalue, 4);
}

static float Read_ADC1(){
	float val=(float)ADCvalue[0];

	return ((float)(val*VSENSE*11));
    	//HAL_Delay(1);
}

static float Read_ADC2(){
	return (ADCvalue[1]*VSENSE*11);
	    	//HAL_Delay(1);
}
static float Read_EXT_B_SENSE(){
	return (ADCvalue[2]*VSENSE*16);
	    	//HAL_Delay(1);
}
static float Read_INT_B_SENSE(){
	return (ADCvalue[3]*VSENSE*2);
	    	//HAL_Delay(1);
}

/* Analog Value Read Function Definition-----------------*/


/* Digital Output Function Definition-----------------*/

static void SET_OUT3_P_LED(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_OUT3_P_LED_GPIO_Port, DO_OUT3_P_LED_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_OUT3_P_LED_GPIO_Port, DO_OUT3_P_LED_Pin, GPIO_PIN_RESET);
	}

}
static void SET_5V_OUT_EN(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_5V_OUT_EN_GPIO_Port, DO_5V_OUT_EN_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_5V_OUT_EN_GPIO_Port, DO_5V_OUT_EN_Pin, GPIO_PIN_RESET);
	}
}


static void SET_OUT2(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_OUT2_GPIO_Port, DO_OUT2_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_OUT2_GPIO_Port, DO_OUT2_Pin, GPIO_PIN_RESET);
	}
}

static void SET_OUT1(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_OUT1_GPIO_Port, DO_OUT1_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_OUT1_GPIO_Port, DO_OUT1_Pin, GPIO_PIN_RESET);
	}
}

static void SET_LED_GPS(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_LED_GPS_GPIO_Port, DO_LED_GPS_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_LED_GPS_GPIO_Port, DO_LED_GPS_Pin, GPIO_PIN_RESET);
	}
}

static void SET_LED_NET(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_LED_NET_GPIO_Port, DO_LED_NET_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_LED_NET_GPIO_Port, DO_LED_NET_Pin, GPIO_PIN_RESET);
	}
}

static void SET_LED_PWR(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_LED_PWR_GPIO_Port, DO_LED_PWR_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_LED_PWR_GPIO_Port, DO_LED_PWR_Pin, GPIO_PIN_RESET);
	}
}

static void SET_GPS_VCC_EN(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_GPS_VCC_EN_GPIO_Port, DO_GPS_VCC_EN_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_GPS_VCC_EN_GPIO_Port, DO_GPS_VCC_EN_Pin, GPIO_PIN_RESET);
	}
}

static void SET_PWRKEY(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_PWRKEY_GPIO_Port, DO_PWRKEY_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_PWRKEY_GPIO_Port, DO_PWRKEY_Pin, GPIO_PIN_RESET);
	}
}

static void SET_GSM_VCC_EN(int val){
	if(val==1){
		HAL_GPIO_WritePin(DO_GSM_VCC_EN_GPIO_Port, DO_GSM_VCC_EN_Pin, GPIO_PIN_SET);
	}
	else if(val==0){
		HAL_GPIO_WritePin(DO_GSM_VCC_EN_GPIO_Port, DO_GSM_VCC_EN_Pin, GPIO_PIN_RESET);
	}
}



/* Digital Output Function Definition-----------------*/





#endif /* INC_GPIO_H_ */
