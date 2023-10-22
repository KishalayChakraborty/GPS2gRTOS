/*
 * testCode.h
 *
 *  Created on: 13-Jul-2021
 *      Author: THE LOGICBOX
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_
/*
#include <time.h>

clock_t start, end;

void tic(){

    start = clock();
}

void toc(){
	end = clock();
	double cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
	char time1[10];
	sprintf(time1, "%f",cpu_time_used);
	Debug_Tx(time1);
}
*/

//#include <time.h>

int ticT;
int tocT;
char tocTxt[30];
char time1[10];
int tic(){
	return HAL_GetTick();
}
void toc(int tc, char Message[]){
	tocT=HAL_GetTick();

	memset(tocTxt,0,30);
	memset(time1,0,10);
	float diff = (float)(tocT-tc)/1000;// / CLOCKS_PER_SEC;

	sprintf(time1, "%f",diff);

	strcat(tocTxt,Message);
	strcat(tocTxt,time1);
	Debug_Tx(tocTxt);


}



void testBlink(void){

	  while (1)
	  {
	    /* USER CODE END WHILE */
		  Debug_Tx("__________________Blink Test________________");
		  HAL_GPIO_TogglePin (GPIOD, DO_LED_GPS_Pin);
		  HAL_GPIO_TogglePin (GPIOD, DO_LED_NET_Pin);
		  HAL_Delay (500);

	    /* USER CODE BEGIN 3 */
	  }
}
int TimeDelay(int tc, float delay){
	tocT=HAL_GetTick();


	float diff = (float)(tocT-tc)/1000;// / CLOCKS_PER_SEC;
	if(delay>diff)return 1;
	else return 0;

}

void TestDigitalOutput(){
	int i=0;
	Debug_Tx("------------------Testing DigitalOutput:");
	for(i=0;i<2;i++){
		Debug_Tx("Out1,Out2,LED_GPS,LED_NET,LED_PWR,OUT3_P_LED Digital Output High:(.5 Sec)");
		  SET_OUT2(1);
		  SET_OUT1(1);
		  SET_LED_GPS(1);
		  SET_LED_NET(1);
		  SET_LED_PWR(1);
		  SET_OUT3_P_LED(1);
		HAL_Delay(1000);
		Debug_Tx("Out1,Out2,LED_GPS,LED_NET,LED_PWR,OUT3_P_LED Digital Output Low:(.5 Sec)");
		  SET_OUT2(0);
		  SET_OUT1(0);
		  SET_LED_GPS(0);
		  SET_LED_NET(0);
		  SET_LED_PWR(0);
		  //SET_OUT3_P_LED(0);
		HAL_Delay(1000);
	}
	HAL_Delay(2000);
}


void TestDigitalInput(){
	int i=0;
	Debug_Tx("------------------Testing DIgitalInput:");
	for(i=0;i<3;i++){
		Debug_Tx("Status of Digital Inputs:");
		int op[7];
		op[0]=Read_DI_IN1();
		op[1]=Read_DI_IN2();
		op[2]=Read_DI_IN3();
		op[3]=Read_DI_MAINS_STATE();
		op[4]=Read_DI_ACC_STATE();
		op[5]=Read_DI_BOX_STATE();
		op[6]=Read_DI_SOS_STATE();
		char di[100];
		sprintf(di, "IN1:%d;\nIN2:%d;\nIN3:%d;\nMAINS_STATE:%d;\nACC_STATE:%d;\nBOX_STATE:%c;\nSOS_STATE:%d;\n",op[0],op[1],op[2],op[3],op[4],op[5],op[6]);
		Debug_Tx(di);
		HAL_Delay(2000);
	}
}


void TestAnalogInput(){
	int i=0;
	Debug_Tx("------------------Testing AnalogInput:");
	for(i=0;i<2;i++){
		Debug_Tx("Voltage at of Analog Pins:");
		float op[4];
		op[0]=Read_ADC1();
		op[1]=Read_ADC2();
		op[2]=Read_EXT_B_SENSE();
		op[3]=Read_INT_B_SENSE();
		char di[100];
		char ADC11[10], ADC21[10], EXT_B_C1[10], INT_B_C1[10];

    	snprintf( ADC11, 6, "%f", op[0]);
    	snprintf( ADC21, 6, "%f", op[1]);
    	snprintf( EXT_B_C1, 6, "%f", op[2]);
    	snprintf( INT_B_C1, 6, "%f", op[3]);
	 	//  char StatusStrng[50];
	 	//  sprintf(StatusStrng, "%d,%d,%s,%s,%d,%c",ACC_STATE,MAINS_STATE,EXT_B_C,INT_B_C,SOS_STATE,box);


		sprintf(di, "ADC1:%s;\nADC2:%s;\nEXT_B_SENSE:%s;\nINT_B_SENSE:%s;\n",ADC11,ADC21,EXT_B_C1,INT_B_C1);
		Debug_Tx(di);
		HAL_Delay(2000);
	}
}


void TestGPS(){
	int i=0;
	Debug_Tx("------------------Testing GPS:");
	for(i=0;i<3;i++){
		Debug_Tx(gpsDataRet);
		//HAL_UART_Receive_IT(&huart2, (uint8_t *)gpsData, 900);
		//HAL_Delay(2000);
	}
}

void TestGSM(){
	Debug_Tx("------------------Testing  GSM:");
	Debug_Tx("GSM IMEI Data:");
	Debug_Tx(GSMIMEI());
	Debug_Tx("GSM SIM Operator Info:");
	Debug_Tx(GSMSimOperator());
	Debug_Tx("GSM Cell Info:");
	Debug_Tx(GSMCellInfo());
	Debug_Tx("GSM CCID:");
	Debug_Tx(GSMSimNo());


	HAL_Delay(2000);
}

void TestACC(){
	int i=0;
	Debug_Tx("------------------Testing AccGyro:");
	for(i=0;i<10;i++){
		Debug_Tx(detectAccStr());
		HAL_Delay(1000);
	}
}

void TestMEM(){
	int i=0;
	Debug_Tx("------------------Testing Memory:");
	for(i=0;i<1;i++){
		Debug_Tx("Clearing memory Chip--");
		ClearQueue();
		Debug_Tx("Writing Configuration--");
		writeConfig("PCB_V1_1\0","in sim no1234567891234in sim no\0","out sim no 123456789123456out sim no\0",
		   	  				  "http://192.168.10.0/aaEmergenncy ip\0","http://192.168.10.0/Reg ip\0","http://192.168.10.0/Track ID\0",
		  	  				  "oooooooootttttthheerrrrrrOtherdatadddaaatttttttttaaaaaaaaa\0");
		Debug_Tx("Reading RegNo--");
		Debug_Tx(readRegNo());
		HAL_Delay(2000);
	}


}

void TestMEMQ(){
	int i=0;
	Debug_Tx("------------------Testing MemoryQQ:");
	char str[256];
	for(i=0;i<40;i++){
		sprintf(str, "%d**data########################################################################################################################################################################################################data*%d", i,i);
		Debug_Tx("Writing Queue");
		Debug_Tx(str);
		WriteQdata((uint8_t*)str, strlen(str)+1);
		//HAL_Delay(500);
	}
	for(i=0;i<2;i++){
		Debug_Tx("Reading Queue");
		ReadQdata();
		Debug_Tx(ReadMDataS);
		//Debug_Tx(ReadMDataS);
		HAL_Delay(1000);
	}
	for(i=40;i<80;i++){
		sprintf(str, "%d**data########################################################################################################################################################################################################data*%d", i,i);
		Debug_Tx("Writing Queue");
		Debug_Tx(str);
		WriteQdata((uint8_t*)str, strlen(str)+1);
		//HAL_Delay(500);
	}
	for(i=0;i<2;i++){
		Debug_Tx("Reading Queue");
		ReadQdata();
		Debug_Tx(ReadMDataS);

		//Debug_Tx(ReadMDataS);
		HAL_Delay(1000);
	}
}



#endif /* INC_TEST_H_ */
