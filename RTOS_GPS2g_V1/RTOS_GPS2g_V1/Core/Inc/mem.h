/*
 * mem.h
 *
 *  Created on: 07-Jul-2021
 *      Author: THE LOGICBOX
 */

#ifndef INC_MEM_H_
#define INC_MEM_H_


#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
extern void Debug_Tx(char*);

extern void SPI_flash_Page_Program(uint8_t*, uint8_t*,uint16_t);
extern void SPI_flash_Read_Data(uint8_t* , uint8_t* , uint16_t );
extern void SPI_flash_Sector_Erase(uint8_t*);
extern void SPI_flash_Chip_Erase();



#define Write_Enable 0x06
#define Write_Disable 0x04
#define Read_Status_Register 0x05
#define Write_Status_Register 0x01
#define Read_Data  0x03
#define Fast_Read  0x0B
#define Fast_Read_Dual_Output  0x3B
#define Page_Program 0x02
#define Block_Erase 0xD8
#define Sector_Erase 0x20
#define Chip_Erase  0xC7
#define Power_down 0xB9
#define Release_Powerdown_Device_ID 0xAB
#define Manufacturer 0x90
#define JEDEC_ID 0x9F

#define SPI_flash_cs_low HAL_GPIO_WritePin(SPI_CS_MEM_GPIO_Port, SPI_CS_MEM_Pin, GPIO_PIN_RESET)
//HAL_GPIO_WritePin (GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define SPI_flash_cs_high HAL_GPIO_WritePin(SPI_CS_MEM_GPIO_Port, SPI_CS_MEM_Pin, GPIO_PIN_SET)
//HAL_GPIO_WritePin(GPIOB, GPIO_PIN 9, GPIO_PIN_SET)



//#define ChipSelect()            HAL_GPIO_WritePin(SPI_CS_MEM_GPIO_Port, SPI_CS_MEM_Pin, GPIO_PIN_RESET)
//#define ChipDeselect()          HAL_GPIO_WritePin(SPI_CS_MEM_GPIO_Port, SPI_CS_MEM_Pin, GPIO_PIN_SET)

#define RegNoLen	16
#define INSMSLen	100
#define OUTSMSLen	60
#define EmgIPLen	40
#define RegIPLen	40
#define TrackIPLen	40
#define OtherDataLen	256

uint8_t dataR[256];
//uint8_t data4k[4096];
uint8_t addRet[3];


uint8_t buffer_SPI_Recive[30];
//The data you want to receive

uint8_t LastLoc[]={0x01,0x00,0x00};
uint8_t WrtLoc[]={0x01,0x00,0x00};

uint8_t WrtAdd[]={0x03,0x00,0x00};
uint8_t ReadMData[4096];
char ReadMDataS[4096];




//(uint8_t* data_address, uint8_t* data, uint16_t size)



uint8_t readQ1(uint8_t*);
uint8_t* incrimentAddress(uint8_t*,int);
//void Copy4k(uint8_t* add1);
void ClearQueue();
void QremoveLast();
char* readQData();
void writeQData(char*);
uint8_t* readDataLoc();
uint8_t* readDataWrtLoc();
void writeDataLoc(int,int);
int writeConfig(char*,char*,char*,char*,char*,char*,char*);
char* readRegNo();
char* readINSMSno();
char* readOUTSMSno();
char* readEmgIP();
char* readRegIP();
char* readTracIP();
char* readOtherData();






/* Base I2C and SPI Function Prototypes-----------------*/


uint8_t buffer[30];
uint8_t buffer_SPI_Sent [30]={'H', 'e','1','1','o',' ','S','t','m', '3', '2'};
//The data you want to sent
uint8_t buffer_SPI_Recive[30];
//The data you want to receive

//This is the definition of an array

void SPI_flash_sent_byte(uint8_t data){
    HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
}

void SPI_flash_Write_Enable() {
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Write_Enable);
    SPI_flash_cs_high;
}

void SPI_flash_Write_Disable() {
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Write_Disable);
    SPI_flash_cs_high;
}

void SPI_flash_sent_address(uint8_t* Sent){
    HAL_SPI_Transmit(&hspi1, Sent, 3,1000);
}



void SPI_flash_TransmitReceive (uint8_t *data, uint16_t size){
    HAL_SPI_TransmitReceive (&hspi1, data,data, size, 1000);
}

void SPI_flash_get_device_ID( ) {
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Manufacturer);//Manufacturer);
    //SPI_flash_sent_address (address);
    uint8_t aa[10];
    aa[0]=0;
    aa[1]=0;
    aa[2]=0;
    aa[3]=0;
    aa[4]=0;
    aa[5]=0;
    SPI_flash_sent_address (aa);
    SPI_flash_TransmitReceive(aa, 2);
    SPI_flash_cs_high;
    char bufd[100];
    sprintf(bufd,"Manufacturer ID[90h]: 0x%X%X \r\n", aa[0],aa[1]);
    Debug_Tx((char*)bufd);

    SPI_flash_cs_low;
       SPI_flash_sent_byte(0x9F);//Manufacturer);
       //SPI_flash_sent_address (address);
       //uint8_t aa[10];
       aa[0]=0;
       aa[1]=0;
       aa[2]=0;
       aa[3]=0;
       aa[4]=0;
       aa[5]=0;
      // SPI_flash_sent_address (aa);
       SPI_flash_TransmitReceive(aa, 3);
       SPI_flash_cs_high;
       //char bufd[100];
       sprintf(bufd,"Device ID [9Fh]: 0x%X%X%X\r\n", aa[0],aa[1],aa[2]);
       Debug_Tx((char*)bufd);



       SPI_flash_cs_low;
       SPI_flash_sent_byte(0x4B);//Manufacturer);
       //SPI_flash_sent_address (address);
       //uint8_t aa[10];
       aa[0]=0;
       aa[1]=0;
       aa[2]=0;
       aa[3]=0;
       aa[4]=0;
       aa[5]=0;
       //SPI_flash_sent_address (aa);
       HAL_SPI_Transmit(&hspi1, aa, 4,1000);
       SPI_flash_TransmitReceive(aa, 6);
       SPI_flash_cs_high;
      // char bufd[100];
       sprintf(bufd,"Unique ID [9Fh]: 0x%X%X%X%X%X%X\r\n", aa[0],aa[1],aa[2],aa[3],aa[4],aa[5] );
       Debug_Tx((char*)bufd);
}


void SPI_flash_Block_Erase(uint8_t* address) {
    SPI_flash_Write_Enable();
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Block_Erase);
    SPI_flash_sent_address (address);
    SPI_flash_cs_high;
    SPI_flash_Write_Disable();

}

void SPI_flash_Sector_Erase(uint8_t* address) {
    SPI_flash_Write_Enable();
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Sector_Erase);
    SPI_flash_sent_address (address);
    SPI_flash_cs_high;
    SPI_flash_Write_Disable();
    //HAL_Delay(100);
	Debug_Rx();
}
void SPI_flash_Chip_Erase(){
    SPI_flash_Write_Enable();
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Chip_Erase);
    SPI_flash_cs_high;
    SPI_flash_Write_Disable();
}

void SPI_flash_Power_down(){
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Power_down);
    SPI_flash_cs_high;
}




void SPI_flash_Page_Program(uint8_t* data_address, uint8_t* data, uint16_t size) {
	//Writes Data too address  upto size
    SPI_flash_Write_Enable();
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Page_Program);
    SPI_flash_sent_address (data_address);
    SPI_flash_TransmitReceive(data, size);
    SPI_flash_cs_high;
    SPI_flash_Write_Disable();

	Debug_Rx();
    //HAL_Delay(1);
}

void SPI_flash_Read_Data(uint8_t* data_address, uint8_t* data, uint16_t size){
	//Reads data from MEM starting from given address of size size
	//gets data to Data
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Read_Data);
    SPI_flash_sent_address(data_address);
    SPI_flash_TransmitReceive(data, size);
    SPI_flash_cs_high;
    //Debug_Tx((char*)data);
}



uint8_t* incrimentAddress(uint8_t* add,int inc){
	int addi=(add[0]*256*256)+(add[1]*256)+(add[2])+inc;
	add[0]=addi/(256*256);
	add[1]=(addi-add[0])/256;
	add[2]=(addi)%256;


	return add;
}

void incrimentAddress2(int inc){
	int addi=(WrtAdd[0]*256*256)+(WrtAdd[1]*256)+(WrtAdd[2])+inc;
	WrtAdd[0]=addi/(256*256);
	WrtAdd[1]=(addi%(256*256))/256;
	WrtAdd[2]=(addi)%256;


}

void printAdd() {
	char str[20];
	memset(str,0,20);
	sprintf(str, "MemoryAdd:%X-%X-%X",WrtAdd[0],WrtAdd[1],WrtAdd[2]);
	Debug_Tx(str);
}

void WriteQdata(uint8_t* data, uint16_t len) {
	//Writes data at the end of queue
	if (len<255){
		//Debug_Tx("---------------------\nWriting  Data");
		//printAdd();
		SPI_flash_Page_Program(WrtAdd, data,len);
		incrimentAddress2(256);
	}
}

int isQempty() {
	int addi=(WrtAdd[0]*256*256)+(WrtAdd[1]*256);
	if(addi<(256*256*3)+256){
		return 1;
	}
	return 0;

}


void GetLastAddress() {
	//Read data from the end of queue
	//printAdd();

	char str[20];


	if((WrtAdd[0]==3) & (WrtAdd[1]==0)& (WrtAdd[1]==0)){
		for(int i=0;i<60000;i++){
			SPI_flash_Read_Data(WrtAdd , ReadMData ,5);
			memset(str,0,20);
			sprintf(str, "Add:%X-%X-%X  Data-%X-%X-%X",WrtAdd[0],WrtAdd[1],WrtAdd[2],ReadMData[0],ReadMData[1],ReadMData[2]);
			Debug_Tx(str);
			if(((ReadMData[0]<255) & (ReadMData[1]<255)& (ReadMData[2]<255)& (ReadMData[3]<255))&((ReadMData[0]>0) & (ReadMData[1]>0)& (ReadMData[2]>0)& (ReadMData[3]>0))){
				incrimentAddress2(256);
			}
			else{
				break;
			}
		}
	}
	//printAdd();

}



int ReadQdata() {
	//Read data from the end of queue
	//printAdd();
	memset(ReadMData,0,4096);
	memset(ReadMDataS,0,4096);
	if(isQempty()==1){
		Debug_Tx("Memory Empty");
		return 0;
	}
	incrimentAddress2(-1);
	int len=WrtAdd[1]%16;
	len=len+1;
	WrtAdd[1]=WrtAdd[1]/16;
	WrtAdd[1]=WrtAdd[1]*16;
	WrtAdd[2]=0;
	SPI_flash_Read_Data(WrtAdd , ReadMData , len*256);

	char temp[256];
	for (int i=1;i<=len;i++){
		memset(temp,0,256);
		for (int j=0;j<256;j++){
			temp[j]=ReadMData[(len-i)*256+j];
			if (temp[j]==255){
				temp[j]=0;
				break;
			}
		}
		temp[255]=0;
		strcat(ReadMDataS,temp);
		strcat(ReadMDataS,"\r\n");
	}
	SPI_flash_Sector_Erase(WrtAdd);
	return 1;
}

void InitMEMQ(){
	if(isQempty()==1){
		int i=0;
		Debug_Tx("------------------Init Mem:");
		char str[256];
		for(i=0;i<5;i++){
			sprintf(str, "%d**data********************************************************************************************************************************************************************************************************data*%d", i,i);
			WriteQdata((uint8_t*)str, strlen(str)+1);
		}
		for(i=0;i<2;i++){
			ReadQdata();
		}
	}

}































void ClearQueue(){
	//uint8_t dat[]={0x01, 0x00,0x00,0x01, 0x00,0x00,0x00,0x00,0x00,0x00,0x01};
	//uint8_t add[]={0x00,0x10,0x00};

	//SPI_flash_Sector_Erase(add);
	SPI_flash_Chip_Erase();
	HAL_Delay(30000);

	//SPI_flash_Page_Program(add,dat,11);
}


































int writeConfig(char* RegNo,char* INSMS,char* OUTSMS,char* EmgIP,char* RegIP,char* TrackIP,char* OtherData){
	uint8_t data1[256];
	uint8_t data2[256];
	uint8_t data3[256];
	uint8_t add[]={0x00,0x00,0x00};

	SPI_flash_Read_Data(add , data1 , 256);
	add[1]=0x01;
	SPI_flash_Read_Data(add , data2 , 256);
	add[1]=0x02;
	SPI_flash_Read_Data(add , data3 , 256);



	int i=0;
	int j=0;
	for(i=0;i<=strlen(RegNo)&&i<RegNoLen;i++){
		data1[j+i]=RegNo[i];
	}
	j=j+RegNoLen;

	for(i=0;i<=strlen(INSMS)&&i<INSMSLen;i++){
		data1[j+i]=INSMS[i];
	}
	j=j+INSMSLen;

	for(i=0;i<=strlen(OUTSMS)&&i<OUTSMSLen;i++){
		data1[j+i]=OUTSMS[i];
	}
	j=0;
	for(i=0;i<=strlen(EmgIP)&&i<EmgIPLen;i++){
		data2[j+i]=EmgIP[i];
	}
	j=j+EmgIPLen;

	for(i=0;i<=strlen(RegIP)&&i<RegIPLen;i++){
		data2[j+i]=RegIP[i];
	}
	j=j+RegIPLen;


	for(i=0;i<=strlen(TrackIP)&&i<TrackIPLen;i++){
		data2[j+i]=TrackIP[i];
	}
	j=0;


	for(i=0;i<=strlen(OtherData)&&i<OtherDataLen;i++){
		data3[j+i]=OtherData[i];
	}
	j=j+OtherDataLen;


	add[1]=0x00;

	SPI_flash_Sector_Erase(add);

	SPI_flash_Page_Program(add, data1,256);
	add[1]=0x01;
	SPI_flash_Page_Program(add, data2,256);
	add[1]=0x02;
	SPI_flash_Page_Program(add, data3,256);
	return 1;

}



char* readRegNo(){
	uint8_t add4[]={0x00, 0x00,0x00};

    memset(dataR,0,256);
	SPI_flash_Read_Data(add4 , (uint8_t*)dataR , RegNoLen);
	if (strlen((char*) dataR)>1){

		//Debug_Tx((char*) dataR);
		return (char*) dataR;
	}
	else{
		return "--Err RegNo--";
	}

}

char* readINSMSno(){
	uint8_t add4[]={0x00, 0x00,0x00};
	add4[2]=RegNoLen;
	SPI_flash_Read_Data(add4 , (uint8_t*)dataR , INSMSLen);
	return (char*) dataR;
}

char* readOUTSMSno(){

	uint8_t  add4[]={0x00, 0x00,0x00};
	add4[2]=RegNoLen+INSMSLen;
	SPI_flash_Read_Data(add4 , (uint8_t*)dataR , OUTSMSLen);
	return (char*) dataR;
}

char* readEmgIP(){
	uint8_t  add4[]={0x00, 0x01,0x00};
	SPI_flash_Read_Data(add4 , (uint8_t*)dataR , EmgIPLen);
	return (char*) dataR;
}

char* readRegIP(){
	uint8_t  add4[]={0x00, 0x01,0x00};
	add4[2]=EmgIPLen;
	SPI_flash_Read_Data(add4 , (uint8_t*)dataR , RegIPLen);
	return (char*) dataR;
}

char* readTracIP(){
	uint8_t  add4[]={0x00, 0x01,0x00};
	add4[2]=EmgIPLen+RegIPLen;
	SPI_flash_Read_Data(add4 , (uint8_t*)dataR , TrackIPLen);
	return (char*) dataR;
}

char* readOtherData(){
	uint8_t  add4[]={0x00, 0x02,0x00};
	SPI_flash_Read_Data(add4 , (uint8_t*)dataR , OtherDataLen);
	return (char*) dataR;

}


#endif /* INC_MEM_H_ */
