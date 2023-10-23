/*
 * memold.h
 *
 *  Created on: 24-Oct-2021
 *      Author: THE LOGICBOX
 */
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
uint8_t data4k[4096];
uint8_t addRet[3];


uint8_t buffer_SPI_Recive[30];
//The data you want to receive

uint8_t LastLoc[]={0x01,0x00,0x00};
uint8_t WrtLoc[]={0x01,0x00,0x00};


//(uint8_t* data_address, uint8_t* data, uint16_t size)



uint8_t readQ1(uint8_t*);
uint8_t* incrimentAddress(uint8_t*,int);
void Copy4k(uint8_t* add1);
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
uint8_t buffer_address []={0x00, 0x00,0x00}; //It's just a zero address, you can change it into whatever you like

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

void SPI_flash_get_device_ID(uint8_t *address) {
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Manufacturer);
    SPI_flash_sent_address (address);
    SPI_flash_TransmitReceive(address, 2);
    SPI_flash_cs_high;
    char bufd[100];
    sprintf(bufd,"Manufacturer ID: 0x%X%X\r\n", address[0],address[1]);
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
    SPI_flash_cs_low;
    SPI_flash_sent_byte(Read_Data);
    SPI_flash_sent_address(data_address);
    SPI_flash_TransmitReceive(data, size);
    SPI_flash_cs_high;
    //Debug_Tx((char*)data);
}





uint8_t readQ1(uint8_t* add){
	char d[1];
	SPI_flash_Read_Data(add , (uint8_t*)d , 1);
	return d[0];
}

uint8_t* incrimentAddress(uint8_t* add,int inc){
	int addi=(add[0]*256*256)+(add[1]*256)+(add[2])+inc;
	add[0]=addi/(256*256);
	add[1]=(addi-add[0])/256;
	add[2]=(addi)%256;
	return add;
}
void Copy4k(uint8_t* add1){

	uint8_t d[1];

	addRet[0]=add1[0];
	addRet[1]=(add1[1]/16)*16;
	addRet[2]=0x00;
	int i=0;
	int c=0;
	while(c<1){
		data4k[i]=readQ1(addRet);
		memcpy(addRet,incrimentAddress(addRet,1),3);
		if (add1[0]<=addRet[0] && add1[1]<=addRet[1] && add1[2]<=addRet[2]){
			c=1;
		}
		i++;
	}

	addRet[0]=add1[0];
	addRet[1]=(add1[1]/16)*16;
	addRet[2]=0x00;
	SPI_flash_Sector_Erase(addRet);
	d[0]=data4k[0];
	SPI_flash_Page_Program(addRet,d,1);
	for(c=1;c<i;c++){
		d[0]=data4k[c];
		memcpy(addRet,incrimentAddress(addRet,1),3);

		SPI_flash_Page_Program(addRet,d,1);
	}




}
void ClearQueue(){
	uint8_t dat[]={0x01, 0x00,0x00,0x01, 0x00,0x00,0x00,0x00,0x00,0x00,0x01};
	uint8_t add[]={0x00,0x10,0x00};


	//SPI_flash_Sector_Erase(add);
	SPI_flash_Chip_Erase();


	//HAL_Delay(20000);
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	Debug_Rx();
	SPI_flash_Page_Program(add,dat,11);



}



void QremoveLast(){
	uint8_t add1[]={0x00,0x00,0x00};
	uint8_t add2[]={0x00,0x00,0x00};

	memcpy(add1,readDataLoc(),3);

	memcpy(add2, add1, 3);


	int i=0;
	char d='a';
	while(d!='$' && i<=256){
		d=readQ1(add1);
		memcpy(add1,incrimentAddress(add1,-1),3);

		i++;
	}


	add1[1]=(add1[1]/16)*16;
	add2[1]=(add2[1]/16)*16;
	if(add1[1]+16<=add2[1]){
		add1[0]=0x00;
		SPI_flash_Sector_Erase(add1);
	}
	i=i+1;
	writeDataLoc((-1)*i,0);

}



char* readQData(){
	uint8_t add[3];
	memcpy(add,readDataLoc(),3);
	char data[256];
	int i=0;
	char d='a';
	while(d!='$' && i<=256){
		d=readQ1(add);
		memcpy(add,incrimentAddress(add,-1),3);
		data[i]=d;
		i++;
	}
	int j=0;
	for(j=0;j<i;j++){
		dataR[j]=data[i-j-1];
	}
	dataR[i]='\0';
	return (char*)dataR;
}

void writeQData(char *data){
	int len=strlen(data);

	uint8_t AddL[3];
	uint8_t AddW[3];

	memcpy(AddL,readDataLoc(),3);
	memcpy(AddW,readDataWrtLoc(),3);
	if(AddL[0]==AddW[0] && AddL[1]==AddW[1] && AddL[2]==AddW[2]){
		Debug_Tx("Data address equal");
	}
	else{
		Debug_Tx("Data address un equal refreshing segment");
		Copy4k(AddL);
	}


	memcpy(AddL,readDataLoc(),3);
	memcpy(AddW,readDataWrtLoc(),3);
	if ((AddL[2]+len)<=256){
		Debug_Tx("len ok no seg");
		memcpy(AddL,incrimentAddress(AddL,1),3);

		uint8_t buff[256];
		memcpy(buff,data,len);
		SPI_flash_Page_Program(AddL, buff,len);
		writeDataLoc(len,len);
		Debug_Tx("write done");
	}
	else{
		Debug_Tx("seg req");
		int rem=256-AddL[2];
		memcpy(AddL,incrimentAddress(AddL,1),3);
		uint8_t buff[256];
		memcpy(buff,data,len);
		uint8_t subbuff[256];
		memcpy( subbuff, &buff[0], rem );

		SPI_flash_Page_Program(AddL, (uint8_t*)subbuff,rem);

		memcpy(AddL, incrimentAddress(AddL,rem), 3);

		memcpy( subbuff, &buff[rem-1], len-rem );
		subbuff[len-rem] = '\0';
		SPI_flash_Page_Program(AddL, (uint8_t*)subbuff,len-rem+1);
		writeDataLoc(len,len);
	}
}



uint8_t* readDataLoc(){
	uint8_t data1[11];
	int eraseCount=0;
	int i=0;
	int j=0;

	uint8_t add[]={0x00,0x00,0x00};

	while(i<14){
		add[0]=0x00;
		add[1]=add[1]+0x10;
		add[2]=0x00;
		SPI_flash_Read_Data(add , data1 , 11);
		for (j=0;j<3;j++){
			LastLoc[j]=data1[j];
		}
		eraseCount=0;
		for (j=0;j<5;j++){
			eraseCount=(eraseCount*j*16)+data1[j+6];
		}
		if(eraseCount<90000){
			i=15;
		}
		i++;


	}
	return LastLoc;
}




uint8_t* readDataWrtLoc(){
	uint8_t data1[11];
	int eraseCount=0;
	int i=0;
	int j=0;
	uint8_t add[]={0x00,0x00,0x00};
	while(i<14){
		add[0]=0x00;
		add[1]=add[1]+0x10;
		add[2]=0x00;
		SPI_flash_Read_Data(add , data1 , 11);
		for (j=0;j<3;j++){
			WrtLoc[j]=data1[j+3];
		}

		eraseCount=0;
		for (j=0;j<5;j++){
			eraseCount=(eraseCount*j*16)+data1[j+6];
		}
		if(eraseCount<90000){
			i=15;
		}
		char dd[12];

		for(j=0;j<11;j++){
			dd[j]=(char)data1[j]+48;
		}
		dd[11]=0;
		i++;
		Debug_Tx(dd);
	}

	return WrtLoc;
}







void writeDataLoc(int len,int wrt){
	uint8_t data1[256];

	LastLoc[0]=0x01;
	LastLoc[1]=0x00;
	LastLoc[2]=0x00;
	WrtLoc[0]=0x01;
	WrtLoc[1]=0x00;
	WrtLoc[2]=0x00;
	//, 0x00,0x00};
	uint8_t add[]={0x00,0x00,0x00};
	int eraseCount=0;
	int i=0;
	int j=0;
	while(i<14){
		add[1]=add[1]+0x10;
		SPI_flash_Read_Data(add , data1 , 256);
		for (j=0;j<3;j++){
			LastLoc[j]=data1[j];
		}

		for (j=0;j<3;j++){
			WrtLoc[j]=data1[j+3];
		}

		eraseCount=0;
		for (j=0;j<5;j++){
			eraseCount=(eraseCount*j*16)+data1[j+6];
		}
		if(eraseCount<90000){
			i=15;
		}
		i++;
	}
	eraseCount++;
	uint8_t ce[]={0x00,0x00,0x00,0x00,0x00};
	int a=0;
	//a=eraseCount%16;
	for (i=4;i>=0;i--){
		a=eraseCount%16;
		ce[i]=a;
		eraseCount=eraseCount/16;
	}
	if(len<0 && wrt<0){
		//memcpy(LastLoc,incrimentAddress(LastLoc,len),3);
		//memcpy(WrtLoc,incrimentAddress(WrtLoc,wrt),3);
		uint8_t dt[11];
		int l=0,k=0;
		for(k=0;k<3;k++)dt[l++]=WrtLoc[k];
		for(k=0;k<3;k++)dt[l++]=WrtLoc[k];
		for(k=0;k<5;k++)dt[l++]=ce[k];
		SPI_flash_Sector_Erase(add);
		SPI_flash_Page_Program(add,dt,11);

	}
	else{
		memcpy(LastLoc,incrimentAddress(LastLoc,len),3);
		memcpy(WrtLoc,incrimentAddress(WrtLoc,wrt),3);
		SPI_flash_Sector_Erase(add);
		uint8_t dt[11];
		int l=0,k=0;
		for(k=0;k<3;k++)dt[l++]=LastLoc[k];
		for(k=0;k<3;k++)dt[l++]=WrtLoc[k];
		for(k=0;k<5;k++)dt[l++]=ce[k];
		SPI_flash_Page_Program(add,dt,11);
	}

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
