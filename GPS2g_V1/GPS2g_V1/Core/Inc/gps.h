/*
 * gps.h
 *
 *  Created on: 26-Jun-2021
 *      Author: THE LOGICBOX
 */


#ifndef INC_GPS_H_
#define INC_GPS_H_
#define _SVID_SOURCE
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdlib.h>

#include <time.h>


extern void SET_LED_GPS(int);


//extern void GPS_Tx(const char*);
extern void Debug_Tx(char*);

#include <assert.h>
void ProcessGPS();



struct GPS
{
   float lat, lon,alt,hdop,pdop,head,speed;
   int fix,sat, Updt_ms;
   char latD[3],lonD[3],time[15],date[15];
};

struct  GPS GPSInfo;
static uint8_t gpsData[1000];
char gpsDataRet[70];
char dd[10];
char printBuf[50];

//char* buffer
//int ProcessRunning_GPS=0;

void initGPS(){
	//GPS_Tx("$PSTMSETCONSTMASK,1025\r\n\0");


	//GPS_Tx("$PSTMCFGTDATA,1964,3443,1,00000012\r\n\0");
	//GPS_Tx("PSTMCFGMSGL,1,1,0x00000044,0x00000000\r\n\0");
	//GPS_Tx("$PSTMSAVEPAR\r\n\0");

	//GPS_Tx("$PSTMSRR\r\n\0");


	GPSInfo.lat=0.0;
	GPSInfo.lon=0.0;
	GPSInfo.alt=0.0;
	GPSInfo.hdop=0.0;
	GPSInfo.pdop=0.0;
	GPSInfo.head=0.0;
	GPSInfo.speed=0.0;
	GPSInfo.fix=0;
	GPSInfo.sat=0;
	strcpy(GPSInfo.latD,"x");
	strcpy(GPSInfo.lonD,"x");
	strcpy(GPSInfo.time,"x");
	strcpy(GPSInfo.date,"x");
}
void getGPSString(){
	memset(gpsDataRet,0,70);

	if(GPSInfo.fix==1){
		SET_LED_GPS(1);
	}
	else{
		SET_LED_GPS(0);
	}




	sprintf(gpsDataRet, "%1d,%s,%s,%0.6f,%1s,%0.6f,%s,%0.1f,%0.2f,%d,%0.1f,%0.1f,%0.1f",
	    			GPSInfo.fix,GPSInfo.date,GPSInfo.time,GPSInfo.lat,GPSInfo.latD,GPSInfo.lon,GPSInfo.lonD,GPSInfo.speed,GPSInfo.head,GPSInfo.sat,GPSInfo.alt,GPSInfo.pdop,GPSInfo.hdop);
}



float DegreeDecimalConvert(float ddmmmm){
    // printf("insfunction2 %f",ddmmmm);
    int dd1= ddmmmm/100;
    float mm=ddmmmm -(dd1*100);
    mm=mm/60;
    mm=(float)dd1+mm;
    return(mm);
}



void printFlt(float x){
	memset(printBuf,0,50);
	snprintf(printBuf, 6, "%5.0f", x);
	//gcvt(x, 6, buf);
	Debug_Tx(printBuf);

}
void printInt(int x){
	memset(printBuf,0,50);
	snprintf(printBuf, 6, "%d", x);
	//gcvt(x, 6, buf);
	Debug_Tx(printBuf);

}
int nmea0183_checksum(char *s){//one extra blank char was found so last 4 char had to remove
    int c = 0;
    while (*s)
        c ^= *s++;

    return c;

}

/*
 char *strtok_new(char * string, char const * delimiter){
   static char *source = NULL;
   char *p, *riturn = 0;
   if(string != NULL)         source = string;
   if(source == NULL)         return NULL;

   if((p = strpbrk (source, delimiter)) != NULL) {
      *p  = 0;
      riturn = source;
      source = ++p;
   }
   return riturn;
}
char * append(char * str1, char * str2){   char * new_str ;
    if((new_str = malloc(strlen(str1)+strlen(str2)+1)) != NULL){
        new_str[0] = '\0';   // ensures the memory is an empty string
        strcat(new_str,str1);
        strcat(new_str,str2);
    }
    return new_str;
}


void getGNGGA(char * ptra){
	char *delim = ",";
	ptra=append(ptra,",");
	char *dataTab[50];
	char *ptr = strtok_new(ptra, delim);
  	int i=0;// printf("received str%d'%s'\n", i,ptr);
 	while (ptr != NULL && i<50){
 		dataTab[i]=ptr;
	 	ptr = strtok_new(NULL, delim);
	    i++;
 	}
	dataTab[i]=ptr;
    int n=i;
    for(i=1;i<n;i++){
        char str2[strlen(dataTab[i])];
        strcpy(str2,dataTab[i]);
	 	if(i==1){ //Getting time
	 	    //char delim[]=".";
            //char *ptr2 = strtok(str2, delim);
            //strcpy(GPSInfo.time,ptr2);
	    }
	    if(i==2){ //Getting Latitude
	        //float ddmmmm=stor(str2);
	        //GPSInfo.lat=DegreeDecimalConvert(ddmmmm);
	    }
	    if(i==3){ //Getting Latitude ind

	        //strcpy(GPSInfo.latD,str2);
	    }
	    if(i==4){ //Getting Longitude
	    	//float ddmmmm=stor(str2);
	    	//GPSInfo.lon=DegreeDecimalConvert(ddmmmm);
	    }
	    if(i==5){ //Getting Longitude ind
	        //strcpy(GPSInfo.lonD,str2);
	    }
	    if(i==6){ //Getting GPS status
	        if(strcmp(str2,"0")==0){GPSInfo.fix=0;}
	        else{GPSInfo.fix=1;}
	    }
	    if(i==7){ //Getting Satellites no
	    	//GPSInfo.sat = atoi(str2);
	    }
	    if(i==8){ //Getting HDOP
	    	//GPSInfo.sat = stor(str2);
	    }
	    if(i==9){ //Getting Altitude
	    	//GPSInfo.alt=stor(str2);
	    }

    }

}
void getGNVTG(char * ptra){

	char * delim = ",";
	ptra=append(ptra,",");
	// printf("received str2'%s'\n", ptra);
	char *dataTab[50];
	char *ptr = strtok_new(ptra, delim);
  	int i=0;// printf("received str%d'%s'\n", i,ptr);
 	while (ptr != NULL && i<50){
 	    //// printf("received str4'%s'\n", ptr);
	    dataTab[i]=ptr;
	 	ptr = strtok_new(NULL, delim);
	    i++;
	    // printf("received str%d'%s'\n", i,ptr);
	}
	dataTab[i]=ptr;
	//dataTab[i]="*end*";

    int n=i;
    for(i=1;i<n;i++){
        char str2[strlen(dataTab[i])];
        strcpy(str2,dataTab[i]);
        // printf("\n\n daaaaaa%s \n",str2);


	    if(i==9){ //Getting GPS status
	        if(strcmp(str2,"N")==0){strcpy(gpsData[0],"0");}
	        else{strcpy(gpsData[0],"1");}
	    }

	    if(i==5){ //Getting Speed
	        static char  buf[6];
	        strcpy(buf,str2);
            strcpy(gpsData[7],buf);
	    }
	    if(i==1){ //Getting Course
	        static char  buf[1];
	        strcpy(buf,str2);
            strcpy(gpsData[8],buf);
	    }
	    for(int j=0;j<14;j++){
            // printf("%s,",gpsData[j]);
        }
    }
}
*/

void getGNRMC(const char * ptra){
	//Debug_Tx((char*)ptra);
	int i=0;
    char *token = strtok((char *)ptra, ",");
    while( token != NULL ) {
  	    if(i==1){ //Getting time
  	    	strncpy(GPSInfo.time,token,6);
  	    }
  	    if(i==10){ //Getting GPS status
  	        if(strstr(token,"N") != NULL){GPSInfo.fix=0;}
  	        else{GPSInfo.fix=1;}
  	    }
  	    if(i==10){ //Getting GPS status
  	        //if(strstr(token,"N") != NULL){GPSInfo.fix=0;}
  	    }
  	    if(i==3){ //Getting Latitude
  	    	 GPSInfo.lat=DegreeDecimalConvert(stor(token));
  	    }
  	    if(i==4){ //Getting Latitude ind
  	        strcpy(GPSInfo.latD,token);
  	    }
  	    if(i==5){ //Getting Longitude
	        GPSInfo.lon=DegreeDecimalConvert(stor(token));
  	    }
  	    if(i==6){ //Getting Longitude ind
  	    	strcpy(GPSInfo.lonD,token);
  	    }
  	    if(i==9){ //Getting Date
  			memset(dd,0,10);
  	        for(int l=0;l<4;l++){dd[l]=token[l];}
  	        dd[4]='2';
  	        dd[5]='0';
  	        for(int l=6;l<8;l++){dd[l]=token[l-2];}
  	        dd[8]='\0';
  	        strcpy(GPSInfo.date,dd);
  	    }
  	    if(i==7){ //Getting Speed
  	        GPSInfo.speed=stor(token);
  	    }
  	    if(i==8){ //Getting Course
  	        GPSInfo.head=stor(token);
  	    }
  	    token = strtok(NULL,",");
        i++;
    }
	//free(token);
}
void getGNGSA(const  char * ptra){
	//Debug_Tx((char*)ptra);
	int i=0;
    char *token = strtok((char *)ptra, ",");
    while( token != NULL ) {

        if(i==15){ //Getting PDOP
    	    GPSInfo.pdop=stor(token);
    	}
    	if(i==16){ //Getting HDOP
    	    GPSInfo.hdop=stor(token);
        }
  	    token = strtok(NULL,",");
        i++;
    }
	//free(token);
}



void getGNGGA(const  char * ptra){
	//Debug_Tx((char*)ptra);

	int i=0;
    char *token = strtok((char *)ptra, ",");
    while( token != NULL ) {
    	if(i==7){ //Getting Satellites no
            GPSInfo.sat = atoi(token);
        }

    	if(i==9){ //Getting Saltitute
            GPSInfo.alt = stor(token);
        }


  	    if(i==6){ //Getting GPS status
  	        //if(strstr(token,"0") != NULL){GPSInfo.fix=0;}
  	        //else{GPSInfo.fix=1;}
  	    }
  	    token = strtok(NULL,",");
        i++;
    }
	//free(token);
}



/*
void getGNGLL(char * ptra){

	char *delim = ",";
	ptra=append(ptra,",");
	// printf("received str2'%s'\n", ptra);
	char *dataTab[50];
	char *ptr = strtok_new(ptra, delim);
  	int i=0;// printf("received str%d'%s'\n", i,ptr);
 	while (ptr != NULL && i<50){
 	    //// printf("received str4'%s'\n", ptr);
	    dataTab[i]=ptr;
	 	ptr = strtok_new(NULL, delim);
	    i++;
	    // printf("received str%d'%s'\n", i,ptr);
	}
	dataTab[i]=ptr;
	//dataTab[i]="*end*";

    int n=i;
    for(i=1;i<n;i++){
        char str2[strlen(dataTab[i])];
        strcpy(str2,dataTab[i]);
        // printf("\n\n daaaaaa%s \n",str2);


	    if(i==5){ //Getting time
	        char delim[]=".";
            char *ptr2 = strtok(str2, delim);
            static char ptr3[6];
            strcpy(ptr3,ptr2);
	        strcpy(GPSInfo.time,ptr3);
	    }

	    if(i==1){ //Getting Latitudefloat ddmmmm=stor(str2);
	    		        float ddmmmm=stor(str2);
	    		        GPSInfo.lat=DegreeDecimalConvert(ddmmmm);
	    }
	    if(i==2){ //Getting Latitude ind
	        strcpy(GPSInfo.latD,str2);
	    }
	    if(i==3){ //Getting Longitude
	    		        float ddmmmm=stor(str2);
	    		        GPSInfo.lon=DegreeDecimalConvert(ddmmmm);
	    }
	    if(i==4){ //Getting Longitude ind
	        strcpy(GPSInfo.lonD,str2);
	    }
	    if(i==6){ //Getting GPS status
	        if(strcmp(str2,"V")==0){GPSInfo.fix=0;}
	        else{GPSInfo.fix=1;}
	    }
	    if(i==7){ //Getting GPS status2
	        if(strcmp(str2,"N")==0){GPSInfo.fix=0;}
	    }

    }
}
*/
uint8_t nmea_valid_checksum(const char *message) {
    uint8_t checksum= (uint8_t)strtol(strchr(message, '*')+1, NULL, 16);

    char p;
    uint8_t sum = 0;
    ++message;
    while ((p = *message++) != '*') {
        sum ^= p;
    }

    if (sum != checksum) {
        return 0;
    }

    return 1;
}
int validateGPS(const char * str){
	int GPStyp=0;

	uint8_t checksum = 0;
	    if ((checksum = nmea_valid_checksum(str)) != 1) {
	        return 0;
	    }
	    if(strlen(str)<4){return 0;}

	    if (strstr(str, "GNRMC") != NULL) {
	    	//Debug_Tx(str);
	    	GPStyp=1;getGNRMC(str);
	    }
	    else if (strstr(str, "GNGGA") != NULL) {
	    	//Debug_Tx(str);
	    	GPStyp=2; getGNGGA(str);
	    }
	    else if (strstr(str, "GNGSA") != NULL) {
	    	//Debug_Tx(str);
	    	GPStyp=3;getGNGSA(str);
	    }
	    else if (strstr(str, "GNGSV") != NULL) {
	    	GPStyp=4;
	    }
	    else if (strstr(str, "GNGLL") != NULL) {
	    	GPStyp=5;
	    }
	    else if (strstr(str, "GNVTG") != NULL) {
	    	GPStyp=6;
	    }
    return(GPStyp);
}

float getSpeed(){
	return GPSInfo.speed;
}
void TriggerGPS(){

	memset(gpsData,0,1000);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)gpsData,900);
}


void ProcessGPS(){
	    //Debug_Tx(gpsData);
		int valid=0 ;
		if(strlen((char*)gpsData)>5){

			char *tk= NULL;
			char* token = strtok_r((char*)gpsData, "\r\n", &tk);//strtok(gpsData, "\r\n");//
			while( token != NULL ) {
				valid=valid+validateGPS(token);
				token = strtok_r(NULL, "\r\n", &tk);//strtok(NULL, "\r\n"); //
			}
			//free(token);
		}
}

#endif /* INC_GPS_H_ */
