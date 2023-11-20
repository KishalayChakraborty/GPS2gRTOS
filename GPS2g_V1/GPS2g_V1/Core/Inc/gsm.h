/*
 * gsm.h
 *
 *  Created on: 27-Jun-2021
 *      Author: THE LOGICBOX
 */

#ifndef INC_GSM_H_
#define INC_GSM_H_

// #include "uartFun.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
// #include "FOTA.h"
extern char *GSM_Rx();
extern char *GSM_RxL();
extern void GSM_Tx(const char *);
extern void GSM_TxL(const char *);
extern void Debug_Tx(char *);
extern void SET_LED_NET(int);
extern void SET_GSM_VCC_EN(int);
extern void SET_PWRKEY(int);
extern void restartGSMuart(void);

extern int tic(void);
extern void toc(int tc, char Message[]);
void SendTCPdata();
void ResetTCP();
char GSMInData[1000];
char GSMData[4500];
char GSMDData[3001];
static uint8_t GSMBuff[1];
char GSMDataC[100];
char GSMReply3[500];

char data_LOGIN[100];

int gprsok = 0;
int debug = 1;
char GSMTXC[100];
char GSMTXD[300];
int dnlfile = 0;

char gsminfo[80];
int gsmER = 0;
char GSMReply[100];

#include <errno.h>
int errorlen = 0;
float GSMSignal = 0;
int gpsto_net = 75000;
int gpsto_dev = 400;

char SS0[4], MCC0[5], MNC0[5], LAC0[6], CID0[6];
char SS1[4], LAC1[6], CID1[6];
char SS2[4], LAC2[6], CID2[6];
char SS3[4], LAC3[6], CID3[6];
char SS4[4], LAC4[6], CID4[6];
int GSMProf = 1;//1 airtel, 0 bsnl
int FTPdnS = 0;

char ip[] ="216.10.242.75\",6507"; //"20.210.207.21\",5001";//"216.10.242.75\",6507"; // "20.210.207.21\",5001";//"216.10.242.75\",6507"; //S"20.210.207.21\",5001";//"216.10.242.75\",6507";	 // "20.210.207.21\",5001";//34.74.249.18\",300";216.10.242.75,PORT1-6507
char ip2[] = "";//"20.210.207.21\",5001";//"216.10.242.75\",6507"; // SS"20.210.207.21\",5001";//"216.10.242.75\",6507"; //  "216.10.243.86\",6055";
									 // and port DataString_em1
char DataString_em1[160];
char DataString_em2[160];
int ServerConnected = 0;

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern void flash_erase_pag(uint32_t PageAddress);

static char encoding_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
								'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
								'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
								'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
								'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
								'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
								'w', 'x', 'y', 'z', '0', '1', '2', '3',
								'4', '5', '6', '7', '8', '9', '+', '/'};
static char *decoding_table = NULL;

void build_decoding_table()
{

	decoding_table = malloc(256);

	for (int i = 0; i < 64; i++)
		decoding_table[(unsigned char)encoding_table[i]] = i;
}

uint32_t StartPageAddress = 0x0801E000;
uint32_t Flash_Write_Data(uint64_t Data)
{
	int sofar = 0;
	HAL_FLASH_Unlock();
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, Data) == HAL_OK)
	{
		StartPageAddress += 8; // use StartPageAddress += 2 for half word and 8 for double word
		sofar += 1;
	}
	else
	{
		Debug_Tx("err flash writing");
		return HAL_FLASH_GetError();
	}
	HAL_FLASH_Lock();

	return 0;
}
int Flash_Write(uint8_t *data)
{
	for (int j = 0; j < 2048 / 8; j++)
	{
		uint64_t op = 0;
		for (int i = 0; i < 8; i++)
		{
			op = op << 8 | data[(j * 8) - i + 7];
		}
		Flash_Write_Data(op);
	}
	return 0;
}
int Flash_erase()
{
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);
	for (int i = 60; i <= 63; i++)
	{
		// FLASH_Erase_Sector(i);0x08007000
		// FLASH_PageErase(i);

		uint32_t PAGEerror = 0;

		FLASH_EraseInitTypeDef eraseInitStruct;
		eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
		eraseInitStruct.Page = i;
		eraseInitStruct.NbPages = 1;

		// HAL_FLASH_Unlock();
		if (HAL_FLASHEx_Erase(&eraseInitStruct, &PAGEerror) != HAL_OK)
		{
		}
	}
	HAL_FLASH_Lock();

	return 0;
}

int base64_decode(const uint8_t *data, int loc, size_t input_length)
{

	if (decoding_table == NULL)
		build_decoding_table();

	if (input_length % 4 != 0)
	{
		Debug_Tx("inputlenth err");
		return 0;
	}

	size_t output_length = input_length / 4 * 3;
	if (data[input_length - 1] == '=')
		(output_length)--;
	if (data[input_length - 2] == '=')
		(output_length)--;

	// unsigned char *decoded_data = malloc(output_length);
	uint8_t decoded_data[516];
	if (decoded_data == NULL)
	{
		Debug_Tx("decodeddata err");
		return 0;
	}

	for (int i = 0, j = 0; i < input_length;)
	{

		char sextet_a = data[i] == '=' ? 0 & i++ : decoding_table[data[i++]];
		char sextet_b = data[i] == '=' ? 0 & i++ : decoding_table[data[i++]];
		char sextet_c = data[i] == '=' ? 0 & i++ : decoding_table[data[i++]];
		char sextet_d = data[i] == '=' ? 0 & i++ : decoding_table[data[i++]];

		uint32_t triple = (sextet_a << 3 * 6) + (sextet_b << 2 * 6) + (sextet_c << 1 * 6) + (sextet_d << 0 * 6);

		if (j < output_length)
			decoded_data[j++] = (triple >> 2 * 8) & 0xFF;
		if (j < output_length)
			decoded_data[j++] = (triple >> 1 * 8) & 0xFF;
		if (j < output_length)
			decoded_data[j++] = (triple >> 0 * 8) & 0xFF;
	}

	// Debug_Tx("decoded data");
	// Debug_Tx((char*)decoded_data);
	for (int i = 0; i < 512; i++)
	{
		GSMDData[i + (loc * 512)] = decoded_data[i];
	}
	return 1;
}

void RestartGSM()
{
	SET_GSM_VCC_EN(1);
	SET_PWRKEY(0);
	HAL_Delay(700);
	SET_PWRKEY(1);
	HAL_Delay(500);
	SET_PWRKEY(0);
	HAL_Delay(700);
	SET_GSM_VCC_EN(0);
	HAL_Delay(700);
	SET_PWRKEY(1);
	HAL_Delay(500);
	SET_GSM_VCC_EN(1);
	HAL_Delay(200);
	SET_PWRKEY(0);
}

void SendGSMCode(const char cmd[])
{

	memset(GSMData, 0, 990);
	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
	HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
	memset(GSMTXC, 0, 100);
	if ((strlen(cmd) > 90) & (debug == 1))
	{
		Debug_Tx("Error:GSM Code Length Exceed");
	}
	strcpy(GSMTXC, cmd);
	strcat(GSMTXC, "\r\n");

	// Debug_Tx(GSMTXC);
	GSM_Tx(GSMTXC);
	// return GSM_Rx();
}

int SendGSMCodeFOTA(const char cmd[])
{
	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
	HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
	memset(GSMData, 0, 1000);
	// memset(GSMDData,0,3000);
	memset(GSMTXC, 0, 100);
	if ((strlen(cmd) > 90) & (debug == 1))
	{
		Debug_Tx("Error:GSM Code Length Exceed");
	}
	strcpy(GSMTXC, cmd);
	strcat(GSMTXC, "\r\n");
	dnlfile = 1;
	Debug_Tx("sending gsm download command and waiting ");
	Debug_Tx(GSMTXC);
	GSM_Tx(GSMTXC);
	//
	int try = 20;
	while ((strstr(GSMData, "+QFTPGET:") == NULL) && try > 0)
	{
		HAL_Delay(500);
		try = try - 1;
	}

	HAL_Delay(1000);
	Debug_Tx(GSMData);
	Debug_Tx("Data read loop over");
	if (strstr(GSMData, "+QFTPGET:") == NULL)
	{

		Debug_Tx("Error Incomplete data");
		Debug_Tx(GSMData);
		return 0;
	}

	Debug_Tx("incomplete check over");
	if (strstr(GSMData, "END DOWNLOAD") != NULL)
	{
		if (FTPdnS > 0)
		{
			Debug_Tx(GSMDData);
			Debug_Tx("data writing");
			//Flash_Write((uint8_t *)GSMDData);
			Debug_Tx("data writing complete");
		}

		Debug_Tx("End of file detected ");
		Debug_Tx(GSMData);
		return 2;
	}
	Debug_Tx(GSMData);
	Debug_Tx("complition check over");
	char *pos = strstr(GSMData, "CONNECT");
	pos = pos + 9;
	strcpy(GSMData, pos);
	char *p = strstr(GSMData, "+QFTPGET:");
	if (GSMData[p - GSMData - 2] == '=')
		GSMData[p - GSMData - 1] = 0;
	else
		GSMData[p - GSMData] = 0;

	int j = 0;

	Debug_Tx("loop2");
	for (int i = 0; i < strlen(GSMData); i++)
	{

		// char ee[10];
		// sprintf(ee, "%d,%c,%d", i,GSMData[i],GSMData[i]);
		// Debug_Tx(ee);
		// if(((GSMData[i]>=47)&&(GSMData[i]<=57))||(GSMData[i]==43)||(GSMData[i]==61)||((GSMData[i]>=64)&&(GSMData[i]<=90))){
		if (GSMData[i] != 136)
		{
			GSMData[j] = GSMData[i];
			j = j + 1;
		}
		else
		{
			// Debug_Tx("eeerrr");
		}
	}

	Debug_Tx("loop2 ov");
	GSMData[j] = 0;

	if (FTPdnS == 0)
		memset(GSMDData, 0, 3000);
	char ee[10];
	sprintf(ee, "%d", j - 2);
	Debug_Tx(ee);
	base64_decode((uint8_t *)GSMData, FTPdnS, j - 2);
	if (FTPdnS < 3)
	{
		FTPdnS = FTPdnS + 1;
	}
	else
	{
		Debug_Tx(GSMDData);
		Debug_Tx("data writing");
		//Flash_Write((uint8_t *)GSMDData);
		Debug_Tx("data writing complete");
		FTPdnS = 0;
	}

	dnlfile = 0;

	SendGSMCode(" AT");
	HAL_Delay(1000);
	return 1;
}

void SendGSMCodeD(const char cmd[])
{ // only for sms

	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
	HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
	memset(GSMData, 0, 990);
	GSM_Tx(cmd);
	// return GSM_Rx();
}

void SendGSMCodeL(const char cmd[])
{
	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
	HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);

	memset(GSMData, 0, 990);
	memset(GSMTXC, 0, 100);
	if ((strlen(cmd) > 90) & (debug == 1))
	{
		Debug_Tx("Error:GSM Code Length Exceed");
	}
	strcpy(GSMTXC, cmd);
	strcat(GSMTXC, "\r\n");
	// Debug_Tx(GSMTXC);
	GSM_Tx(GSMTXC);

	// return GSM_RxL();
}

void SendGSMData(const char data[])
{
	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
	HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);

	memset(GSMData, 0, 990);
	uint8_t end[3];
	memset(end, 0, 3);
	end[0] = 0x1A;
	// memset(GSMTXD,0,300);
	// if((strlen(data)>) &(debug==1)){Debug_Tx("Error:GSM Data Length Exceed");}
	// strcpy(GSMTXD,data);
	// strcat(GSMTXD,(char*)end);
	GSM_TxL(data);
	GSM_Tx((char *)end);

	// return GSM_Rx();
}

void EndTransfer()
{

	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
	HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
	memset(GSMData, 0, 990);
	uint8_t end[3];
	memset(end, 0, 3);
	end[0] = 0x1A;
	GSM_Tx((char *)end);
	// return GSM_Rx();
}
int waitForResponse(const char* expectedResponse, int timeout) {
	memset(GSMReply3, 0, 500);
    uint16_t rxBufferIndex = 0;
    int ret=0;

	int stT = HAL_GetTick();
	while (((HAL_GetTick() - stT) <= timeout)){
		//HAL_UART_Receive(&huart1, (uint8_t*)(GSMReply3 + rxBufferIndex), 1, HAL_MAX_DELAY);
        //rxBufferIndex++;
		//strlen(GSMData)
        if (strstr(GSMData, expectedResponse) != NULL) {
        	ret=1;
            break;
        }
    }
	if(ret==0){Debug_Tx("gsm no reply timeout>>");Debug_Tx(GSMData);}
	return ret;

}

char *GetGSMReply(int extra, const char *find, int gap, const char *LineEnd, const char *ErrorMsg, int timeout, const char *finChar)
{
	memset(GSMReply, 0, 100);
	memset(GSMInData, 0, 1000);
	int stT = HAL_GetTick();
	while (((HAL_GetTick() - stT) <= timeout))
	{
		HAL_Delay(70);
		if (strlen(GSMData) > 0)
		{
			strcpy(GSMInData, GSMData);
			char *pq = strstr(GSMInData, finChar);
			if ((pq != NULL))
			{ // && (strlen(strstr(GSMInData,find))>gap)){

				if ((strlen(find) < 1))
				{ // Return for
					strcpy(GSMReply, pq);
					break;
				}
				char *p = strstr(GSMInData, find);
				if ((strlen(LineEnd) < 1))
				{ // Return for
					strcpy(GSMReply, p);
					break;
				}
				else
				{
					if ((strstr(GSMInData, LineEnd) != NULL))
					{
						if ((strlen(p + gap) > strlen(LineEnd)))
						{
							char *p1 = strtok_r(p + gap, LineEnd, NULL); // strtok(GSMData, "\n");strtok(p+gap, );
							if (strlen(p1) < 88)
							{
								strcpy(GSMReply, p1);
								break;
							}
						}
					}
				}
			}
			else
			{
				if ((strstr(GSMInData, "ERROR") != NULL) || (strstr(GSMInData, "FAIL") != NULL))
				{

					Debug_Tx("Err Found");
					break;
				}
			}
			gsmER = 0;
		}
		else
		{
			gsmER++;
			if (gsmER > 9)
			{
				RestartGSM();
				restartGSMuart();
				gsmER = 0;
			}
		}
	}

	// Debug_Tx(GSMInData);
	if (strlen(GSMReply) < 1)
	{
		Debug_Tx("****");
		Debug_Tx(GSMInData);
		Debug_Tx((char *)ErrorMsg);
	}
	else
	{
		// Debug_Tx("****");
		// Debug_Tx(GSMInData);
		// Debug_Tx("****");
		// Debug_Tx(GSMData);
		// Debug_Tx("****");
		// Debug_Tx(GSMReply);
		// Debug_Tx("____returnOK_____");
	}
	HAL_Delay(100);
	// if (strlen(GSMData)<1)restartGSMuart();
	return GSMReply;
}

int GSMSigQuality()
{
	GSMSignal = 0;
	SendGSMCode(" AT+CSQ");
	HAL_Delay(100);
	GSMSignal = strtod(GetGSMReply(0, "+CSQ:", 5, ",", "Error: AT+CSQ GSM Sig Quality", gpsto_dev, ",0"), NULL);

	if (GSMSignal > 5)
	{
		SET_LED_NET(1);
		ServerConnected = 1;
		Debug_Tx("GSM HIGH");
	}
	else
	{
		Debug_Tx("GSM low");
		Debug_Tx("GSM No Signal");
		gprsok = 0;
		ServerConnected = 0;
		if (0)
			RestartGSM();
		restartGSMuart();
		Debug_Tx("GSMTRstarted");
		SET_LED_NET(0);
	}
	return (GSMSignal); // must be higher than 5 ,range 0-33
}

char *GSMSimOperator()
{

	SendGSMCode("  AT+COPS?");
	return (GetGSMReply(0, "+COPS:", 12, "\"", "Error: AT+COPS? sim operator error", 5000, "OK"));
}
char *GSMSimOperator_test()
{

	SendGSMCode("  AT+COPS?");
	return (GetGSMReply(0, "+COPS:", 12, "\"", "Error: AT+COPS? sim operator error", 5000, "OK"));
}

int GSMSimOperatorChkList()
{
	SendGSMCode("  AT+COPS=?");
	char *re = GetGSMReply(0, "+COPS: (", 35, "\"", "Error: AT+COPS=? sim operator List error", gpsto_net, "OK");
	// Debug_Tx(re);
	if ((strstr(re, "Cellone") != NULL))
	{
		Debug_Tx("celone");
		// return(1);
	}
	if ((strstr(re, "airtel") != NULL))
	{
		Debug_Tx("airtel");
		// return(0);
	}

	return (0);
}

char *GSMIMEI()
{
	SendGSMCode(" AT+QGSN");
	return (GetGSMReply(0, "+QGSN:", 8, "\"", "Error: AT+QGSN IMEI Read error", gpsto_dev, "OK"));
}

char *GSMSimNo()
{
	SendGSMCode(" AT+CCID");
	return (GetGSMReply(0, "+CCID:", 8, "\"", "Error: AT+CCID Sim NO Read error", gpsto_dev, "OK"));
}
int DownloadChunkFTP(const char cmd[]){
	Debug_Tx(cmd);
	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF | UART_CLEAR_OREF);
	HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
	memset(GSMData, 0, 4500);
	dnlfile = 1;
	GSM_Tx(cmd);
	HAL_Delay(500);
	int delay1=0;
	while(delay1<100 && strlen(GSMData)<1024){
		HAL_Delay(500);
		delay1=delay1+1;
	}
	HAL_Delay(5000);

	Debug_Tx(GSMData);
}
int DownloadFile()
{
	int rr = 0;
	SendGSMCode(" AT+QIFGCNT=0");
	if (strlen(GetGSMReply(0, "", 0, "", "Error:  AT+QIFGCNT=0 init ftp ", gpsto_net, "OK")) > 0)
	{
		Debug_Tx("*ftp init");
		SendGSMCode(" AT+QFTPCFG=2,0");
		if (strlen(GetGSMReply(0, "", 0, "", "Error:  AT+QFTPCFG set binery mode ", gpsto_net, "+QFTPCFG:0")) > 0)
		{

			//SendGSMCode(" AT+QFTPCFG=1,1");
			//SendGSMCode(" AT+QFTPCFG=6,1");
			Debug_Tx("*set Bin mode");
			//SendGSMCode(" AT+QFTPCFG=\"35.185.111.158\",300,\"uuu\",\"qqqwwweee\"");
			SendGSMCode(" AT+QFTPUSER=\"sammy\"");
			if (waitForResponse("OK",4000)> 0)//if (strlen(GetGSMReply(0, "", 0, "", "Error:  AT+QFTPUSER set ftp user ", gpsto_net, "OK")) > 0)
			{HAL_Delay(2000);

			Debug_Tx(GSMReply);
					Debug_Tx("*set ftp user");HAL_Delay(2000);HAL_Delay(2000);

					Debug_Tx(GSMReply);
				SendGSMCode(" AT+QFTPPASS=\"12345678\"\n");
				if (waitForResponse("OK",6000)> 0)//AT+QFTPCFG
				{ 	Debug_Tx(GSMReply);
					Debug_Tx("*ftp pass");HAL_Delay(2000);
					SendGSMCode(" AT+QFTPPATH=\"/\"");
					if (strlen(GetGSMReply(0, "", 0, "", "Error:  AT+QFTPPATH set savepath ", gpsto_net, "+QFTPPATH: 0")) > 0)
					{
						Debug_Tx("*ftp path");
						SendGSMCode(" AT+QFTPOPEN=\"35.185.111.158\",21");
						if (strlen(GetGSMReply(0, "", 0, "", "Error:  AT+QFTPOPEN open ip port ", gpsto_net, "+QFTPOPEN:0")) > 0)
						{
							Debug_Tx("*ftp portopen");
							Debug_Tx(GSMReply);
							char adrs[40];
							Debug_Tx("**ClearingFlash");
							//Flash_erase();
							//Debug_Tx("**flash clear done");
							Debug_Tx("**get file");

							//SendGSMCode(" AT+QFTPLIST=\"/\"");
							HAL_Delay(2000);
							//Debug_Tx("debug file list");
									//	Debug_Tx(GSMReply);
							rr = 1;
							int g = 0;
							int l = 1;

						int	sp=0			;
						while (sp<10)//g != 2)
							{
								g = 0;
								sprintf(adrs, " len:%d",(sp));

								Debug_Tx("cunk no ##################");
								Debug_Tx(adrs);

								//SendGSMCode(adrs);HAL_Delay(2000);
								int try = 0;
								sprintf(adrs, "  AT+QFTPGET=\"/test%d.txt\"\r\n",sp);//

								DownloadChunkFTP(adrs);


								sp=sp+1;

								//sprintf(adrs, " AT+QFTPGET=\"/bin_%d.bin2en\"", l);//
								/*
								while (g == 0 && try < 2)
								{
									g = SendGSMCodeFOTA(adrs);
									try++;
								}
								if (try >= 2)
								{
									rr = 0;
									break;
								}*/
								l = l + 1;
							}
						}
					}
				}
				Debug_Tx("debug");
				Debug_Tx(GSMReply);
			}Debug_Tx("debug");
			Debug_Tx(GSMReply);
		}
	}
	SendGSMCode(" AT+QFTPCLOSE");
	GetGSMReply(0, "", 0, "", "Error:  AT+QFTPCLOSE ftp closeeror ", gpsto_net, "+QFTPCLOSE");
	SendGSMCode(" AT");
	GetGSMReply(0, "", 0, "", "Error:  AT eror ", gpsto_dev, "OK");
	Debug_Tx("**done**");
	dnlfile = 0;
	// SendGSMCode(" AT+QIFGCNT=0AT+QIOPEN=3,\"TCP\",\"34.74.249.18\",300");

	return rr;
}

int DownloadFileTCP()
{
	/*
	int ck=64*16;
	int dok=1;
	int rret=0;
	SendGSMCode(" AT+QIOPEN=3,\"TCP\",\"34.74.249.18\",300");
	//strcat(GSMDataC,ip);
	//ck=strlen(GetGSMReply(0,"",0,"","Error: AT+QFTPOPEN 0 ftpopen  error ",gpsto_net,"+QFTPOPEN:"));
	Debug_Tx("Starting data receive");
	HAL_Delay(2000);
	Debug_Tx(GSMData);

	int end=0;
	//memset(GSMDData,0,2500);
	while(ck>0){

		SendGSMCode(" AT+QISEND=3");
		gprsok=strlen(GetGSMReply(0,"",0,"","Error: AT+QISEND Send TCP data input",gpsto_net,">"));
		if(dok==1){
		SendGSMData("SendData");//Debug_Tx(GSMData);
		}
		else{
			SendGSMData("ReSend");//Debug_Tx(GSMData);

			Debug_Tx("Resend Data+++++++++++");
		}
		gprsok=strlen(GetGSMReply(0,"",0,"","Error: AT+QISEND Send TCP data",gpsto_net,"SEND OK"));
		HAL_Delay(3000);
		//Debug_Tx("data rcv part done");
		if ((strstr(GSMData,"ENDFile")!= NULL)){ck=0;Debug_Tx("ENDFile");end=1;}
		if ((strstr(GSMData,"CLOSED")!= NULL)){ck=0;Debug_Tx("CLOSED");rret=-1;}
		if ((strstr(GSMData,"ERROR")!= NULL)){ck=0;Debug_Tx("ERROR");rret=-1;}
		if (gprsok==0){ck=0;Debug_Tx("gprsok fail");rret=-1;}
		dok=0;
		int no=0;
		if (end==1 || ck>0){
			char *tk= NULL;


			char* token = strtok_r((char*)GSMData, "@#$%@#", &tk);//strtok(gpsData, "\r\n");//
			while( token != NULL ) {
				if(strlen(token)>0)
				if(no==1){
					//printf("Size of variable a : %d\n",sizeof(token));
					char sss[20];
					memset(sss,0,20);
					sprintf(sss, "size1::%d",strlen(token));
					Debug_Tx(sss);



					if(strlen(token)==128){
						if(strlen(GSMDData)<2000)strcat(GSMDData,token);
						else{
							writeFlashTest(GSMDData);
							memset(sss,0,20);
							sprintf(sss, "size2::%d",strlen(GSMDData));
							Debug_Tx(sss);
							memset(GSMDData,0,2500);
							strcat(GSMDData,token);
						}
						dok=1;
					}
					else{
						Debug_Tx("************************Dataend or error");
						Debug_Tx((char*)GSMData);
					}


				}
				token = strtok_r(NULL, "@#$%@#", &tk);//strtok(NULL, "\r\n"); //
				no=no+1;
			}
		}
		ck=ck-1;

	}
	Debug_Tx("Ending data receive");
	HAL_Delay(3000);
	SendGSMCode("AT+QICLOSE=3");
	//ck=strlen(GetGSMReply(0,"",0,"","Error: AT+QFTPOPEN 0 ftpopen  error ",gpsto_net,"+QFTPOPEN:"));
	HAL_Delay(2000);
	Debug_Tx(GSMData);
	return rret;
*/
	/*SendGSMCode("AT+QFTPGET=\"test.txt\"");
	//ck=strlen(GetGSMReply(0,"",0,"","Error: AT+QFTPOPEN 0 ftpopen  error ",gpsto_net,"+QFTPOPEN:"));
	HAL_Delay(5000);
	Debug_Tx(GSMData);


	SendGSMCode("AT");
	ck=strlen(GetGSMReply(0,"",0,"","Error: AT+QFTPOPEN 0 ftpopen  error ",gpsto_net,"OK"));
*/
	return 0;
}

void StartTCPConnection()
{
	int ck = 1;
	if (strlen(ip) > 4)
	{
		memset(GSMDataC, 0, 100);
		strcpy(GSMDataC, (char *)" AT+QIOPEN=0,\"TCP\",\"");
		strcat(GSMDataC, ip);
		gprsok = 0;
		while ((ck > 0) && (gprsok < 1))
		{
			ck = ck - 1;
			SendGSMCode(GSMDataC);
			// Debug_Tx(GetGSMReply(0,"",0,"","Error: AT+QIOPEN 0 TCP Connection open ",gpsto_net,"CONNECT OK"));

			gprsok = strlen(GetGSMReply(0, "", 0, "", "Error: AT+QIOPEN 0 TCP Connection open ", gpsto_net, "CONNECT OK"));
		}
	}


	HAL_Delay(1500);
	if(strlen(ip2)>4){
		memset(GSMDataC,0,100);
		strcpy(GSMDataC,(char*)" AT+QIOPEN=1,\"TCP\",\"");
		strcat(GSMDataC,ip2);
		ck=1;
		gprsok=0;
		while((ck>0) && (gprsok<1)){
			ck=ck-1;
			SendGSMCode(GSMDataC);
			gprsok=strlen(GetGSMReply(0,"",0,"","Error: AT+QIOPEN 1 TCP Connection open ",gpsto_net,"CONNECT OK"));
		}
	}
	if (gprsok)
	{
		Debug_Tx("connected ip2 ");
		ServerConnected = 1;
	}
	else
		ServerConnected = 0;
	HAL_Delay(1500);
	SendTCPdata(data_LOGIN);
	HAL_Delay(500);
}
void StopTCPConnection()
{
	int ck = 1;
	gprsok = 0;
	while ((ck > 0) && (gprsok < 1))
	{
		ck = ck - 1;
		if (strlen(ip) > 4)
		{
			SendGSMCode(" AT+QICLOSE=0");
			gprsok = strlen(GetGSMReply(5, "", 0, "", "Error: AT+QICLOSE TCP Send Close", gpsto_dev, "CLOSE OK"));
		}
		if (strlen(ip2) > 4)
		{
			SendGSMCode(" AT+QICLOSE=1");
			gprsok = strlen(GetGSMReply(5, "", 0, "", "Error: AT+QICLOSE TCP Send Close", gpsto_dev, "CLOSE OK"));
		}
	}
	ServerConnected = 0;
}
void SendTCPdata(char *data)
{
	int ck = 1;
	if (gprsok > 0)
	{
		//Debug_Tx("GPRSOK");
		if (strlen(ip) > 4)
		{

			//Debug_Tx("IPOK");
			ck = 1;
			gprsok = 0;
			//while ((ck > 0) && (gprsok < 1))
			{
				//Debug_Tx("CONNECTING TO SEND");
				ck = ck - 1;
				SendGSMCode(" AT+QISEND=0");

				gprsok = waitForResponse(">",1000);
				//gprsok = strlen(GetGSMReply(0, "", 0, "", "Error: AT+QISEND Send TCP data input1", 4000, ">"));
			}
			if (gprsok > 0)
			{
				ck = 1;
				gprsok = 0;
				//while ((ck > 0) && (gprsok < 1))
				{
					ck = ck - 1;
					//Debug_Tx("SENDINGDATA");

					SendGSMData(data); // Debug_Tx(GSMData);
					gprsok = waitForResponse("SEND OK",3000);//strlen(GetGSMReply(0, "", 0, "", "Error: AT+QISEND Send TCP data", 10*gpsto_dev, "SEND OK"));

								}
				if (gprsok<1){//SendGSMData("    ");
				Debug_Tx("UNABLE TO11 SEND DATA STOPED CONNECTion");EndTransfer();
}
				else{Debug_Tx("DATASENT");Debug_Tx(data);}

			}
			else
			{
				SendGSMData("    ");
				Debug_Tx("UNABLE TO SEND11 DATA STOPED CONNECTion");
				Debug_Tx(data);
				EndTransfer();

			}
		}
		if (strlen(ip2) > 4)
		{

			//Debug_Tx("IPOK");
			ck = 1;
			gprsok = 0;
			//while ((ck > 0) && (gprsok < 1))
			{
				//Debug_Tx("CONNECTING TO SEND");
				ck = ck - 1;
				SendGSMCode(" AT+QISEND=1");

				gprsok = waitForResponse(">",1000);
				//gprsok = strlen(GetGSMReply(0, "", 0, "", "Error: AT+QISEND Send TCP data input1", 4000, ">"));
			}
			if (gprsok > 0)
			{
				ck = 1;
				gprsok = 0;
				//while ((ck > 0) && (gprsok < 1))
				{
					ck = ck - 1;
					//Debug_Tx("SENDINGDATA");

					SendGSMData(data); // Debug_Tx(GSMData);
					gprsok = waitForResponse("SEND OK",3000);//strlen(GetGSMReply(0, "", 0, "", "Error: AT+QISEND Send TCP data", 10*gpsto_dev, "SEND OK"));

								}
				if (gprsok<1){//SendGSMData("    ");
				Debug_Tx("UNABLE TO22 SEND DATA STOPED CONNECTion");EndTransfer();
}
				else{Debug_Tx("DATASENT");Debug_Tx(data);}

			}
			else
			{
				SendGSMData("    ");
				Debug_Tx("UNABLE TO 22SEND DATA STOPED CONNECTion");
				Debug_Tx(data);
				EndTransfer();

			}

			/*ck = 1;
			gprsok = 0;
			while ((ck > 0) && (gprsok < 1))
			{
				ck = ck - 1;
				SendGSMCode(" AT+QISEND=1");
				gprsok = waitForResponse(">",1000);//gprsok = strlen(GetGSMReply(0, "", 0, "", "Error: AT+QISEND Send TCP data input", gpsto_dev, ">"));
			}
			if (gprsok > 0)
			{
				ck = 1;
				gprsok = 0;
				while ((ck > 0) && (gprsok < 1))
				{
					ck = ck - 1;
					SendGSMData(data); // Debug_Tx(GSMData);
					gprsok = strlen(GetGSMReply(0, "", 0, "", "Error: AT+QISEND Send TCP data", gpsto_dev, "SEND OK"));
				}
			}*/
		}
	}
	else
	{
		Debug_Tx("DISCONNECTED FROM SERVER .resetting connection ");
		ResetTCP();
	}
}

char *GSMCellInfo()
{

	memset(SS0, 0, 4);
	memset(SS1, 0, 4);
	memset(SS2, 0, 4);
	memset(SS3, 0, 4);
	memset(SS4, 0, 4);
	memset(MCC0, 0, 5);
	memset(MNC0, 0, 5);
	memset(LAC0, 0, 6);
	memset(LAC1, 0, 6);
	memset(LAC2, 0, 6);
	memset(LAC3, 0, 6);
	memset(LAC4, 0, 6);
	memset(CID4, 0, 6);
	memset(CID3, 0, 6);
	memset(CID2, 0, 6);
	memset(CID1, 0, 6);
	memset(CID0, 0, 6);
	strcpy(MCC0, "x\0");
	strcpy(MNC0, "x\0");
	strcpy(LAC0, "x\0");
	strcpy(CID0, "x\0");
	strcpy(SS0, "x\0");
	strcpy(SS1, "x\0");
	strcpy(CID1, "x\0");
	strcpy(LAC1, "x\0");

	strcpy(SS2, "x\0");
	strcpy(CID2, "x\0");
	strcpy(LAC2, "x\0");

	strcpy(SS3, "x\0");
	strcpy(CID3, "x\0");
	strcpy(LAC3, "x\0");

	strcpy(SS4, "x\0");
	strcpy(CID4, "x\0");
	strcpy(LAC4, "x\0");

	memset(GSMData, 0, 800);

	SendGSMCodeL(" AT+QENG?");
	// Debug_Tx(GSMData);

	HAL_Delay(500);

	char *m0 = strstr(GSMData, "+QENG: 0");
	if ((m0 != NULL) & (strlen(m0) > 1))
	{

		// Debug_Tx(m0);
		int k = 0;
		char *part;
		while ((part = strtok_r(m0, ",", &m0)))
		{
			if (k == 1)
				strcpy(MCC0, part);
			else if (k == 2)
				strcpy(MNC0, part);
			else if (k == 3)
				strcpy(LAC0, part);
			else if (k == 4)
				strcpy(CID0, part);
			else if (k == 7)
				strcpy(SS0, part);
			k++;
		}
	}
	else
	{
		Debug_Tx("GSM tower data  Signal");
	}
	char *m1 = strstr(GSMData, "+QENG: 1");
	if ((m1 != NULL) & (strlen(m1) > 1))
	{
		// Debug_Tx(m1);
		int k = 0;
		char *part;
		while ((part = strtok_r(m1, ",", &m1)))
		{
			if (k == 3)
				strcpy(SS1, part);
			else if (k == 10)
				strcpy(CID1, part);
			else if (k == 9)
				strcpy(LAC1, part);

			else if (k == 13)
				strcpy(SS2, part);
			else if (k == 20)
				strcpy(CID2, part);
			else if (k == 19)
				strcpy(LAC2, part);

			else if (k == 23)
				strcpy(SS3, part);
			else if (k == 30)
				strcpy(CID3, part);
			else if (k == 29)
				strcpy(LAC3, part);

			else if (k == 33)
				strcpy(SS4, part);
			else if (k == 40)
				strcpy(CID4, part);
			else if (k == 39)
				strcpy(LAC4, part);
			k++;
		}
	}

	//

	memset(gsminfo, 0, 80);
	sprintf(gsminfo, "%d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
			(int)GSMSignal, MCC0, MNC0, LAC0, CID0, CID1, LAC1, SS1, CID2, LAC2, SS2, CID3, LAC3, SS3, CID4, LAC4, SS4);
	// Debug_Tx(gsminfo);

	return (gsminfo);
}

/* no use
char* GSMSimRegNet(){
	strcpy(GSMData,SendGSMCode(" AT+COPS?"));
	return(GSMData);
}


char* GSMCellEnv(){
	strcpy(GSMData,SendGSMCode(" AT+CCED"));
	return(GSMData);
}
*/

char *SetTCPMux()
{
	SendGSMCode(" AT+QIMUX=1");
	return (GetGSMReply(0, "", 0, "", "Error: AT+QIMUX=1 set tcpMux", gpsto_dev, "OK"));
}

void ResetTCP()
{
	//EndTransfer();
	StopTCPConnection();
	StartTCPConnection();
}

void ProcessTCPAll(char *data)
{
	int tcpSENDDATA = tic();

	if (ServerConnected > 0)
	{
		//$AS01FFA0138,$123456789012345,$1.0.4,$1.0.0,28.609803N077.103198E,F7,*

		Debug_Tx("sending data to ip ");
		Debug_Tx(ip2);
		// SendTCPdata(data_LOGIN);
		SendTCPdata(data);
		Debug_Tx("dat send done  ");
	}
	else
	{
		if (debug == 1)
		{
			Debug_Tx("Error: ServerSession disconnected ");
		}

		Debug_Tx("error insending data to ip ");
		Debug_Tx(ip2);
		//ResetTCP();
	}

	toc(tcpSENDDATA, "_________________________TCP SEND DATA");
}

int sendSMS(char *no, char *data)
{
	SendGSMCodeD("AT+CMGS=\"");
	SendGSMCodeD(no);
	SendGSMCodeD("\"\r");

	int smsok = strlen(GetGSMReply(0, "", 0, "", "Error: AT+CMGS send SMS", gpsto_dev, ">"));
	if (smsok > 0)
	{
		SendGSMCodeD(data);
		SendGSMCodeD("\x1A");
		smsok = strlen(GetGSMReply(0, "", 0, "", "Error: AT+CMGS send SMS", gpsto_dev, ">"));
	}
	return smsok;
}

void SetupGPRS(char *apn)
{

	int ck = 5;

	gprsok = 0;
	while ((ck > 0) && (gprsok < 1))
	{
		HAL_Delay(2000);
		ck = ck - 1;
		// SendGSMCode(" AT+CGACT=0,1");
		// gprsok=strlen(GetGSMReply(0,"OK",0,"","Error: AT+CGACT=0,1 GPRS Setup",500));
		memset(GSMDataC, 0, 100);
		strcpy(GSMDataC, (char *)" AT+QICSGP=1,\"");
		strcat(GSMDataC, apn);
		strcat(GSMDataC, "\",\"\",\"\",0");
		SendGSMCode(GSMDataC);
		gprsok = strlen(GetGSMReply(0, "", 0, "", "Error: AT+QICSGP=1 APN Setup", gpsto_dev, "OK"));

		// gprsok=strlen(GetGSMReply(0,"OK",0,"","Error: AT+CGACT=0,1 GPRS Setup",10));
		if (gprsok > 0)
		{
			// gprsok=0;
			// HAL_Delay(4000);

			SendGSMCode(" AT+QIMODE=0");
			gprsok = strlen(GetGSMReply(0, "", 0, "", "Error: QIMODE non transperent mode", gpsto_dev, "OK"));
			// SendGSMCode(" AT+CGATT=0");
			// gprsok=strlen(GetGSMReply(0,"OK",0,"","Error: AT+CGATT=1 GPRS Setup",100));

			// SendGSMCode(" AT+CGATT=1");
			// gprsok=strlen(GetGSMReply(0,"OK",0,"","Error: AT+CGATT=1 GPRS Setup",200));
			if (gprsok > 0)
			{
				// HAL_Delay(1000);

				// SendGSMCode(" AT+QIDNSIP=1");
				// gprsok=strlen(GetGSMReply(0,"OK",0,"","Error: AT+QIDNSIP=1 GPRS Setup",10));

				if (gprsok > 0)
				{
					// HAL_Delay(1000);
				}
			}
		}
	}
}

void ShiftGSMProfile()
{
	// RestartGSM();
	int ook;
	Debug_Tx("______________Begin_____________________________");
	// HAL_Delay(5000);

	// Debug_Tx("______________print operator_____________________________");
	// Debug_Tx(GSMSimOperator());

	// Debug_Tx("______________setup stk_____________________________");
	int ck = 10;
	ook = 0;
	while ((ck > 0) && (ook < 1))
	{
		HAL_Delay(3000);
		ck = ck - 1;
		SendGSMCode(" AT+QSTK=1");
		ook = strlen(GetGSMReply(0, "", 0, "", "Error: AT+QSTK=1 Setup STK", gpsto_net, "OK"));
		if (ook > 0)
		{
			Debug_Tx("______________Setup Menu_____________________________");
			ook = 0;
			SendGSMCode(" AT+STKTR=\"810301250082028281830100\"");
			ook = strlen(GetGSMReply(0, "", 0, "", "Error: AT+STKTR= Setup Menue", gpsto_net, "OK"));
			HAL_Delay(3000);
			if (ook > 0)
			{
				Debug_Tx("______________FOR PROFILE CONFIG_____________________________");
				ook = 0;
				SendGSMCode(" AT+STKENV=\"D30782020181900101\"");
				ook = strlen(GetGSMReply(0, "", 0, "", "Error: AT+STKENV= Profile COnfig", gpsto_net, "OK"));
				if (ook > 0)
				{

					if (GSMProf == 0)
					{
						Debug_Tx("_____________BSNL profile Selection_____________________________");
						ook = 0;
						SendGSMCode(" AT+STKTR=\"810301240082028281830100900102\"");
						ook = strlen(GetGSMReply(0, "", 0, "", "Error:  AT+STKTR=BSNL Config", gpsto_net, "OK"));
						GSMProf = 1;
					}
					else if (GSMProf == 1)
					{
						Debug_Tx("_____________Airtel profile Selection_____________________________");

						ook = 0;
						SendGSMCode(" AT+STKTR=\"810301240082028281830100900101\"");
						ook = strlen(GetGSMReply(0, "", 0, "", "Error:  AT+STKTR=AIRTEL Config", gpsto_net, "OK"));
						GSMProf = 0;
					}
					if (ook > 0)
					{
						Debug_Tx("_____________Refresh_____________________________");
						ook = 0;
						SendGSMCode(" AT+STKTR=\"810301010482028281830100\"");
						ook = strlen(GetGSMReply(0, "", 0, "", "Error:  AT+STKTR Refresh", gpsto_net, "OK"));
					}
				}
			}
		}
	}

	Debug_Tx("_____________Restart_____________________________");

	ck = 2;
	ook = 0;
	while ((ck > 0) && (ook < 1))
	{
		ck = ck - 1;
		SendGSMCode(" AT+QPOWD=1");
		ook = strlen(GetGSMReply(15, "", 0, "", "Error:  AT+QPOWD=1 Restart", gpsto_dev, "NORMAL POWER DOWN"));
	}
	HAL_Delay(6000);
	if (ook > 0)
	{
	}

	// HAL_Delay(5000);
}

void InitGSM()
{
	// RestartGSM();
	HAL_Delay(5000);

	int i = 0;
	int ck = 10;
	// SendGSMCode(" AT+QPOWD=1");
	// i=strlen(GetGSMReply(0,"NORMAL POWER DOWN",0,"","Error:  AT+QPOWD=1 Restart",20));

	// HAL_Delay(10000);
	ck = 2;
	i = 0;
	while ((ck > 0) && (i < 1))
	{
		ck = ck - 1;
		SendGSMCode(" ATE0");
		i = strlen(GetGSMReply(15, "", 0, "", "Error: ATE no Resp", gpsto_dev, "OK"));
	}
	ck = 2;
	i = 0;
	while ((ck > 0) && (i < 1))
	{
		ck = ck - 1;
		SendGSMCode(" AT");
		i = strlen(GetGSMReply(15, "", 0, "", "Error: AT no Resp", gpsto_dev, "OK"));
	}

	HAL_Delay(2000);
	ShiftGSMProfile();

	// HAL_Delay(5000);
	ck = 2;
	i = 0;
	while ((ck > 0) && (i < 1))
	{
		ck = ck - 1;
		SendGSMCode(" ATE0");
		i = strlen(GetGSMReply(10, "", 0, "", "Error: ATe no Resp", gpsto_dev, "OK"));
	}
	ck = 2;
	i = 0;
	while ((ck > 0) && (i < 1))
	{
		ck = ck - 1;
		SendGSMCode(" AT");
		i = strlen(GetGSMReply(10, "", 0, "", "Error: AT no Resp", gpsto_dev, "OK"));
	}

	// SendGSMCode(" AT+CRES");
	// SendGSMCode(" AT+COLP=1"); //Connected Line Identification Presentation
	// i=strlen(GetGSMReply(0,"OK",0,"","Error: AT+COLP=1 Connected  Line Identification ",10));
	// SendGSMCode(" AT+CSCA=\"+919810051914\",145");	//+CSCA: "+919810051914",145 //+919818023015

	ck = 10;
	i = 0;
	while ((ck > 0) && (i < 1))
	{
		ck = ck - 1;
		SendGSMCode(" AT+IFC=1,1"); // Set TE-TA Control Character Framing
		i = strlen(GetGSMReply(5, "", 0, "", "Error: AT+IFC=1,1 Software Flow COntrol", gpsto_dev, "OK"));
		// i=strlen(GetGSMReply(0,"OK",0,"","Error: AT+IFC=1,1 Software Flow COntrol",50));
		HAL_Delay(1000);
		if (i > 0)
		{
			SendGSMCode(" AT+CFUN=1"); // Select sms format
			i = strlen(GetGSMReply(0, "", 0, "", "Error: AT+CFUN=1 set gsm full function", gpsto_dev, "OK"));

			if (i > 0)
			{
				SendGSMCode(" AT+CMGF=1"); // Select sms format
				i = strlen(GetGSMReply(0, "", 0, "", "Error: AT+CMGF=1 SMS Mode", gpsto_dev, "OK"));

				if (i > 0)
				{
					SendGSMCode(" AT+CLIP=1"); // Calling Line Identification Presantation
					i = strlen(GetGSMReply(0, "", 0, "", "Error: AT+CLIP=1 Calling Line Identification", gpsto_dev, "OK"));
					if (i > 0)
					{
						SendGSMCode(" AT+CSCS=\"GSM\""); // Select TE Character Set
						i = strlen(GetGSMReply(0, "", 0, "", "Error: AT+CSCS=\"GSM\"  SMS TE charecter set ", gpsto_dev, "OK"));
						if (i > 0)
						{
							HAL_Delay(3000);
							// SendGSMCode(" AT+CSMP=17,167,0,16");
							// i=strlen(GetGSMReply(0,"OK",0,"","Error: AT+CSMP=17,167,0,16  SMS Text mode parameter ",10));
							if (i > 0)
							{
								SendGSMCode(" AT+QENG=1,4");
								i = strlen(GetGSMReply(0, "", 0, "", "Error: QENG=1,4 set eng mode for info ", gpsto_dev, "OK"));

								if (i > 0)
								{

									HAL_Delay(1000);
									if (GSMProf == 0)
									{
										Debug_Tx("_____________AIRTEL APN SET_____________________________");
										SetupGPRS("taisysnet"); // taisysnet");
									}
									else if (GSMProf == 1)
									{
										Debug_Tx("_____________BSNL APN SET_____________________________");
										SetupGPRS("bsnlnet"); // taisysnet");
									}
									HAL_Delay(1000);

									Debug_Tx("_____________Operator again_____________________________");
									// Debug_Tx(GSMSimOperator());
									HAL_Delay(1000);
									SetTCPMux();
									HAL_Delay(1000);
									SetTCPMux();
								}
							}
						}
					}
				}
			}
		}
	}
}

#endif /* INC_GSM_H_ */
