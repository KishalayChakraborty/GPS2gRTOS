/*
 * runtime.h
 *
 *  Created on: 09-Nov-2021
 *      Author: THE LOGICBOX
 */

#ifndef RUNTIME_H_
#define RUNTIME_H_

	int Dig_in[3];
	char Dig_io[30];
    int Digout1=0, Digout2=0, MAINS_STATE=0, ACC_STATE=0,SOS_STATE=0;
	char BOX_STATE='O';
    float adc[2];
	float EXT_B=0,INT_B=0;
	char InitStr[]="$,E,ATMV\0"; //   "$,T,TGBT\0"
	char VerStr[]="1.0.0\0";
	char AlartStr_NormalPkt[]=",NR,01\0";
	char AlartStr_HistoryPVTData[]=",NR,02\0";
	char AlartStr_MainBatteryDisconnect[]=",BD,03\0";
	char AlartStr_MainBatteryReconnect[]=",BR,06\0";
	char AlartStr_InternalBatterLow[]=",BL,04\0";
	char AlartStr_InternalBatteryChargedAgain[]=",BH,05\0";
	char AlartStr_BoxTemper[]=",TA,09\0";
	char AlartStr_IgnitionTurnedON[]=",IN,07\0";
	char AlartStr_IgnitionTurnedOFF[]=",IF,08\0";
	char AlartStr_EmergencyStateON[]=",EA,10\0";
	char AlartStr_EmergencyStateOFF[]=",EA,11\0";
	char AlartStr_HarshAcceleration[]=",HA,14\0";
	char AlartStr_HarshBreaking[]=",HB,13\0";
	char AlartStr_RashTurning[]=",RT,15\0";
	char AlartStr_OTAParameterChange[]=",CC,12\0";
	char AlartStr_HealthPacket[]=",HP,00\0";
	char AlartStr_EmergencyWireBreak[]=",DT,16\0";
	char AlartStr_OverSpeed[]=",OS,17\0";
	char PacketStatusStrLive[]=",L\0";


	const char PacketStatusStrHist[]=",H\0";
	char Head[100];



	char StatusStrng[20];

	int HistoryPVTData=0;
		int HealthPacket=0;
		int OTAParameterChange=0;

		int EmergencyStateON=-1;
		int EmergencyStateOFF=0;
		int IgnitionTurnedON=0;
		int IgnitionTurnedOFF=0;
		int InternalBatterLow=0;
		int InternalBatteryChargedAgain=0;
		int MainBatteryDisconnect=0;
		int MainBatteryReconnect=0;
		int EmergencyWireBreak=0;

		char checksum[3];

#endif /* RUNTIME_H_ */
