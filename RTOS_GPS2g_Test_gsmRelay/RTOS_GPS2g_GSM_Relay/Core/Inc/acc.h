/*
 * acc.h
 *
 *  Created on: 13-Jul-2021
 *      Author: THE LOGICBOX
 */

#ifndef INC_ACC_H_
#define INC_ACC_H_

/* Base I2C and SPI Function Prototypes-----------------*/
static void initAcc(void);
static float* readAcc(void);


/* Variables for Acc */
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;
float ACC_GYRO_data[6];
/* Variables for Acc */


char dummy[250];
/* Constants for Acc*/
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

/* Constants for Acc*/


static void initAcc(){
	//https://controllerstech.com/how-to-interface-mpu6050-gy-521-with-stm32/#google_vignette

	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 ->  2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 ->  250 /s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
		Debug_Tx("ACC Interface OK");
	}
	else{
		Data = 0;
				HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

				// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
				Data = 0x07;
				HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

				// Set accelerometer configuration in ACCEL_CONFIG Register
				// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 ->  2g
				Data = 0x00;
				HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

				// Set Gyroscopic configuration in GYRO_CONFIG Register
				// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 ->  250 /s
				Data = 0x00;
				HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
		Debug_Tx("ACC Interface ");
	}

	ACC_GYRO_data[0]=0;
	ACC_GYRO_data[1]=0;
	ACC_GYRO_data[2]=0;
	ACC_GYRO_data[3]=0;
	ACC_GYRO_data[4]=0;
	ACC_GYRO_data[5]=0;

}





static float* readAcc(){

	ACC_GYRO_data[0]=0;
	ACC_GYRO_data[1]=0;
	ACC_GYRO_data[2]=0;
	ACC_GYRO_data[3]=0;
	ACC_GYRO_data[4]=0;
	ACC_GYRO_data[5]=0;
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;


	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (�/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
	ACC_GYRO_data[0]=Ax;
	ACC_GYRO_data[1]=Ay;
	ACC_GYRO_data[2]=Az;
	ACC_GYRO_data[3]=Gx;
	ACC_GYRO_data[4]=Gy;
	ACC_GYRO_data[5]=Gz;
	return(ACC_GYRO_data);

	//sprintf (buf, "%.2f", Ax);
}


int detectAcc(){
	readAcc();
	//sprintf(dummy, "Ax=%d;Ay=%d;Az=%d;    Gx=%d;Gy=%d;Gz=%d;", (int) ((ACC_GYRO_data[0])*1000),(int)((ACC_GYRO_data[1])*1000),(int) ((ACC_GYRO_data[2])*1000),(int)((ACC_GYRO_data[3])*1000), (int)((ACC_GYRO_data[4])*1000),(int)((ACC_GYRO_data[5])*1000));
	//Debug_Tx(dummy);
	/*
	 *
	else if(AccGyroStatus==1){
		strcat(Head,AlartStr_HarshAcceleration);
	}

	else if(AccGyroStatus==2){
		strcat(Head,AlartStr_HarshBreaking);
	}
	else if(AccGyroStatus==3){
		strcat(Head,AlartStr_RashTurning);
	}*/

	if((ACC_GYRO_data[0]+ACC_GYRO_data[1]+ACC_GYRO_data[2])>2000){return 1;}
	if((ACC_GYRO_data[0]+ACC_GYRO_data[1]+ACC_GYRO_data[2])<-2000){return 2;}
	if((ACC_GYRO_data[3]+ACC_GYRO_data[4]+ACC_GYRO_data[5])>2000){return 3;}


	return 0;
}


char* detectAccStr(){
	readAcc();
	sprintf(dummy, "Ax=%d;Ay=%d;Az=%d;    Gx=%d;Gy=%d;Gz=%d;", (int) ((ACC_GYRO_data[0])*1000),(int)((ACC_GYRO_data[1])*1000),(int) ((ACC_GYRO_data[2])*1000),(int)((ACC_GYRO_data[3])*1000), (int)((ACC_GYRO_data[4])*1000),(int)((ACC_GYRO_data[5])*1000));
		//Debug_Tx(dummy);

	return dummy;
}



#endif /* INC_ACC_H_ */
