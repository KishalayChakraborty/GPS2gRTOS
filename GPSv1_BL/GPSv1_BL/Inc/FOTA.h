/*
 * FOTA.h
 *
 *  Created on: 02-Dec-2021
 *      Author: THE LOGICBOX
 */

#ifndef INC_FOTA_H_
#define INC_FOTA_H_

//extern void FLASH_PageErase(uint32_t PageAddress);

//uint64_t writeFlashData = 0x1234567890UL;
uint32_t addr = 0x0800F000;



extern void Debug_Tx(char*);





void flash_write(uint32_t address, uint64_t data)

{
	//Debug_Tx("fr1");
    HAL_FLASH_Unlock();
    //Debug_Tx("fr2");
    HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, address , (uint64_t)data);
   // Debug_Tx("fr3");
    HAL_FLASH_Lock();
   // Debug_Tx("fr4");
}



//FLASH write data test
void writeFlashTest(char * data){
	 __disable_irq();
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR);

    Debug_Tx("write");
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);


	int ln=strlen(data);
	for(int i=0;i<ln/2048;i++){
		int x=i+(addr/2048);
		if(x<=63)FLASH_PageErase(x);
		else break;
	}



    //Debug_Tx("write1");
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

    //Debug_Tx("write2");
    HAL_FLASH_Lock();

   // Debug_Tx("write3");
    int j=0;
    uint64_t tem=0;
    while(ln>j){
    	tem=0;
    	for(int k=0;k<8;k++){
    		if((j)+k<ln)tem = (tem << 8) | data[k+(j)];
    		else tem = (tem << 8) | 0;
    	}
    	//Debug_Tx(data);
    	flash_write(addr, tem);

    	addr=addr+8;
    	j=j+8;
    	//ln=ln-8;

        //Debug_Tx("write4");
    }

    //Debug_Tx("write5");
	 __enable_irq();
	 addr=addr+2048;

}




void writeTest(void)
{
	HAL_Delay (1000);
	writeFlashTest("123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765123456788765432112345678876512345678876543211234567887651234567887654321123456788765");

}


#endif /* INC_FOTA_H_ */