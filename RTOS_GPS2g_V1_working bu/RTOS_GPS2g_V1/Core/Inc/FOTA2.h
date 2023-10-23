/*
 * Flash.h
 *
 *  Created on: 26-Nov-2021
 *      Author: THE LOGICBOX
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_


#include <stdint.h>
#include "stm32g0xx_hal.h"
#define STM32_FLASH_BASE            ((uint32_t)0x0801F000) /*!< FLASH base address in the alias region */


#define STM_SECTOR_SIZE 2048
#define FLASH_WAITETIME	50000

typedef   signed          char int8_t;  //  The standard expression sign char is equivalent to int8_t.
typedef   signed short int int16_t;
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


#define   __I     volatile const

#define     __O     volatile
#define     __IO    volatile




typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;
typedef const int16_t sc16;
typedef const int8_t sc8;

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;
typedef __I int16_t vsc16;
typedef __I int8_t vsc8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;
typedef const uint16_t uc16;
typedef const uint8_t uc8;

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;
typedef __I uint16_t vuc16;
typedef __I uint8_t vuc8;



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

























void flash_initialize();
void flash_deinitialize();
void flash_erase(uint8_t sector);
uint8_t flash_write(volatile uint32_t* address, uint32_t *data, uint16_t size);



uint32_t FWAdd =0x0801F000;




void flash_page_erase4(uint32_t addrx){
	Debug_Tx("erase 11");
	if(STMFLASH_ReadWord(addrx)==0XFFFFFFFF) return;

HAL_Delay(2000);
HAL_FLASH_Unlock();
HAL_Delay(2000);
__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP |FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR|FLASH_CR_PER);

HAL_Delay(2000);
FLASH_PageErase(addrx/2048);
HAL_Delay(2000);
//FLASH_Erase_Sector(FLASH_SECTOR_3, VOLTAGE_RANGE_3);
HAL_Delay(2000);
HAL_FLASH_Lock();
HAL_Delay(2000);
return ;
    HAL_FLASH_Lock();
    Debug_Tx("erase 21");
    HAL_StatusTypeDef test = HAL_FLASH_Unlock();

    Debug_Tx("erase 1");
    //test = FLASH_WaitForLastOperation(1000); //1s timeout
    HAL_Delay(2000);
    Debug_Tx("erase 2");
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP |FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR|FLASH_CR_PER);

    __disable_irq();
    Debug_Tx("erase 3");
    FLASH_PageErase(62);
    __enable_irq();
    Debug_Tx("erase 4");
    HAL_Delay (2000);
    //test = FLASH_WaitForLastOperation(1000);

    Debug_Tx("erase 5");
    HAL_FLASH_Lock();
    Debug_Tx("erase done");


}



#define STM32_FLASH_BASE            ((uint32_t)0x0801F000) /*!< FLASH base address in the alias region */

	void flash_initialize();
	void flash_deinitialize();
	void flash_erase(uint8_t sector);

	void FLASH_PageErase2(uint32_t PageAddress)
	{
	  /* Clean the error context */
	  pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;

	    //add
	     /* Clear pending flags (if any) */
	//  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
	    //add end

	    /* Proceed to erase the page */
	    SET_BIT(FLASH->CR, FLASH_CR_PER);
	    WRITE_REG(FLASH->ACR, PageAddress);
	    SET_BIT(FLASH->CR, FLASH_CR_STRT);

	    //add
	    FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
	  CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	    //add end
	}
	uint8_t flash_write(volatile uint32_t* address, uint32_t *data, uint16_t size);

	void flash_initialize()
	{
		HAL_FLASH_Unlock();
	}

	void flash_deinitialize()
	{
		HAL_FLASH_Lock();
	}

	u32 STMFLASH_ReadWord(u32 faddr)
	{
		return *(vu32 *)faddr;
	}


	void STMFLASH_Erase8(u32 WriteAddr,u32 NumToWrite)
	{
		FLASH_EraseInitTypeDef FlashEraseInit;
		//HAL_StatusTypeDef FlashStatus=HAL_OK;
		u32 PageError=0;
		u32 addrx=0;
		u32 endaddr=0;
		if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//Illegal address

		HAL_FLASH_Unlock();             //Unlock
		addrx=WriteAddr;				//The starting address of writing
		endaddr=WriteAddr+NumToWrite*8;	//The end address of writing
		if(addrx<0X1FFF0000)
		{
			FlashEraseInit.TypeErase=FLASH_TYPEERASE_PAGES;    //Erase type, page erase
			FlashEraseInit.Page=addrx/2048;//STMFLASH_GetFlashPage(addrx);   						   //From which page to erase
			FlashEraseInit.NbPages=1;                          //Only erase one page at a time
			if(HAL_FLASHEx_Erase(&FlashEraseInit,&PageError)!=HAL_OK)Debug_Tx("Error in erase");
			FLASH_WaitForLastOperation(FLASH_WAITETIME);
		}
		//FLASH_WaitForLastOperation(FLASH_WAITETIME);        //Wait for the completion of the last operation

		HAL_FLASH_Lock();

	}

	void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)
	{
		FLASH_EraseInitTypeDef FlashEraseInit;
		HAL_StatusTypeDef FlashStatus=HAL_OK;
		u32 PageError=0;
		u32 addrx=0;
		u32 endaddr=0;
		if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//Illegal address

		//HAL_FLASH_Unlock();             //Unlock
		addrx=WriteAddr;				//The starting address of writing
		endaddr=WriteAddr+NumToWrite*8;	//The end address of writing

		if(FlashStatus==HAL_OK)
		{
			 while(WriteAddr<endaddr)									//Write data
			 {
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,WriteAddr,*(uint64_t*) pBuffer)!=HAL_OK)//data input
				{
					break;												//Write exception
				}
				WriteAddr+=8;
				pBuffer+=2;
			}
		}
		//          									//Locked
	}

	void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   		//Continuous reading
	{
		u32 i;
		for(i=0;i<NumToRead;i++)
		{
			pBuffer[i]=STMFLASH_ReadWord(ReadAddr);	//Read 4 bytes.
			ReadAddr+=4;												//Offset 4 bytes.
		}
	}



#define FW_ADDR    0x0800F000


static inline __attribute__((always_inline))  runapp(){
	SCB -> VTOR = FW_ADDR;
	SYSCFG->CFGR1 = 0x01;
	 __disable_irq();
    __set_MSP(*(volatile uint32_t *)FW_ADDR);

    //__asm__ volatile("msr msp, %0"::"g"(*(volatile uint32_t *)FW_ADDR))

    void (*start)(void) = (void *)*(volatile uint32_t *)(FW_ADDR + 4);
    start();


}

void WriteFile(char* buf){
	//__enable_irq ();
	//flash_initialize();
	STMFLASH_Write((uint32_t)FWAdd,(u32*)buf,sizeof(buf)/8);
	FWAdd=FWAdd+sizeof(buf)/8;
	//flash_deinitialize();
}

void STMFLASH_Erase(u32 WriteAddr,u32 NumToWrite)
{
	FLASH_EraseInitTypeDef FlashEraseInit;
	//HAL_StatusTypeDef FlashStatus=HAL_OK;
	u32 PageError=0;
	u32 addrx=0;
	u32 endaddr=0;
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//Illegal address

	HAL_FLASH_Unlock();             //Unlock
	addrx=WriteAddr;				//The starting address of writing
	endaddr=WriteAddr+NumToWrite*8;	//The end address of writing
	if(addrx<0X1FFF0000)
	{
		while(addrx<endaddr)		//Clear all obstacles. (For non-FFFFFFFF, erase first)
		{
			 if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)	//There is a place other than 0XFFFFFFFF, to erase this sector
			 {
				FlashEraseInit.TypeErase=FLASH_TYPEERASE_PAGES;    //Erase type, page erase
				FlashEraseInit.Page=addrx/2048;//STMFLASH_GetFlashPage(addrx);   						   //From which page to erase
				FlashEraseInit.NbPages=1;                          //Only erase one page at a time
				//FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;
				if(HAL_FLASHEx_Erase(&FlashEraseInit,&PageError)!=HAL_OK)
				{
					break;//An error has occurred
				}
			}else addrx+=4;
			FLASH_WaitForLastOperation(FLASH_WAITETIME);            //Wait for the completion of the last operation
		}
	}
	FLASH_WaitForLastOperation(FLASH_WAITETIME);        //Wait for the completion of the last operation

	HAL_FLASH_Lock();

}

void JumpToFw(void){
__enable_irq ();
/* USER CODE BEGIN 2 */
//int aa[4]={1,2,3,4};
//STMFLASH_Write((uint32_t)0x0801F000,(u32*)aa,sizeof(aa)/8);
/* USER CODE END 2 */

// start_app((uint32_t)0x08010001,(uint32_t)0x08010000);
//asm("msr msp, %0; bx %1;" : : "r"((uint32_t)0x08010000), "r"((uint32_t)0x08010000));
//runapp();
}




#endif /* INC_FLASH_H_ */
