/**********************************Mecono.CN***********************************/
/*                                                                            */
/*  @File Name : APP_W25Q128.h                                                */
/*  @Author    : Mecono                                                       */
/*  @version   : V0.02                                                        */
/*  @Date      : 2016-03-28                                                   */
/*  @attention : Flash W25Q128 Ó¦ÓÃº¯Êý                                       */
/*                                                                            */
/******************************************************************************/
#ifndef _APP_W25Q128_
#define _APP_W25Q128_

#include "DRI_SPI.h"
#include "stdio.h"
#include "string.h"

/* Results of Disk Functions */
typedef enum {
    FLASH_OK = 0,         /* 0: Successful */
    FLASH_ERROR = 1,      /* 1: R/W Error */
    FLASH_WRPRT,          /* 2: Write Protected */
    FLASH_BUSY,           /* 3: BUSY */
    FLASH_PARERR          /* 4: Invalid Parameter */
} Flash_STATUS;
/*  Used the 4K Sector Size */
/*  1 4K size Sector        */
/*  0 512 size Secotr       */
#define SECTOR_4K 0

/*  Flash JEDEC ID  */
#define JEDEC_ID 0xEF4018

/*  Flash CMD       */
#define Write_Enable     0x06
#define Write_Disable    0x04
#define Read_Status_Reg  0x05
#define Wirte_Status_Reg 0x01
#define Read_Data        0x03
#define Fast_Read        0x0B
#define Fast_Read_Double_Out    0x3B
#define Page_Program     0x02
#define Block_Erase      0xD8
#define Sector_Erase     0x20
#define Chip_Erase       0xC7
#define Power_down       0xB9
#define Release_Power_down      0xAB
#define Device_ID        0x90

#define FLASH_PAGE_SIZE         256
#if SECTOR_4K
#define FLASH_SECTOR_SIZE       4096
#define FLASH_SECTOR_COUNT      4096
#else
#define FLASH_SECTOR_SIZE       512
#define FLASH_SECTOR_COUNT      0x8000
#endif
#define FLASH_BLOCK_SIZE        65536
#define FLASH_BLCOK_COUNT       256
#define FLASH_PAGES_PER_SECTOR  FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE


//Flash Hold Pin PC15
/*  Flash HOLD ON  */
#define Flash_HOLD_LOW()  HAL_GPIO_WritePin(GPIOC, FLASH_HOLD_Pin, \
                                                   GPIO_PIN_RESET)

/*  Flash HOLD OFF */
#define Flash_HOLD_HIGH()  HAL_GPIO_WritePin(GPIOC, FLASH_HOLD_Pin, \
                                                    GPIO_PIN_SET)

/*  W25X40_Init     :                                                         */
Flash_STATUS W25X40_Init(void);
/*  W25X40_WriteEN  : Flash write Enable                                      */
void W25X40_WriteEN(void);
/*  W25X40_WriteAddress  : Flash write address                                */
Flash_STATUS W25X40_WriteAddress(uint32_t address);
/*  W25X40_WriteEN  : Flash write Disable                                     */
void W25X40_WriteDis(void);
/*  W25X40_WriteEN  : Flash read                                              */
Flash_STATUS W25X40_Read(uint32_t address , uint16_t len , uint8_t *buffer);
/*  W25X40_WriteEN  : Flash write                                             */
Flash_STATUS W25X40_Write(uint32_t address , uint16_t len , uint8_t *buffer);
/*  W25X40_WriteEN  : Flash wait busy                                         */
Flash_STATUS W25X40_Wait(void);
/*  W25X40_WriteEN  : Flash Sector Erase                                      */
Flash_STATUS W25X40_Sector_Erase(uint32_t address);
/*  W25X40_WriteEN  : Flash Block Erase                                       */
Flash_STATUS W25X40_Block_Erase(uint32_t address);
/*  W25X40_WriteEN  : Flash Sector Wirte                                      */
Flash_STATUS W25X40_Sector_Write(uint32_t address , uint8_t *buffer);
/*  W25X40_WriteEN  : Flash Sector Read                                       */
Flash_STATUS W25X40_Sector_Read(uint32_t address , uint8_t *buffer);
/*  W25X40_Cmp_Data : Flash数据比较                                           */
Flash_STATUS W25X40_Cmp_Data(uint32_t Addr, uint8_t *_ucpTar, uint32_t size);
/*  W25X40_Need_Erase :*/
Flash_STATUS W25X40_Need_Erase(uint8_t * OldBuf, uint8_t * NewBuf, uint16_t Len);

Flash_STATUS W25X40_Auto_Write(uint32_t Addr, uint16_t Len, uint8_t * Buff);

Flash_STATUS W25X40_Write_Buff(uint32_t Addr, uint32_t Len, uint8_t* Buff);
#endif
/**********************************File  END***********************************/
