/**********************************Mecono.CN***********************************/
/*                                                                            */
/*  @File Name : DRI_FPAG.h                                                   */
/*  @Author    : Mecono                                                       */
/*  @version   : V0.02                                                        */
/*  @Date      : 2016-03-20                                                   */
/*  @attention :                                                              */
/*                                                                            */
/******************************************************************************/
#ifndef _DRI_FPGA_
#define _DRI_FPGA_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "ff.h"


typedef enum {
    FPGA_OK = 0,         /* 0: Successful */
    FPGA_ERROR = 1,      /* 1: R/W Error */
    FPGA_WRPRT,          /* 2: Write Protected */
    FPGA_BUSY,           /* 3: BUSY */
    FPGA_PARERR          /* 4: Invalid Parameter */
} FPGA_STATUS;

// FNCONFIG_Pin   PE1
// FDATA0_Pin     PE0
// FNSTATUS_Pin   PB9
// FDCLK_Pin      PB8
// FCONG_DONE_Pin PB4
/*
#define nCONFIG_HIGH()    HAL_GPIO_WritePin(GPIOE, FNCONFIG_Pin, GPIO_PIN_SET)
#define nCONFIG_LOW()     HAL_GPIO_WritePin(GPIOE, FNCONFIG_Pin, GPIO_PIN_RESET)


#define FDATA0_LOW()   HAL_GPIO_WritePin(GPIOE, FDATA0_Pin, GPIO_PIN_RESET)
#define FDATA0_HIGH()  HAL_GPIO_WritePin(GPIOE, FDATA0_Pin, GPIO_PIN_SET)

#define FNSTATUS_LOW()   HAL_GPIO_WritePin(GPIOB, FNSTATUS_Pin, GPIO_PIN_RESET)
#define FNSTATUS_HIGH()  HAL_GPIO_WritePin(GPIOB, FNSTATUS_Pin, GPIO_PIN_SET)

#define FDCLK_LOW()    HAL_GPIO_WritePin(GPIOB, FDCLK_Pin, GPIO_PIN_RESET)
#define FDCLK_HIGH()   HAL_GPIO_WritePin(GPIOB, FDCLK_Pin, GPIO_PIN_SET)

#define FCONG_DONE_LOW()    HAL_GPIO_WritePin(GPIOB, FCONG_DONE_Pin, GPIO_PIN_RESET)
#define FCONG_DONE_HIGH()   HAL_GPIO_WritePin(GPIOB, FCONG_DONE_Pin, GPIO_PIN_SET)
*/
/*  FPGA_Init : FPGA≥ı ºªØ                                              */
FPGA_STATUS FPGA_Init(FIL* fp, const TCHAR* path);

#endif
/**********************************File  END***********************************/
