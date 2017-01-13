/**********************************Mecono.CN***********************************/
/*                                                                            */
/*  @File Name : DRI_COG.h                                                    */
/*  @Author    : Mecono                                                       */
/*  @version   : V0.02                                                        */
/*  @Date      : 2016-03-20                                                   */
/*  @attention :                                                              */
/*                                                                            */
/******************************************************************************/
#ifndef _DRI_COG_
#define _DRI_COG_

#include "stm32f4xx_hal.h"

// COG_LED_pin   PC1
// COG_SCLK_pin  PA0
// COG_SDA_pin   PA1
// COG_RS_pin    PA2

/*  背光开 */
#define COG_LED_ON()    HAL_GPIO_WritePin(GPIOC, COG_LED_Pin, GPIO_PIN_SET)
/*  背光关 */
#define COG_LED_OFF()   HAL_GPIO_WritePin(GPIOC, COG_LED_Pin, GPIO_PIN_RESET)

/*  SCK LOW */
#define COG_SCK_LOW()   HAL_GPIO_WritePin(GPIOA, COG_SCLK_Pin, GPIO_PIN_RESET)
/*  SCK HIGH */
#define COG_SCK_HIGH()  HAL_GPIO_WritePin(GPIOA, COG_SCLK_Pin, GPIO_PIN_SET)

/*  SDA LOW */
#define COG_SDA_LOW()   HAL_GPIO_WritePin(GPIOA, COG_SDA_Pin, GPIO_PIN_RESET)
/*  SDA HIGH */
#define COG_SDA_HIGH()  HAL_GPIO_WritePin(GPIOA, COG_SDA_Pin, GPIO_PIN_SET)

/*  RS LOW */
#define COG_RS_LOW()    HAL_GPIO_WritePin(GPIOA, COG_RS_Pin, GPIO_PIN_RESET)
/*  RS HIGH */
#define COG_RS_HIGH()   HAL_GPIO_WritePin(GPIOA, COG_RS_Pin, GPIO_PIN_SET)

/*  COG_BSP_Init : COG管脚初始化                                              */
void COG_BSP_Init(void);

/*  COG_SendByte : 发送一节指令                                               */
void COG_SendCMD(uint8_t);
/*  COG_SendByte : 发送一节数据                                               */
void COG_SendDATA(uint8_t);
/*  COG_SendByte : 读取一节                                                   */
uint8_t COG_ReadByte(void);

#endif
/**********************************File  END***********************************/
