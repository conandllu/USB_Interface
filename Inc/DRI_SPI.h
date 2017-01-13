/**********************************Mecono.CN***********************************/
/*                                                                            */
/*  @File Name : DRI_SPI.h                                                    */
/*  @Author    : Mecono                                                       */
/*  @version   : V0.01                                                        */
/*  @Date      : 2016-04-06                                                   */
/*  @attention : SPI1 ��ʼ��                                                  */
/*                                                                            */
/******************************************************************************/
#ifndef _DRI_SPI_
#define _DRI_SPI_

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

/*  SPI1 CSƬѡʹ�� */
#define SPI1_CS_LOW()    HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, \
                                                   GPIO_PIN_RESET)
/* SPI1 CSƬѡ���� */
#define SPI1_CS_HIGH()    HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, \
                                                   GPIO_PIN_SET)

/*  SPI1_SendByte : ����һ��                                                  */
HAL_StatusTypeDef SPI1_SendByte(uint8_t);
/*  SPI1_SendByte : ��ȡһ��                                                  */
HAL_StatusTypeDef SPI1_ReadByte(void);

#endif
/**********************************File  END***********************************/
