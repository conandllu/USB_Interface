/**********************************Mecono.CN***********************************/
/*                                                                            */
/*  @File Name : APP_COG.h                                                    */
/*  @Author    : Mecono                                                       */
/*  @version   : V0.01                                                        */
/*  @Date      : 2014-06-17                                                   */
/*  @attention : COG 应用函数                                                 */
/*                                                                            */
/******************************************************************************/
#ifndef _APP_COG_
#define _APP_COG_

#include "DRI_COG.h"


/*  COG_Init     : COG初始化                                                  */
void COG_Init(void);
/*  COG_Address  : 光点坐标                                                   */
void COG_Address(uint16_t x, uint8_t y);
/*  COG_Clear  : 清屏                                                         */
void COG_Clear(void);
/*  COG_SetPixel  : Set COG Pixel On or OFF                                   */
void COG_SetPixel(uint16_t x, uint8_t y,uint8_t set);
/*  COG_DPChar8x6  : 显示ASCII 8x6 字符                                       */
uint8_t COG_DPChar8x6(uint16_t x, uint8_t y, char ascii);
uint8_t COG_DPTemp(uint16_t Temp);
/*  COG_WriteEN  : Flash write                                                */
//DRESULT COG_Write(uint32_t address , uint16_t len , uint8_t *buffer);
/*  COG_WriteEN  : Flash wait busy                                            */
//DRESULT COG_Wait(void);
/*  COG_WriteEN  : Flash Sector Erase                                         */
//DRESULT COG_Sector_Erase(uint32_t address)
#endif
/**********************************File  END***********************************/
