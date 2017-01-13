/**********************************Mecono.CN***********************************/
/*                                                                            */
/*  @File Name : DRI_COG.c                                                    */
/*  @Author    : Mecono                                                       */
/*  @version   : V0.01                                                        */
/*  @Date      : 2014-06-03                                                   */
/*  @attention :                                                              */
/*                                                                            */
/******************************************************************************/
#include "DRI_COG.h"



/******************************************************************************/
/*  @函数名   : COG_BSP_Init                                                  */
/*  @输入     :                                                               */
/*  @输出     :                                                               */
/*  @修改时间 : 2014-06-17                                                    */
/*  @描述     : COG管脚初始化                                                 */
/******************************************************************************/

/*void COG_BSP_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
*/
  /*!< COG pins configuration *************************************************/
/*    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC , ENABLE);
 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
*/
    /* COG_LED PIN */
/*    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOC, &GPIO_InitStructure);*/
		
    /* COG_SCK PIN */
/*	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
*/
    /* COG_SDA PIN */
/*    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    */
    /* COG_RS PIN */
/*    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
*/
/******************************************************************************/
/*  @函数名   : COG_SendCMD                                                   */
/*  @输入     : byte:输出数据                                                 */
/*  @输出     :                                                               */
/*  @修改时间 : 2014-06-17                                                    */
/*  @描述     : COG单字节发送                                                 */
/******************************************************************************/
void COG_SendCMD(uint8_t byte)
{
    uint8_t i = 0;
    COG_RS_LOW();
    
    for(i = 0; i < 8; i++){    /* 发送数据 高位在前 */
        COG_SCK_LOW();
        if(byte & 0x80)
            COG_SDA_HIGH();
        else
            COG_SDA_LOW();
        COG_SCK_HIGH();
        byte = byte << 1;
    }
}

/******************************************************************************/
/*  @函数名   : COG_SendDATA                                                  */
/*  @输入     : byte:输出数据                                                 */
/*  @输出     :                                                               */
/*  @修改时间 : 2014-06-17                                                    */
/*  @描述     : COG单字节发送                                                 */
/******************************************************************************/
void COG_SendDATA(uint8_t byte)
{
    uint8_t i = 0;
    COG_RS_HIGH();
    
    for(i = 0; i < 8; i++){    /* 发送数据 高位在前 */
        COG_SCK_LOW();
        if(byte & 0x80)
            COG_SDA_HIGH();
        else
            COG_SDA_LOW();
        COG_SCK_HIGH();
        byte = byte << 1;
    }
}

/******************************************************************************/
/*  @函数名   : COG_ReadByte                                                  */
/*  @输入     :                                                               */
/*  @输出     : 返回数据                                                      */
/*  @修改时间 : 2014-06-17                                                    */
/*  @描述     : COG单字节发送                                                 */
/******************************************************************************/
uint8_t COG_ReadByte(void)
{
    return 0;
}
/**********************************File  END***********************************/
