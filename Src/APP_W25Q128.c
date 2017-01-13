/**********************************Mecono.CN***********************************/
/*                                                                            */
/*  @File Name : APP_W25Q128.c                                                */
/*  @Author    : Mecono                                                       */
/*  @version   : V0.01                                                        */
/*  @Date      : 2013-07-20                                                   */
/*  @attention : Uart1 初始化  使用PA9，PA10引脚                              */
/*               Shell 添加                                                   */
/*                                                                            */
/******************************************************************************/
#include "APP_W25Q128.h"
#include "stdio.h"

/******************************************************************************/
/*  @函数名   : W25X40_Init()                                                 */
/*  @输入     :                                                               */
/*  @输出     :                                                               */
/*  @修改时间 : 2014-06-02                                                    */
/*  @描述     : W25X40初始化                                                  */
/******************************************************************************/
Flash_STATUS W25X40_Init()
{
    uint32_t temp[3] = {0};
    Flash_HOLD_HIGH();
    SPI1_CS_LOW();
    SPI1_SendByte(0x9F);
    temp[0] = SPI1_ReadByte();
		printf("%X",temp[0]);
    temp[1] = SPI1_ReadByte();
		printf("%X",temp[1]);
    temp[2] = SPI1_ReadByte();
		printf("%X\r\n",temp[2]);

	  SPI1_CS_HIGH();
 // Flash_HOLD_LOW();
    
    if(((temp[0] << 16) |(temp[1] << 8) | temp[2]) == JEDEC_ID) 
      return FLASH_OK;
    else
      return FLASH_ERROR;

}

/******************************************************************************/
/*  @函数名   : W25X40_WriteEN                                                */
/*  @输入     :                                                               */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash write Enable                                            */
/******************************************************************************/
void W25X40_WriteEN()
{
    SPI1_CS_LOW();
    SPI1_SendByte(Write_Enable);
    SPI1_CS_HIGH();
}

/******************************************************************************/
/*  @函数名   : W25X40_WriteDis                                               */
/*  @输入     :                                                               */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash write Disable                                           */
/******************************************************************************/
void W25X40_WriteDis()
{
    SPI1_CS_LOW();
    SPI1_SendByte(Write_Disable);
    SPI1_CS_HIGH();
}

/******************************************************************************/
/*  @函数名   : W25X40_Wait                                                   */
/*  @输入     :                                                               */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash busy wait                                               */
/******************************************************************************/
Flash_STATUS W25X40_Wait()
{
    SPI1_CS_LOW();
    SPI1_SendByte(Read_Status_Reg);
    while((SPI1_ReadByte() & 0x01));
	SPI1_CS_HIGH();
	return FLASH_OK;

}

/******************************************************************************/
/*  @函数名   : W25X40_WriteAddress                                           */
/*  @输入     : uint32_t address : 24 bit address                             */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash Write Address                                           */
/******************************************************************************/
Flash_STATUS W25X40_WriteAddress(uint32_t address)
{
    SPI1_SendByte((uint8_t)((address >> 16) & 0xFF));
    SPI1_SendByte((uint8_t)((address >> 8) & 0xFF));
    SPI1_SendByte((uint8_t)(address & 0xFF));
    return FLASH_OK;
}

/******************************************************************************/
/*  @函数名   : W25X40_Read                                                   */
/*  @输入     : uint32_t address : 24 bit address                             */
/*              uint16_t len :读取长度                                        */
/*              uint8_t *buffer : 读取缓冲buffer首指针                        */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash Read                                                    */
/******************************************************************************/
Flash_STATUS W25X40_Read(uint32_t address , uint16_t len , uint8_t *buffer)
{
    uint16_t i = 0;

    SPI1_CS_LOW();
    SPI1_SendByte(Fast_Read);
    
    W25X40_WriteAddress(address);
	  SPI1_ReadByte();
    for(i = 0 ; i < len ; i++){
        buffer[i] = SPI1_ReadByte();
    }

    SPI1_CS_HIGH();

    return FLASH_OK;
}

/******************************************************************************/
/*  @函数名   : W25X40_Write                                                  */
/*  @输入     : uint32_t address : 24 bit address                             */
/*              uint16_t len :写入长度  小于 256                              */
/*              uint8_t *buffer : 写入缓冲buffer首指针                        */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash Write                                                   */
/******************************************************************************/
Flash_STATUS W25X40_Write(uint32_t address , uint16_t len , uint8_t * buffer)
{
    uint32_t  j = 0;

    uint16_t uiPageCount;
    uint8_t ucPageOffset;
    ucPageOffset = address & 0xFF;                     /* 页内偏移地址        */

    
    W25X40_WriteEN();

    SPI1_CS_LOW();
  
    while(len != 0) {
        if(ucPageOffset == 0) {                        /* 写入地址为页起始    */
            uiPageCount = len >> 8;                    /* 写入页的数量        */
            for(uiPageCount = len >> 8; uiPageCount > 0; uiPageCount--) {
                W25X40_WriteEN();
                SPI1_CS_LOW();
                SPI1_SendByte(Page_Program);
                W25X40_WriteAddress(address);          /* 满页的多页写入      */
                for(j = 0; j < 256; j++) {
                    SPI1_SendByte(*buffer++);
                }
                SPI1_CS_HIGH();
                W25X40_Wait();
                address += 256;
             //   buffer += 256;
                len -= 256;
            }
            W25X40_WriteEN();                          /* 残页的写入          */                 
            SPI1_CS_LOW();
            SPI1_SendByte(Page_Program);
            W25X40_WriteAddress(address);
            for(j = 0; j < len; j++) {
                    SPI1_SendByte(*buffer++);
            }
            len = 0;
            SPI1_CS_HIGH();
            W25X40_Wait();
        } else {                                       /* 写入地址不为页起始  */
            if((ucPageOffset + len) < 256){            /* 写入不足一页        */
                W25X40_WriteEN();
                SPI1_CS_LOW();
                SPI1_SendByte(Page_Program);
                W25X40_WriteAddress(address);
                for(j = 0; j < len; j++) {
                    SPI1_SendByte(*buffer++);
                }
                len = 0;
                SPI1_CS_HIGH();
                W25X40_Wait();
            } else {                                   /* 写入超出一页        */
                W25X40_WriteEN();
                SPI1_CS_LOW();
                SPI1_SendByte(Page_Program);
                W25X40_WriteAddress(address);
                for(j = 0; j < (256 - ucPageOffset); j++) {
                    SPI1_SendByte(*buffer++);
                }
                SPI1_CS_HIGH();
                W25X40_Wait();
                address += (256 - ucPageOffset);
              //  buffer += (256 - ucPageOffset);
                len -= (256 - ucPageOffset);
            }           
            ucPageOffset = 0;
        }
    }
    
    SPI1_CS_HIGH(); 
    W25X40_Wait();

    return FLASH_OK;
}

/******************************************************************************/
/*  @函数名   : W25X40_Sector_Erase                                           */
/*  @输入     : uint32_t address : 24 bit address                             */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash Sector Erase                                            */
/******************************************************************************/
Flash_STATUS W25X40_Sector_Erase(uint32_t address)
{
    #if SECTOR_4K                                      /* 4K 扇区擦除         */

    if(address > FLASH_SECTOR_COUNT) {
        return FLASH_ERROR;
    }

    address *= FLASH_SECTOR_SIZE;

    W25X40_WriteEN();

    SPI1_CS_LOW();
    SPI1_SendByte(Sector_Erase);
    W25X40_WriteAddress(address);
    SPI1_CS_HIGH();
    while (W25X40_Wait());

    return FLASH_OK;  

    #else                                              /*  512 扇区擦除       */
    uint8_t ucBuffer[4096], i, ucSectorCount;
    uint32_t uiSectorAddress;
    if(address > FLASH_SECTOR_COUNT) {
        return FLASH_ERROR;
    }

    ucSectorCount = address % 8;            /*  逻辑扇区在物理扇区的位置 0~7  */
    address *= FLASH_SECTOR_SIZE;
    uiSectorAddress = address & 0xFFF000;              /*  物理4K扇区首地址   */
    W25X40_Read(uiSectorAddress, 4096, ucBuffer);

    W25X40_WriteEN();                                  /*  擦除物理4K扇区     */

    SPI1_CS_LOW();
    SPI1_SendByte(Sector_Erase);
    W25X40_WriteAddress(uiSectorAddress);
    SPI1_CS_HIGH();

    W25X40_Wait();

    for(i = 0; i < 8; i++) {
        if(i == ucSectorCount) {

        } else {
            W25X40_Write((uiSectorAddress + 0x200 * i), 512, &ucBuffer[0x200 * i]);
        }                                              /*  写回未删除的逻辑扇区 */
    }
    return FLASH_OK;

    #endif
}

/******************************************************************************/
/*  @函数名   : W25X40_Block_Erase                                            */
/*  @输入     : uint32_t address : 24 bit address                             */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash Sector Erase                                            */
/******************************************************************************/
Flash_STATUS W25X40_Block_Erase(uint32_t address)
{
    if(address > FLASH_BLCOK_COUNT) {
        return FLASH_ERROR;
    }

    address *= FLASH_BLOCK_SIZE;
    W25X40_WriteEN();
    Flash_HOLD_HIGH();
    SPI1_CS_LOW();
    SPI1_SendByte(Block_Erase);
    W25X40_WriteAddress(address);
    SPI1_CS_HIGH();
    while (W25X40_Wait());
    Flash_HOLD_LOW();
    return FLASH_OK;  
}

/******************************************************************************/
/*  @函数名   : W25X40_Sector_Write                                           */
/*  @输入     : uint32_t address : 24 bit address                             */
/*              uint8_t *buffer : 写入缓冲buffer首指针                        */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash Write                                                   */
/******************************************************************************/
Flash_STATUS W25X40_Sector_Write(uint32_t address , uint8_t *buffer)
{

    if(address > FLASH_SECTOR_COUNT) {
        return FLASH_ERROR;
    }

    W25X40_Sector_Erase(address);
    address *= FLASH_SECTOR_SIZE;

    return W25X40_Write(address, FLASH_SECTOR_SIZE, buffer);

}

/******************************************************************************/
/*  @函数名   : W25X40_Sector_Read                                            */
/*  @输入     : uint32_t address : 24 bit address                             */
/*              uint8_t *buffer : 读取缓冲buffer首指针                        */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : Flash Write                                                   */
/******************************************************************************/
Flash_STATUS W25X40_Sector_Read(uint32_t address , uint8_t * buffer)
{
    if(address > FLASH_SECTOR_COUNT) {
        return FLASH_ERROR;
    }

    address *= FLASH_SECTOR_SIZE;
    
    W25X40_Read(address, FLASH_SECTOR_SIZE, buffer);

    return FLASH_OK;
}

/******************************************************************************/
/*  @函数名   : W25X40_Cmp_Data                                               */
/*  @输入     : Addr : Flash地址                                              */
/*              *_ucpTar : 数据缓冲区                                         */
/*               size : 数据个数 不能超过芯片容量大小                         */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : 比较Flash的数据                                               */
/******************************************************************************/
Flash_STATUS W25X40_Cmp_Data(uint32_t Addr, uint8_t *_ucpTar, uint32_t size)
{
    uint8_t ucValue;

    //如果读取的数据长度为0或超出芯片容量返回
    if ((Addr + size) > FLASH_SECTOR_COUNT * FLASH_SECTOR_SIZE)
    {
        return FLASH_ERROR;
    }

    if (size == 0)
    {
        return FLASH_OK;
    }

    Flash_HOLD_HIGH();
    SPI1_CS_LOW();                                  
    SPI1_SendByte(Fast_Read);                          
    W25X40_WriteAddress(Addr);                 
    while (size--)
    {
        ucValue = SPI1_ReadByte();
        if (*_ucpTar++ != ucValue)
        {
            SPI1_CS_HIGH();
            Flash_HOLD_LOW();
            return FLASH_ERROR;
        }
    }
    SPI1_CS_HIGH();
    Flash_HOLD_LOW();
    return FLASH_OK;
}

/******************************************************************************/
/*  @函数名   : W25X40_Need_Erase                                             */
/*  @输入     : OldBuf : 旧数据                                               */
/*              NewBuf : 数据缓冲区                                           */
/*              Len : 数据个数 不能超过扇区大小                               */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : 比较Flash的数据                                               */
/******************************************************************************/
Flash_STATUS W25X40_Need_Erase(uint8_t * OldBuf, uint8_t * NewBuf, uint16_t Len)
{
    uint16_t i;
    uint8_t ucOld;

    /*
        算法第一步：old求反，new不变
          old    new
          1101   0101
    ~     1111
        = 0010   0101

        算法第二步：old求反结果与new位与
          0010   old
    &     0101   new
         =0000

        算法第三步：结果为0无需擦除，否则擦除
    */

    for (i = 0; i < Len; i++)
    {
        ucOld = *OldBuf++;
        ucOld = ~ucOld;

        if ((ucOld & (*NewBuf++)) != 0)
        {
            return FLASH_ERROR;
        }
    }
    return FLASH_OK;
}

/******************************************************************************/
/*  @函数名   : W25X40_Auto_Write                                             */
/*  @输入     : Addr : 目标区域首地址                                         */
/*              Len  : 数据长度                                               */
/*              Buff : 数据指针                                               */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : 比较Flash的数据                                               */
/******************************************************************************/
Flash_STATUS W25X40_Auto_Write(uint32_t Addr, uint16_t Len, uint8_t * Buff)
{
    uint16_t i;
    uint16_t j;                
    uint32_t uiFirstAddr;       /* 扇区首地址· */
    uint8_t ucNeedErase;        /* 1表示需要擦除 */
    static uint8_t s_spiBuf[FLASH_SECTOR_SIZE];
    /* 长度为不继续操作 返回成功*/
    if (Len == 0)
    {
        return FLASH_OK;
    }

    /* 如果偏移地址超过芯片容量则返回失败 */
    if (Addr > FLASH_SECTOR_COUNT * FLASH_SECTOR_SIZE)
    {
        return FLASH_ERROR;
    }

    /* 如果数据长度大于扇区容量，则返回失败 */
    if (Len > FLASH_SECTOR_SIZE)
    {
        return FLASH_ERROR;
    }

    /* 如果数据没有变化则不写Flash */
 //   W25X40_Read(Addr, Len, s_spiBuf);
 //   if (memcmp(s_spiBuf, Buff, Len) == 0)
 //   {
 //       return FLASH_OK;
 //   }

    /* 判断是否需要擦除扇区 */

  //  ucNeedErase = 0;
  //  if (W25X40_Need_Erase(s_spiBuf, Buff, Len))
  //  {
        ucNeedErase = 1;
  //  }

    uiFirstAddr = Addr & 0x1000;
/*
    if (Len == FLASH_SECTOR_SIZE)     /* 整个扇区改写 */
 /*   {
        for (i = 0; i < FLASH_SECTOR_SIZE; i++)
        {
            s_spiBuf[i] = Buff[i];
        }
    }*/
//    else                                /* 改写部分数据 */
 //   {
        /* 先读出整个扇区的数据 */
        W25X40_Read(uiFirstAddr, FLASH_SECTOR_SIZE, s_spiBuf);

        /* 再用新数据覆盖 */
        i = Addr & 0x0FFF;
				for(j = 0;j < Len; j++)
					s_spiBuf[i +j] = Buff[i];


    /* 写完之后进行校验，如果不正确则重写，最多3次 */
    for (i = 0; i < 1; i++)
    {

        if (ucNeedErase == 1)
        {
            W25X40_Sector_Erase(uiFirstAddr >> 12);        /* 擦除扇区 */
        }

        /* 写入数据*/
        return W25X40_Write(uiFirstAddr, FLASH_SECTOR_SIZE, s_spiBuf);

     //   if (W25X40_Cmp_Data(Addr, Buff, Len) == FLASH_OK)
     ///   {
      //      return FLASH_OK;
     //   }
    //    else
    //    {
    //        if (W25X40_Cmp_Data(Addr, Buff, Len) == FLASH_OK)
    //        {
    //            return FLASH_OK;
    //        }

            /* 失败后延时一段时间再试 */
    //        for (j = 0; j < 10000; j++);
       // }
    }

    return FLASH_ERROR;
}

/******************************************************************************/
/*  @函数名   : W25X40_Write_Buff                                             */
/*  @输入     : Addr : 目标区域首地址                                         */
/*              Len  : 数据长度                                               */
/*              Buff : 数据指针                                               */
/*  @输出     ：无                                                            */
/*  @修改时间 : 2014-06-08                                                    */
/*  @描述     : 比较Flash的数据                                               */
/******************************************************************************/
Flash_STATUS W25X40_Write_Buff(uint32_t Add, uint32_t Len, uint8_t* Buff)
{
    uint16_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr = Add % FLASH_SECTOR_SIZE;
    count = FLASH_SECTOR_SIZE - Addr;
    NumOfPage =  Len / FLASH_SECTOR_SIZE;
	#if 0
    NumOfSingle = Len % FLASH_SECTOR_SIZE;
		printf("ADD %X,Len %d\r\n",Add,Len);
		printf("Page %d,Single %d\r\n",NumOfPage,NumOfSingle);
	#endif
    if (Addr == 0) /* 起始地址是扇区首地址 */
    {
        if (NumOfPage == 0) /* 数据长度小于页面长度 */
        {
            if (W25X40_Auto_Write(Add, Len , Buff) == FLASH_ERROR)
            {
                return FLASH_ERROR;
            }
        }
        else    /* 数据长度大于页面长度 */
        {
            while (NumOfPage--)
            {
                if (W25X40_Auto_Write(Add, FLASH_SECTOR_SIZE, Buff) == FLASH_ERROR)
                {
                    return FLASH_ERROR;
                }
                Add +=  FLASH_SECTOR_SIZE;
                Buff += FLASH_SECTOR_SIZE;
            }
            if (W25X40_Auto_Write(Add, NumOfSingle, Buff) == FLASH_ERROR)
            {
                return FLASH_ERROR;
            }
        }
    }
    else  /* 起始地址不是扇区首地址*/
    {
        if (NumOfPage == 0) /* 数据长度小于扇区长度 */
        {
            if (NumOfSingle > count) /* (Len + Addr) > SPI_FLASH_PAGESIZE */
            {
                temp = NumOfSingle - count;

                if (W25X40_Auto_Write(Add, count, Buff) == FLASH_ERROR)
                {
                    return FLASH_ERROR;
                }

                Add +=  count;
                Buff += count;

                if (W25X40_Auto_Write(Add, temp, Buff) == FLASH_ERROR)
                {
                    return FLASH_ERROR;
                }
            }
            else
            {
                if (W25X40_Auto_Write(Add, Len, Buff) == FLASH_ERROR)
                {
                    return FLASH_ERROR;
                }
            }
        }
        else    /* 数据长度大于页面大小 */
        {
            Len -= count;
            NumOfPage =  Len / FLASH_SECTOR_SIZE;
            NumOfSingle = Len % FLASH_SECTOR_SIZE;

            if (W25X40_Auto_Write(Add, count, Buff) == FLASH_ERROR)
            {
                return FLASH_ERROR;
            }

            Add += count;
            Buff += count;

            while (NumOfPage--)
            {
                if (W25X40_Auto_Write(Add, FLASH_SECTOR_SIZE, Buff) == FLASH_ERROR)
                {
                    return FLASH_ERROR;
                }
                Add += FLASH_SECTOR_SIZE;
                Buff += FLASH_SECTOR_SIZE;
            }

            if (NumOfSingle != 0)
            {
                if (W25X40_Auto_Write(Add, NumOfSingle, Buff) == FLASH_ERROR)
                {
                    return FLASH_ERROR;
                }
            }
        }
    }

    return FLASH_OK;  
}

/**********************************File  END***********************************/
