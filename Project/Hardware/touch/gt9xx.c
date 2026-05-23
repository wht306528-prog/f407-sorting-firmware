/**
  ******************************************************************************
  * @file    gt5xx.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   i2c??????????????gt9157???
  ******************************************************************************
  * @attention
  *
  * ?????:???  STM32 F407 ?????? 
  * ???    :http://www.firebbs.cn
  * ???    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bsp_nt35510_lcd.h"
#include "gt9xx.h"
#include "bsp_i2c_touch.h"
#include "gui_drv_cfg.h"

/*
 * 【初学者导读】
 * - `I2C_Transfer`：按 Linux i2c_msg 语义拆分「写寄存器地址 / 读数据」等多段事务；
 *   `I2C_M_RD` 置位表示当前段为读，其余为写。
 * - `CTP_CFG_GT917S`：触控芯片内部配置表（灵敏度、阈值等），勿随意篡改字节以免漂移。
 * - 坐标读取流程：检测 INT → I2C 读状态寄存器 → 解析触点个数 → 回调 GUI。
 */

// 7????GT917S????????
const uint8_t CTP_CFG_GT917S[] =  {
  0x84,0x20,0x03,0xE0,0x01,0x05,0x05,0x00,0x00,0x40,
  0x00,0x0F,0x78,0x64,0x53,0x11,0x00,0x00,0x00,0x00,
  0x23,0x17,0x19,0x1D,0x0F,0x04,0x00,0x00,0x00,0x00,
  0x00,0x00,0x04,0x51,0x14,0x00,0x00,0x00,0x00,0x00,
  0x32,0x00,0x00,0x50,0x38,0x28,0x8A,0x20,0x11,0x37,
  0x39,0xA2,0x07,0x38,0x6D,0x28,0x11,0x03,0x24,0x00,
  0x01,0x28,0x50,0xC0,0x94,0x02,0x00,0x00,0x53,0xB8,
  0x2E,0xA2,0x35,0x8F,0x3B,0x80,0x42,0x75,0x49,0x6B,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x4C,0x3C,
  0xFF,0xFF,0x07,0x14,0x14,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x73,
  0x50,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x1F,0x1D,0x1B,0x1A,0x19,0x18,0x17,0x16,0x15,0x09,
  0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0x1C,0x1B,0x1A,0x19,0x18,0x17,0x15,0x14,
  0x13,0x12,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x05,0x00,0x00,0x0F,
  0x00,0x00,0x00,0x80,0x46,0x08,0x96,0x50,0x32,0x0A,
  0x0A,0x64,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x32,0x03,0x0C,0x08,0x23,0x00,0x14,0x23,0x00,0x28,
  0x46,0x30,0x3C,0xD0,0x07,0x50,0x70,0xB0,0x01
};

//uint8_t config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
//                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

TOUCH_IC touchIC = GT917S;			

const TOUCH_PARAM_TypeDef touch_param[1] = 
{
	 /* GT917S,5???? */
  {
    .max_width = 800,
    .max_height = 480,
    .config_reg_addr = 0x8050,
  },
};

static int8_t GTP_I2C_Test(void);

static void Delay(__IO uint32_t nCount)	 //??????????
{
	for(; nCount != 0; nCount--);
}


/**
  * @brief   ???IIC???????????
  * @param
  *		@arg i2c_msg:??????????
  *		@arg num:??????????????
  * @retval  ????????????????????????????????0xff
  */
static int I2C_Transfer( struct i2c_msg *msgs,int num)
{
	int im = 0;
	int ret = 0;

	GTP_DEBUG_FUNC();

	for (im = 0; ret == 0 && im != num; im++)
	{
		/* I2C_M_RD：本段为读事务；否则为写寄存器/命令 */
		if ((msgs[im].flags&I2C_M_RD))																//????flag????????????????????
		{
			ret = I2C_ReadBytes(msgs[im].addr, msgs[im].buf, msgs[im].len);		//IIC???????
		} else
		{
			ret = I2C_WriteBytes(msgs[im].addr,  msgs[im].buf, msgs[im].len);	//IIC????????
		}
	}

	/* 任一段返回非 0：沿用上层错误码快速退出 */
	if(ret)
		return ret;

	return im;   													//????????????????
}

/**
  * @brief   ??IIC????????????
  * @param
  *		@arg client_addr:??????
  *		@arg  buf[0~1]: ???????????????????
  *		@arg buf[2~len-1]: ?????????????????buffer
  *		@arg len:    GTP_ADDR_LENGTH + read bytes count??????????????+????????????????
  * @retval  i2c_msgs?????????????2??????????????
  */
static int32_t GTP_I2C_Read(uint8_t client_addr, uint8_t *buf, int32_t len)
{
    struct i2c_msg msgs[2];
    int32_t ret=-1;
    int32_t retries = 0;

    GTP_DEBUG_FUNC();
    /*?????????????????????????????:
     * 1. IIC  ???? ?????????????
     * 2. IIC  ???  ????
     * */

    msgs[0].flags = !I2C_M_RD;					//????
    msgs[0].addr  = client_addr;					//IIC??????
    msgs[0].len   = GTP_ADDR_LENGTH;	//?????????2???(????????????????)
    msgs[0].buf   = &buf[0];						//buf[0~1]????????????????????
    
    msgs[1].flags = I2C_M_RD;					//???
    msgs[1].addr  = client_addr;					//IIC??????
    msgs[1].len   = len - GTP_ADDR_LENGTH;	//?????????????
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];	//buf[GTP_ADDR_LENGTH]??????????????????????

    while(retries < 5)
    {
        ret = I2C_Transfer( msgs, 2);					//????IIC?????????????????2?????????
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5))
    {
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((uint16_t)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}



/**
  * @brief   ??IIC???????????
  * @param
  *		@arg client_addr:??????
  *		@arg  buf[0~1]: ??????????????????????
  *		@arg buf[2~len-1]: ??????????
  *		@arg len:    GTP_ADDR_LENGTH + write bytes count??????????????+????????????????
  * @retval  i2c_msgs?????????????1??????????????
  */
static int32_t GTP_I2C_Write(uint8_t client_addr,uint8_t *buf,int32_t len)
{
    struct i2c_msg msg;
    int32_t ret = -1;
    int32_t retries = 0;

    GTP_DEBUG_FUNC();
    /*???????????????????????????:
     * 1. IIC???? ???? ?????????????????
     * */
    msg.flags = !I2C_M_RD;			//????
    msg.addr  = client_addr;			//????????
    msg.len   = len;							//??????????(????????????+??????????????)
    msg.buf   = buf;						//???????????????????????(?????????????)

    while(retries < 5)
    {
        ret = I2C_Transfer(&msg, 1);	//????IIC???????????????1?????????
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {

        GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((uint16_t)(buf[0] << 8)) | buf[1]), len-2, ret);

    }
    return ret;
}



/**
  * @brief   ???IIC??????????????????????
  * @param
  *		@arg client:??????
  *		@arg  addr: ????????
  *		@arg rxbuf: ?????????????
  *		@arg len:    ??????????
  * @retval
  * 	@arg FAIL
  * 	@arg SUCCESS
  */
 int32_t GTP_I2C_Read_dbl_check(uint8_t client_addr, uint16_t addr, uint8_t *rxbuf, int len)
{
    uint8_t buf[16] = {0};
    uint8_t confirm_buf[16] = {0};
    uint8_t retry = 0;
    
    GTP_DEBUG_FUNC();

    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (uint8_t)(addr >> 8);
        buf[1] = (uint8_t)(addr & 0xFF);
        GTP_I2C_Read(client_addr, buf, len + 2);
        
        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (uint8_t)(addr >> 8);
        confirm_buf[1] = (uint8_t)(addr & 0xFF);
        GTP_I2C_Read(client_addr, confirm_buf, len + 2);

      
        if (!memcmp(buf, confirm_buf, len+2))
        {
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }    
    GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}


/**
  * @brief   ???GT91xx????
  * @param ??
  * @retval ??
  */
void GTP_IRQ_Disable(void)
{

    GTP_DEBUG_FUNC();

    I2C_GTP_IRQDisable();
}

/**
  * @brief   ???GT91xx????
  * @param ??
  * @retval ??
  */
void GTP_IRQ_Enable(void)
{
    GTP_DEBUG_FUNC();
     
	  I2C_GTP_IRQEnable();    
}

#if 1 /* ??????????????? */

/**
  * @brief   ????????????????????
  * @param
  *    @arg     id: ???????trackID
  *    @arg     x:  ?????? x ????
  *    @arg     y:  ?????? y ????
  *    @arg     w:  ?????? ????
  * @retval ??
  */
/*???????????????(????)??????????????????????????????????????*/
static int16_t pre_x[GTP_MAX_TOUCH] ={-1,-1,-1,-1,-1};
static int16_t pre_y[GTP_MAX_TOUCH] ={-1,-1,-1,-1,-1};

static void GTP_Touch_Down(int32_t id,int32_t x,int32_t y,int32_t w)
{
  
	GTP_DEBUG_FUNC();

	/*?x??y????????????????*/
    GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);

	
    /* ???????????????????????? */
    Touch_Button_Down(x,y);


#if 0 /* emXGUI ?????????????????? */
    Draw_Trail(pre_x[id],pre_y[id],x,y,&brush);
#endif
	
		/************************************/
		/*????????????????????????????????*/
		/* (x,y) ????????????? *************/
		/************************************/
	
		/*prex,prey??????????????????????id??????(???????????)*/
    pre_x[id] = x; pre_y[id] =y;
	
}


/**
  * @brief   ????????????????
  * @param ?????id??
  * @retval ??
  */
static void GTP_Touch_Up( int32_t id)
{
	

    /*???????????,???????????*/
    Touch_Button_Up(pre_x[id],pre_y[id]);

		/*****************************************/
		/*??????????????????????????????????*/
		/* pre_x[id],pre_y[id] ???????????? ****/
		/*******************************************/	
		/***id??????(???????????)********/
	
	
    /*??????????pre xy ???????*/
	  pre_x[id] = -1;
	  pre_y[id] = -1;		
  
    GTP_DEBUG("Touch id[%2d] release!", id);

}


/**
  * @brief   ?????????????????????????????????
  * @param ??
  * @retval ??
  */
static void Goodix_TS_Work_Func(void)
{
    uint8_t  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    uint8_t  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    uint8_t  touch_num = 0;
    uint8_t  finger = 0;
    static uint16_t pre_touch = 0;
    static uint8_t pre_id[GTP_MAX_TOUCH] = {0};

    uint8_t client_addr=GTP_ADDRESS;
    uint8_t* coor_data = NULL;
    int32_t input_x = 0;
    int32_t input_y = 0;
    int32_t input_w = 0;
    uint8_t id = 0;
 
    int32_t i  = 0;
    int32_t ret = -1;

    GTP_DEBUG_FUNC();

    ret = GTP_I2C_Read(client_addr, point_data, 12);//10?????????2?????
    if (ret < 0)
    {
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);

        return;
    }
    
    finger = point_data[GTP_ADDR_LENGTH];//???????????

    if (finger == 0x00)		//???????????
    {
        return;
    }

    if((finger & 0x80) == 0)//????buffer status??
    {
        goto exit_work_func;//????????????????????
    }

    touch_num = finger & 0x0f;//???????
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;//??????????????????????
    }

    if (touch_num > 1)//????????
    {
        uint8_t buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = GTP_I2C_Read(client_addr, buf, 2 + 8 * (touch_num - 1));
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));			//??????????????????point_data
    }

    
    
    if (pre_touch>touch_num)				//pre_touch>touch_num,?????????????
    {
        for (i = 0; i < pre_touch; i++)						//????????????
         {
            uint8_t j;
           for(j=0; j<touch_num; j++)
           {
               coor_data = &point_data[j * 8 + 3];
               id = coor_data[0] & 0x0F;									//track id
              if(pre_id[i] == id)
                break;

              if(j >= touch_num-1)											//???????????id???????pre_id[i]??????????
              {
                 GTP_Touch_Up( pre_id[i]);
              }
           }
       }
    }


    if (touch_num)
    {
        for (i = 0; i < touch_num; i++)						//????????????
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;									//track id
            pre_id[i] = id;

            input_x  = coor_data[1] | (coor_data[2] << 8);	//x????
            input_y  = coor_data[3] | (coor_data[4] << 8);	//y????
            input_w  = coor_data[5] | (coor_data[6] << 8);	//size
        
            {
				/* NT35510 与 GT911 统一 800×480 时直接用芯片上报坐标，勿再按扫描模式镜像 */
                GTP_Touch_Down( id, input_x, input_y, input_w);//???????
            }
        }
    }
    else if (pre_touch)		//touch_ num=0 ??pre_touch??=0
    {
      for(i=0;i<pre_touch;i++)
      {
          GTP_Touch_Up(pre_id[i]);
      }
    }


    pre_touch = touch_num;


exit_work_func:
    {
        ret = GTP_I2C_Write(client_addr, end_cmd, 3);
        if (ret < 0)
        {
            GTP_INFO("I2C write end_cmd error!");
        }
    }

}

void GTP_TouchPoll(void)
{
	Goodix_TS_Work_Func();
}

#endif

/**
  * @brief   ????????????????
  * @param ??
  * @retval ??
  */
 int8_t GTP_Reset_Guitar(void)
{
    GTP_DEBUG_FUNC();
#if 1
    I2C_ResetChip();
    return 0;
#else 		//????????
    int8_t ret = -1;
    int8_t retry = 0;
    uint8_t reset_command[3]={(uint8_t)GTP_REG_COMMAND>>8,(uint8_t)GTP_REG_COMMAND&0xFF,2};

    //??????????
    while(retry++ < 5)
    {
        ret = GTP_I2C_Write(GTP_ADDRESS, reset_command, 3);
        if (ret > 0)
        {
            GTP_INFO("GTP enter sleep!");

            return ret;
        }

    }
    GTP_ERROR("GTP send sleep cmd failed.");
    return ret;
#endif

}



 /**
   * @brief   ?????????
   * @param ??
   * @retval 1??????????????
   */
//int8_t GTP_Enter_Sleep(void)
//{
//    int8_t ret = -1;
//    int8_t retry = 0;
//    uint8_t reset_comment[3] = {(uint8_t)(GTP_REG_COMMENT >> 8), (uint8_t)GTP_REG_COMMENT&0xFF, 5};//5
//
//    GTP_DEBUG_FUNC();
//
//    while(retry++ < 5)
//    {
//        ret = GTP_I2C_Write(GTP_ADDRESS, reset_comment, 3);
//        if (ret > 0)
//        {
//            GTP_INFO("GTP enter sleep!");
//
//            return ret;
//        }
//
//    }
//    GTP_ERROR("GTP send sleep cmd failed.");
//    return ret;
//}


int8_t GTP_Send_Command(uint8_t command)
{
    int8_t ret = -1;
    int8_t retry = 0;
    uint8_t command_buf[3] = {(uint8_t)(GTP_REG_COMMAND >> 8), (uint8_t)GTP_REG_COMMAND&0xFF, GTP_COMMAND_READSTATUS};

    GTP_DEBUG_FUNC();

    while(retry++ < 5)
    {
        ret = GTP_I2C_Write(GTP_ADDRESS, command_buf, 3);
        if (ret > 0)
        {
            GTP_INFO("send command success!");

            return ret;
        }

    }
    GTP_ERROR("send command fail!");
    return ret;
}

/**
  * @brief   ?????????
  * @param ??
  * @retval 0??????????????
  */
int8_t GTP_WakeUp_Sleep(void)
{
    uint8_t retry = 0;
    int8_t ret = -1;

    GTP_DEBUG_FUNC();

    while(retry++ < 10)
    {
        ret = GTP_I2C_Test();
        if (ret > 0)
        {
            GTP_INFO("GTP wakeup sleep.");
            return ret;
        }
        GTP_Reset_Guitar();
    }

    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

static int32_t GTP_Get_Info(void)
{
    uint8_t opr_buf[10] = {0};
    int32_t ret = 0;

    uint16_t abs_x_max = GTP_MAX_WIDTH;
    uint16_t abs_y_max = GTP_MAX_HEIGHT;
    uint8_t int_trigger_type = GTP_INT_TRIGGER;
        
    opr_buf[0] = (uint8_t)((GTP_REG_CONFIG_DATA+1) >> 8);
    opr_buf[1] = (uint8_t)((GTP_REG_CONFIG_DATA+1) & 0xFF);
    
    ret = GTP_I2C_Read(GTP_ADDRESS, opr_buf, 10);
    if (ret < 0)
    {
        return FAIL;
    }
    
    abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
    abs_y_max = (opr_buf[5] << 8) + opr_buf[4];
		GTP_DEBUG("RES");   
		GTP_DEBUG_ARRAY(&opr_buf[0],10);

    opr_buf[0] = (uint8_t)((GTP_REG_CONFIG_DATA+6) >> 8);
    opr_buf[1] = (uint8_t)((GTP_REG_CONFIG_DATA+6) & 0xFF);
    ret = GTP_I2C_Read(GTP_ADDRESS, opr_buf, 3);
    if (ret < 0)
    {
        return FAIL;
    }
    int_trigger_type = opr_buf[2] & 0x03;
    
    GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            abs_x_max,abs_y_max,int_trigger_type);
    
    return SUCCESS;    
}

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
        0: succeed, otherwise: failed
*******************************************************/
int32_t GTP_Init_Panel(void)
{
  int32_t ret = -1;

  int32_t i = 0;
  uint16_t check_sum = 0;
  int32_t retry = 0;

  const uint8_t* cfg_info;
  uint8_t cfg_info_len  ;
  uint8_t* config;

  uint8_t cfg_num =0 ;		//????????????????

  GTP_DEBUG_FUNC();

  /* GT911 复位与引脚在此完成；main.c 不再额外调用 I2C_Touch_Init，避免重复复位长延时 */
  I2C_Touch_Init();

  ret = GTP_I2C_Test();
  if (ret < 0)
  {
      GTP_ERROR("I2C communication ERROR!");
      return ret;
  }

  //???????IC?????
  GTP_Read_Version(); 
    
#if UPDATE_CONFIG

  config = (uint8_t *)malloc (GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);

  config[0] = GTP_REG_CONFIG_DATA >> 8;
  config[1] =  GTP_REG_CONFIG_DATA & 0xff;	

  //????IC???????????????
  if(touchIC == GT917S)
  {
    cfg_info =  CTP_CFG_GT917S; //???????????
    cfg_info_len = CFG_GROUP_LEN(CTP_CFG_GT917S);//??????????????
  }

  memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
  memcpy(&config[GTP_ADDR_LENGTH], cfg_info, cfg_info_len);


  cfg_num = cfg_info_len;

  GTP_DEBUG("cfg_info_len = %d ",cfg_info_len);
  GTP_DEBUG("cfg_num = %d ",cfg_num);
  GTP_DEBUG_ARRAY(config,6);

  /*????LCD?????????????????*/
  config[GTP_ADDR_LENGTH+1] = LCD_X_LENGTH & 0xFF;
  config[GTP_ADDR_LENGTH+2] = LCD_X_LENGTH >> 8;
  config[GTP_ADDR_LENGTH+3] = LCD_Y_LENGTH & 0xFF;
  config[GTP_ADDR_LENGTH+4] = LCD_Y_LENGTH >> 8;
#if 1
  /*?????????????X2Y????*/
  config[GTP_ADDR_LENGTH+6] &= ~(X2Y_LOC);
#endif
  //?????????checksum????????
  check_sum = 0;

  /* ????check sum????? */
  if(touchIC == GT917S) 
  {
    for (i = GTP_ADDR_LENGTH; i < (cfg_num+GTP_ADDR_LENGTH-3); i += 2) 
    {
      check_sum += (config[i] << 8) + config[i + 1];
    }
    check_sum = 0 - check_sum;
    GTP_DEBUG("Config checksum: 0x%04X", check_sum);
    //????checksum
    config[(cfg_num+GTP_ADDR_LENGTH -3)] = (check_sum >> 8) & 0xFF;
    config[(cfg_num+GTP_ADDR_LENGTH -2)] = check_sum & 0xFF;
    config[(cfg_num+GTP_ADDR_LENGTH -1)] = 0x01;
  }

  //???????????
  for (retry = 0; retry < 5; retry++)
  {
    ret = GTP_I2C_Write(GTP_ADDRESS, config , cfg_num + GTP_ADDR_LENGTH+2);
    if (ret > 0)
    {
      break;
    }
  }
  Delay(0xfffff);				//????????????

#if 1	//????????????????????????????
    //??????????????????????????

  uint8_t buf[300];
  buf[0] = config[0];
  buf[1] =config[1];    //????????

  GTP_DEBUG_FUNC();

  ret = GTP_I2C_Read(GTP_ADDRESS, buf, sizeof(buf));

  GTP_DEBUG("read ");

  GTP_DEBUG_ARRAY(buf,cfg_num);

  GTP_DEBUG("write ");

  GTP_DEBUG_ARRAY(config,cfg_num);

  //?????????
  for(i=1;i<cfg_num+GTP_ADDR_LENGTH-3;i++)
  {

  if(config[i] != buf[i])
  {
  GTP_ERROR("Config fail ! i = %d ",i);
  free(config);
  return -1;
  }
  }
  if(i==cfg_num+GTP_ADDR_LENGTH-3)
  GTP_DEBUG("Config success ! i = %d ",i);

#endif
free(config);	
#endif
		
  /* ?????????emXGUI ???????? */
  I2C_GTP_IRQDisable();

  GTP_Get_Info();

  return 0;
}


/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
        2: succeed, otherwise: failed
*******************************************************/
int32_t GTP_Read_Version(void)
{
    int32_t ret = -1;
    uint8_t buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};    //????????

    GTP_DEBUG_FUNC();

    ret = GTP_I2C_Read(GTP_ADDRESS, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }
    if (buf[4] == '7')
    {
        GTP_INFO("IC3 Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
				// GT917S
				if(buf[2] == '9' && buf[3] == '1' && buf[4] == '7' && buf[5] == 'S')
				{	
					touchIC = GT917S; 
				}	
		}
    else
    {
        GTP_INFO("Unknown IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
		}
    return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
        2: succeed, otherwise failed.
*******************************************************/
static int8_t GTP_I2C_Test( void)
{
    uint8_t test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    uint8_t retry = 0;
    int8_t ret = -1;

    GTP_DEBUG_FUNC();
  
    while(retry++ < 5)
    {
        ret = GTP_I2C_Read(GTP_ADDRESS, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.",retry);
    }
    return ret;
}

////?????????????????
//void GTP_TouchProcess(void)
//{
//  GTP_DEBUG_FUNC();
//  Goodix_TS_Work_Func();

//}


int GTP_Execu(int *x,int *y)
{
	  u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	  u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
	  u8  touch_num = 0;
	  u8  finger = 0;
	  static u16 pre_touch = 0;
	  static u8 pre_id[GTP_MAX_TOUCH] = {0};

	  u8 client_addr=GTP_ADDRESS;
	  u8* coor_data = NULL;
	  s32 input_x = 0;
	  s32 input_y = 0;
	  s32 input_w = 0;
	  u8 id = 0;

	  s32 i  = 0;
	  s32 ret = 0;

	    GTP_DEBUG_FUNC();

	    ret = GTP_I2C_Read(client_addr, point_data, 12);//10?????????2?????
	    if (ret < 0)
	    {
	        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
	        return touch_num;
	    }

	    finger = point_data[GTP_ADDR_LENGTH];//触摸点数 + buffer 状态

	    /* Goodix：buffer 状态位 bit7=1 表示坐标已就绪 readable；=0 表示尚未更新完 */
	    if (finger == 0x00)
	    {
	    	return touch_num;
	    }

	    if ((finger & 0x80) == 0)
	    {
	        goto exit;//????????????????????
	    }

	    touch_num = finger & 0x0f;//???????
	    if (touch_num > GTP_MAX_TOUCH)
	    {
	        goto exit;//??????????????????????
	    }

	    if (touch_num > 1)//????????
	    {
          //????????????????????????????????????static??????
          static u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

	        ret = GTP_I2C_Read(client_addr, buf, 2 + 8 * (touch_num - 1));
	        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));			//??????????????????point_data
	    }

	    if (touch_num>0)
	    {
	        for (i = 0; i < touch_num; i++)						//????????????
	        {
	            coor_data = &point_data[i * 8 + 3];

	            id = coor_data[0] & 0x0F;									//track id
	            pre_id[i] = id;

              
            
	            input_x  = coor_data[1] | (coor_data[2] << 8);	//x????
	            input_y  = coor_data[3] | (coor_data[4] << 8);	//y????
	            input_w  = coor_data[5] | (coor_data[6] << 8);	//size
            
//            	/*?????????????X/Y???????*/

//              input_x  = LCD_X_LENGTH - input_x;
//              input_y  = LCD_Y_LENGTH - input_y;
            
              *x = input_x;
              *y = input_y;

	        }
	    }
	    pre_touch = touch_num;

exit:
	    ret = GTP_I2C_Write(client_addr, end_cmd, 3);
	    if (ret==FALSE)
	    {
	        GTP_INFO("I2C write end_cmd error!");
	    }
	    return touch_num;
}



//MODULE_DESCRIPTION("GTP Series Driver");
//MODULE_LICENSE("GPL");
