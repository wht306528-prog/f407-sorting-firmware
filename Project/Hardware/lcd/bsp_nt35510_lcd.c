/**
  ******************************************************************************
  * @file    bsp_NT35510_lcd.c
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   NT35510Һ��������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F407 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "bsp_nt35510_lcd.h"
#include "fonts.h"

/*
 * 【初学者导读 / 工程用法】
 * - NT35510 挂在 FSMC：命令寄存器与显示 RAM 分别映射到两个相邻 Bank 地址，
 *   `NT35510_Write_Cmd` 写索引，`NT35510_Write_Data` 写参数或像素。
 * - `LCD_SCAN_MODE` 影响行列地址递增方向；更换横竖屏时需同步修改触摸坐标映射。
 * - 大块填充走 `NT35510_FillColor`/`GRAM` 连续写；刷新频率与 FSMC 时序、像素时钟相关。
 */

//����Һ��ɨ�跽����仯��XY���ؿ���
//����NT35510_GramScan�������÷���ʱ���Զ�����
uint16_t LCD_X_LENGTH = NT35510_MORE_PIXEL;
uint16_t LCD_Y_LENGTH = NT35510_LESS_PIXEL;

//Һ����ɨ��ģʽ����������Ҫ���ڷ���ѡ�������ļ������
//������ѡֵΪ0-7
//����NT35510_GramScan�������÷���ʱ���Զ�����
//LCD�ճ�ʼ�����ʱ��ʹ�ñ�Ĭ��ֵ
/** 竖屏 480×800（偶数模）。触摸在 app_display 中做多候选归一化以命中底栏 */
uint8_t LCD_SCAN_MODE = 6u;


static sFONT *LCD_Currentfonts = &Font16x32;  //Ӣ������
static uint16_t CurrentTextColor   = WHITE;//ǰ��ɫ
static uint16_t CurrentBackColor   = BLACK;//����ɫ

//�����ȡ��������ģ����
static uint8_t ucBuffer [ WIDTH_CH_CHAR*HEIGHT_CH_CHAR/8 ];	


__inline void                 NT35510_Write_Cmd           ( uint16_t usCmd );
__inline void                 NT35510_Write_Data          ( uint16_t usData );
__inline uint16_t             NT35510_Read_Data           ( void );
static void                   NT35510_Delay               ( __IO uint32_t nCount );
static void                   NT35510_GPIO_Config         ( void );
static void                   NT35510_FSMC_Config         ( void );
static void                   NT35510_REG_Config          ( void );
static void                   NT35510_SetCursor           ( uint16_t usX, uint16_t usY );
static __inline void          NT35510_FillColor           ( uint32_t ulAmout_Point, uint16_t usColor );
static uint16_t               NT35510_Read_PixelData      ( void );


/**
  * @brief  ����ʱ����
  * @param  nCount ����ʱ����ֵ
  * @retval ��
  */	
static void Delay ( __IO uint32_t nCount )
{
  for ( ; nCount != 0; nCount -- );
	
}

/**
  * @brief  ��NT35510д������
  * @param  usCmd :Ҫд���������Ĵ�����ַ��
  * @retval ��
  */	
__inline void NT35510_Write_Cmd ( uint16_t usCmd )
{
	* ( __IO uint16_t * ) ( FSMC_Addr_NT35510_CMD ) = usCmd;
	
}


/**
  * @brief  ��NT35510д������
  * @param  usData :Ҫд�������
  * @retval ��
  */	
__inline void NT35510_Write_Data ( uint16_t usData )
{
	* ( __IO uint16_t * ) ( FSMC_Addr_NT35510_DATA ) = usData;
	
}


/**
  * @brief  ��NT35510��ȡ����
  * @param  ��
  * @retval ��ȡ��������
  */	
__inline uint16_t NT35510_Read_Data ( void )
{
	return ( * ( __IO uint16_t * ) ( FSMC_Addr_NT35510_DATA ) );
	
}


/**
  * @brief  ���� NT35510 ����ʱ����
  * @param  nCount ����ʱ����ֵ
  * @retval ��
  */	
static void NT35510_Delay ( __IO uint32_t nCount )
{
  for ( ; nCount != 0; nCount -- );
	
}


/**
  * @brief  ��ʼ��NT35510��IO����
  * @param  ��
  * @retval ��
  */
static void NT35510_GPIO_Config ( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��FSMC��Ӧ��Ӧ�ܽ�ʱ��*/
	RCC_AHB1PeriphClockCmd ( 	
													/*�����ź�*/
													NT35510_CS_CLK|NT35510_DC_CLK|NT35510_WR_CLK|
													NT35510_RD_CLK	|NT35510_BK_CLK|NT35510_RST_CLK|
													/*�����ź�*/
													NT35510_D0_CLK|NT35510_D1_CLK|	NT35510_D2_CLK | 
													NT35510_D3_CLK | NT35510_D4_CLK|NT35510_D5_CLK|
													NT35510_D6_CLK | NT35510_D7_CLK|NT35510_D8_CLK|
													NT35510_D9_CLK | NT35510_D10_CLK|NT35510_D11_CLK|
													NT35510_D12_CLK | NT35510_D13_CLK|NT35510_D14_CLK|
													NT35510_D15_CLK	, ENABLE );
		
	
	/* ����FSMC���Ӧ��������,FSMC-D0~D15 */	
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_InitStructure.GPIO_Pin = NT35510_D0_PIN; 
    GPIO_Init(NT35510_D0_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D0_PORT,NT35510_D0_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D1_PIN; 
    GPIO_Init(NT35510_D1_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D1_PORT,NT35510_D1_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D2_PIN; 
    GPIO_Init(NT35510_D2_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D2_PORT,NT35510_D2_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D3_PIN; 
    GPIO_Init(NT35510_D3_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D3_PORT,NT35510_D3_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D4_PIN; 
    GPIO_Init(NT35510_D4_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D4_PORT,NT35510_D4_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D5_PIN; 
    GPIO_Init(NT35510_D5_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D5_PORT,NT35510_D5_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D6_PIN; 
    GPIO_Init(NT35510_D6_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D6_PORT,NT35510_D6_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D7_PIN; 
    GPIO_Init(NT35510_D7_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D7_PORT,NT35510_D7_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D8_PIN; 
    GPIO_Init(NT35510_D8_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D8_PORT,NT35510_D8_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D9_PIN; 
    GPIO_Init(NT35510_D9_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D9_PORT,NT35510_D9_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D10_PIN; 
    GPIO_Init(NT35510_D10_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D10_PORT,NT35510_D10_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D11_PIN; 
    GPIO_Init(NT35510_D11_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D11_PORT,NT35510_D11_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D12_PIN; 
    GPIO_Init(NT35510_D12_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D12_PORT,NT35510_D12_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D13_PIN; 
    GPIO_Init(NT35510_D13_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D13_PORT,NT35510_D13_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D14_PIN; 
    GPIO_Init(NT35510_D14_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D14_PORT,NT35510_D14_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D15_PIN; 
    GPIO_Init(NT35510_D15_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D15_PORT,NT35510_D15_PinSource,FSMC_AF);

	/* ����FSMC���Ӧ�Ŀ�����
	 * FSMC_NOE   :LCD-RD
	 * FSMC_NWE   :LCD-WR
	 * FSMC_NE1   :LCD-CS
	 * FSMC_A0    :LCD-DC
	 */
    GPIO_InitStructure.GPIO_Pin = NT35510_RD_PIN; 
    GPIO_Init(NT35510_RD_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_RD_PORT,NT35510_RD_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_WR_PIN; 
    GPIO_Init(NT35510_WR_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_WR_PORT,NT35510_WR_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_CS_PIN; 
    GPIO_Init(NT35510_CS_PORT, &GPIO_InitStructure);   
    GPIO_PinAFConfig(NT35510_CS_PORT,NT35510_CS_PinSource,FSMC_AF);  

    GPIO_InitStructure.GPIO_Pin = NT35510_DC_PIN; 
    GPIO_Init(NT35510_DC_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_DC_PORT,NT35510_DC_PinSource,FSMC_AF);
	

  /* ����LCD��λRST���ƹܽ�*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = NT35510_RST_PIN; 
	GPIO_Init ( NT35510_RST_PORT, & GPIO_InitStructure );
		
	/* ����LCD������ƹܽ�BK*/
	GPIO_InitStructure.GPIO_Pin = NT35510_BK_PIN; 
	GPIO_Init ( NT35510_BK_PORT, & GPIO_InitStructure );

}


 /**
  * @brief  LCD  FSMC ģʽ����
  * @param  ��
  * @retval ��
  */
static void NT35510_FSMC_Config ( void )
{
#if 1
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 	
	
	/* ʹ��FSMCʱ��*/
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);

	//��ַ����ʱ�䣨ADDSET��Ϊ1��HCLK 5/168M=30ns
	readWriteTiming.FSMC_AddressSetupTime      = 0x04;	 //��ַ����ʱ��
	//���ݱ���ʱ�䣨DATAST��+ 1��HCLK = 12/168M=72ns	
	readWriteTiming.FSMC_DataSetupTime         = 0x0f;	 //���ݽ���ʱ��
	//ѡ����Ƶ�ģʽ
	//ģʽB,�첽NOR FLASHģʽ����NT35510��8080ʱ��ƥ��
	readWriteTiming.FSMC_AccessMode            = FSMC_AccessMode_B;	
	
	/*����������ģʽB�޹�*/
	//��ַ����ʱ�䣨ADDHLD��ģʽAδ�õ�
	readWriteTiming.FSMC_AddressHoldTime       = 0x00;	 //��ַ����ʱ��
	//��������ת�����ڣ������ڸ���ģʽ��NOR����
	readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
	//����ʱ�ӷ�Ƶ��������ͬ�����͵Ĵ洢��
	readWriteTiming.FSMC_CLKDivision           = 0x00;
	//���ݱ���ʱ�䣬������ͬ���͵�NOR	
	readWriteTiming.FSMC_DataLatency           = 0x00;	

	
	FSMC_NORSRAMInitStructure.FSMC_Bank                  = FSMC_Bank1_NORSRAMx;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux        = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType            = FSMC_MemoryType_NOR;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &readWriteTiming;  
	
	FSMC_NORSRAMInit ( & FSMC_NORSRAMInitStructure ); 
	
	
	/* ʹ�� FSMC_Bank1_NORSRAM3 */
	FSMC_NORSRAMCmd ( FSMC_Bank1_NORSRAMx, ENABLE );  

#else
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 
	FSMC_NORSRAMTimingInitTypeDef  writeTiming;
	
	/* ʹ��FSMCʱ��*/
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);
  
	readWriteTiming.FSMC_AddressSetupTime = 0x00;	 //��ַ����ʱ�䣨ADDSET��Ϊ2��HCLK 1/36M=27ns
	readWriteTiming.FSMC_AddressHoldTime = 0x00;	 //��ַ����ʱ�䣨ADDHLD��ģʽAδ�õ�	
	readWriteTiming.FSMC_DataSetupTime = 0x0f;		 // ���ݱ���ʱ��Ϊ16��HCLK,��ΪҺ������IC�Ķ����ݵ�ʱ���ٶȲ���̫�죬�����1289���IC��
	readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
	readWriteTiming.FSMC_CLKDivision = 0x00;
	readWriteTiming.FSMC_DataLatency = 0x00;
	readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //ģʽA 	

	writeTiming.FSMC_AddressSetupTime = 0x01;	 //��ַ����ʱ�䣨ADDSET��Ϊ1��HCLK  
	writeTiming.FSMC_AddressHoldTime = 0x00;	 //��ַ����ʱ�䣨A		
	writeTiming.FSMC_DataSetupTime = 0x04;		 ////���ݱ���ʱ��Ϊ4��HCLK	
	writeTiming.FSMC_BusTurnAroundDuration = 0x00;
	writeTiming.FSMC_CLKDivision = 0x00;
	writeTiming.FSMC_DataLatency = 0x00;
	writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //ģʽA  

	FSMC_NORSRAMInitStructure.FSMC_Bank           = FSMC_Bank1_NORSRAMx;// FSMC_Bank1_NORSRAM4;//  ����ʹ��NE1 ��Ҳ�Ͷ�ӦBTCR[6],[7]��
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; // ���������ݵ�ַ
	FSMC_NORSRAMInitStructure.FSMC_MemoryType     =FSMC_MemoryType_SRAM;// FSMC_MemoryType_SRAM;  //SRAM 
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;  //�洢�����ݿ���Ϊ16bit	
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;// FSMC_BurstAccessMode_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait      = FSMC_AsynchronousWait_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;  
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;	//  �洢��дʹ��
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Enable; // ��дʹ�ò�ͬ��ʱ��
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming; //��дʱ��
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &writeTiming;  //дʱ��

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //��ʼ��FSMC����

	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAMx, ENABLE);  // ʹ��BANK1  
#endif
		
}


/**
 * @brief  ��ʼ��NT35510�Ĵ���
 * @param  ��
 * @retval ��
 */
/**
 * @brief  ��ʼ��NT35510�Ĵ���
 * @param  ��
 * @retval ��
 */
static void NT35510_REG_Config ( void )
{	
///NT35510-HSD43
  //PAGE1
  NT35510_Write_Cmd(0xF000);    NT35510_Write_Data(0x0055);
  NT35510_Write_Cmd(0xF001);    NT35510_Write_Data(0x00AA);
  NT35510_Write_Cmd(0xF002);    NT35510_Write_Data(0x0052);
  NT35510_Write_Cmd(0xF003);    NT35510_Write_Data(0x0008);
  NT35510_Write_Cmd(0xF004);    NT35510_Write_Data(0x0001);

  //Set AVDD 5.2V
  NT35510_Write_Cmd(0xB000);    NT35510_Write_Data(0x000D);
  NT35510_Write_Cmd(0xB001);    NT35510_Write_Data(0x000D);
  NT35510_Write_Cmd(0xB002);    NT35510_Write_Data(0x000D);

  //Set AVEE 5.2V
  NT35510_Write_Cmd(0xB100);    NT35510_Write_Data(0x000D);
  NT35510_Write_Cmd(0xB101);    NT35510_Write_Data(0x000D);
  NT35510_Write_Cmd(0xB102);    NT35510_Write_Data(0x000D);

  //Set VCL -2.5V
  NT35510_Write_Cmd(0xB200);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xB201);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xB202);    NT35510_Write_Data(0x0000);				

  //Set AVDD Ratio
  NT35510_Write_Cmd(0xB600);    NT35510_Write_Data(0x0044);
  NT35510_Write_Cmd(0xB601);    NT35510_Write_Data(0x0044);
  NT35510_Write_Cmd(0xB602);    NT35510_Write_Data(0x0044);

  //Set AVEE Ratio
  NT35510_Write_Cmd(0xB700);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB701);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB702);    NT35510_Write_Data(0x0034);

  //Set VCL -2.5V
  NT35510_Write_Cmd(0xB800);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB801);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB802);    NT35510_Write_Data(0x0034);
        
  //Control VGH booster voltage rang
  NT35510_Write_Cmd(0xBF00);    NT35510_Write_Data(0x0001); //VGH:7~18V	

  //VGH=15V(1V/step)	Free pump
  NT35510_Write_Cmd(0xB300);    NT35510_Write_Data(0x000f);		//08
  NT35510_Write_Cmd(0xB301);    NT35510_Write_Data(0x000f);		//08
  NT35510_Write_Cmd(0xB302);    NT35510_Write_Data(0x000f);		//08

  //VGH Ratio
  NT35510_Write_Cmd(0xB900);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB901);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB902);    NT35510_Write_Data(0x0034);

  //VGL_REG=-10(1V/step)
  NT35510_Write_Cmd(0xB500);    NT35510_Write_Data(0x0008);
  NT35510_Write_Cmd(0xB501);    NT35510_Write_Data(0x0008);
  NT35510_Write_Cmd(0xB502);    NT35510_Write_Data(0x0008);

  NT35510_Write_Cmd(0xC200);    NT35510_Write_Data(0x0003);

  //VGLX Ratio
  NT35510_Write_Cmd(0xBA00);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xBA01);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xBA02);    NT35510_Write_Data(0x0034);

    //VGMP/VGSP=4.5V/0V
  NT35510_Write_Cmd(0xBC00);    NT35510_Write_Data(0x0000);		//00
  NT35510_Write_Cmd(0xBC01);    NT35510_Write_Data(0x0078);		//C8 =5.5V/90=4.8V
  NT35510_Write_Cmd(0xBC02);    NT35510_Write_Data(0x0000);		//01

  //VGMN/VGSN=-4.5V/0V
  NT35510_Write_Cmd(0xBD00);    NT35510_Write_Data(0x0000); //00
  NT35510_Write_Cmd(0xBD01);    NT35510_Write_Data(0x0078); //90
  NT35510_Write_Cmd(0xBD02);    NT35510_Write_Data(0x0000);

  //Vcom=-1.4V(12.5mV/step)
  NT35510_Write_Cmd(0xBE00);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xBE01);    NT35510_Write_Data(0x0064); //HSD:64;Novatek:50=-1.0V, 80  5f

  //Gamma (R+)
  NT35510_Write_Cmd(0xD100);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD101);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD102);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD103);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD104);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD105);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD106);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD107);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD108);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD109);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD10A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD10B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD10C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD10D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD10E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD10F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD110);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD111);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD112);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD113);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD114);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD115);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD116);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD117);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD118);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD119);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD11A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD11B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD11C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD11D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD11E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD11F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD120);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD121);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD122);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD123);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD124);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD125);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD126);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD127);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD128);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD129);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD12A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD12B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD12C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD12D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD12E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD12F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD130);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD131);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD132);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD133);    NT35510_Write_Data(0x006D);

  //Gamma (G+)
  NT35510_Write_Cmd(0xD200);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD201);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD202);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD203);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD204);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD205);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD206);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD207);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD208);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD209);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD20A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD20B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD20C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD20D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD20E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD20F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD210);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD211);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD212);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD213);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD214);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD215);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD216);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD217);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD218);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD219);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD21A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD21B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD21C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD21D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD21E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD21F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD220);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD221);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD222);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD223);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD224);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD225);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD226);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD227);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD228);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD229);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD22A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD22B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD22C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD22D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD22E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD22F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD230);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD231);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD232);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD233);    NT35510_Write_Data(0x006D);

  //Gamma (B+)
  NT35510_Write_Cmd(0xD300);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD301);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD302);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD303);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD304);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD305);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD306);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD307);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD308);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD309);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD30A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD30B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD30C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD30D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD30E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD30F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD310);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD311);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD312);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD313);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD314);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD315);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD316);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD317);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD318);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD319);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD31A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD31B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD31C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD31D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD31E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD31F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD320);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD321);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD322);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD323);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD324);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD325);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD326);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD327);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD328);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD329);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD32A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD32B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD32C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD32D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD32E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD32F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD330);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD331);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD332);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD333);    NT35510_Write_Data(0x006D);

  //Gamma (R-)
  NT35510_Write_Cmd(0xD400);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD401);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD402);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD403);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD404);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD405);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD406);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD407);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD408);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD409);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD40A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD40B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD40C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD40D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD40E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD40F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD410);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD411);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD412);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD413);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD414);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD415);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD416);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD417);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD418);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD419);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD41A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD41B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD41C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD41D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD41E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD41F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD420);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD421);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD422);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD423);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD424);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD425);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD426);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD427);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD428);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD429);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD42A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD42B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD42C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD42D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD42E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD42F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD430);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD431);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD432);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD433);    NT35510_Write_Data(0x006D);

  //Gamma (G-)
  NT35510_Write_Cmd(0xD500);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD501);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD502);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD503);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD504);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD505);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD506);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD507);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD508);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD509);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD50A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD50B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD50C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD50D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD50E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD50F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD510);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD511);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD512);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD513);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD514);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD515);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD516);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD517);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD518);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD519);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD51A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD51B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD51C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD51D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD51E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD51F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD520);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD521);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD522);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD523);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD524);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD525);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD526);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD527);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD528);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD529);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD52A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD52B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD52C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD52D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD52E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD52F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD530);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD531);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD532);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD533);    NT35510_Write_Data(0x006D);

  //Gamma (B-)
  NT35510_Write_Cmd(0xD600);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD601);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD602);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD603);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD604);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD605);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD606);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD607);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD608);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD609);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD60A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD60B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD60C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD60D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD60E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD60F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD610);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD611);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD612);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD613);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD614);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD615);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD616);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD617);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD618);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD619);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD61A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD61B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD61C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD61D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD61E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD61F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD620);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD621);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD622);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD623);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD624);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD625);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD626);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD627);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD628);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD629);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD62A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD62B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD62C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD62D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD62E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD62F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD630);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD631);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD632);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD633);    NT35510_Write_Data(0x006D);

  //PAGE0
  NT35510_Write_Cmd(0xF000);    NT35510_Write_Data(0x0055);
  NT35510_Write_Cmd(0xF001);    NT35510_Write_Data(0x00AA);
  NT35510_Write_Cmd(0xF002);    NT35510_Write_Data(0x0052);
  NT35510_Write_Cmd(0xF003);    NT35510_Write_Data(0x0008);	
  NT35510_Write_Cmd(0xF004);    NT35510_Write_Data(0x0000); 

  //480x800
  NT35510_Write_Cmd(0xB500);    NT35510_Write_Data(0x0050);

  //NT35510_Write_Cmd(0x2C00);    NT35510_Write_Data(0x0006); //8BIT 6-6-6?

  //Dispay control
  NT35510_Write_Cmd(0xB100);    NT35510_Write_Data(0x00CC);	
  NT35510_Write_Cmd(0xB101);    NT35510_Write_Data(0x0000); // S1->S1440:00;S1440->S1:02

  //Source hold time (Nova non-used)
  NT35510_Write_Cmd(0xB600);    NT35510_Write_Data(0x0005);

  //Gate EQ control	 (Nova non-used)
  NT35510_Write_Cmd(0xB700);    NT35510_Write_Data(0x0077);  //HSD:70;Nova:77	 
  NT35510_Write_Cmd(0xB701);    NT35510_Write_Data(0x0077);	//HSD:70;Nova:77

  //Source EQ control (Nova non-used)
  NT35510_Write_Cmd(0xB800);    NT35510_Write_Data(0x0001);  
  NT35510_Write_Cmd(0xB801);    NT35510_Write_Data(0x0003);	//HSD:05;Nova:07
  NT35510_Write_Cmd(0xB802);    NT35510_Write_Data(0x0003);	//HSD:05;Nova:07
  NT35510_Write_Cmd(0xB803);    NT35510_Write_Data(0x0003);	//HSD:05;Nova:07

  //Inversion mode: column
  NT35510_Write_Cmd(0xBC00);    NT35510_Write_Data(0x0002);	//00: column
  NT35510_Write_Cmd(0xBC01);    NT35510_Write_Data(0x0000);	//01:1dot
  NT35510_Write_Cmd(0xBC02);    NT35510_Write_Data(0x0000); 

  //Frame rate	(Nova non-used)
  NT35510_Write_Cmd(0xBD00);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xBD01);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xBD02);    NT35510_Write_Data(0x001c); //HSD:06;Nova:1C
  NT35510_Write_Cmd(0xBD03);    NT35510_Write_Data(0x001c); //HSD:04;Nova:1C
  NT35510_Write_Cmd(0xBD04);    NT35510_Write_Data(0x0000);

  //LGD timing control(4H/4-delay_ms)
  NT35510_Write_Cmd(0xC900);    NT35510_Write_Data(0x00D0);	//3H:0x50;4H:0xD0	 //D
  NT35510_Write_Cmd(0xC901);    NT35510_Write_Data(0x0002);  //HSD:05;Nova:02
  NT35510_Write_Cmd(0xC902);    NT35510_Write_Data(0x0050);	//HSD:05;Nova:50
  NT35510_Write_Cmd(0xC903);    NT35510_Write_Data(0x0050);	//HSD:05;Nova:50	;STV delay_ms time
  NT35510_Write_Cmd(0xC904);    NT35510_Write_Data(0x0050);	//HSD:05;Nova:50	;CLK delay_ms time

  NT35510_Write_Cmd(0x3600);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0x3500);    NT35510_Write_Data(0x0000);

  NT35510_Write_Cmd(0xFF00);    NT35510_Write_Data(0x00AA);
  NT35510_Write_Cmd(0xFF01);    NT35510_Write_Data(0x0055);
  NT35510_Write_Cmd(0xFF02);    NT35510_Write_Data(0x0025);
  NT35510_Write_Cmd(0xFF03);    NT35510_Write_Data(0x0001);

  NT35510_Write_Cmd(0xFC00);    NT35510_Write_Data(0x0016);
  NT35510_Write_Cmd(0xFC01);    NT35510_Write_Data(0x00A2);
  NT35510_Write_Cmd(0xFC02);    NT35510_Write_Data(0x0026);
  /* RGB565：保留单次写入，避免被后续写入覆盖像素格式（NT35510 DAT 低字节有效） */
  NT35510_Write_Cmd(0x3A00);    NT35510_Write_Data(0x0055);

  //Sleep out
  NT35510_Write_Cmd(0x1100);	   //?
  Delay(0xFFFFFF);

  //Display on
  NT35510_Write_Cmd(0x2900);
}


/**
 * @brief  NT35510��ʼ�����������Ҫ�õ�lcd��һ��Ҫ�����������
 * @param  ��
 * @retval ��
 */
void NT35510_Init ( void )
{
	NT35510_GPIO_Config ();
	NT35510_FSMC_Config ();
	
	
	NT35510_Rst ();
	NT35510_REG_Config ();
	
	//����Ĭ��ɨ�跽������ 6 ģʽΪ�󲿷�Һ�����̵�Ĭ����ʾ����  
	NT35510_GramScan(LCD_SCAN_MODE);
    
	NT35510_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* ��������ʾȫ�� */
	NT35510_BackLed_Control ( ENABLE );      //����LCD�����
}


/**
 * @brief  NT35510����LED����
 * @param  enumState �������Ƿ�ʹ�ܱ���LED
  *   �ò���Ϊ����ֵ֮һ��
  *     @arg ENABLE :ʹ�ܱ���LED
  *     @arg DISABLE :���ñ���LED
 * @retval ��
 */
void NT35510_BackLed_Control ( FunctionalState enumState )
{
	if ( enumState )
		 GPIO_SetBits( NT35510_BK_PORT, NT35510_BK_PIN );	
	else
		 GPIO_ResetBits( NT35510_BK_PORT, NT35510_BK_PIN );
		
}



/**
 * @brief  NT35510 ������λ
 * @param  ��
 * @retval ��
 */
void NT35510_Rst ( void )
{			
	/*ע�⸴λʱ��͸ߵ�ƽ�����ʱ���ܹ���*/
	
	GPIO_ResetBits ( NT35510_RST_PORT, NT35510_RST_PIN );	 //�͵�ƽ��λ

	NT35510_Delay ( 0xFFFFF ); 					   

	GPIO_SetBits ( NT35510_RST_PORT, NT35510_RST_PIN );		 	 

	NT35510_Delay ( 0x1FFFFF ); 
 	
	
}



/**
 * @brief  ����NT35510��GRAM��ɨ�跽�� 
 * @param  ucOption ��ѡ��GRAM��ɨ�跽�� 
 *     @arg 0-7 :������ѡֵΪ0-7��˸�����
 *
 *	����������0��3��5��6 ģʽ�ʺϴ���������ʾ���֣�
 *				���Ƽ�ʹ������ģʽ��ʾ����	����ģʽ��ʾ���ֻ��о���Ч��			
 *		
 *	����0��2��4��6 ģʽ��X��������Ϊ480��Y��������Ϊ854
 *	����1��3��5��7 ģʽ��X��������Ϊ854��Y��������Ϊ480
 *
 *	���� 6 ģʽΪ�󲿷�Һ�����̵�Ĭ����ʾ����
 *	���� 3 ģʽΪ����ͷ����ʹ�õķ���
 *	���� 0 ģʽΪBMPͼƬ��ʾ����ʹ�õķ���
 *
 * @retval ��
 * @note  ����ͼ����A��ʾ���ϣ�V��ʾ���£�<��ʾ����>��ʾ����
					X��ʾX�ᣬY��ʾY��

------------------------------------------------------------
ģʽ0��				.		ģʽ1��		.	ģʽ2��			.	ģʽ3��					
					A		.					A		.		A					.		A									
					|		.					|		.		|					.		|							
					Y		.					X		.		Y					.		X					
					0		.					1		.		2					.		3					
	<--- X0 o		.	<----Y1	o		.		o 2X--->  .		o 3Y--->	
------------------------------------------------------------	
ģʽ4��				.	ģʽ5��			.	ģʽ6��			.	ģʽ7��					
	<--- X4 o		.	<--- Y5 o		.		o 6X--->  .		o 7Y--->	
					4		.					5		.		6					.		7	
					Y		.					X		.		Y					.		X						
					|		.					|		.		|					.		|							
					V		.					V		.		V					.		V		
---------------------------------------------------------				
											 LCD��ʾ��
								|-----------------|
								|			Ұ��Logo		|
								|									|
								|									|
								|									|
								|									|
								|									|
								|									|
								|									|
								|									|
								|-----------------|
								��Ļ���棨��480����854��

 *******************************************************/
void NT35510_GramScan ( uint8_t ucOption )
{	
	//������飬ֻ������0-7
	if(ucOption >7 )
		return;
	
	//����ģʽ����LCD_SCAN_MODE��ֵ����Ҫ���ڴ�����ѡ��������
	LCD_SCAN_MODE = ucOption;
	
	//����ģʽ����XY��������ؿ���
	if(ucOption%2 == 0)	
	{
		//0 2 4 6ģʽ��X�������ؿ���Ϊ480��Y����Ϊ854
		LCD_X_LENGTH = NT35510_LESS_PIXEL;
		LCD_Y_LENGTH =	NT35510_MORE_PIXEL;
	}
	else				
	{
		//1 3 5 7ģʽ��X�������ؿ���Ϊ854��Y����Ϊ480
		LCD_X_LENGTH = NT35510_MORE_PIXEL;
		LCD_Y_LENGTH =	NT35510_LESS_PIXEL; 
	}

	//0x36��������ĸ�3λ����������GRAMɨ�跽��	
	NT35510_Write_Cmd ( 0x3600 ); 
	NT35510_Write_Data (0x00 | (ucOption<<5));//����ucOption��ֵ����LCD��������0-7��ģʽ
	NT35510_Write_Cmd ( CMD_SetCoordinateX ); 
	NT35510_Write_Data ( 0x00 );		/* x ��ʼ�����8λ */
  NT35510_Write_Cmd ( CMD_SetCoordinateX + 1 ); 
	NT35510_Write_Data ( 0x00 );		/* x ��ʼ�����8λ */
  NT35510_Write_Cmd ( CMD_SetCoordinateX + 2 ); 
	NT35510_Write_Data ( ((LCD_X_LENGTH-1)>>8)&0xFF ); /* x ���������8λ */	
  NT35510_Write_Cmd ( CMD_SetCoordinateX + 3 ); 
	NT35510_Write_Data ( (LCD_X_LENGTH-1)&0xFF );				/* x ���������8λ */

	NT35510_Write_Cmd ( CMD_SetCoordinateY ); 
	NT35510_Write_Data ( 0x00 );		/* y ��ʼ�����8λ */
  NT35510_Write_Cmd ( CMD_SetCoordinateY + 1 ); 
	NT35510_Write_Data ( 0x00 );		/* y ��ʼ�����8λ */
  NT35510_Write_Cmd ( CMD_SetCoordinateY + 2 ); 
	NT35510_Write_Data ( ((LCD_Y_LENGTH-1)>>8)&0xFF );	/* y ���������8λ */	 
  NT35510_Write_Cmd ( CMD_SetCoordinateY + 3 ); 
	NT35510_Write_Data ( (LCD_Y_LENGTH-1)&0xFF );				/* y ���������8λ */

	/* write gram start */
	NT35510_Write_Cmd ( CMD_SetPixel );	
}



/**
 * @brief  ��NT35510��ʾ���Ͽ���һ������
 * @param  usX �����ض�ɨ�跽���´��ڵ����X����
 * @param  usY �����ض�ɨ�跽���´��ڵ����Y����
 * @param  usWidth �����ڵĿ���
 * @param  usHeight �����ڵĸ߶�
 * @retval ��
 */
void NT35510_OpenWindow ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight )
{	
	NT35510_Write_Cmd ( CMD_SetCoordinateX ); 				 /* ����X���� */
	NT35510_Write_Data ( usX >> 8  );	 /* �ȸ�8λ��Ȼ���8λ */
  NT35510_Write_Cmd ( CMD_SetCoordinateX + 1 ); 
	NT35510_Write_Data ( usX & 0xff  );	 /* ������ʼ��ͽ�����*/
  NT35510_Write_Cmd ( CMD_SetCoordinateX + 2 );
	NT35510_Write_Data ( ( usX + usWidth - 1 ) >> 8  );
  NT35510_Write_Cmd ( CMD_SetCoordinateX + 3 );
	NT35510_Write_Data ( ( usX + usWidth - 1 ) & 0xff  );

	NT35510_Write_Cmd ( CMD_SetCoordinateY ); 			     /* ����Y����*/
	NT35510_Write_Data ( usY >> 8  );
  NT35510_Write_Cmd ( CMD_SetCoordinateY + 1);
	NT35510_Write_Data ( usY & 0xff  );
  NT35510_Write_Cmd ( CMD_SetCoordinateY + 2);
	NT35510_Write_Data ( ( usY + usHeight - 1 ) >> 8 );
  NT35510_Write_Cmd ( CMD_SetCoordinateY + 3);
	NT35510_Write_Data ( ( usY + usHeight - 1) & 0xff );
	
}


/**
 * @brief  �趨NT35510�Ĺ������
 * @param  usX �����ض�ɨ�跽���¹���X����
 * @param  usY �����ض�ɨ�跽���¹���Y����
 * @retval ��
 */
static void NT35510_SetCursor ( uint16_t usX, uint16_t usY )	
{
	NT35510_OpenWindow ( usX, usY, 1, 1 );
}


/**
 * @brief  ��NT35510��ʾ������ĳһ��ɫ������ص�
 * @param  ulAmout_Point ��Ҫ�����ɫ�����ص������Ŀ
 * @param  usColor ����ɫ
 * @retval ��
 */
static __inline void NT35510_FillColor ( uint32_t ulAmout_Point, uint16_t usColor )
{
	uint32_t i = 0;
	
	
	/* memory write */
	NT35510_Write_Cmd ( CMD_SetPixel );	
		
	for ( i = 0; i < ulAmout_Point; i ++ )
		NT35510_Write_Data ( usColor );
		
	
}


/**
 * @brief  ��NT35510��ʾ����ĳһ������ĳ����ɫ��������
 * @param  usX �����ض�ɨ�跽���´��ڵ����X����
 * @param  usY �����ض�ɨ�跽���´��ڵ����Y����
 * @param  usWidth �����ڵĿ���
 * @param  usHeight �����ڵĸ߶�
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_Clear ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight )
{
	NT35510_OpenWindow ( usX, usY, usWidth, usHeight );

	NT35510_FillColor ( usWidth * usHeight, CurrentBackColor );		
	
}


/**
 * @brief  ��NT35510��ʾ����ĳһ����ĳ����ɫ�������
 * @param  usX �����ض�ɨ�跽���¸õ��X����
 * @param  usY �����ض�ɨ�跽���¸õ��Y����
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_SetPointPixel ( uint16_t usX, uint16_t usY )	
{	
	if ( ( usX < LCD_X_LENGTH ) && ( usY < LCD_Y_LENGTH ) )
  {
		NT35510_SetCursor ( usX, usY );
		
		NT35510_FillColor ( 1, CurrentTextColor );
	}
	
}


/**
 * @brief  ��ȡNT35510 GRAN ��һ����������
 * @param  ��
 * @retval ��������
 */
static uint16_t NT35510_Read_PixelData ( void )	
{	
	uint16_t us_RG=0, usB=0 ;

	
	NT35510_Write_Cmd ( 0x2E00 );   /* ������ */
	//ǰ��ȡ���ν��ȥ��
	us_RG = NT35510_Read_Data (); 	/*FIRST READ OUT DUMMY DATA*/
	us_RG = NT35510_Read_Data (); 	/*FIRST READ OUT DUMMY DATA*/
	us_RG = NT35510_Read_Data (); 	/*FIRST READ OUT DUMMY DATA*/
	
	us_RG = NT35510_Read_Data ();  	/*READ OUT RED AND GREEN DATA  */
	usB = NT35510_Read_Data ();  		/*READ OUT BLUE DATA*/
	
  return   (us_RG&0xF800)| ((us_RG<<3)&0x7E0) | (usB>>11) ;
	
}


/**
 * @brief  ��ȡ NT35510 ��ʾ����ĳһ����������������
 * @param  usX �����ض�ɨ�跽���¸õ��X����
 * @param  usY �����ض�ɨ�跽���¸õ��Y����
 * @retval ��������
 */
uint16_t NT35510_GetPointPixel ( uint16_t usX, uint16_t usY )
{ 
	uint16_t usPixelData;

	
	NT35510_SetCursor ( usX, usY );
	
	usPixelData = NT35510_Read_PixelData ();
	
	return usPixelData;
	
}


/**
 * @brief  �� NT35510 ��ʾ����ʹ�� Bresenham �㷨���߶� 
 * @param  usX1 �����ض�ɨ�跽�����߶ε�һ���˵�X����
 * @param  usY1 �����ض�ɨ�跽�����߶ε�һ���˵�Y����
 * @param  usX2 �����ض�ɨ�跽�����߶ε���һ���˵�X����
 * @param  usY2 �����ض�ɨ�跽�����߶ε���һ���˵�Y����
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DrawLine ( uint16_t usX1, uint16_t usY1, uint16_t usX2, uint16_t usY2 )
{
	uint16_t us; 
	uint16_t usX_Current, usY_Current;
	
	int32_t lError_X = 0, lError_Y = 0, lDelta_X, lDelta_Y, lDistance; 
	int32_t lIncrease_X, lIncrease_Y; 	
	
	
	lDelta_X = usX2 - usX1; //������������ 
	lDelta_Y = usY2 - usY1; 
	
	usX_Current = usX1; 
	usY_Current = usY1; 
	
	
	if ( lDelta_X > 0 ) 
		lIncrease_X = 1; //���õ������� 
	
	else if ( lDelta_X == 0 ) 
		lIncrease_X = 0;//��ֱ�� 
	
	else 
  { 
    lIncrease_X = -1;
    lDelta_X = - lDelta_X;
  } 

	
	if ( lDelta_Y > 0 )
		lIncrease_Y = 1; 
	
	else if ( lDelta_Y == 0 )
		lIncrease_Y = 0;//ˮƽ�� 
	
	else 
  {
    lIncrease_Y = -1;
    lDelta_Y = - lDelta_Y;
  } 

	
	if (  lDelta_X > lDelta_Y )
		lDistance = lDelta_X; //ѡȡ�������������� 
	
	else 
		lDistance = lDelta_Y; 

	
	for ( us = 0; us <= lDistance + 1; us ++ )//������� 
	{  
		NT35510_SetPointPixel ( usX_Current, usY_Current );//���� 
		
		lError_X += lDelta_X ; 
		lError_Y += lDelta_Y ; 
		
		if ( lError_X > lDistance ) 
		{ 
			lError_X -= lDistance; 
			usX_Current += lIncrease_X; 
		}  
		
		if ( lError_Y > lDistance ) 
		{ 
			lError_Y -= lDistance; 
			usY_Current += lIncrease_Y; 
		} 
		
	}  
	
	
}   


/**
 * @brief  �� NT35510 ��ʾ���ϻ�һ������
 * @param  usX_Start �����ض�ɨ�跽���¾��ε���ʼ��X����
 * @param  usY_Start �����ض�ɨ�跽���¾��ε���ʼ��Y����
 * @param  usWidth�����εĿ��ȣ���λ�����أ�
 * @param  usHeight�����εĸ߶ȣ���λ�����أ�
 * @param  ucFilled ��ѡ���Ƿ����þ���
  *   �ò���Ϊ����ֵ֮һ��
  *     @arg 0 :���ľ���
  *     @arg 1 :ʵ�ľ��� 
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DrawRectangle ( uint16_t usX_Start, uint16_t usY_Start, uint16_t usWidth, uint16_t usHeight, uint8_t ucFilled )
{
	if ( ucFilled )
	{
		NT35510_OpenWindow ( usX_Start, usY_Start, usWidth, usHeight );
		NT35510_FillColor ( usWidth * usHeight ,CurrentTextColor);	
	}
	else
	{
		NT35510_DrawLine ( usX_Start, usY_Start, usX_Start + usWidth - 1, usY_Start );
		NT35510_DrawLine ( usX_Start, usY_Start + usHeight - 1, usX_Start + usWidth - 1, usY_Start + usHeight - 1 );
		NT35510_DrawLine ( usX_Start, usY_Start, usX_Start, usY_Start + usHeight - 1 );
		NT35510_DrawLine ( usX_Start + usWidth - 1, usY_Start, usX_Start + usWidth - 1, usY_Start + usHeight - 1 );		
	}

}


/**
 * @brief  �� NT35510 ��ʾ����ʹ�� Bresenham �㷨��Բ
 * @param  usX_Center �����ض�ɨ�跽����Բ�ĵ�X����
 * @param  usY_Center �����ض�ɨ�跽����Բ�ĵ�Y����
 * @param  usRadius��Բ�İ뾶����λ�����أ�
 * @param  ucFilled ��ѡ���Ƿ�����Բ
  *   �ò���Ϊ����ֵ֮һ��
  *     @arg 0 :����Բ
  *     @arg 1 :ʵ��Բ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DrawCircle ( uint16_t usX_Center, uint16_t usY_Center, uint16_t usRadius, uint8_t ucFilled )
{
	int16_t sCurrentX, sCurrentY;
	int16_t sError;
	
	
	sCurrentX = 0; sCurrentY = usRadius;	  
	
	sError = 3 - ( usRadius << 1 );     //�ж��¸���λ�õı�־
	
	
	while ( sCurrentX <= sCurrentY )
	{
		int16_t sCountY;
		
		
		if ( ucFilled ) 			
			for ( sCountY = sCurrentX; sCountY <= sCurrentY; sCountY ++ ) 
			{                      
				NT35510_SetPointPixel ( usX_Center + sCurrentX, usY_Center + sCountY );           //1���о����� 
				NT35510_SetPointPixel ( usX_Center - sCurrentX, usY_Center + sCountY );           //2       
				NT35510_SetPointPixel ( usX_Center - sCountY,   usY_Center + sCurrentX );           //3
				NT35510_SetPointPixel ( usX_Center - sCountY,   usY_Center - sCurrentX );           //4
				NT35510_SetPointPixel ( usX_Center - sCurrentX, usY_Center - sCountY );           //5    
        NT35510_SetPointPixel ( usX_Center + sCurrentX, usY_Center - sCountY );           //6
				NT35510_SetPointPixel ( usX_Center + sCountY,   usY_Center - sCurrentX );           //7 	
        NT35510_SetPointPixel ( usX_Center + sCountY,   usY_Center + sCurrentX );           //0				
			}
		
		else
		{          
			NT35510_SetPointPixel ( usX_Center + sCurrentX, usY_Center + sCurrentY );             //1���о�����
			NT35510_SetPointPixel ( usX_Center - sCurrentX, usY_Center + sCurrentY );             //2      
			NT35510_SetPointPixel ( usX_Center - sCurrentY, usY_Center + sCurrentX );             //3
			NT35510_SetPointPixel ( usX_Center - sCurrentY, usY_Center - sCurrentX );             //4
			NT35510_SetPointPixel ( usX_Center - sCurrentX, usY_Center - sCurrentY );             //5       
			NT35510_SetPointPixel ( usX_Center + sCurrentX, usY_Center - sCurrentY );             //6
			NT35510_SetPointPixel ( usX_Center + sCurrentY, usY_Center - sCurrentX );             //7 
			NT35510_SetPointPixel ( usX_Center + sCurrentY, usY_Center + sCurrentX );             //0
    }			
		
		
		sCurrentX ++;

		
		if ( sError < 0 ) 
			sError += 4 * sCurrentX + 6;	  
		
		else
		{
			sError += 10 + 4 * ( sCurrentX - sCurrentY );   
			sCurrentY --;
		} 	
		
		
	}
	
	
}

/**
 * @brief  �� NT35510 ��ʾ������ʾһ��Ӣ���ַ�
 * @param  usX �����ض�ɨ�跽�����ַ�����ʼX����
 * @param  usY �����ض�ɨ�跽���¸õ����ʼY����
 * @param  cChar ��Ҫ��ʾ��Ӣ���ַ�
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DispChar_EN ( uint16_t usX, uint16_t usY, const char cChar )
{
	uint8_t  byteCount, bitCount,fontLength;	
	uint16_t ucRelativePositon;
	uint8_t *Pfont;
	
	//��ascii���ƫ�ƣ���ģ��������ASCII����ǰ32����ͼ�η��ţ�
	ucRelativePositon = cChar - ' ';
	
	//ÿ����ģ���ֽ���
	fontLength = (LCD_Currentfonts->Width*LCD_Currentfonts->Height)/8;
		
	//��ģ�׵�ַ
	/*ascii���ƫ��ֵ����ÿ����ģ���ֽ����������ģ��ƫ��λ��*/
	Pfont = (uint8_t *)&LCD_Currentfonts->table[ucRelativePositon * fontLength];
	
	//������ʾ����
	NT35510_OpenWindow ( usX, usY, LCD_Currentfonts->Width, LCD_Currentfonts->Height);
	
	NT35510_Write_Cmd ( CMD_SetPixel );			

	//���ֽڶ�ȡ��ģ����
	//����ǰ��ֱ����������ʾ���ڣ���ʾ���ݻ��Զ�����
	for ( byteCount = 0; byteCount < fontLength; byteCount++ )
	{
			//һλһλ����Ҫ��ʾ����ɫ
			for ( bitCount = 0; bitCount < 8; bitCount++ )
			{
					if ( Pfont[byteCount] & (0x80>>bitCount) )
						NT35510_Write_Data ( CurrentTextColor );			
					else
						NT35510_Write_Data ( CurrentBackColor );
			}	
	}	
}


/**
 * @brief  �� NT35510 ��ʾ������ʾӢ���ַ���
 * @param  line �����ض�ɨ�跽�����ַ�������ʼY����
  *   ��������ʹ�ú�LINE(0)��LINE(1)�ȷ�ʽָ���������꣬
  *   ��LINE(x)����ݵ�ǰѡ�������������Y����ֵ��
	*		��ʾ������ʹ��LINE��ʱ����Ҫ��Ӣ���������ó�Font8x16
 * @param  pStr ��Ҫ��ʾ��Ӣ���ַ������׵�ַ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DispStringLine_EN (  uint16_t line,  char * pStr )
{
	uint16_t usX = 0;
	
	while ( * pStr != '\0' )
	{
		if ( ( usX - NT35510_DispWindow_X_Star + LCD_Currentfonts->Width ) > LCD_X_LENGTH )
		{
			usX = NT35510_DispWindow_X_Star;
			line += LCD_Currentfonts->Height;
		}
		
		if ( ( line - NT35510_DispWindow_Y_Star + LCD_Currentfonts->Height ) > LCD_Y_LENGTH )
		{
			usX = NT35510_DispWindow_X_Star;
			line = NT35510_DispWindow_Y_Star;
		}
		
		NT35510_DispChar_EN ( usX, line, * pStr);
		
		pStr ++;
		
		usX += LCD_Currentfonts->Width;
		
	}
	
}


/**
 * @brief  �� NT35510 ��ʾ������ʾӢ���ַ���
 * @param  usX �����ض�ɨ�跽�����ַ�����ʼX����
 * @param  usY �����ض�ɨ�跽�����ַ�����ʼY����
 * @param  pStr ��Ҫ��ʾ��Ӣ���ַ������׵�ַ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DispString_EN ( 	uint16_t usX ,uint16_t usY,  char * pStr )
{
	while ( * pStr != '\0' )
	{
		if ( ( usX - NT35510_DispWindow_X_Star + LCD_Currentfonts->Width ) > LCD_X_LENGTH )
		{
			usX = NT35510_DispWindow_X_Star;
			usY += LCD_Currentfonts->Height;
		}
		
		if ( ( usY - NT35510_DispWindow_Y_Star + LCD_Currentfonts->Height ) > LCD_Y_LENGTH )
		{
			usX = NT35510_DispWindow_X_Star;
			usY = NT35510_DispWindow_Y_Star;
		}
		
		NT35510_DispChar_EN ( usX, usY, * pStr);
		
		pStr ++;
		
		usX += LCD_Currentfonts->Width;
		
	}
	
}


/**
 * @brief  �� NT35510 ��ʾ������ʾӢ���ַ���(��Y�᷽��)
 * @param  usX �����ض�ɨ�跽�����ַ�����ʼX����
 * @param  usY �����ض�ɨ�跽�����ַ�����ʼY����
 * @param  pStr ��Ҫ��ʾ��Ӣ���ַ������׵�ַ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DispString_EN_YDir (	 uint16_t usX,uint16_t usY ,  char * pStr )
{	
	while ( * pStr != '\0' )
	{
		if ( ( usY - NT35510_DispWindow_Y_Star + LCD_Currentfonts->Height ) >LCD_Y_LENGTH  )
		{
			usY = NT35510_DispWindow_Y_Star;
			usX += LCD_Currentfonts->Width;
		}
		
		if ( ( usX - NT35510_DispWindow_X_Star + LCD_Currentfonts->Width ) >  LCD_X_LENGTH)
		{
			usX = NT35510_DispWindow_X_Star;
			usY = NT35510_DispWindow_Y_Star;
		}
		
		NT35510_DispChar_EN ( usX, usY, * pStr);
		
		pStr ++;
		
		usY += LCD_Currentfonts->Height;		
	}	
}



/**
 * @brief  �� NT35510 ��ʾ������ʾһ�������ַ�
 * @param  usX �����ض�ɨ�跽�����ַ�����ʼX����
 * @param  usY �����ض�ɨ�跽�����ַ�����ʼY����
 * @param  usChar ��Ҫ��ʾ�������ַ��������룩
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */ 
void NT35510_DispChar_CH ( uint16_t usX, uint16_t usY, uint16_t usChar )
{
	uint8_t rowCount, bitCount;
  uint32_t usTemp; 
	
//	ռ�ÿռ�̫�󣬸ĳ�ȫ�ֱ��� 
//	uint8_t ucBuffer [ WIDTH_CH_CHAR*HEIGHT_CH_CHAR/8 ];	
	
	//������ʾ����
	NT35510_OpenWindow ( usX, usY, WIDTH_CH_CHAR, HEIGHT_CH_CHAR );
	
	NT35510_Write_Cmd ( CMD_SetPixel );
	
	//ȡ��ģ����  
  GetGBKCode ( ucBuffer, usChar );	
	
	for ( rowCount = 0; rowCount < HEIGHT_CH_CHAR; rowCount++ )
	{
    /* ȡ���ĸ��ֽڵ����ݣ���lcd�ϼ���һ�����ֵ�һ�� */
		usTemp = ucBuffer [ rowCount * 4 ];
		usTemp = ( usTemp << 8 );
		usTemp |= ucBuffer [ rowCount * 4 + 1 ];
		usTemp = ( usTemp << 8 );
		usTemp |= ucBuffer [ rowCount * 4 + 2 ];
		usTemp = ( usTemp << 8 );
		usTemp |= ucBuffer [ rowCount * 4 + 3 ];
		
		for ( bitCount = 0; bitCount < WIDTH_CH_CHAR; bitCount ++ )
		{			
			if ( usTemp & ( 0x80000000 >> bitCount ) )  //��λ��ǰ 
			  NT35510_Write_Data ( CurrentTextColor );				
			else
				NT35510_Write_Data ( CurrentBackColor );			
		}		
	}
	
}


/**
 * @brief  �� NT35510 ��ʾ������ʾ�����ַ���
 * @param  line �����ض�ɨ�跽�����ַ�������ʼY����
  *   ��������ʹ�ú�LINE(0)��LINE(1)�ȷ�ʽָ���������꣬
  *   ��LINE(x)����ݵ�ǰѡ�������������Y����ֵ��
	*		��ʾ������ʹ��LINE��ʱ����Ҫ��Ӣ���������ó�Font8x16
 * @param  pStr ��Ҫ��ʾ��Ӣ���ַ������׵�ַ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DispString_CH ( 	uint16_t usX , uint16_t usY, char * pStr )
{	
	uint16_t usCh;

	
	while( * pStr != '\0' )
	{		
		if ( ( usX - NT35510_DispWindow_X_Star + WIDTH_CH_CHAR ) > LCD_X_LENGTH )
		{
			usX = NT35510_DispWindow_X_Star;
			usY += HEIGHT_CH_CHAR;
		}
		
		if ( ( usY - NT35510_DispWindow_Y_Star + HEIGHT_CH_CHAR ) > LCD_Y_LENGTH )
		{
			usX = NT35510_DispWindow_X_Star;
			usY = NT35510_DispWindow_Y_Star;
		}	
		
		usCh = * ( uint16_t * ) pStr;	
	  usCh = ( usCh << 8 ) + ( usCh >> 8 );

		NT35510_DispChar_CH ( usX, usY, usCh );
		
		usX += WIDTH_CH_CHAR;
		
		pStr += 2;           //һ�����������ֽ� 

	}	   
	
}


/**
 * @brief  �� NT35510 ��ʾ������ʾ��Ӣ���ַ���
 * @param  line �����ض�ɨ�跽�����ַ�������ʼY����
  *   ��������ʹ�ú�LINE(0)��LINE(1)�ȷ�ʽָ���������꣬
  *   ��LINE(x)����ݵ�ǰѡ�������������Y����ֵ��
	*		��ʾ������ʹ��LINE��ʱ����Ҫ��Ӣ���������ó�Font8x16
 * @param  pStr ��Ҫ��ʾ���ַ������׵�ַ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DispStringLine_EN_CH (  uint16_t line, char * pStr )
{
	uint16_t usCh;
	uint16_t usX = 0;
	
	while( * pStr != '\0' )
	{
		if ( * pStr <= 126 )	           	//Ӣ���ַ�
		{
			if ( ( usX - NT35510_DispWindow_X_Star + LCD_Currentfonts->Width ) > LCD_X_LENGTH )
			{
				usX = NT35510_DispWindow_X_Star;
				line += LCD_Currentfonts->Height;
			}
			
			if ( ( line - NT35510_DispWindow_Y_Star + LCD_Currentfonts->Height ) > LCD_Y_LENGTH )
			{
				usX = NT35510_DispWindow_X_Star;
				line = NT35510_DispWindow_Y_Star;
			}			
		
		  NT35510_DispChar_EN ( usX, line, * pStr );
			
			usX +=  LCD_Currentfonts->Width;
		
		  pStr ++;

		}
		
		else	                            //�����ַ�
		{
			if ( ( usX - NT35510_DispWindow_X_Star + WIDTH_CH_CHAR ) > LCD_X_LENGTH )
			{
				usX = NT35510_DispWindow_X_Star;
				line += HEIGHT_CH_CHAR;
			}
			
			if ( ( line - NT35510_DispWindow_Y_Star + HEIGHT_CH_CHAR ) > LCD_Y_LENGTH )
			{
				usX = NT35510_DispWindow_X_Star;
				line = NT35510_DispWindow_Y_Star;
			}	
			
			usCh = * ( uint16_t * ) pStr;	
			
			usCh = ( usCh << 8 ) + ( usCh >> 8 );		

			NT35510_DispChar_CH ( usX, line, usCh );
			
			usX += WIDTH_CH_CHAR;
			
			pStr += 2;           //һ�����������ֽ� 
		
    }
		
  }	
} 

/**
 * @brief  �� NT35510 ��ʾ������ʾ��Ӣ���ַ���
 * @param  usX �����ض�ɨ�跽�����ַ�����ʼX����
 * @param  usY �����ض�ɨ�跽�����ַ�����ʼY����
 * @param  pStr ��Ҫ��ʾ���ַ������׵�ַ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DispString_EN_CH ( 	uint16_t usX , uint16_t usY, char * pStr )
{
	uint16_t usCh;
	
	while( * pStr != '\0' )
	{
		if ( * pStr <= 126 )	           	//Ӣ���ַ�
		{
			if ( ( usX - NT35510_DispWindow_X_Star + LCD_Currentfonts->Width ) > LCD_X_LENGTH )
			{
				usX = NT35510_DispWindow_X_Star;
				usY += LCD_Currentfonts->Height;
			}
			
			if ( ( usY - NT35510_DispWindow_Y_Star + LCD_Currentfonts->Height ) > LCD_Y_LENGTH )
			{
				usX = NT35510_DispWindow_X_Star;
				usY = NT35510_DispWindow_Y_Star;
			}			
		
		  NT35510_DispChar_EN ( usX, usY, * pStr );
			
			usX +=  LCD_Currentfonts->Width;
		
		  pStr ++;

		}
		
		else	                            //�����ַ�
		{
			if ( ( usX - NT35510_DispWindow_X_Star + WIDTH_CH_CHAR ) > LCD_X_LENGTH )
			{
				usX = NT35510_DispWindow_X_Star;
				usY += HEIGHT_CH_CHAR;
			}
			
			if ( ( usY - NT35510_DispWindow_Y_Star + HEIGHT_CH_CHAR ) > LCD_Y_LENGTH )
			{
				usX = NT35510_DispWindow_X_Star;
				usY = NT35510_DispWindow_Y_Star;
			}	
			
			usCh = * ( uint16_t * ) pStr;	
			
			usCh = ( usCh << 8 ) + ( usCh >> 8 );		

			NT35510_DispChar_CH ( usX, usY, usCh );
			
			usX += WIDTH_CH_CHAR;
			
			pStr += 2;           //һ�����������ֽ� 
		
    }
		
  }	
} 

/**
 * @brief  �� NT35510 ��ʾ������ʾ��Ӣ���ַ���(��Y�᷽��)
 * @param  usX �����ض�ɨ�跽�����ַ�����ʼX����
 * @param  usY �����ض�ɨ�跽�����ַ�����ʼY����
 * @param  pStr ��Ҫ��ʾ����Ӣ���ַ������׵�ַ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void NT35510_DispString_EN_CH_YDir (  uint16_t usX,uint16_t usY , char * pStr )
{
	uint16_t usCh;
	
	while( * pStr != '\0' )
	{			
			//ͳһʹ�ú��ֵĿ��������㻻��
			if ( ( usY - NT35510_DispWindow_Y_Star + HEIGHT_CH_CHAR ) >LCD_Y_LENGTH  )
			{
				usY = NT35510_DispWindow_Y_Star;
				usX += WIDTH_CH_CHAR;
			}			
			if ( ( usX - NT35510_DispWindow_X_Star + WIDTH_CH_CHAR ) >  LCD_X_LENGTH)
			{
				usX = NT35510_DispWindow_X_Star;
				usY = NT35510_DispWindow_Y_Star;
			}
			
		//��ʾ	
		if ( * pStr <= 126 )	           	//Ӣ���ַ�
		{			
			NT35510_DispChar_EN ( usX, usY, * pStr);
			
			pStr ++;
			
			usY += HEIGHT_CH_CHAR;		
		}
		else	                            //�����ַ�
		{			
			usCh = * ( uint16_t * ) pStr;	
			
			usCh = ( usCh << 8 ) + ( usCh >> 8 );		

			NT35510_DispChar_CH ( usX,usY , usCh );
			
			usY += HEIGHT_CH_CHAR;
			
			pStr += 2;           //һ�����������ֽ� 
		
    }
		
  }	
} 

/***********************��������****************************/
#define ZOOMMAXBUFF 16384
uint8_t zoomBuff[ZOOMMAXBUFF] = {0};	//�������ŵĻ��棬���֧�ֵ�128*128
uint8_t zoomTempBuff[1024] = {0};

/**
 * @brief  ������ģ�����ź����ģ��1�����ص���8������λ����ʾ
										0x01��ʾ�ʼ���0x00��ʾ�հ���
 * @param  in_width ��ԭʼ�ַ�����
 * @param  in_heig ��ԭʼ�ַ��߶�
 * @param  out_width �����ź���ַ�����
 * @param  out_heig�����ź���ַ��߶�
 * @param  in_ptr ���ֿ�����ָ��	ע�⣺1pixel 1bit
 * @param  out_ptr �����ź���ַ����ָ�� ע��: 1pixel 8bit
 *		out_ptrʵ����û������������ĳ���ֱ�������ȫ��ָ��zoomBuff��
 * @param  en_cn ��0ΪӢ�ģ�1Ϊ����
 * @retval ��
 */
void NT35510_zoomChar(uint16_t in_width,	//ԭʼ�ַ�����
									uint16_t in_heig,		//ԭʼ�ַ��߶�
									uint16_t out_width,	//���ź���ַ�����
									uint16_t out_heig,	//���ź���ַ��߶�
									uint8_t *in_ptr,	//�ֿ�����ָ��	ע�⣺1pixel 1bit
									uint8_t *out_ptr, //���ź���ַ����ָ�� ע��: 1pixel 8bit
									uint8_t en_cn)		//0ΪӢ�ģ�1Ϊ����	
{
	uint8_t *pts,*ots;
	//����Դ��ģ��Ŀ����ģ��С���趨����������ӣ�����16��Ϊ�˰Ѹ�������ת�ɶ�������
	unsigned int xrIntFloat_16=(in_width<<16)/out_width+1; 
  unsigned int yrIntFloat_16=(in_heig<<16)/out_heig+1;
	
	unsigned int srcy_16=0;
	unsigned int y,x;
	uint8_t *pSrcLine;
	
	uint16_t byteCount,bitCount;
	
	//�������Ƿ�Ϸ�
	if(in_width > 48) return;												//�ֿⲻ��������48����
	if(in_width * in_heig == 0) return;	
	if(in_width * in_heig > 48*48 ) return; 					//����������� 48*48
	
	if(out_width * out_heig == 0) return;	
	if(out_width * out_heig >= ZOOMMAXBUFF ) return; //����������� 128*128
	pts = (uint8_t*)&zoomTempBuff;
	
	//Ϊ�������㣬�ֿ��������1 pixel/1bit ӳ�䵽1pixel/8bit
	//0x01��ʾ�ʼ���0x00��ʾ�հ���
	if(en_cn == 0x00)//Ӣ��
	{
		//Ӣ�ĺ������ֿ����±߽粻�ԣ����ڴ˴���������Ҫע��tempBuff��ֹ���
			for(byteCount=0;byteCount<in_heig*in_width/8;byteCount++)	
			{
				for(bitCount=0;bitCount<8;bitCount++)
					{						
						//��Դ��ģ������λӳ�䵽�ֽ�
						//in_ptr��bitXΪ1����pts�������ֽ�ֵΪ1
						//in_ptr��bitXΪ0����pts�������ֽ�ֵΪ0
						*pts++ = (in_ptr[byteCount] & (0x80>>bitCount))?1:0; 
					}
			}				
	}
	else //����
	{			
			for(byteCount=0;byteCount<in_heig*in_width/8;byteCount++)	
			{
				for(bitCount=0;bitCount<8;bitCount++)
					{						
						//��Դ��ģ������λӳ�䵽�ֽ�
						//in_ptr��bitXΪ1����pts�������ֽ�ֵΪ1
						//in_ptr��bitXΪ0����pts�������ֽ�ֵΪ0
						*pts++ = (in_ptr[byteCount] & (0x80>>bitCount))?1:0; 
					}
			}		
	}

	//zoom����
	pts = (uint8_t*)&zoomTempBuff;	//ӳ����Դ����ָ��
	ots = (uint8_t*)&zoomBuff;	//������ݵ�ָ��
	for (y=0;y<out_heig;y++)	/*�б���*/
    {
				unsigned int srcx_16=0;
        pSrcLine=pts+in_width*(srcy_16>>16);				
        for (x=0;x<out_width;x++) /*�������ر���*/
        {
            ots[x]=pSrcLine[srcx_16>>16]; //��Դ��ģ���ݸ��Ƶ�Ŀ��ָ����
            srcx_16+=xrIntFloat_16;			//������ƫ��Դ���ص�
        }
        srcy_16+=yrIntFloat_16;				  //������ƫ��Դ���ص�
        ots+=out_width;						
    }
	/*���������ź����ģ����ֱ�Ӵ洢��ȫ��ָ��zoomBuff����*/
	out_ptr = (uint8_t*)&zoomBuff;	//out_ptrû����ȷ�������������ֱ�Ӹĳ���ȫ�ֱ���ָ�룡
	
	/*ʵ�������ʹ��out_ptr����Ҫ������һ�䣡����
		ֻ����Ϊout_ptrû��ʹ�ã��ᵼ��warning��ǿ��֢*/
	out_ptr++; 
}			


/**
 * @brief  �������ź����ģ��ʾ�ַ�
 * @param  Xpos ���ַ���ʾλ��x
 * @param  Ypos ���ַ���ʾλ��y
 * @param  Font_width ���ַ�����
 * @param  Font_Heig���ַ��߶�
 * @param  c ��Ҫ��ʾ����ģ����
 * @param  DrawModel ���Ƿ�ɫ��ʾ 
 * @retval ��
 */
void NT35510_DrawChar_Ex(uint16_t usX, //�ַ���ʾλ��x
												uint16_t usY, //�ַ���ʾλ��y
												uint16_t Font_width, //�ַ�����
												uint16_t Font_Height,  //�ַ��߶� 
												uint8_t *c,						//��ģ����
												uint16_t DrawModel)		//�Ƿ�ɫ��ʾ
{
  uint32_t index = 0, counter = 0;

	//������ʾ����
	NT35510_OpenWindow ( usX, usY, Font_width, Font_Height);
	
	NT35510_Write_Cmd ( CMD_SetPixel );		
	
	//���ֽڶ�ȡ��ģ����
	//����ǰ��ֱ����������ʾ���ڣ���ʾ���ݻ��Զ�����
	for ( index = 0; index < Font_Height; index++ )
	{
			//һλһλ����Ҫ��ʾ����ɫ
			for ( counter = 0; counter < Font_width; counter++ )
			{
					//���ź����ģ���ݣ���һ���ֽڱ�ʾһ������λ
					//�����ֽ�ֵΪ1��ʾ������Ϊ�ʼ�
					//�����ֽ�ֵΪ0��ʾ������Ϊ����
					if ( *c++ == DrawModel )
						NT35510_Write_Data ( CurrentBackColor );			
					else
						NT35510_Write_Data ( CurrentTextColor );
			}	
	}	
}


/**
 * @brief  �������ź����ģ��ʾ�ַ���
 * @param  Xpos ���ַ���ʾλ��x
 * @param  Ypos ���ַ���ʾλ��y
 * @param  Font_width ���ַ����ȣ�Ӣ���ַ��ڴ˻�����/2��ע��Ϊż��
 * @param  Font_Heig���ַ��߶ȣ�ע��Ϊż��
 * @param  c ��Ҫ��ʾ���ַ���
 * @param  DrawModel ���Ƿ�ɫ��ʾ 
 * @retval ��
 */
void NT35510_DisplayStringEx(uint16_t x, 		//�ַ���ʾλ��x
														 uint16_t y, 				//�ַ���ʾλ��y
														 uint16_t Font_width,	//Ҫ��ʾ��������ȣ�Ӣ���ַ��ڴ˻�����/2��ע��Ϊż��
														 uint16_t Font_Height,	//Ҫ��ʾ������߶ȣ�ע��Ϊż��
														 uint8_t *ptr,					//��ʾ���ַ�����
														 uint16_t DrawModel)  //�Ƿ�ɫ��ʾ



{
	uint16_t Charwidth = Font_width; //Ĭ��ΪFont_width��Ӣ�Ŀ���Ϊ���Ŀ��ȵ�һ��
	uint8_t Ascii;	//Ӣ��
	uint16_t usCh;  //����
	
	//ռ�ÿռ�̫�󣬸ĳ�ȫ�ֱ���	
	//	uint8_t ucBuffer [ WIDTH_CH_CHAR*HEIGHT_CH_CHAR/8 ];	
	
	while ( *ptr != '\0')
	{
			/****��������*****/
			if ( ( x - NT35510_DispWindow_X_Star + Charwidth ) > LCD_X_LENGTH )
			{
				x = NT35510_DispWindow_X_Star;
				y += Font_Height;
			}
			
			if ( ( y - NT35510_DispWindow_Y_Star + Font_Height ) > LCD_Y_LENGTH )
			{
				x = NT35510_DispWindow_X_Star;
				y = NT35510_DispWindow_Y_Star;
			}	
			
		if(*ptr > 0x80) //���������
		{			
			Charwidth = Font_width;
			usCh = * ( uint16_t * ) ptr;				
			usCh = ( usCh << 8 ) + ( usCh >> 8 );
			GetGBKCode ( ucBuffer, usCh );	//ȡ��ģ����
			//������ģ���ݣ�Դ��ģΪ32*32
			NT35510_zoomChar(WIDTH_CH_CHAR,HEIGHT_CH_CHAR,Charwidth,Font_Height,(uint8_t *)&ucBuffer,(uint8_t *)&zoomBuff,1); 
			//��ʾ�����ַ�
			NT35510_DrawChar_Ex(x,y,Charwidth,Font_Height,(uint8_t*)&zoomBuff,DrawModel);
			x+=Charwidth;
			ptr+=2;
		}
		else
		{
				Charwidth = Font_width / 2;
				Ascii = *ptr - 32;
				//ʹ��16*32����������ģ����
				NT35510_zoomChar(16,32,Charwidth,Font_Height,(uint8_t *)&Font16x32.table[Ascii * Font16x32.Height*Font16x32.Width/8],(uint8_t *)&zoomBuff,0);
			  //��ʾ�����ַ�
				NT35510_DrawChar_Ex(x,y,Charwidth,Font_Height,(uint8_t*)&zoomBuff,DrawModel);
				x+=Charwidth;
				ptr++;
		}
	}
}


/**
 * @brief  �������ź����ģ��ʾ�ַ���(��Y�᷽��)
 * @param  Xpos ���ַ���ʾλ��x
 * @param  Ypos ���ַ���ʾλ��y
 * @param  Font_width ���ַ����ȣ�Ӣ���ַ��ڴ˻�����/2��ע��Ϊż��
 * @param  Font_Heig���ַ��߶ȣ�ע��Ϊż��
 * @param  c ��Ҫ��ʾ���ַ���
 * @param  DrawModel ���Ƿ�ɫ��ʾ 
 * @retval ��
 */
void NT35510_DisplayStringEx_YDir(uint16_t x, 		//�ַ���ʾλ��x
																		 uint16_t y, 				//�ַ���ʾλ��y
																		 uint16_t Font_width,	//Ҫ��ʾ��������ȣ�Ӣ���ַ��ڴ˻�����/2��ע��Ϊż��
																		 uint16_t Font_Height,	//Ҫ��ʾ������߶ȣ�ע��Ϊż��
																		 uint8_t *ptr,					//��ʾ���ַ�����
																		 uint16_t DrawModel)  //�Ƿ�ɫ��ʾ
{
	uint16_t Charwidth = Font_width; //Ĭ��ΪFont_width��Ӣ�Ŀ���Ϊ���Ŀ��ȵ�һ��
	uint8_t Ascii;	//Ӣ��
	uint16_t usCh;  //����
	uint8_t ucBuffer [ WIDTH_CH_CHAR*HEIGHT_CH_CHAR/8 ];	
	
	while ( *ptr != '\0')
	{			
			//ͳһʹ�ú��ֵĿ��������㻻��
			if ( ( y - NT35510_DispWindow_X_Star + Font_width ) > LCD_X_LENGTH )
			{
				y = NT35510_DispWindow_X_Star;
				x += Font_width;
			}
			
			if ( ( x - NT35510_DispWindow_Y_Star + Font_Height ) > LCD_Y_LENGTH )
			{
				y = NT35510_DispWindow_X_Star;
				x = NT35510_DispWindow_Y_Star;
			}	
			
		if(*ptr > 0x80) //���������
		{			
			Charwidth = Font_width;
			usCh = * ( uint16_t * ) ptr;				
			usCh = ( usCh << 8 ) + ( usCh >> 8 );
			GetGBKCode ( ucBuffer, usCh );	//ȡ��ģ����
			//������ģ���ݣ�Դ��ģΪ16*16
			NT35510_zoomChar(WIDTH_CH_CHAR,HEIGHT_CH_CHAR,Charwidth,Font_Height,(uint8_t *)&ucBuffer,(uint8_t *)&zoomBuff,1); 
			//��ʾ�����ַ�
			NT35510_DrawChar_Ex(x,y,Charwidth,Font_Height,(uint8_t*)&zoomBuff,DrawModel);
			y+=Font_Height;
			ptr+=2;
		}
		else
		{
				Charwidth = Font_width / 2;
				Ascii = *ptr - 32;
				//ʹ��16*24����������ģ����
				NT35510_zoomChar(16,24,Charwidth,Font_Height,(uint8_t *)&Font16x32.table[Ascii * Font16x32.Height*Font16x32.Width/8],(uint8_t *)&zoomBuff,0);
			  //��ʾ�����ַ�
				NT35510_DrawChar_Ex(x,y,Charwidth,Font_Height,(uint8_t*)&zoomBuff,DrawModel);
				y+=Font_Height;
				ptr++;
		}
	}
}

/**
  * @brief  ����Ӣ����������
  * @param  fonts: ָ��Ҫѡ�������
	*		����Ϊ����ֵ֮һ
  * 	@arg��Font24x32;
  * 	@arg��Font16x24;
  * 	@arg��Font8x16;
  * @retval None
  */
void LCD_SetFont(sFONT *fonts)
{
  LCD_Currentfonts = fonts;
}

/**
  * @brief  ��ȡ��ǰ��������
  * @param  None.
  * @retval ���ص�ǰ��������
  */
sFONT *LCD_GetFont(void)
{
  return LCD_Currentfonts;
}


/**
  * @brief  ����LCD��ǰ��(����)��������ɫ,RGB565
  * @param  TextColor: ָ��ǰ��(����)��ɫ
  * @param  BackColor: ָ��������ɫ
  * @retval None
  */
void LCD_SetColors(uint16_t TextColor, uint16_t BackColor) 
{
  CurrentTextColor = TextColor; 
  CurrentBackColor = BackColor;
}

/**
  * @brief  ��ȡLCD��ǰ��(����)��������ɫ,RGB565
  * @param  TextColor: �����洢ǰ��(����)��ɫ��ָ�����
  * @param  BackColor: �����洢������ɫ��ָ�����
  * @retval None
  */
void LCD_GetColors(uint16_t *TextColor, uint16_t *BackColor)
{
  *TextColor = CurrentTextColor;
  *BackColor = CurrentBackColor;
}

/**
  * @brief  ����LCD��ǰ��(����)��ɫ,RGB565
  * @param  Color: ָ��ǰ��(����)��ɫ 
  * @retval None
  */
void LCD_SetTextColor(uint16_t Color)
{
  CurrentTextColor = Color;
}

/**
  * @brief  ����LCD�ı�����ɫ,RGB565
  * @param  Color: ָ��������ɫ 
  * @retval None
  */
void LCD_SetBackColor(uint16_t Color)
{
  CurrentBackColor = Color;
}

/**
  * @brief  ���ĳ������
  * @param  Line: ָ��Ҫɾ������
  *   ��������ʹ�ú�LINE(0)��LINE(1)�ȷ�ʽָ��Ҫɾ�����У�
  *   ��LINE(x)����ݵ�ǰѡ�������������Y����ֵ����ɾ����ǰ����߶ȵĵ�x�С�
  * @retval None
  */
void NT35510_ClearLine(uint16_t Line)
{
  NT35510_Clear(0,Line,LCD_X_LENGTH,((sFONT *)LCD_GetFont())->Height);	/* ��������ʾȫ�� */

}
/*********************end of file*************************/



