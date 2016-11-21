/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Custom HID demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "Misc.h"
#include "stm32f10x_it.h" 
#include "Stm32f10x_exti.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "Usb_desc.h"

#include "Remote.h"
#include "delay.h"

extern void TIM2_IRQHandler(void);
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define         ID1          (0x1FFFF7E8)
#define         ID2          (0x1FFFF7EC)
#define         ID3          (0x1FFFF7F0)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
__IO uint8_t PrevXferComplete = 1;
__IO uint32_t TimingDelay = 0;
uint8_t gu8KeyNeedClear = 0;

/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);

void Delay(__IO uint32_t nCount);
void TIMER_cfg(void);
void RCC_cfg(void);
void NVIC_cfg(void);

void Set_System(void);
void Set_USBClock(void);
void USB_Interrupts_Config(void);
void EXTI_Configuration(void);
void Leave_LowPowerMode(void);
void Get_SerialNum(void);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
	uint8_t key = 0;
	uint8_t Keyboad_Buf[8] = {0,0,0,0,0,0,0,0};	

	delay_init();
#if 1
  Set_System();

	USB_Interrupts_Config();

	Set_USBClock();

	USB_Init();

#endif	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2	 

	Remote_Init();
	
  while(1)
  {
	#if 1
		key = Remote_Scan();
		
		switch (key)		
		{
			case 0xA2:
				if ((PrevXferComplete) && (CONFIGURED == bDeviceState))
				{
					if ((uint8_t)0 == gu8KeyNeedClear)
					{
						Keyboad_Buf[2] = (uint8_t)0x50;
					    USB_SIL_Write(EP1_IN, Keyboad_Buf, 8);
						SetEPTxValid(ENDP1);

						gu8KeyNeedClear = (uint8_t)1;

						PrevXferComplete = 0;
					}			
				}
			
				break;
			
			case 0xE2:
				if ((PrevXferComplete) && (CONFIGURED == bDeviceState))
				{
					if ((uint8_t)0 == gu8KeyNeedClear)
					{
						Keyboad_Buf[2] = (uint8_t)0x4f;
					    USB_SIL_Write(EP1_IN, Keyboad_Buf, 8);
						SetEPTxValid(ENDP1);

						gu8KeyNeedClear = (uint8_t)1;

						PrevXferComplete = 0;
					}			
				}
				
				break;
	    
      case 0x62:
				if ((PrevXferComplete) && (CONFIGURED == bDeviceState))
				{
					if ((uint8_t)0 == gu8KeyNeedClear)
					{
						Keyboad_Buf[2] = (uint8_t)0x3e;
					    USB_SIL_Write(EP1_IN, Keyboad_Buf, 8);
						SetEPTxValid(ENDP1);

						gu8KeyNeedClear = (uint8_t)1;

						PrevXferComplete = 0;
					}			
				}
				
				break;				

  		case 0xE0:
				if ((PrevXferComplete) && (CONFIGURED == bDeviceState))
				{
					if ((uint8_t)0 == gu8KeyNeedClear)
					{
						Keyboad_Buf[2] = (uint8_t)0x05;
					    USB_SIL_Write(EP1_IN, Keyboad_Buf, 8);
						SetEPTxValid(ENDP1);

						gu8KeyNeedClear = (uint8_t)1;

						PrevXferComplete = 0;
					}			
				}
				
				break;			

      case 0xA8:
				if ((PrevXferComplete) && (CONFIGURED == bDeviceState))
				{
					if ((uint8_t)0 == gu8KeyNeedClear)
					{
						Keyboad_Buf[2] = (uint8_t)0x1A;
					    USB_SIL_Write(EP1_IN, Keyboad_Buf, 8);
						SetEPTxValid(ENDP1);

						gu8KeyNeedClear = (uint8_t)1;

						PrevXferComplete = 0;
					}			
				}
				
				break;		

			case 0x90:
				if ((PrevXferComplete) && (CONFIGURED == bDeviceState))
				{
					if ((uint8_t)0 == gu8KeyNeedClear)
					{
						Keyboad_Buf[2] = (uint8_t)0x29;
					    USB_SIL_Write(EP1_IN, Keyboad_Buf, 8);
						SetEPTxValid(ENDP1);

						gu8KeyNeedClear = (uint8_t)1;

						PrevXferComplete = 0;
					}			
				}
				
				break;							
				
			case 0:
				break;
			
			default:
				break;
		}
		#endif	
	}
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;
  
  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;
  
  Device_Serial0 += Device_Serial2;
  
  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &CustomHID_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &CustomHID_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode.
* Description    : Restores system clocks and power while exiting suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;
  
  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else 
  {
    bDeviceState = ATTACHED;
  }
  /*Enable SystemCoreClock*/
  SystemInit();
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
  /* Enable The HSI (16Mhz) */
  RCC_HSICmd(ENABLE); 
#endif
#if defined(STM32F30X)
  ADC30x_Configuration();
#endif
}

void Set_System(void)
{ 
  GPIO_InitTypeDef  GPIO_InitStructure;
	
  /*Set PA11,12 as IN - USB_DM,DP*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  /* Additional EXTI configuration (configure both edges) */
  EXTI_Configuration();   
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  
#else
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config.
* Description    : Configures the USB interrupts.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
  
  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
  
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_FS_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#elif defined(STM32F37X)
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#else
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif /* STM32L1XX_XD */
  
  /* Enable the EXTI9_5 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the EXTI15_10 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the DMA1 Channel1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_Init(&NVIC_InitStructure);
  
}

/*******************************************************************************
* Function Name : EXTI_Configuration.
* Description   : Configure the EXTI lines for Key and Tamper push buttons.
* Input         : None.
* Output        : None.
* Return value  : The direction value.
*******************************************************************************/
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
   
  /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void TIMER_cfg()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//���½�Timer����Ϊȱʡֵ
	TIM_DeInit(TIM2);

	//�����ڲ�ʱ�Ӹ�TIM2�ṩʱ��Դ
	TIM_InternalClockConfig(TIM2);

	//Ԥ��Ƶϵ��Ϊ36000-1������������ʱ��Ϊ72MHz/36000 = 2kHz
	TIM_TimeBaseStructure.TIM_Prescaler = 36000 - 1;

	//����ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	//���ü�����ģʽΪ���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	//���ü��������С��ÿ��2000�����Ͳ���һ�������¼�
	TIM_TimeBaseStructure.TIM_Period = 2000 - 1;

	//������Ӧ�õ�TIM2��
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	//�������жϱ�־
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	//��ֹARRԤװ�ػ�����
	TIM_ARRPreloadConfig(TIM2, DISABLE);

	//����TIM2���ж�
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
}

void RCC_cfg()
{
	//�������״̬����
	ErrorStatus HSEStartUpStatus;

	//��RCC�Ĵ�����������ΪĬ��ֵ
	RCC_DeInit();

	//���ⲿ����ʱ�Ӿ���
	RCC_HSEConfig(RCC_HSE_ON);

	//�ȴ��ⲿ����ʱ�Ӿ�����
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
		//����AHBʱ��(HCLK)Ϊϵͳʱ��
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		//���ø���AHBʱ��(APB2)ΪHCLKʱ��
		RCC_PCLK2Config(RCC_HCLK_Div1);

		//���õ���AHBʱ��(APB1)ΪHCLK��2��Ƶ
		RCC_PCLK1Config(RCC_HCLK_Div2);

		//����FLASH������ʱ
		// FLASH_SetLatency(FLASH_Latency_2);

		//ʹ��Ԥȡָ����
		// FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		//����PLLʱ�ӣ�ΪHSE��9��Ƶ 8MHz * 9 = 72MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		//ʹ��PLL
		RCC_PLLCmd(ENABLE);

		//�ȴ�PLL׼������
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

		//����PLLΪϵͳʱ��Դ
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		//�ж�PLL�Ƿ���ϵͳʱ��
		while(RCC_GetSYSCLKSource() != 0x08);

	}

	//����TIM2��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	//����GPIO��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
}

void NVIC_cfg()
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//ѡ���жϷ���1
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	//ѡ��TIM2���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;      

	//��ռʽ�ж����ȼ�����Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;

	//��Ӧʽ�ж����ȼ�����Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	//ʹ���ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(__IO uint32_t nCount)
{
  TimingDelay = nCount;
  for(; nCount!= 0;nCount--);
}

#ifdef  USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while(1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
