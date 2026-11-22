/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include "string.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Target=2000,Actual=0,OUT=0;
float Kp=0,Ki=0,Kd=0;
float Error0=0,Error1=0,ErrorInt=0;
int16_t cur=0;
float OUT_MAX=3000;
float OUT_MIN=-3000;
float INT_MAX=1100;
float INT_MIN=-1100;
char buf[64];
uint32_t print_cnt=0;


volatile uint8_t dbus_buf[18];
volatile uint8_t dbus_ready=0;
volatile uint8_t S1,S2;
int16_t ch0_raw;

uint32_t last_time=0;
volatile uint8_t mode_flag=0;
uint8_t last_mode=0;
float last_target=2000;
float C=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Send_Msg1(uint32_t CAN_TX_ID,uint8_t *data,uint8_t len);
uint8_t CAN_Receive_Msg(CAN_RxHeaderTypeDef *rxHeader,uint8_t *rxData);
float PID_Control(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CAN_Filter_Init(CAN_HandleTypeDef *hcan,uint32_t filter_bank)
{
  CAN_FilterTypeDef sFilterConfig = {0};
  
  sFilterConfig.FilterBank = filter_bank; //选择过滤器组
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//选择掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//选择过滤器位宽
  sFilterConfig.FilterIdHigh = 0x0000;//参考id
  sFilterConfig.FilterIdLow = 0x0000;//参考id
  sFilterConfig.FilterMaskIdHigh = 0x0000;//MASK掩码
  sFilterConfig.FilterMaskIdLow = 0x0000;//MASK掩码
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;//送到哪一个FIFO
  sFilterConfig.FilterActivation = ENABLE;//过滤器开关
  
 
  sFilterConfig.SlaveStartFilterBank = 14;  // 指定 CAN2 的起始滤波器组（0~13 给 CAN1，14~27 给 CAN2）
  
  HAL_CAN_ConfigFilter(hcan, &sFilterConfig); //把过滤器配置写入寄存器
}


void DBUS_Decode(uint8_t *buf2)
{
	S1=(buf2[5]>>6)&0x03;
	S2=(buf2[5]>>4)&0x03;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
CAN_Filter_Init(&hcan1,0);
HAL_CAN_Start(&hcan1);


HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)dbus_buf, 18);
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(dbus_ready)
	  {
		  dbus_ready=0;
		  if(S1==1)
		  {
		  if(S2==2)
			  Target=8190;
		  if(S2==1)
			  Target=2047;
		  Ki=0;
		  Kp=0.1;
		  Kd=0.25;
		  mode_flag=1;
	      }
		  if(S1==2)
		  {
			  if(S2==2)
			  Target=2000;
		      if(S2==1)
			  Target=-2000;
			  mode_flag=2;
			  Kp=0.6;
			  Ki=0.6;
			  Kd=0.05;
		  }
	  }
	  
		  if(mode_flag!=last_mode || Target!=last_target)
		  {
			  Error0=0;
			  Error1=0;
			  ErrorInt=0;
			  OUT=0;
			  last_mode=mode_flag;
			  last_target=Target;
		  }

		  uint32_t now=HAL_GetTick();
	  if(now-last_time>10)
	  {
	  cur=PID_Control();
		  if(mode_flag==1)
		  {
		  	if(cur>0)
		    cur+=400;
	        if(cur<0)
		    cur-=400;
	        if(fabs(Error0)<20)
		    cur=0;
          }

	  uint8_t send_buf[8]={(cur>>8)&0xFF,cur&0xFF,0,0,0,0,0,0};
	  CAN_Send_Msg1(0x200,send_buf,8);
	  last_time=now;
  }
	  
	  CAN_RxHeaderTypeDef my_header;//定义结构体
	  uint8_t my_data[8];
	  if(CAN_Receive_Msg(&my_header,my_data)==1 && my_header.StdId==0x201)
	  {
		  uint16_t raw;
		  if(mode_flag==1)
		  {
		  raw=((uint16_t)my_data[0]<<8)|my_data[1];
		  }
		  if(mode_flag==2)
		  {
			  raw=((uint16_t)my_data[2]<<8)|my_data[3];
		  }
		  Actual=(float)(int16_t)raw;
		  		 
	  }
print_cnt++;
      if (print_cnt >= 1000) 
      {
          print_cnt = 0;
		  sprintf(buf,"%.2f,%.2f,%.2f\n",Actual,OUT,Target); 
          HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf),1000);	
	  }		  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CAN_Send_Msg1(uint32_t CAN_TX_ID,uint8_t *data,uint8_t len)
{
	CAN_TxHeaderTypeDef txHeader;//定义结构体
	uint32_t txMailbox;//邮箱编号
	txHeader.StdId=CAN_TX_ID;//设置id
	txHeader.IDE=CAN_ID_STD;//选择标准帧
	txHeader.RTR=CAN_RTR_DATA;//选择数据帧
	txHeader.DLC=len;//手写报文长度
	HAL_CAN_AddTxMessage(&hcan1,&txHeader,data,&txMailbox);

}
uint8_t CAN_Receive_Msg(CAN_RxHeaderTypeDef *rxHeader,uint8_t *rxData)
{
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0)>0)//确认收到了数据
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,rxHeader,rxData);
		return 1;
	}
	return 0;
}
float PID_Control(void)
{
	Error1=Error0;
	Error0=Target-Actual;
	if(mode_flag==1)
	{

		if(Error0>4096)
			Error0-=8192;
		else if(Error0<-4096)
			Error0+=8192;
	}
	ErrorInt+=Error0;
	if(ErrorInt>INT_MAX)
		ErrorInt=INT_MAX;
	if(ErrorInt<INT_MIN)
		ErrorInt=INT_MIN;
	
	OUT=Kp*Error0+Ki*ErrorInt+Kd*(Error0-Error1);
	if(OUT>OUT_MAX)
		OUT=OUT_MAX;
	if(OUT<OUT_MIN)
		OUT=OUT_MIN;

	
	return OUT;
}


/*中断回调*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	if(huart==&huart3)
	{
		if(Size == 18)          /* 只有完整一帧才解码，半帧丢弃 */
		{
			DBUS_Decode((uint8_t *)dbus_buf);
			dbus_ready = 1;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)dbus_buf, 18);
	}
}

/* UART 出错后必须重新武装接收，否则接收永久停止 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart3)
	{
		__HAL_UART_CLEAR_OREFLAG(huart);
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)dbus_buf, 18);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
