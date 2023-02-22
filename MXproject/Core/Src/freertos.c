/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"

#include "usbd_cdc_if.h"
#include "adc.h"
#include "i2c.h"

#include "ssd1306.h"
#include "ssd1306_tests.h"

#include "ads1115.h"

#include "mcp4726.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define SSDDEBUG
#define ack_len 5

#define sendAck(ackData, num, task) {\
	ackData[0] = num;\
	ackData[1] = task;\
	CDC_Transmit_FS((uint8_t *) ackData, ack_len);\
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId ssdThreadId;
osThreadId comThreadId;

uint8_t buffer[64];
uint8_t newData;

uint32_t adc_dma[14];

struct AcdPV {
	uint16_t adc_pvu;
	uint16_t adc_pvi;
	uint8_t adc_ampi;
};

struct SSDData {
	uint16_t dac_led;
	uint16_t dac_pv;
	float adc_pvu;
	float adc_pvi;
	float adc_led1;
	float adc_led2;
	float adc_36;
	uint16_t pv_gate;
	uint8_t err_code;
} ssdData;

struct COMData {
	struct SSDData * ssd;
	uint8_t buff[64];
	uint8_t res[16];
	uint8_t recv[2];
} comData;



/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vSSDThread(void const * argument);
void vCOMThread(void const * argument);

#if defined(SSDDEBUG)
void ssdDataClear(struct SSDData *);
void ssdUpdateDacLed(struct COMData *, uint16_t);
void ssdUpdateDacPV(struct COMData *, uint16_t);
void ssdUpdateAdcPV(struct COMData *);
void ssdUpdateGate(struct COMData *, uint8_t);
#endif

uint8_t comReset();
uint8_t comSetDacLed(uint16_t);
uint8_t comSetDacPV(uint16_t);
uint8_t comSendAdcPV(struct COMData *, uint8_t);
uint8_t comSendAdcOther(uint8_t *, uint8_t);
uint8_t comSetPVGate(uint8_t);
uint8_t comSetLEDGate(uint8_t);
uint8_t comSetPVFan(uint8_t);
uint8_t comSetLEDFan(uint8_t);
/* USER CODE END FunctionPrototypes */

void vDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, vDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(ssdThread, vSSDThread, osPriorityNormal, 1, 512);
  osThreadDef(comThread, vCOMThread, osPriorityNormal, 1, 512);
  comData.ssd = &ssdData;
  ssdThreadId = osThreadCreate(osThread(ssdThread), comData.ssd);
  comThreadId = osThreadCreate(osThread(comThread), &comData);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_vDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vDefaultTask */
void vDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN vDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END vDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void vSSDThread(void const * arg)
{

//#if defined(SSDDEBUG)
	struct SSDData * ssdData = (struct SSDData * ) arg;
	char buff[10];
//#endif
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
#if defined(SSDDEBUG)
		ssd1306_Fill(0x00);

		snprintf(buff, sizeof(buff), "LD:%u", ssdData->dac_led);
		ssd1306_SetCursor(2, 0);
		ssd1306_WriteString(buff, Font_6x8, White);

		snprintf(buff, sizeof(buff), "PV:%u", ssdData->dac_pv);
		ssd1306_SetCursor(56, 0);
		ssd1306_WriteString(buff, Font_6x8, White);

		snprintf(buff, sizeof(buff), "U:%.5f", ssdData->adc_pvu);
		ssd1306_SetCursor(62, 16);
		ssd1306_WriteString(buff, Font_6x8, White);

		snprintf(buff, sizeof(buff), "I:%.5f", ssdData->adc_pvi);
		ssd1306_SetCursor(62, 24);
		ssd1306_WriteString(buff, Font_6x8, White);

		ssdData->adc_led1 = adc_dma[0] * 0.0008058608;
		snprintf(buff, sizeof(buff), "1L:%.3f", ssdData->adc_led1);
		ssd1306_SetCursor(2, 8);
		ssd1306_WriteString(buff, Font_6x8, White);

		ssdData->adc_led2 = adc_dma[1] * 0.0008058608;
		snprintf(buff, sizeof(buff), "2L:%.3f", ssdData->adc_led2);
		ssd1306_SetCursor(56, 8);
		ssd1306_WriteString(buff, Font_6x8, White);

		ssdData->adc_36 = adc_dma[2] * 0.0008058608;
		snprintf(buff, sizeof(buff), "36V%.3f", ssdData->adc_36);
		ssd1306_SetCursor(2, 16);
		ssd1306_WriteString(buff, Font_6x8, White);

		snprintf(buff, sizeof(buff), "G:%u", ssdData->pv_gate);
		ssd1306_SetCursor(110, 8);
		ssd1306_WriteString(buff, Font_6x8, White);

		snprintf(buff, sizeof(buff), "E:%u", ssdData->err_code);
		ssd1306_SetCursor(110, 0);
		ssd1306_WriteString(buff, Font_6x8, White);
		//recv buffer

		ssd1306_SetCursor(2, 24);
		for( int i = 0; i < 4; i++)
		{
			snprintf(buff, 3, "%02x", buffer[i]);
			ssd1306_WriteChar(buff[0], Font_6x8, White);
			ssd1306_WriteChar(buff[1], Font_6x8, White);
		}

		//ssd1306_WriteString(buffer, Font_6x8, White);

		ssd1306_UpdateScreen();
#endif
		HAL_ADC_Start_DMA(&hadc1, adc_dma, 7);
		HAL_GPIO_TogglePin (GPIOC, GPIO_PIN_13);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

}

void vCOMThread(void const * arg)
{
	struct COMData * comData = (struct COMData * ) arg;
//#if defined(SSDDEBUG)
//	struct SSDData * ssdData = comData->ssdData;
#if defined(SSDDEBUG)
	ssdDataClear(comData->ssd);
#endif

	char ackData[ack_len] = "12ACK";
	uint8_t num = 1;

	for(;;)
	{		if(newData == 1)
		{
			memset(comData->buff, '\0', 64);
			memcpy(comData->buff, buffer, 64);
			newData = 0;
			uint16_t level;
			//parse

			if ( num == comData->buff[0]) //correct data
			{
				switch (comData->buff[1])
				{
				case 0u: //reset
					comReset();
#if defined(SSDDEBUG)
					ssdDataClear(comData->ssd);
#endif
					sendAck(ackData, num, 0u);
					break;
				case 1u: //set dac led
					taskENTER_CRITICAL();
					level = ((uint16_t)comData->buff[2] << 8) | (uint16_t)comData->buff[3];
					comSetDacLed(level);
					taskEXIT_CRITICAL();
#if defined(SSDDEBUG)
					ssdUpdateDacLed(comData, level);
#endif
					sendAck(ackData, num, 1u);
					break;
				case 2u: //set dac pv
					taskENTER_CRITICAL();
					level = ((uint16_t)comData->buff[2] << 8) | (uint16_t)comData->buff[3];
					comSetDacPV(level);
					taskEXIT_CRITICAL();
#if defined(SSDDEBUG)
					ssdUpdateDacPV(comData, level);
#endif
					sendAck(ackData, num, 2u);
					break;
				case 3u: //read adc pv
					taskENTER_CRITICAL();
					comSendAdcPV(comData, num);
					taskEXIT_CRITICAL();
#if defined(SSDDEBUG)
					ssdUpdateAdcPV(comData);
#endif
					break;
				case 4u: //read adc other
					taskENTER_CRITICAL();
					comSendAdcOther(comData->res, num);
					taskEXIT_CRITICAL();
					break;
				case 5u: //set pv gate
					comSetPVGate(comData->buff[2]);
#if defined(SSDDEBUG)
					ssdUpdateGate(comData, comData->buff[2]);
#endif
					sendAck(ackData, num, 5u);
					break;
				case 6u: //set led gate
					comSetLEDGate(comData->buff[2]);
#if defined(SSDDEBUG)
					ssdUpdateGate(comData, comData->buff[2]);
#endif
					sendAck(ackData, num, 6u);
					break;
				case 7u: //set pv fan
					comSetPVFan(comData->buff[2]);
#if defined(SSDDEBUG)
					ssdUpdateGate(comData, comData->buff[2]);
#endif
					sendAck(ackData, num, 7u);
					break;
				case 8u: //set led fan
					comSetLEDFan(comData->buff[2]);
#if defined(SSDDEBUG)
					ssdUpdateGate(comData, comData->buff[2]);
#endif
					sendAck(ackData, num, 8u);
					break;
				case 0x88: //Echo 7 bytes
					CDC_Transmit_FS((uint8_t *) comData->buff, 7);
					break;
				default:
					CDC_Transmit_FS((uint8_t *) comData->buff, 64);
					break;
				}

				num==0xff?num=1:num++; //inc num
			}
			else
			{
				if (comData->buff[0] == 0 && comData->buff[1] == 0x80)
				{
					num = 0x01;
				}
				else
				{
					char data[19] = "\0";
					data[1] = 1u;
					CDC_Transmit_FS((uint8_t *) data, 2);
				}
			}


		}
	}
}

//COM reactions

uint8_t comReset()
{
	comSetDacLed(0);
	comSetDacPV(0);
	comSetPVGate(0);
	return 0;
}

uint8_t comSetDacLed(uint16_t level)
{
	HAL_StatusTypeDef res = mcp4726_WriteOutputV(&hi2c1, level);
	return (uint8_t) res;
}

uint8_t comSetDacPV(uint16_t level)
{
	HAL_StatusTypeDef res = mcp4726_WriteOutputV(&hi2c2, level);
	return (uint8_t) res;
}

uint8_t comSendAdcPV(struct COMData * com, uint8_t num)
{
	com->res[0] = num;
	com->res[1] = 0x03;
	//Get PV voltage (MUX 0-1, PGA 2V)
	if (ads1115_GetValue(com->recv, ADS1115_MUX_0_GND, ADS1115_PGA_2V) != HAL_OK)
	{
		com->res[2] = 0xff;
		com->res[3] = 0xff;
	} else {
		com->res[2] = com->recv[0];
		com->res[3] = com->recv[1];
	}
	 //Get PV voltage (MUX 2-3, PGA 2V for now)
	if (ads1115_GetValue(com->recv, ADS1115_MUX_2_GND, ADS1115_PGA_2V) != HAL_OK)
	{
		com->res[4] = 0xff;
		com->res[5] = 0xff;
	} else {
		com->res[4] = com->recv[0];
		com->res[5] = com->recv[1];
	}
	com->res[6] = 0x00; //amp not implemented yet
	return CDC_Transmit_FS(com->res, 7);
}

uint8_t comSendAdcOther(uint8_t * res, uint8_t num)
{
	res[0] = num;
	res[1] = 0x04;
	res[2] = (uint8_t)((adc_dma[0] & 0x0f00) >> 8);
	res[3] = (uint8_t)(adc_dma[0] & 0x00ff);
	res[4] = (uint8_t)((adc_dma[1] & 0x0f00) >> 8);
	res[5] = (uint8_t)(adc_dma[1] & 0x00ff);
	res[6] = (uint8_t)((adc_dma[2] & 0x0f00) >> 8);
	res[7] = (uint8_t)(adc_dma[2] & 0x00ff);
	res[8] = (uint8_t)((adc_dma[3] & 0x0f00) >> 8);
	res[9] = (uint8_t)(adc_dma[3] & 0x00ff);
	res[10] = (uint8_t)((adc_dma[4] & 0x0f00) >> 8);
	res[11] = (uint8_t)(adc_dma[4] & 0x00ff);
	res[12] = (uint8_t)((adc_dma[5] & 0x0f00) >> 8);
	res[13] = (uint8_t)(adc_dma[5] & 0x00ff);
	res[14] = (uint8_t)((adc_dma[6] & 0x0f00) >> 8);
	res[15] = (uint8_t)(adc_dma[6] & 0x00ff);
	return CDC_Transmit_FS(res, 16);
}

uint8_t comSetPVGate(uint8_t level)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, level?GPIO_PIN_SET:GPIO_PIN_RESET);
	return 0;
}

uint8_t comSetLEDGate(uint8_t level)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, level?GPIO_PIN_SET:GPIO_PIN_RESET);
	return 0;
}

uint8_t comSetPVFan(uint8_t level)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, level?GPIO_PIN_SET:GPIO_PIN_RESET);
	return 0;
}

uint8_t comSetLEDFan(uint8_t level)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, level?GPIO_PIN_SET:GPIO_PIN_RESET);
	return 0;
}

//DEBUG

#if defined(SSDDEBUG)
void ssdDataClear(struct SSDData * ssdData)
{
	ssdData->adc_36 = 0;
	ssdData->adc_led1 = 0;
	ssdData->adc_led2 = 0;
	ssdData->adc_pvi = 0;
	ssdData->adc_pvu = 0;
	ssdData->dac_led = 0;
	ssdData->dac_pv = 0;
	ssdData->pv_gate = 0;
	ssdData->err_code = 0;
}

void ssdUpdateDacLed(struct COMData *comData, uint16_t level)
{
	comData->ssd->dac_led = level;
}

void ssdUpdateDacPV(struct COMData *comData, uint16_t level)
{
	comData->ssd->dac_pv = level;
}

void ssdUpdateAdcPV(struct COMData *comData)
{
	comData->ssd->adc_pvu = (float)(int16_t)(((uint16_t)comData->res[2] << 8) | (uint16_t) comData->res[3]);
	comData->ssd->adc_pvi = (float)(int16_t)(((uint16_t)comData->res[4] << 8) | (uint16_t) comData->res[5]);
	comData->ssd->adc_pvu *= 0.0000625f;
	comData->ssd->adc_pvi *= 0.0000625f;
}

void ssdUpdateGate(struct COMData *comData, uint8_t level)
{
	comData->ssd->pv_gate = level;
}
#endif

/* USER CODE END Application */
