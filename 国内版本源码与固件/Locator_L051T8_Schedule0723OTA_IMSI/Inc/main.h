/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED3_G_Pin GPIO_PIN_4
#define LED3_G_GPIO_Port GPIOB
#define LED1_R_Pin GPIO_PIN_8
#define LED1_R_GPIO_Port GPIOB
#define GPS_INT_Pin GPIO_PIN_12
#define GPS_INT_GPIO_Port GPIOA
#define LED2_B_Pin GPIO_PIN_15
#define LED2_B_GPIO_Port GPIOA
#define GPS_SW_Pin GPIO_PIN_11
#define GPS_SW_GPIO_Port GPIOA
#define NB_RI_Pin GPIO_PIN_1
#define NB_RI_GPIO_Port GPIOB
#define PS_INT_Pin GPIO_PIN_5
#define PS_INT_GPIO_Port GPIOB
#define NB_SW_Pin GPIO_PIN_0
#define NB_SW_GPIO_Port GPIOB
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI0_1_IRQn
#define GPS_RST_Pin GPIO_PIN_8
#define GPS_RST_GPIO_Port GPIOA
#define NCHG_Pin GPIO_PIN_6
#define NCHG_GPIO_Port GPIOA
#define NCHG_EXTI_IRQn EXTI4_15_IRQn
#define BAT_V_DET_Pin GPIO_PIN_4
#define BAT_V_DET_GPIO_Port GPIOA
#define NB_RESET_Pin GPIO_PIN_2
#define NB_RESET_GPIO_Port GPIOB
#define NPPR_Pin GPIO_PIN_5
#define NPPR_GPIO_Port GPIOA
#define NPPR_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */
extern void Device_Open_GPS(void);
extern void ScheduleModeProcess(void);
extern void Device_Close_GPS(void);
/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
