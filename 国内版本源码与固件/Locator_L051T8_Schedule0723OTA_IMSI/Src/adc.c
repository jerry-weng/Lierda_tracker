/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
//uint16_t Battery_Votege_Level_Normal[11] = {4100, 4000, 3900, 3830, 3760, 3720, 3680, 3660, 3620, 3560, 3000};
//uint16_t Battery_Votege_Level_Work[11] =   {4000, 3840, 3730, 3650, 3600, 3560, 3520, 3480, 3440, 3270, 3000};
      
//uint16_t Battery_Votege_Level_Normal[10] = {4034, 3940, 3864, 3801, 3756, 3722, 3693, 3658, 3613, 3000}; // 420mAH电池曲线
uint16_t Battery_Votege_Level_Normal[10] = {4144, 4036, 3927, 3843, 3779, 3733, 3698, 3665, 3608, 3000}; // 590mAH电池曲线
//uint16_t Battery_Votege_Level_Work[10] =   {4000, 3840, 3730, 3650, 3600, 3560, 3520, 3480, 3440, 3000};

/* USER CODE END 0 */

ADC_HandleTypeDef hadc;

/* ADC init function */
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC GPIO Configuration    
    PA4     ------> ADC_IN4 
    */
    GPIO_InitStruct.Pin = BAT_V_DET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BAT_V_DET_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC GPIO Configuration    
    PA4     ------> ADC_IN4 
    */
    HAL_GPIO_DeInit(BAT_V_DET_GPIO_Port, BAT_V_DET_Pin);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/*
* @fn      BatValueConvert
* @brief   电池电压 - 电量转化  
* @param   当前电压，之前电量，充电标志, NB工作标志
* @return   
*/
uint8_t BatValueConvert(uint32_t Bat_Value, uint8_t Pre_Bat_Level, uint8_t chargeFlag, uint8_t NB_state)
{
    uint8_t Bat_Level= 100;
    //大电流（NB工作时段），电池电压会降低
    for(uint8_t i=0; i<10; i++)
    {
        //电量 %100
        if(Bat_Value > Battery_Votege_Level_Normal[0])
        {
            Bat_Level = 100;
            break;
        }        
        //电量0%
        else if(Bat_Value <= Battery_Votege_Level_Normal[9])
        {
            Bat_Level = 0;
            break;
        }
        else
        {
            //根据 NB工作标志 判断使用哪种转换 标准
//            if(NB_state == RESET)
//            {
                if(Bat_Value < Battery_Votege_Level_Normal[i] && Bat_Value > Battery_Votege_Level_Normal[i+1])
                {
                    Bat_Level = 100 - (i+1)*10;
                    if(Bat_Level == 10)
                    {
                        if(Bat_Value < 3550)
                        {
                            Bat_Level = 5;
                        }
                    }
                    break;
                }
//            }
//            else if(NB_state == SET)
//            {
//                if(Bat_Value < Battery_Votege_Level_Work[i] && Bat_Value > Battery_Votege_Level_Work[i+1])
//                {
//                    Bat_Level = 100 - i*10;
//                    break;
//                }
//            }
        }
    }
    
    //比较当前电量,之前电量,确保电量值 不回退
    //    //若未在充电
    //    if(chargeFlag == RESET)
    //    {
    //        //当前电量 > 之前电量， 取之前电量
    //        if(Bat_Level > Pre_Bat_Level)
    //        {
    //            Bat_Level = Pre_Bat_Level;
    //        }
    //    }
    //    //若在充电
    //    else if(chargeFlag == SET)
    //    {
    //        //当前电量 < 之前电量， 取之前电量
    //        if(Bat_Level < Pre_Bat_Level)
    //        {
    //            Bat_Level = Pre_Bat_Level;
    //        }
    //    }
    
    return Bat_Level;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
