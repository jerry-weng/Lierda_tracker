/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "User_ClockDriver.h"
#include "i2c.h"
#include "stm32l0xx_it.h"
#include "tools.h"
#include "gps.h"
#include "nb.h"
#include "schedule.h"
#include "flash.h"
#include "FOTA.h"
#include "crc16.h"
#include "lsd_mems.h"
#include "lis3dh.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//定时器事件
#define KEYPRESS_EVENT                      0x0001
#define LED_EVENT                           0x0002
//#define GPS_TIMEOUT_EVENT                   0x0004
#define IN_NET_EVENT                        0x0004
#define ADC_CONV_EVENT                      0x0008
#define DATA_RSP_EVENT                      0x0010
#define GPS_COLLECTION_EVENT                0x0020
#define AT_CMD_TIMEOUT_EVENT                0x0040
#define ONE_TIMEOUT_EVENT                   0x0080
#define LED_BLINK_EVENT                     0x0100
#define BOARD_CLOSE_EVENT                   0x0200
#define DEVICE_REGIST_EVENT                 0x0400
#define MODE_DISPLAY_EVENT                  0x0800
#define GPS_TIMEOUT_EVENT                   0x1000
#define UNBIND_EVENT                        0x2000
#define EVEN_PRESS_EVENT                    0x4000
#define GPS_KEEP_EVENT                      0x8000
#define GPS_RMC_FORMAT_EVENT                0x00010000

#define DATA_REPORT_EVENT                   0x00040000
#define NB_RESET_EVENT                      0x00080000      
      

//超时时间
#define KEYPRESS_TIMEOUT                    3000
#define LEDBLINK_TIMEOUT                    150
#define BATTERY_LV_LOW_LED_TIMEOUT          4000
#define MODE_SWICTH_LED_TIMEOUT             200
#define CFG_MODE_LED_TIMEOUT                6000
#define SERCHING_NET_TIMEOUT                3000
#define NM_MODE_LED_TIMEOUT                 10000
#define POS_MODE_LED_TIMEOUT                5000

#define GPS_SERCH_TIMEOUT_DEFAULT           30000 
#define ADC_CONVERT_TIMEOUT                 60000
#define DATA_RSP_TIMEOUT                    1000
#define DATA_RSP_TIMEOUT_MAX                5000 
#define GPS_COLLECTION_TIMEOUT_DEFAULT      1440
#define AT_CMD_CFUN0_TIMEOUT                6000
#define AT_CMD_CFUN1_TIMEOUT                3000
#define AT_CMD_ADDR_TIMEOUT                 3000
#define AT_CMD_CGDCONT_TIMEOUT              5000 
#define ONE_TIMEOUT                         2000    
#define AT_BOARD_CLOSE_TIMEOUT              45000
#define DEVICE_REGIST_TIMEOUT               2000
#define MODE_DISPLAY_TIMEOUT                800
#define IN_NET_TIMEOUT                      65000
#define UNBIND_TIMEOUT                      10000
#define EVEN_PRESS_TIMEOUT                  800

#define GPS_KEEP_TINEOUT_20S                30000//30000  //20000
#define GPS_KEEP_TINEOUT_6S                 3000  //6000
#define GPS_RMC_FORMAT_TIMEOUT              1000
#define DATA_REPORT_TIMEOUT                 5000

#define NB_RESET_TIMEOUT                    800
      
//模式切换LED闪烁次数*2
#define LED_BLINK_NUM               8

//电量
#define CHARGE_COMPIETE_BATTERY_LV  100      
#define BATTERY_LOW_LV              20
#define BATTERY_VERY_LOW_LV         5

//分压电阻阻值
#define R_GND                       100
#define R_VCC                       68//33

#define ENTER_CONFIG_MODE_KEYNUM    5
#define REGIST_RETRYNUM_MAX         5
//AT指令 最大重发次数
#define AT_RETRY_NUM_MAX            20      

#define KEY_PRESS_EVENT             0x0001
#define KEY_RELEASE_EVENT           0x0002
#define TIM2_TIMEOUT_EVENT          0x0004
#define TIM21_TIMEOUT_EVENT         0x0008
#define TIM22_TIMEOUT_EVENT         0x0010
#define MOVE_ALARM_EVENT            0x0020


#define ADC_20S_NUM                 90//30min

//按键动作
typedef struct
{
    uint8_t keyConut;
    uint8_t keyPressFlag;
    uint8_t keyLongPressFlag;    
} KeyAction_t;
KeyAction_t KeyAction = {0};

//任务
//uint32_t event;

//定时器开启标志位
uint8_t KeyPressTimerStartFlag = RESET;
uint8_t AntiShakeTimerStartFlag = RESET;
uint8_t SleepTimerStartFlag = RESET;
uint8_t GPSSerchTimerStartFlag = RESET;
uint8_t EvenPressTimerStartFlag = RESET;

uint8_t GPSKeepTimerStartFlag = RESET;
uint8_t GPSKeepTimeoutFlag = RESET;
//连按标记
uint8_t EvenPressFlag = RESET;
//定时器
Clock_Struct KeyPressclock;
Clock_Struct LEDBlinkclock;
Clock_Struct ADCConverclock;
Clock_Struct OneTimeoutclock;
//Clock_Struct DeviceRegistclock;
Clock_Struct ModeDisplayclock;

//判断连按
Clock_Struct EvenPressclock;
//GPS保持定时器
Clock_Struct GPSKeepclock;
//GPS显示格式转换
Clock_Struct GPSRMCclock;

//NB复位
Clock_Struct NBResetclock;

Clock_Struct Testclock;

//GPS坐标点指针
GPS_TO_NB_Data_t* GPS_TO_NB_Data = NULL;

static uint32_t TimeOutInit = 0xFFFF;
//初始化闪烁5次
static uint8_t LED_Init_Blink_Count = 0;
//闪烁
static uint8_t LED_Blink_Count = 0;
//充电口已插入标志
static uint8_t chargeEnable = RESET;
//充电完成标志
uint8_t chargeFinish = RESET;

//任务
uint32_t event = 0;
//是否开机
uint8_t BoardStart = RESET;
//开启注册动作
uint8_t RegistActionFlag = RESET;
//GPS采集周期到
uint8_t GPSTimeOutFlag = RESET;
//NB上报周期到
uint8_t NBTimeOutFlag = RESET;

uint8_t NB_ERROR_counter = 0;

//初次配置NB模块
uint8_t NB_NetWork_First_Init = SET;

uint8_t NSORF_counter = 0;
uint8_t Schedule_Mode_GPSFlag = RESET;

//注册命令计数
uint8_t RegistCount = 0;
//模式显示计数
uint8_t ModeDisplayCount = 0;

//模式切换标志
uint8_t ModeSwitchFlag = RESET;

//额外上报标记
//uint8_t ExtraDataupFlag = RESET;
uint8_t ExtraDataupLEDFlag = RESET;
uint8_t ExtraDataupTimerFlag = RESET;
uint8_t ExtraDataupPreMode; 

uint8_t MoveAlarmPreMode; 

uint8_t adc_timeoutFlag = SET;
uint8_t keyFlag = RESET;
uint8_t move_continue_Flag = RESET;
//当前电池电量
uint32_t Bat_Value;
//前一次采集的电池电量
uint8_t Pre_Bat_Level = 100;

uint16_t regist_counter = 0;
uint16_t adc_counter = 0;
//定时器-应用层转化为分钟
uint16_t GPS_min_counter = 0;
uint16_t gps_serch_counter = 0;
uint16_t sleep_counter = 0;    
uint8_t curAT_RSP_ID_t;

//ADC
uint32_t AD_Value = 0;
uint32_t AD_ValueSum = 0;
//GPS保持定时器
uint32_t GPSKeepCounter = 0;
//转换为只显示RMC
//static char ConfigRMC[28] = "$PCAS03,0,0,0,0,1,0,0,0*03\r\n";
uint8_t GPSRMCCounter = RESET;

uint8_t InitFlag = RESET;
uint8_t InitFlag_c = RESET;

uint8_t InNetCounter = 0;

typedef struct
{
    uint8_t ip[4];
    uint16_t port;
} OTA_flag_t;
OTA_flag_t  OTA_flag;


//电量低无法开机标志
uint8_t Bat_OK_Flag = RESET;

uint8_t schedule_recoverFlag = RESET;
//关闭edrx
static char CloseEdrx[28] = "AT+NPTWEDRXS=0,5,0111,0010\r\n";
//开启PSM
static char OpenPsm[32] = "AT+CPSMS=1,,,00000001,00000001\r\n";
//清除小区频点
static char ClearFreq[14] = "AT+NCSEARFCN\r\n";
//开启小区重选
static char CellReselection[35] = "AT+NCONFIG=CELL_RESELECTION, TRUE\r\n";

uint8_t  PT_flag[1];

uint8_t cmeErrorCounter = 0;
uint8_t InnetTimeOutFlag = RESET;
uint32_t Systime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//初始化
void UserDataInit(void);
void UserTimerInit(void);
void NB_NetWork_Init(void);
//定时器回调
void KeyPressTimerCb(void);
void LEDCb(void);
void LEDBlinkCb(void);
void GPSInfoCb(void);
void GPS_SerchTimeoutCb(void);
void GPSCollectionTimerCb(void);
void ADCTimerCb(void);
void DataRspTimerCb(void);
void ATTimeoutCb(void);
void OneTimeoutCb(void);
void BoardCloseCb(void);
void DeviceRegistCb(void);
void ModeDisplayCb(void);
void InNetCb(void);
void UnbindCb(void);
void EvenPressCb(void);
void GPSKeepCb(void);
void GPSRMCCb(void);
void DataRptCb(void);
void NB_RESETCb(void);

void TSETCb(void);
//流程与数据处理
uint8_t NueStatesProcess(uint8_t *atMessage);
void Device_Regist_Process(void);
void User_NB_TaskProcess(void);
void User_GPS_TaskProcess(void);
uint32_t UserTaskProcess(void);
uint16_t User_Event_Process(void);
void User_UartProcess(void);
void User_AdcProcess(void);
void GPSInfoProcess(uint8_t *gpsRxMessage);
void LEDProcess(uint8_t Bat_Level);

void Device_Open_NB(void);
void Device_Close_GPS(void);
void Handel_LED(void);
void HandelKeyInterrupt(uint16_t GPIO_Pin);
void Handel_NB_AT_RSP(uint8_t *atMessage);
static void SystemPower_Config(void);
void UserStartLedClock(void);
void DataReport(void);
void Device_Close_GPS(void);
//串口相关
//NB
void LPUART1_Rx_Enable(void);
HAL_StatusTypeDef LPUART1_Rx_Disable(void);
//GPS
void UART1_Rx_Enable(void);
HAL_StatusTypeDef UART1_Rx_Disable(void);
void Device_Open_GPS(void);
void huart1_to_IOEXIT(void);
void IOEXIT_to_huart1(void);
void UserStopLedClock(void);

void NB_Data_Report(void);
//低功耗
void SleepMode_Measure(void);
void StopMode_Measure(void);
static void SystemClockConfig_STOP(void);
void closeAllLed(void);
void UserIOInit(void);
void BoardSleep(void);
void KEY_GPIO_Init(void);
void RTC_Timer_Start(void);

void State_Init(void);

void ScheduleModeProcess(void);
void SleepIO_Config(void);
void UART2_Rx_Enable(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  __enable_irq();
  HAL_ResumeTick();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemPower_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_LPUART1_UART_Init();
  //MX_TIM2_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
  MX_I2C1_Init();
  UserIOInit();
  //huart1_to_IOEXIT();
  UserDataInit();
  UserTimerInit();
  Time_syn_RetryTimerInit();
  
  //UART2_Rx_Enable();
  
  printf("\r\nBoard_Start\r\n");
  HAL_Delay(500);
  //User_AdcProcess();

//  if(HAL_GPIO_ReadPin(NPPR_GPIO_Port, NPPR_Pin) == GPIO_PIN_RESET)
//  {
//      HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
//      chargeEnable = SET;
//  }

  OTA_flag.ip[0] = 160;
  OTA_flag.ip[1] = 44;
  OTA_flag.ip[2] = 206;
  OTA_flag.ip[3] = 237;
  OTA_flag.port = 5683;
  EEPROM_WriteBytes(8, (uint8_t*)&OTA_flag, 6);

#warning 产测标志位  
   EEPROM_ReadBytes(0, PT_flag, 1);
   printf("PT_flag, %d\r\n", PT_flag[0]);
   
#warning 调试用，屏蔽出厂检测!!   
   PT_flag[0] = SET;
   memcpy(fota_info.ver_str, APP_VERSION, strlen(APP_VERSION));
   FOTA_Info_Save();

  //User_StartClock(&Testclock);
   
   Lsd_MEMS_Init();
   Lsd_MEMS_SetWakeup_Mode(LIS3DH_ODR_25Hz, false, LIS3DH_FULLSCALE_8, 3);
   asm("nop");
   //HAL_I2C_MspDeInit(&hi2c1);
   
   
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//        while(1)
//        {
//            User_Timer_TaskProcess();
//            User_UartProcess();
//            User_Event_Process();
//            User_NB_TaskProcess();
//            User_AdcProcess();
//        }
        
        //低电量不开机
        while(Bat_OK_Flag == RESET)
        {
            User_AdcProcess();
        }
        while(Bat_OK_Flag == SET)
        {
            if(InitFlag == RESET)
            {
                printf("11");
                HAL_Delay(3000);
                LPUART1_Rx_Enable();
                NB_AT_CMD_Send(AT_CFUN0);
                while(1)
                {
                    if(UsartType.RX_flag)    	// Receive flag
                    {  
                        HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
                        UsartType.RX_flag=0;	// clean flag
                        //LPUART1_Rx_Disable();
                        InitFlag_c = strncmp((char*)UsartType.RX_pData, AT_RSP_OK, AT_RSP_OK_LEN);
                        if(InitFlag_c != RESET)
                        {
                            InitFlag = SET;
                            printf("CF0\r\n");
                            break;
                        }
                    }
                }
                LPUART1_Rx_Disable();
            }
            printf("BoardSleep\r\n");
            //休眠状态处理
            State_Init();
            
            HAL_UART_MspDeInit(&hlpuart1);
            HAL_UART_MspDeInit(&huart1);
            HAL_UART_MspDeInit(&huart2);
            
            //huart1_to_IOEXIT();
            //if(RegistFlag == SET)
            //{
                SleepIO_Config();
            //}
            
            RTC_Timer_Start();
            StopMode_Measure();
            
            /* Wait until USER button is pressed to enter the Wake up mode */
            while(SleepStatus == AWAKE)
            {   
                HAL_UART_MspInit(&hlpuart1);
                HAL_UART_MspInit(&huart1);
                HAL_UART_MspInit(&huart2);
                
                //hlpuart1.gState = HAL_UART_STATE_READY;
                printf("Init\r\n");
                //LED显示，根据配置 NB定时上报，GPS定时上报等
                while(SleepStatus == AWAKE)
                {
                    User_Event_Process();
                    User_Timer_TaskProcess();
                    User_UartProcess();
                    User_NB_TaskProcess();
                    User_AdcProcess();
                }
            }
        }
    }
    
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;//RCC_LPUART1CLKSOURCE_LSE;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//初始化数据
void UserDataInit(void)
{
    NB_Report_to_ServerData.start = SERVER_START;
    NB_Report_to_ServerData.cmd = SERVER_ID_DATA_REPOET;
    NB_Report_to_ServerData.version = CMD_VERSION;
    NB_Report_to_ServerData.softWare_version = SW_VERSION;
    Locator_Data.GPS_data_len = 0;
    Locator_Data.mode = currentMode;
    Locator_Data.FindnowCycle = DEFAULT_F_CYCLE;
    Locator_Data.TrackerCycle = DEFAULT_T_CYCLE;
    Locator_Data.batteryLv = 100;
    Locator_Data.avaliableTime = 255;   
    Locator_pram.GPSCollectionCycle = 1;
    Locator_pram.NB_dataCycle = 1;
}
//用户定时器初始化
void UserTimerInit(void)
{
    MX_TIM2_Init(TimeOutInit);
    //按键防抖时钟
    KeyPressclock.eventID = KEYPRESS_EVENT;
    KeyPressclock.timeOut = KEYPRESS_TIMEOUT;
    KeyPressclock.TaskHook = KeyPressTimerCb;
    //LED时钟
    LEDclock.eventID = LED_EVENT;
    LEDclock.timeOut = MODE_SWICTH_LED_TIMEOUT;
    LEDclock.TaskHook = LEDCb;
    //LED闪烁
    LEDBlinkclock.eventID = LED_BLINK_EVENT;
    LEDBlinkclock.timeOut = LEDBLINK_TIMEOUT;
    LEDBlinkclock.TaskHook = LEDBlinkCb;
    //ADC采样时钟
    ADCConverclock.eventID = ADC_CONV_EVENT;
    ADCConverclock.timeOut = ADC_CONVERT_TIMEOUT;
    ADCConverclock.TaskHook = ADCTimerCb;
    //服务器数据回复时钟
    DataRspclock.eventID = DATA_RSP_EVENT;
    DataRspclock.timeOut = DATA_RSP_TIMEOUT; 
    DataRspclock.TaskHook = DataRspTimerCb;
    
    //GPS搜星时钟
    GPSTimeOutclock.eventID = GPS_TIMEOUT_EVENT;
    GPSTimeOutclock.timeOut = GPS_SERCH_TIMEOUT_DEFAULT;
    GPSTimeOutclock.TaskHook = GPS_SerchTimeoutCb;
    
    //AT指令发送超时时钟
    ATTimeoutclock.eventID = AT_CMD_TIMEOUT_EVENT;
    ATTimeoutclock.timeOut = AT_CMD_CFUN0_TIMEOUT;
    ATTimeoutclock.TaskHook = ATTimeoutCb;
    
    //测试时钟
    OneTimeoutclock.eventID = ONE_TIMEOUT_EVENT;
    OneTimeoutclock.timeOut = ONE_TIMEOUT;
    OneTimeoutclock.TaskHook = OneTimeoutCb;
    
    //休眠
    BoardCloseTimeoutclock.eventID = BOARD_CLOSE_EVENT;
    BoardCloseTimeoutclock.timeOut = AT_BOARD_CLOSE_TIMEOUT;
    BoardCloseTimeoutclock.TaskHook = BoardCloseCb;
    
    //注册命令重发定时器
    DeviceRegistclock.eventID = DEVICE_REGIST_EVENT;
    DeviceRegistclock.timeOut = DEVICE_REGIST_TIMEOUT;   
    DeviceRegistclock.TaskHook = DeviceRegistCb;
    
    //按一下按键亮一次灯显示模式
    ModeDisplayclock.eventID = MODE_DISPLAY_EVENT;
    ModeDisplayclock.timeOut = MODE_DISPLAY_TIMEOUT;
    ModeDisplayclock.TaskHook = ModeDisplayCb;
    
    //入网超时
    InNetTimeoutclock.eventID = IN_NET_EVENT;
    InNetTimeoutclock.timeOut = IN_NET_TIMEOUT;
    InNetTimeoutclock.TaskHook = InNetCb;
    
    //关机延时
    APPUnbindclock.eventID = UNBIND_EVENT;
    APPUnbindclock.timeOut = UNBIND_TIMEOUT;
    APPUnbindclock.TaskHook = UnbindCb;
    
    EvenPressclock.eventID = EVEN_PRESS_EVENT;
    EvenPressclock.timeOut = EVEN_PRESS_TIMEOUT;
    EvenPressclock.TaskHook = EvenPressCb;
    
    //GPS保持
    GPSKeepclock.eventID = GPS_KEEP_EVENT;
    GPSKeepclock.timeOut = GPS_KEEP_TINEOUT_20S;
    GPSKeepclock.TaskHook = GPSKeepCb;

    //RMC格式切换
    GPSRMCclock.eventID = GPS_RMC_FORMAT_EVENT;
    GPSRMCclock.timeOut = GPS_RMC_FORMAT_TIMEOUT;
    GPSRMCclock.TaskHook = GPSRMCCb;

    //数据重发
    DataRptclock.eventID = DATA_REPORT_EVENT;
    DataRptclock.timeOut = DATA_REPORT_TIMEOUT;
    DataRptclock.TaskHook =  DataRptCb;
    
    NBResetclock.eventID = NB_RESET_EVENT;
    NBResetclock.timeOut = NB_RESET_TIMEOUT;
    NBResetclock.TaskHook = NB_RESETCb;

    Testclock.eventID = 0x00100000;
    Testclock.timeOut = 5000;
    Testclock.TaskHook = TSETCb;
}
////搜星超时定时器
//void GPSSerchTimerInit(uint8_t timeout)
//{
//    GPSTimeOutclock.eventID = GPS_TIMEOUT_EVENT;
//    GPSTimeOutclock.timeOut = GPS_SERCH_TIMEOUT_DEFAULT;
//    GPSTimeOutclock.TaskHook = GPS_SerchTimeoutCb;
//}
//IO中断
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#warning 第一步，按键唤醒
    HandelKeyInterrupt(GPIO_Pin);
    /* NOTE: This function Should not be modified, when the callback is needed,
    the HAL_GPIO_EXTI_Callback could be implemented in the user file
    */ 
}
//时钟
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    //时钟唤醒
    //SleepStatus = AWAKE;
    if (htim->Instance == htim2.Instance)
    {
        event |= TIM2_TIMEOUT_EVENT;
        //printf("T2\n");
    }
    if (htim->Instance == htim21.Instance)
    {
        event |= TIM21_TIMEOUT_EVENT;
    }
    if (htim->Instance == htim22.Instance)
    {
        event |= TIM22_TIMEOUT_EVENT;
    }
}

//进入休眠
void SleepMode_Measure(void)
{
    
    /* Suspend Tick increment to prevent wakeup by Systick interrupt. 
    Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base) */
    HAL_SuspendTick();
    
    /* Request to enter SLEEP mode */
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    
    /* Resume Tick interrupt if disabled prior to sleep mode entry */
    HAL_ResumeTick();
    
}

//进入STOP
void StopMode_Measure(void)
{
    /* Enter Stop Mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    
    /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
    PLL as system clock source (HSE and PLL are disabled in STOP mode) */
    SystemClockConfig_STOP();
}
//按键处理
void KeyPressTimerCb(void)
{
    //判断长按或短按
    if(KeyAction.keyLongPressFlag == RESET)
    {
        if(BoardStart == RESET)
        {
            //若在未开机状态，短按开机无效，重新进入休眠
            //printf("sleep\r\n"); 
            if(PT_flag[0] == SET)
            {
                SleepStatus = SLEEP;
            }
            else if(KeyAction.keyConut < 5)
            {
                //未产测过，短按次数不足，休眠
                SleepStatus = SLEEP;
            }
        }
        else
        {
//            //注册成功后按键显示模式
//            if(KeyAction.keyConut == 1)
//            {
//                if(RegistFlag == SET)
//                {                 
//                    //if(Bat_Level > BATTERY_VERY_LOW_LV)
//                    //{
//                        if(currentMode == POSITIONING_MODE)
//                        {
//                            HAL_GPIO_WritePin(LED2_B_GPIO_Port, LED2_B_Pin, GPIO_PIN_RESET);
//                        }
//                        else if(currentMode == NORMAL_MODE)
//                        {
//                            HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
//                        }
//                        //User_StopClock(&ModeDisplayclock);
//                        User_StartClock(&ModeDisplayclock);
//                    //}
//                }
//            }
        }
    }
    else if(KeyAction.keyLongPressFlag == SET)
    {
        if(BoardStart == RESET)
        {
#warning 第二步，长按三秒后开机
            BoardStart = SET;
            if(RegistFlag == RESET)
            {
                //printf("3s");
                if(PT_flag[0] == SET)
                {
                    Device_Open_NB();
                }
                //Device_Open_GPS();
            }
            //开机后，开启LED定时器
            if(PT_flag[0] == SET)
            {
                User_StartClock(&LEDclock);
            }
        }
#warning 添加0105        
        else if(BoardStart == SET)
        {
            //开机后长按，额外上报一次
            //printf("ExtraDataup");
            if(RegistFlag == SET)
            {
                if(gps_open_Flag == SET)
                {
                    User_StopClock(&GPSTimeOutclock);
                    Device_Close_GPS();
                }
                if(NB_open_Flag == SET)
                {
                    User_StopClock(&ATTimeoutclock);
                    User_StopClock(&InNetTimeoutclock);
                    User_StopClock(&DataRptclock);
                    User_StopClock(&DataRspclock);
                    User_StopClock(&BoardCloseTimeoutclock);
                    
                    NB_open_Flag = RESET;
                    Close_NB();
                }
                //额外上报标记
                ExtraDataupFlag = SET;
                ExtraDataupLEDFlag = SET;
                ExtraDataupTimerFlag = SET;
                
                Locator_Data.userAlarmFlag = SET;
#warning   注意此处模式字段
                if(move_alarm_enable_Flag == SET)
                {
                    ExtraDataupPreMode = MoveAlarmPreMode;
                }
                else
                {
                    ExtraDataupPreMode = currentMode;
                }
                currentMode = POSITIONING_MODE;
                Locator_Data.mode = POSITIONING_MODE;
                LEDclock.timeOut = 100;
                User_StartClock(&LEDclock);
            }
            
        }        
    }
    //清空计数
    KeyAction.keyConut = 0;
    //printf("KeyTimeOut\r\n");
    KeyPressTimerStartFlag = RESET;
    //开机后按键结束，判断是否需要进入休眠
    if(BoardStart == SET)
    {
        if(NB_open_Flag == RESET && gps_open_Flag == RESET)
        {
            if(ExtraDataupFlag == RESET)
            {
                printf("sleep0\r\n");
                SleepStatus = SLEEP;
            }
        }
    }
}
//LED定时器回调
void LEDCb(void)
{
    Handel_LED();
}
/*
* @fn      LEDBlinkCb
* @brief   LED闪烁定时器
* @param   
* @return   
*/
void LEDBlinkCb(void)
{
    closeAllLed();
    if(LED_Blink_Count < 2)
    {
        LED_Blink_Count++;
        //未注册过
        if(RegistFlag != SET)
        {
            //未连上网，红灯
            if(NB_InNetWork_Flag == RESET)
            {
                HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
            }
            //连上网，绿灯
            else if(NB_InNetWork_Flag == SET)
            {
                HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
            }
        }
        else if(RegistFlag == SET)
        {
            //if(Bat_Level <= BATTERY_VERY_LOW_LV)
            //{
            //    HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
            //}
        }
        User_StartClock(&LEDBlinkclock);
    }
}
//ADC采集
void ADCTimerCb(void)
{

}

void DataRspTimerCb(void)
{
    if(RegistFlag == RESET)
    {
        //异常掉电重新注册逻辑
        RegistFinishFlag = SET;
    }
    if(Schedule_StateStartFlag == SET)
    {
        Schedule_StateSend_counter++;
        
        curAT_RSP_ID = AT_DATA_RSP;
        NB_Report_to_ServerData.cmd = SCHEDULE_UPLOAD_ID;
        NB_Report_to_ServerData.cmdlen = sizeof(NB_Report_to_ServerData_t)+sizeof(Schedule_Data_upload_t)+ Schedule_Data_upload.GPS_List_num * sizeof(GPS_List_t) + CHECK_SUM_LEN + END_NUM_LEN;
        if(RegistFlag == RESET)
        {
            Schedule_Data_upload.GPS_List_num = 0;
            Schedule_Data_upload.GPS_data_len = 1;
        }
        Schedule_Data_Report(NB_Report_to_ServerData, Schedule_Data_upload, GPS_Info_schedule);
        printf("Schedule_DATA_UP: %d", NB_Report_to_ServerData.cmdlen);
        
        if(Schedule_StateSend_counter >= 5)
        {
            Schedule_StateSend_counter = 0;
            Schedule_StateStartFlag = RESET;
            if(RegistFlag == RESET)
            {
                NVIC_SystemReset();
            }
            else if(RegistFlag == SET)
            {
                printf("StateTimeOut2");
                
                User_StopClock(&ATTimeoutclock);
                User_StopClock(&InNetTimeoutclock);
                User_StopClock(&DataRptclock);
                User_StopClock(&BoardCloseTimeoutclock);
                Close_NB();
                wakeupCounter = 0;
                //upload_OK_Flag = RESET;
                cmd_OK_Flag = RESET;
                SleepStatus = SLEEP;
            }
        }
        else
        {
            DataRspclock.timeOut = 5000;
            User_StartClock(&DataRspclock);
        }
    }
    else if(Normal_StateFlag == SET)
    {
        if(Locator_pram.enterSleep == SET)
        {
            Locator_pram.enterSleep = RESET;
            NVIC_SystemReset();
        }
        Locator_Data.GPS_List_num = 0;//GPS_Info.stores_num;
        Locator_Data.GPS_data_len = Locator_Data.GPS_List_num*sizeof(GPS_List_t)+1;
        
        Schedule_Data_upload.GPS_List_num = 0;
        Schedule_Data_upload.GPS_data_len = 1;
        
        NB_Report_to_ServerData.cmd = SERVER_ID_DATA_REPOET;
        NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
        curAT_RSP_ID = AT_DATA_RSP;
        NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
        printf("D1\r\n");
        NB_StateSend_counter++;
        DataRspclock.timeOut = DATA_RSP_TIMEOUT_MAX;
        if(NB_StateSend_counter > 5)
        {
            Normal_StateFlag = RESET;
            NB_StateSend_counter = 0;
            if(RegistFlag == RESET)
            {
                NVIC_SystemReset();
            }
            else if(RegistFlag == SET)
            {
                printf("StateTimeOut");
                User_StopClock(&ATTimeoutclock);
                User_StopClock(&InNetTimeoutclock);
                User_StopClock(&DataRptclock);
                User_StopClock(&BoardCloseTimeoutclock);
                Close_NB();
                wakeupCounter = 0;
                //upload_OK_Flag = RESET;
                cmd_OK_Flag = RESET;
                SleepStatus = SLEEP;
            }
        }
        else
        {
            DataRspclock.timeOut = 5000;
            User_StartClock(&DataRspclock);
        }
    }
}

//GPS信息处理
void GPSInfoProcess(uint8_t *gpsRxMessage)
{   
    //处理串口信息，组成要向NB上报的信息字段
    //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
    
    GPS_TO_NB_Data = GPS_Rx_Data_process(gpsRxMessage);
    GPS_TO_NB_GSV_Data = GPS_GSV_Data_process(gpsRxMessage);
    if(GPS_TO_NB_GSV_Data->state == SUCCESS)
    {
        if(eff_satCounter < GPS_TO_NB_GSV_Data->SNR30up_satellites_Num)
        {
            eff_satCounter = GPS_TO_NB_GSV_Data->SNR30up_satellites_Num;
            printf("eff_satCounter: %d\r\n", eff_satCounter);
        }
        if(PT_flag[0] == RESET)
        {
            if(eff_satCounter >= 1)
            {
                Device_Close_GPS();
                //RegistFlag = SET;
                GPS_TO_NB_Data->State = INFO_STATE_UNAVILIABLE;
                //GPSKeepTimerStartFlag = SET;
                printf("PT_GPS_OK2\r\n");
                PT_flag[0] = SET;
                EEPROM_WriteBytes(0, PT_flag, 1);
                closeAllLed();
                
                User_StopClock(&LEDclock);
                User_StopClock(&LEDBlinkclock);
                User_StopClock(&GPSTimeOutclock);
                UserStopLedClock();
                
                HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
                Testclock.timeOut = 5000;
                User_StartClock(&Testclock);
                //printf("currentLoadValue: %d", currentLoadValue);
                return;
            }
        }
    }
    printf("GPSSTATE: %d", GPS_TO_NB_Data->State);

    //如果搜星成功
    if(GPS_TO_NB_Data->State == INFO_STATE_AVILIABLE)
    {
        //HAL_UART_Transmit_DMA(&huart2, "GPS_OK", 6);
        //此次搜星成功，停止搜星超时定时
        if(GPSSerchTimerStartFlag == SET)
        {
            User_StopClock(&GPSTimeOutclock);
            GPSSerchTimerStartFlag = RESET;
        }
#warning 添加171229        
        /*
        搜星成功后
        如果搜星间隔2小时(120分钟) 内
        每隔2小时保持一次额外维持20秒搜星，以保证后续快速定位
        不然额外维持3秒，保证精度稳定
        */
        if(GPSKeepTimerStartFlag == RESET)
        {
            GPSKeepTimerStartFlag = SET;
            if(Schedule_ModeFlag == RESET)
            {
                if(Locator_Data.TrackerCycle <= GPS_KEEP_MIN )
                {
                    //如果有命令下发时间间隔切换，或者2小时时间到
                    if(GPSKeepTimeoutFlag == SET || TrackerCycle_short_Flag == SET)
                    {
                        //标志位到后保持20s
                        TrackerCycle_short_Flag = RESET;
                        GPSKeepclock.timeOut = GPS_KEEP_TINEOUT_20S;
                    }
                    else
                    {
                        //不然保持6s
                        GPSKeepclock.timeOut = GPS_KEEP_TINEOUT_6S;
                    }
                }
                else
                {
                    GPSKeepclock.timeOut = GPS_KEEP_TINEOUT_6S;
                }
            }
            else
            {
                if(Schedule_Mode_GPSFlag == RESET)
                {
                    Schedule_Mode_GPSFlag = SET;
                    GPSKeepclock.timeOut = GPS_KEEP_TINEOUT_20S;
                }
                else
                {
                    GPSKeepclock.timeOut = GPS_KEEP_TINEOUT_6S;
                }
            }
            User_StartClock(&GPSKeepclock);
            printf("GPS_T: %d\r\n", GPSKeepclock.timeOut);
        }
    }
}

//GPS搜星超时
void GPS_SerchTimeoutCb(void)
{
    uint8_t gps_counter = 0;
    if(GPSKeepTimeoutFlag == SET || TrackerCycle_short_Flag == SET)
    {
        //5分钟
        gps_counter = 6;
        //gps_counter = 3;
        
        //GPSKeepTimeoutFlag = RESET;
        //TrackerCycle_short_Flag = RESET;
    }
    else
    {
        //1分半
        gps_counter = 3;
    }
    gps_serch_counter++;
    
    if(gps_serch_counter == 1)
    {
        //在室内或者信号弱
        if(eff_satCounter <2)
        {
            printf("GPSSignalError\r\n");
            gps_counter = 0;
            if(PT_flag[0] == RESET)
            {
                Device_Close_GPS();
                printf("PT_GPS_ERROR\r\n");
                User_StopClock(&LEDclock);
                User_StopClock(&LEDBlinkclock);
                UserStopLedClock();
                HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
                Testclock.timeOut = 5000;
                User_StartClock(&Testclock);
                return;
            }
        }
    }
    
    if(gps_serch_counter >= gps_counter)
    {
        GPSKeepTimeoutFlag = RESET;
        TrackerCycle_short_Flag = RESET;
        
        GPSSerchTimerStartFlag = RESET;
        printf("GPSTimeOut\r\n");
        gps_serch_counter = 0;
        if(gps_open_Flag == SET)
        {
            //保存该搜星失败数据
            //GPS_Info_Store(&GPS_TO_NB_TimeOut_Data);
			GPSSerchTimerStartFlag = RESET;
           
            Device_Close_GPS();
            
//            if(Schedule_ModeFlag == RESET)
//            {
//                Locator_Data.GPS_List_num = 0;
//                Locator_Data.GPS_data_len = Locator_Data.GPS_List_num*sizeof(GPS_List_t)+1;
//                
//                NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
//            }
//            else
//            {
//                Schedule_Data_upload.GPS_List_num = 0;
//                Schedule_Data_upload.GPS_data_len = 1;
//                //printf("HH,%d" ,Schedule_Data_upload.GPS_data_len);
//            }
            //NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
            //GPS_Info.stores_num = 0;   
            Device_Open_NB();
        }
    }
    else
    {
        User_StartClock(&GPSTimeOutclock);
    }
}

//GPS定时
void GPSCollectionTimerCb(void)
{
    
}

void OneTimeoutCb(void)
{
    //延时后重开串口
    LPUART1_Rx_Enable();
}

void BoardCloseCb(void)
{
    BoardSleep();
}

//注册命令重发超时
void DeviceRegistCb(void)
{
    if(RegistFlag == RESET)
    {
        regist_counter++;
        //1min 超时
        if(regist_counter < 6)
        {
            NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
            
            NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
            curAT_RSP_ID = AT_DATA_RSP;
            if(regist_counter > 2)
            {
                DeviceRegistclock.timeOut = 10000;
            }
            User_StartClock(&DeviceRegistclock);
        }
        else
        {
            regist_counter = 0;
            NVIC_SystemReset();
        }
    }
}
//按键中断处理
void HandelKeyInterrupt(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == KEY_Pin)
    {   
        //printf("K:%d", Bat_OK_Flag);
        if(InitFlag == SET)
        {
            //if(NB_open_Flag == RESET && gps_open_Flag == RESET)
            //{
            //读取IO电平,为低表示按下
            if( HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
            {
                event |= KEY_PRESS_EVENT;
                //printf("keyPress!\r\n");
                SleepStatus = AWAKE;
            }
            else if( HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
            {
                event |= KEY_RELEASE_EVENT;
                //printf("keyRelease!\r\n");
            }
        }
    //}
    }
    //充电指示
    if(GPIO_Pin == NCHG_Pin )
    {
        //SleepStatus = AWAKE;
        //closeAllLed();
        //User_StopClock(&LEDclock);
        //低-正在充电
        if(HAL_GPIO_ReadPin(NCHG_GPIO_Port, NCHG_Pin) == GPIO_PIN_RESET)
        {
            chargeEnable = SET;
            //红灯亮
            HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
        }
        else if(HAL_GPIO_ReadPin(NCHG_GPIO_Port, NCHG_Pin) == GPIO_PIN_SET)
        {
            chargeEnable = RESET;
            HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_SET);
//            if(chargeEnable == SET)
//            {
//                chargeFinish = SET;
//                //充电完成
//                HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
//                
//                Locator_Data.batteryLv = 100;
//                Schedule_Data_upload.battery_Lv = 100;
//                
//            }
            //else if(chargeEnable == RESET)
            //{
            //    LEDProcess(Bat_Level);
            //}
        }                
    }
    //外部电源插入指示
    if(GPIO_Pin == NPPR_Pin)
    {
//        //SleepStatus = AWAKE;
//        //低表示外部有电源供电
//        if(HAL_GPIO_ReadPin(NPPR_GPIO_Port, NPPR_Pin) == GPIO_PIN_RESET)
//        {
//            chargeEnable = SET;
//        }
//        else if(HAL_GPIO_ReadPin(NPPR_GPIO_Port, NPPR_Pin) == GPIO_PIN_SET)
//        {
//            closeAllLed();
//            chargeEnable = RESET;
//            chargeFinish = RESET;
//        }
    }
    
    //GPS
//    if(GPIO_Pin == GPIO_PIN_10)
//    {
//        if(gps_open_Flag == SET)
//        {
//            //转换成串口功能，唤醒
//            printf("gps:%d\r\n", gps_serch_counter);
//            gps_serch_counter++;
//            IOEXIT_to_huart1();
//            //1min
//            if(gps_serch_counter <= 300)
//            {
//                //IOEXIT_to_huart1();
//            }
//            else
//            {
//                event = GPS_TIMEOUT_EVENT;
//            }
//            SleepStatus = AWAKE;
//        }
//    }
    if(GPIO_Pin == GPIO_PIN_5)
    {
        SleepStatus = AWAKE;
        event |= MOVE_ALARM_EVENT;
    }
}
/*
* @fn      Handel_LED
* @brief   LED显示
* @param   
* @return
*/
void Handel_LED(void)
{
    if(LED_Init_Blink_Count <= LED_BLINK_NUM && LED_Init_Blink_Finish == RESET)
    {
#warning 第三步，绿灯快闪5下 
        if(LED_Init_Blink_Count == 0)
        {
            closeAllLed();
        }
        if((currentMode == REGIST_MODE)||(currentMode == NORMAL_MODE))
        {
            if(PT_flag[0] == SET)
            {
                HAL_GPIO_TogglePin(LED3_G_GPIO_Port, LED3_G_Pin);
            }
            else
            {
                HAL_GPIO_TogglePin(LED2_B_GPIO_Port, LED2_B_Pin);
            }
        }
        else if(currentMode == POSITIONING_MODE)
        {
            HAL_GPIO_TogglePin(LED2_B_GPIO_Port, LED2_B_Pin);
        }
        LED_Init_Blink_Count++;
        LEDclock.timeOut = MODE_SWICTH_LED_TIMEOUT;
        User_StartClock(&LEDclock);
    }
    else
    {
        LED_Init_Blink_Count = 0;
        LED_Init_Blink_Finish = SET;
        //printf("yy\t\n");
        LEDProcess(Bat_Level);
        if(ModeSwitchFlag == SET)
        {
            ModeSwitchFlag = RESET;
            User_StopClock(&LEDclock);
            wakeupCounter = 0;
            NBTimeOutFlag = SET;
            //SleepStatus = AWAKE;
            printf("ModeSwitch\r\n");
        }
    }
}


/*
* @fn      State_Init
* @brief   处理休眠时的设备状态
* @param   
* @return   
*/
void State_Init(void)
{
    closeAllLed();
    if(chargeEnable == SET && chargeFinish == RESET)
    {
        //红灯亮
        HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
    }
    else if(chargeEnable == SET && chargeFinish == SET)
    {
        //充电完成
        HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
    }
    
    //恢复为原模式
    if(ExtraDataupFlag == SET)
    {
        ExtraDataupFlag = RESET;
        if(Schedule_ModeFlag == RESET)
        {
            if(ExtraDataupPreMode == NORMAL_MODE)
            {
                currentMode = ExtraDataupPreMode;
                Locator_Data.mode = currentMode;
            }
        }
    }
    
    //memset(UsartType.RX_pData, 0, RX_LEN);            
    //单位--分钟
    
    if(move_alarm_Flag == RESET)
    {
        if(Schedule_ModeFlag == RESET)
        {
            if(currentMode == NORMAL_MODE)
            {
                dataCycle = Locator_Data.FindnowCycle;
            }
            else if(currentMode == POSITIONING_MODE)
            {
                dataCycle = Locator_Data.TrackerCycle;
            }
        }
    }        
    getRTC_Date_Time();
    if(stimestructure.Minutes != 0)
    {
        DaySwitchCounter = 0;
    }
    printf("DaySwitchCounter:%d\r\n", DaySwitchCounter);
    printf("DataCycle:%d\r\n", dataCycle );
    printf("WakeUpCounter:%d\r\n", wakeupCounter);
    printf("CurrentDayInTheLoop:%d\r\n", CurrentDayInTheLoop);
    printf("TimeSyn:%d\r\n", Time_synFlag);
    printf("BAT:%d\r\n", Bat_Value);
    printf("move_alarm_Flag:%d\r\n", move_alarm_Flag);
    printf("AIS_Coap180821\r\n");
    

    curAT_RSP_ID = AT_INIT_SUCCESS;
    SleepStatus = SLEEP;
    DataUpFlag = RESET;
    NB_InNetWork_Flag = RESET;
    NBTimeOutFlag = RESET;
    NB_open_Flag = RESET;
    //gps_open_Flag = RESET;
    
    //Locator_Data.sensorFlag = RESET;
    KeyAction.keyConut = 0;
    KeyAction.keyPressFlag = RESET;
    EvenPressTimerStartFlag = RESET;
    Time_syning_Flag = RESET;
    Schedule_State = RESET;
    cycleStartFlag = RESET;
    data_confirmFlag = RESET;
    OTAingFlag = RESET;
    keyFlag = RESET;
    move_continue_Flag = RESET;
    Schedule_StateSend_counter = 0;
    NB_StateSend_counter = 0;
    NB_ERROR_counter = 0;
    cmeErrorCounter = 0;
    InNetCounter = 0;   
}

/*
* @fn      LEDProcess
* @brief   处理LED状态
* @param   
* @return   
*/
void LEDProcess(uint8_t Bat_Level)
{
    //if(chargeEnable != SET)
    //{
        closeAllLed();
        LED_Blink_Count = 0;
        //LEDclock.timeOut = TimeOutInit;

        //电池电量低
        //if(Bat_Level < BATTERY_LOW_LV)
        //{
        //    LEDclock.timeOut = BATTERY_LV_LOW_LED_TIMEOUT;
        //    UserStartLedClock();
        //    User_StartClock(&LEDBlinkclock);
        //}
        //else
        //{
            //未初始化注册过
            if(RegistFlag == RESET)
            {
                LEDclock.timeOut = SERCHING_NET_TIMEOUT;
                UserStartLedClock();
                User_StartClock(&LEDBlinkclock);
            }
            else
            {
                //额外上报
                if(ExtraDataupTimerFlag == SET)
                {
                    ExtraDataupTimerFlag = RESET;
                    HAL_GPIO_WritePin(LED2_B_GPIO_Port, LED2_B_Pin, GPIO_PIN_RESET);
                    ModeDisplayclock.timeOut = 2000;
                    User_StartClock(&ModeDisplayclock);
                }
                else if(EvenPressFlag == SET)
                {
                    EvenPressFlag = RESET;
                    LEDclock.timeOut = MODE_DISPLAY_TIMEOUT;
                    if(currentMode == POSITIONING_MODE)
                    {
                        HAL_GPIO_WritePin(LED2_B_GPIO_Port, LED2_B_Pin, GPIO_PIN_RESET);
                    }
                    else if(currentMode == NORMAL_MODE)
                    {
                        HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
                    }
                    //printf("W\r\n");
                    ModeDisplayclock.timeOut = MODE_DISPLAY_TIMEOUT;
                    User_StartClock(&ModeDisplayclock);
                }
            }
        //}
    //}
}
/*
* @fn      closeAllLed
* @brief   关闭 所有LED
* @param   
* @return   
*/
void closeAllLed(void)
{
    HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_B_GPIO_Port, LED2_B_Pin, GPIO_PIN_SET);
}

void User_NB_TaskProcess(void)
{
    if(NBTimeOutFlag == SET)
    {
        NBTimeOutFlag = RESET;
        if(currentMode == POSITIONING_MODE)
        {
            if(gps_open_Flag == RESET && NB_open_Flag == RESET)
            {
                //printf("G\r\n");
                Device_Open_GPS();
            }
        }
        else if(currentMode == NORMAL_MODE)
        {
            if(NB_open_Flag == RESET)
            {
                //printf("N\r\n");
                Device_Open_NB();
            }
        }
    }
    //入网成功
    if(NB_InNetWork_Flag == SET)
    {
        //NB_InNetWork_Flag = RESET;
        //已注册
        if(RegistFlag == SET)
        {
#warning 第六步，周期到，上报数据   
            if(DataUpFlag == RESET)
            {
                DataUpFlag = SET;
                if(testFlag2 == SET)
                {
                    testFlag2 = RESET;
                    testFlag3 = SET;
                    curAT_RSP_ID = AT_DATA_RSP;
                    NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t);
                    printf("PP\r\n");
                    Locator_Data_ReportSleep( NB_Report_to_ServerData);
                    Testclock.timeOut = 30000;
                    User_StartClock(&Testclock);
                }
                else
                {
                    //if(Time_synFlag == SET)
                    //{
                    if(move_alarm_Flag == SET || move_continue_Flag == SET)
                    {
                        curAT_RSP_ID = AT_DATA_RSP;
                        NB_Report_to_ServerData.cmd = SERVER_ID_DATA_REPOET;
                        NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
                        printf("MOVE_DATA_UP: %d", NB_Report_to_ServerData.cmdlen);
                        NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
                        
                        Retry_state = RESET;
                        AT_Retry_Num = 0;
                        User_StartClock(&DataRptclock);
                    }
                    else
                    {
                        if(Schedule_ModeFlag == SET)
                        {
                            curAT_RSP_ID = AT_DATA_RSP;
                            
                            //cycleStartFlag = SET;
                            //ScheduleModeProcess();
                            if(Locator_Data.userAlarmFlag == SET)
                            {
                                Schedule_Data_upload.scheduleID = SET;
                            }
                            ScheduleParmSet(CurrentDayInTheLoop);
                            
                            Schedule_Data_upload.Mode = currentMode;
                            NB_Report_to_ServerData.cmd = SCHEDULE_UPLOAD_ID;
                            NB_Report_to_ServerData.cmdlen = sizeof(NB_Report_to_ServerData_t)+sizeof(Schedule_Data_upload_t)+ Schedule_Data_upload.GPS_List_num * sizeof(GPS_List_t) + CHECK_SUM_LEN + END_NUM_LEN;
                            Schedule_Data_Report(NB_Report_to_ServerData, Schedule_Data_upload, GPS_Info_schedule);
                            printf("Schedule_DATA_UP: %d", NB_Report_to_ServerData.cmdlen);
                            
                            //ScheduleParmSet(CurrentDayInTheLoop);
                            //testFlag = SET;
                            Retry_state = RESET;
                            AT_Retry_Num = 0;
                            User_StartClock(&DataRptclock);
                        }
                        //}
                        //else
                        //{
                        else if(Schedule_ModeFlag == RESET)
                        {
                            curAT_RSP_ID = AT_DATA_RSP;
                            NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
                            printf("NB_DATA_UP: %d", NB_Report_to_ServerData.cmdlen);
                            NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
                            
                            Retry_state = RESET;
                            AT_Retry_Num = 0;
                            User_StartClock(&DataRptclock);
                        }
                        
                        if(Locator_pram.enterSleep == SPECIAL_FINDNOW)
                        {
                            Locator_pram.enterSleep = RESET;
                        }
                        //                    else
                        //                    {
                        //                        //同步时间戳
                        //                        printf("D5\r\n");
                        //                        curAT_RSP_ID = AT_DATA_RSP;
                        //                        Schedule_Device_Confirm_Report(TIME_SYN_REQ_ID, 0, Schedule_CMD_State.packetNum, 0xFF, 0);
                        //                    }                    
                        //                }
                        
                        //                //测
                        //                Retry_state = RESET;
                        //                AT_Retry_Num = 0;
                        //                User_StartClock(&DataRptclock); 
                        //User_StartClock(&BoardCloseTimeoutclock);
                    }
                }
            }
        }
    }
}

/*
发送AT指令   --->  等待串口回复
串口回复超时 --->  重发
重发次数到   --->  失败
*/

//AT指令 发送超时
void ATTimeoutCb(void)
{
    //时间到，未收到正确的回复，重发
    if(Retry_state != SET)
    {
        //超时重发
        if(AT_Retry_Num < AT_RETRY_NUM_MAX-1)
        {
            AT_Retry_Num++;
            //开
            if(curAT_RSP_ID == AT_CFUN0)
            {   
                //重发
                NB_AT_CMD_Send(AT_CFUN0);
                ATTimeoutclock.timeOut = AT_CMD_CFUN0_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
                printf("AT_CFUN0");
            }
            //关
            else if(curAT_RSP_ID == AT_CFUN1)
            {
                //重发
                NB_AT_CMD_Send(AT_CFUN1);
                ATTimeoutclock.timeOut = AT_CMD_CFUN1_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
                printf("AT_CFUN1");
            }
            //查IP
            else if(curAT_RSP_ID == AT_CGPADDR)
            {
                //重发
                NB_AT_CMD_Send(AT_CGPADDR);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            else if(curAT_RSP_ID == AT_CGSN)
            {
                NB_AT_CMD_Send(AT_CGSN);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            else if(curAT_RSP_ID == AT_NRB)
            {
                //重发
                NB_AT_CMD_Send(AT_NRB);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
                printf("NRB");
            }
            else if(curAT_RSP_ID == AT_CGDCONT)
            {
                NB_AT_CMD_Send(AT_CGDCONT);
                //ATTimeoutclock.timeOut = AT_CMD_CGDCONT_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //建立Socket
            else if(curAT_RSP_ID == AT_NSOCR)
            {
                NB_AT_CMD_Send(AT_NSOCR);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //NNMI
            else if(curAT_RSP_ID == AT_NNMI)
            {
                NB_AT_CMD_Send(AT_NNMI);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //PSM
            else if(curAT_RSP_ID == AT_OPEN_PSM)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)OpenPsm, 32/*sizeof(OpenPsm)*/);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //closeEdrx
            else if(curAT_RSP_ID == AT_CLOSE_EDRX)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)CloseEdrx, 28/*sizeof(CloseEdrx)*/);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //IOT地址
            else if(curAT_RSP_ID == AT_NCDP)
            {
                NB_AT_CMD_Send(AT_NCDP);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //coap进入休眠
            else if(curAT_RSP_ID == AT_COAP_ENTER_SLEEP)
            {                           
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)coapEnterSleep, 32);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //小区重选
            else if(curAT_RSP_ID == AT_CELL_RESELECT)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)CellReselection, 35);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //清除频点
            else if(curAT_RSP_ID == AT_CLEAR_FREQ)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)ClearFreq, 14); 
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            else if(curAT_RSP_ID == AT_FOTA_REPORT)
            {
                FOTA_Success_Rsp(0);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            else if(curAT_RSP_ID == AT_NCCID)
            {
                //HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)Nccid, 10); 
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            User_StartClock(&ATTimeoutclock);
        }
        //重发次数到
        else
        {
            Retry_state = RESET;
            AT_Retry_Num = 0;
        }
    }
}

/*
* @fn      NB_NetWork_Init
* @brief   初始化NB模块，连接网络  
* @param   
* @return   
*/
void NB_NetWork_Init(void)
{      
    //NB_NetWork_First_Init字段主要为了获取IMEI号与NB型号
    if(NB_NetWork_First_Init == SET)
    {
        AT_Retry_Num = 0;
        Retry_state = RESET;    
        //1.关闭射频功能
        NB_AT_CMD_Send(AT_CFUN0);
        //开启超时定时器，超时重发，重发次数到后 返回失败。       
        User_StartClock(&ATTimeoutclock);
        curAT_RSP_ID = AT_CFUN0;
        //printf("AT_CFUN0");
    }
    else if(NB_NetWork_First_Init == RESET)
    {
        //AT_Retry_Num = 0;
        //Retry_state = RESET;
        //NB_AT_CMD_Send(AT_NRB);
        curAT_RSP_ID = AT_NRB_RESULT;
        //User_StartClock(&ATTimeoutclock);
        printf("Reboot\r\n");
        LPUART1_Rx_Disable();
        User_StartClock(&OneTimeoutclock);
    }
}

/*
* @fn      Handel_NB_AT_RSP
* @brief   处理NB模块回复  
* @param   当前 等待回复 的指令ID
* @return   
*/
void Handel_NB_AT_RSP(uint8_t *atMessage)
{
    /*
    等待数据回复需要超时重发机制
    */
    //开启关闭协议栈需重发
    printf("R\r\n");
    if(curAT_RSP_ID == AT_CFUN0 || curAT_RSP_ID == AT_CFUN1)
    {
        //OK
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            Retry_state = SET;
            AT_Retry_Num = 0;
            if(curAT_RSP_ID == AT_CFUN0)
            {
                //关闭超时定时器
                User_StopClock(&ATTimeoutclock);
                
                printf("Clear_Freq\r\n");
                //清除频点
                curAT_RSP_ID = AT_CLEAR_FREQ;
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)ClearFreq, 14);
                
                Retry_state = RESET;
                AT_Retry_Num = 0;
                ATTimeoutclock.timeOut = 3000;
                User_StartClock(&ATTimeoutclock);
                
                //下一步查询版本号               
                //NB_AT_CMD_Send(AT_CGMR);
                //curAT_RSP_ID = AT_CGMR;
            }
            else if(curAT_RSP_ID == AT_CFUN1)
            {
                //关闭超时定时器
                User_StopClock(&ATTimeoutclock);
                //下一步开启错误提示
                NB_AT_CMD_Send(AT_CMEE);
                curAT_RSP_ID = AT_CMEE;
            }            
        }
    }
    //清除频点
    else if(curAT_RSP_ID == AT_CLEAR_FREQ)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            AT_Retry_Num = 0;
            User_StopClock(&ATTimeoutclock);
            printf("CELL_RESELECT\r\n");
            //清除频点
            curAT_RSP_ID = AT_CELL_RESELECT;
            HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)CellReselection, 35);
            User_StartClock(&ATTimeoutclock);
        }

    }
    //小区重选
    else if(curAT_RSP_ID == AT_CELL_RESELECT)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            User_StopClock(&ATTimeoutclock);
            //下一步，查询IMEI号
            AT_Retry_Num = 0;
            Retry_state = RESET;
            NB_AT_CMD_Send(AT_CGSN);
            curAT_RSP_ID = AT_CGSN;
            User_StartClock(&ATTimeoutclock);
            printf("IMEI\r\n");
        }
    }
//    //版本号
//    else if(curAT_RSP_ID == AT_CGMR)
//    {
//        char *str;
//        //\r\nSECURITY, 长度为 11
//        str = strtok((char*)atMessage+11, AT_END); 
//        NB_version_len = strlen(str);      
//        tool_memcpy(NB_version, str, NB_version_len);
//        //下一步，查询IMEI号
//        AT_Retry_Num = 0;
//        Retry_state = RESET;
//        NB_AT_CMD_Send(AT_CGSN);
//        curAT_RSP_ID = AT_CGSN;
//        User_StartClock(&ATTimeoutclock);
//        printf("IMEI\r\n");
//    }
    //查询IMEI
    else if(curAT_RSP_ID == AT_CGSN)
    {
        if(strncmp((char*)atMessage, AT_IMEI_RSP, AT_IMEI_RSP_LEN) == STR_CMP_TRUE)
        {
            Retry_state = SET;
            AT_Retry_Num = 0;
            User_StopClock(&ATTimeoutclock);
            
            char *str;
            str = strtok((char*)atMessage+AT_IMEI_RSP_LEN, AT_END); 
            if(strlen(str) == IMEI_NUM_LEN)
            {
                //保存IMEI号
                tool_memcpy(NB_Report_to_ServerData.IMEI_num, str, IMEI_NUM_LEN); 
                
                //下一步设置IOT平台地址
                AT_Retry_Num = 0;
                Retry_state = RESET;
                NB_AT_CMD_Send(AT_NCDP);
                curAT_RSP_ID = AT_NCDP;
                User_StartClock(&ATTimeoutclock);
                printf("IOT\r\n");
            }
        }
    }
    //设置IOT平台地址
    else if(curAT_RSP_ID == AT_NCDP)
    {
        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        //回复的为OK
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            //下一步软重启
            AT_Retry_Num = 0;
            Retry_state = RESET;
            NB_AT_CMD_Send(AT_NRB);
            curAT_RSP_ID = AT_NRB;
            User_StartClock(&ATTimeoutclock);
            //初始化成功
            NB_NetWork_First_Init = RESET;
            printf("NRB\r\n");
        }
    }
    //软重启
    else if(curAT_RSP_ID == AT_NRB)
    {
        if(strncmp((char*)atMessage, AT_NRB_RSP, AT_NRB_RSP_LEN) == STR_CMP_TRUE)
        {
            //下一步等待重启结果
            curAT_RSP_ID = AT_NRB_RESULT;
            printf("REBOOTING\r\n");
            LPUART1_Rx_Disable();
            User_StartClock(&OneTimeoutclock);
        }
    }
    //重启结果
    else if(curAT_RSP_ID == AT_NRB_RESULT)
    {
        char* str;
        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        str = strstr((char*)atMessage, AT_NRB_RESULT_RSP);
        if(str != NULL)
        {
            //下一步开启协议栈
            //NB_AT_CMD_Send(AT_CFUN1);
            curAT_RSP_ID = AT_CFUN1;
            Retry_state = RESET;
            AT_Retry_Num = 0;
            ATTimeoutclock.timeOut = AT_CMD_CFUN1_TIMEOUT;
            User_StartClock(&ATTimeoutclock);
            printf("CFUN1");
        }
    }
    //开启错误提示->设置基站连接通知，此类命令都回复OK
    else if(curAT_RSP_ID == AT_CMEE)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            curAT_RSP_ID = AT_NNMI;
            NB_AT_CMD_Send(curAT_RSP_ID);
            
            Retry_state = RESET;
            AT_Retry_Num = 0;
            ATTimeoutclock.timeOut = AT_CMD_CGDCONT_TIMEOUT;
            User_StartClock(&ATTimeoutclock);
            printf("NNMI\r\n");
        }    
    }
    else if(curAT_RSP_ID == AT_NNMI)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            Retry_state = SET;
            AT_Retry_Num = 0;
            User_StopClock(&ATTimeoutclock);
            
            curAT_RSP_ID = AT_CGATT;
            NB_AT_CMD_Send(curAT_RSP_ID);
            printf("AT_CGATT\r\n");
        }
    }
    else if(curAT_RSP_ID == AT_CGATT)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            curAT_RSP_ID = AT_CGPADDR;
            NB_AT_CMD_Send(curAT_RSP_ID);
            printf("AT_CGPADDR\r\n");
            
            Retry_state = RESET;
            AT_Retry_Num = 0;
            ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
            User_StartClock(&ATTimeoutclock);
        }
    }
    //查询分配的IP地址
    else if(curAT_RSP_ID == AT_CGPADDR)
    {
        //例：
        /*+CGPADDR:0
        +CGPADDR:1,10.44.195.203
        
        OK
        +CGPADDR:0,10.10.10.8
        +CGPADDR:1
        
        OK
        +CGPADDR:0
        +CGPADDR:1
        
        OK
        */
        //if(strncmp((char*)atMessage, AT_NETWORK_STATE, AT_NETWORK_STATE_LEN) == STR_CMP_TRUE)
        //{
            printf("gettingIP\r\n");
            //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
            //printf("ID1: %d", *(atMessage+AT_NETWORK_STATE_LEN));
            //printf("ID2: %d", *(atMessage+AT_NETWORK_STATE_LEN*2));
            //格式正确+有内容回复
            if((strncmp((char*)atMessage+AT_NETWORK_STATE_LEN, AT_COMMA, AT_COMMA_LEN) == STR_CMP_TRUE) || 
               (strncmp((char*)atMessage+AT_NETWORK_STATE_LEN*2, AT_COMMA, AT_COMMA_LEN) == STR_CMP_TRUE))
            {
                printf("gotIP\r\n");
                User_StopClock(&ATTimeoutclock);
                Retry_state = RESET;
                AT_Retry_Num = 0;
                
                //下一步查询网络状态
                //curAT_RSP_ID = AT_NUESTATS;
                //NB_AT_CMD_Send(AT_NUESTATS);
                
                //FOTA
                FOTA_Cmplt_Flag = FOTA_Cmplt_Flag_Read();
                if(FOTA_Cmplt_Flag == 1)
                {
                    printf("AT_FOTA_REPORT\r\n");
                    curAT_RSP_ID = AT_FOTA_REPORT;
                    FOTA_Success_Rsp(0);
                    User_StartClock(&ATTimeoutclock);
                }
                else
                {
                    //下一步查询网络状态
                    curAT_RSP_ID = AT_NUESTATS;
                    NB_AT_CMD_Send(AT_NUESTATS);
                }
                
                //User_StartClock(&ATTimeoutclock);
                //curAT_RSP_ID = AT_CLOSE_EDRX;
                
                //HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)CloseEdrx, 28/*sizeof(CloseEdrx)*/);
            }
        //}       
    }
    else if(curAT_RSP_ID == AT_FOTA_REPORT)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            printf("FOTA_ok\r\n");
        }
        else
        {
            HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
            Handel_coap_Data(atMessage);
        }
    }
    
    
//    else if(curAT_RSP_ID == AT_CLOSE_EDRX)
//    {
//        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
//        {
//            User_StopClock(&ATTimeoutclock);
//            Retry_state = RESET;
//            AT_Retry_Num = 0;
//            User_StartClock(&ATTimeoutclock);
//            
//            curAT_RSP_ID = AT_OPEN_PSM;
//            HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)OpenPsm, 32/*sizeof(OpenPsm)*/);
//            printf("CLOSE_EDRX\r\n");
//        }    
//    }
//    else if(curAT_RSP_ID == AT_OPEN_PSM)
//    {
//        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
//        {
//            Retry_state = SET;
//            AT_Retry_Num = 0;
//            User_StopClock(&ATTimeoutclock);
//            
//            //下一步查询网络状态
//            curAT_RSP_ID = AT_NUESTATS;
//            NB_AT_CMD_Send(AT_NUESTATS);
//            //User_StartClock(&ATTimeoutclock);
//            printf("OPEN_PSM\r\n");
//        }
//    }
    //查询网络状态，保存
    else if(curAT_RSP_ID == AT_NUESTATS)
    {
        Retry_state = SET;
        AT_Retry_Num = 0;
        User_StopClock(&ATTimeoutclock);
        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        //uint8_t curState = ERROR;
        //curState = NueStatesProcess(atMessage);
        NueStatesProcess(atMessage);
        
#warning 新增coap相关
        printf("Innet!\r\n");
        curAT_RSP_ID = AT_INIT_SUCCESS;
        User_StopClock(&InNetTimeoutclock);    
        InnetTimeOutFlag = RESET;
        NB_InNetWork_Flag = SET;
        if(PT_flag[0] == RESET)
        {
            printf("PT_NB_OK\r\n");
            curAT_RSP_ID = AT_NO;
            DataUpFlag = RESET;
            NB_InNetWork_Flag = RESET;
            NB_open_Flag = RESET;
            Close_NB();
            Device_Open_GPS();
        }
        else if(RegistFlag == RESET)
        {
            curAT_RSP_ID = AT_DATA_RSP; 
            
            NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) 
                + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);                
            NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);

            DeviceRegistclock.timeOut = 4000;
            User_StartClock(&DeviceRegistclock);
            User_StartClock(&BoardCloseTimeoutclock);
            printf("registReq\r\n");
        }
        else if(RegistFlag == SET)
        {
            //修改0703
            User_StartClock(&BoardCloseTimeoutclock);
        }
    }
    else if(curAT_RSP_ID == AT_DATA_RSP)
    {
        HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        //strncmp((char*)atMessage, AT_COAP_RSP_OK, AT_COAP_RSP_OK_LEN) == STR_CMP_TRUE
        //if(testFlag == SET)
        if(strncmp((char*)atMessage, AT_PRAM_CME_159, AT_PRAM_CME_159_LEN) == STR_CMP_TRUE || strncmp((char*)atMessage, AT_PRAM_ERROR, AT_PRAM_ERROR_LEN) == STR_CMP_TRUE )
        {
            //testFlag = RESET;
            curAT_RSP_ID = AT_NO;
            NB_InNetWork_Flag = RESET;
            DataUpFlag = RESET;
            
            printf("CME:, NBRESET\r\n");
            //软复位NB模块重新搜网
            User_StopClock(&ATTimeoutclock);
            User_StopClock(&DataRptclock);
            User_StopClock(&DataRspclock);
            User_StopClock(&BoardCloseTimeoutclock);
            User_StopClock(&InNetTimeoutclock);
            
            User_StartClock(&NBResetclock);
        }
        else if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            //发送成功
            if(RegistFlag == RESET)
            {        
                User_StopClock(&DeviceRegistclock);
                User_StopClock(&DataRptclock);
                if(RegistStateFlag == SET)
                {
                    RegistStateFlag = RESET;
                    RegistFinishFlag = SET;
                    DataUpFlag = SET;
                }
            }
            else
            {
                User_StopClock(&DataRptclock);
            }
            //表示处理当前指令ok
            if(cmd_OK_Flag == SET)
            {
                Schedule_StateStartFlag = RESET;
                Normal_StateFlag = RESET;
                User_StopClock(&DataRspclock);
            }
            
            curAT_RSP_ID = AT_NSONMI;
        }
        
        
//#warning  注意处理此时收到服务器数据的处理
//        else
//        {
//            char* str;
//            str = strstr((char*)atMessage, AT_COAP_HEAD_RX);
//            if(str != NULL)
//            {
//                Handel_coap_Data(atMessage);
//            }
//        }
    }
    //收到数据
    else if(curAT_RSP_ID == AT_NSONMI)
    {
#warning 收到coap数据
         //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
         printf("NSONMI\r\n");
         if(testFlag3 == RESET)
         {
             Handel_coap_Data(atMessage);
         }
//         //粘包处理逻辑
//         if(upload_OK_Flag == SET && cmd_OK_Flag == SET)
//         {
//             printf("#\r\n");
//             upload_OK_Flag = RESET;
//             Coap_EnterSleep();
//         }
    }
//    //清除残留命令
//    else if(curAT_RSP_ID == AT_CLEAR_RESERVE)
//    {
//        
//    }
    //信号强度查询
    else if(curAT_RSP_ID == AT_NUESTATS_NORMAL)
    {
        uint8_t curState = ERROR;
        curState = NueStatesProcess(atMessage);
        //若数据正确
        //下一步 发送数据
        if(curState == SUCCESS)
        {
            printf("SignalOK\r\n");
            curAT_RSP_ID = AT_DATA_RSP;
            NB_InNetWork_Flag = SET;
        }
        else
        {
            printf("SignalERROR\r\n");
            curAT_RSP_ID = AT_DATA_RSP;
            NB_InNetWork_Flag = SET;
        }
    }
    
    else if(curAT_RSP_ID == AT_COAP_ENTER_SLEEP)
    {
        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            Retry_state = SET;
            AT_Retry_Num = 0;
            User_StopClock(&ATTimeoutclock);
            
            curAT_RSP_ID = AT_INIT_SUCCESS;
            User_StopClock(&DataRspclock);
            User_StopClock(&DataRptclock);
            User_StopClock(&BoardCloseTimeoutclock);
            User_StopClock(&ATTimeoutclock);
            User_StopClock(&InNetTimeoutclock);
            //发送完成后清除GPS信息
            //if(currentMode == POSITIONING_MODE)
            //{
                GPS_Info_Delete();
                if(Schedule_ModeFlag == RESET)
                {
                    Locator_Data.GPS_List_num = 0;
                    Locator_Data.GPS_data_len = Locator_Data.GPS_List_num*sizeof(GPS_List_t)+1;
                    
                    NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
                }
                else
                {
                    Schedule_Data_upload.GPS_List_num = 0;
                    Schedule_Data_upload.GPS_data_len = 1;
                    //printf("HH,%d" ,Schedule_Data_upload.GPS_data_len);
                }
                GPS_Info_schedule.stores_num = 0;
            //}
            Close_NB();
            //memset(UsartType.RX_pData, 0, RX_LEN);
            if(RegistFinishFlag == SET)
            {
                DataUpFlag = SET;
                RegistFlag = SET;
                RegistFinishFlag = RESET;
                if(cmd_OK_Flag == RESET)
                {
                    currentMode = POSITIONING_MODE;
                    wakeupCounter = 4320;
                }
                TrackerCycle_short_Flag = SET;
                Locator_Data.initFlag = SET;
                Locator_Data.mode = currentMode;
                
                printf("registSuccess\r\n");
                
                //testFlag = SET;
            }
            else
            {
                wakeupCounter = 0;
            }
            cmd_OK_Flag = RESET;
            SleepStatus = SLEEP;
        }
    }
    
}

/*
* @fn      NueStatesProcess
* @brief   处理网络状态帧 
* @param   串口内容
* @return   
*/
uint8_t NueStatesProcess(uint8_t *atMessage)
{
//    /*
//    例：
//    Signal power:-801
//    Total power:-677
//    TX power:50
//    TX time:673
//    RX time:8823
//    Cell ID:79044177
//    ECL:1
//    SNR:-12
//    EARFCN:2506
//    PCI:3
//    RSRQ:-146
//    
//    OK
//    
//    */
    if(strncmp((char*)atMessage, "\r\n", 2) == STR_CMP_TRUE)
    {
        //NB_Report_to_ServerData
        char *str;
        uint8_t dataCounter = 0;
        str = strtok((char*)atMessage+2, "\r\n");
        while(str)
        {
            //信号强度
            if(strncmp(str, AT_NUESTATS_RSPR, AT_NUESTATS_RSPR_LEN) == STR_CMP_TRUE)
            {
                uint16_t rspr = 0;
                rspr = atoi(str+AT_NUESTATS_RSPR_LEN)/10;
                Signal_Level.RSPR = BigtoLittle16(rspr);
                dataCounter++;
            }
            //小区ID
            else if(strncmp(str, AT_NUESTATS_CELL_ID, AT_NUESTATS_CELL_ID_LEN) == STR_CMP_TRUE)
            {
                uint32_t cell_id = 0;
                cell_id = atoi(str+AT_NUESTATS_CELL_ID_LEN);
                Signal_Level.cellID = cell_id;
                dataCounter++;
            }
            //覆盖等级
            else if(strncmp(str, AT_NUESTATS_ECL, AT_NUESTATS_ECL_LEN) == STR_CMP_TRUE)
            {
                uint8_t ecl = 0;
                ecl = atoi(str+AT_NUESTATS_ECL_LEN);
                Signal_Level.ECL =  ecl;
                dataCounter++;
            }
            //信噪比
            else if(strncmp(str, AT_NUESTATS_SNR, AT_NUESTATS_SNR_LEN) == STR_CMP_TRUE)
            {
                uint16_t snr = 0;
                snr = atoi(str+AT_NUESTATS_SNR_LEN)/10;
                Signal_Level.SINR = BigtoLittle16(snr);
                dataCounter++;
            }
            //信号质量
            else if(strncmp(str, AT_NUESTATS_RSRQ, AT_NUESTATS_RSRQ_LEN) == STR_CMP_TRUE)
            {
                uint16_t rsrq = 0;
                rsrq = atoi(str+AT_NUESTATS_RSRQ_LEN);
                Signal_Level.RSRQ = BigtoLittle16(rsrq);
                dataCounter++;
            }
            str = strtok(NULL,"\r\n");
        }
        if(dataCounter == NUESTATES_NUM)
        {
            return SUCCESS;
        }
    }
    return ERROR;
    //return SUCCESS;
}

//上报数据
void NB_Data_Report(void)
{
    //    if(NB_Init_Flag != SUCCESS)
    //    {
    //        Open_NB();
    //        NB_NetWork_Init();
    //    }
    //    //联网成功
    //    else if( NB_Init_Flag == SUCCESS)
    //    {
    //    if(currentMode == CONFIG_MODE)
    //    {
//#warning 添加  Locator_Data 中的模式信息     
    //    }
    //    else if(currentMode == NORMAL_MODE)
    //    {
    //        
    //    }
    //    else if(currentMode == POSITIONING_MODE)
    //    {
    //        //上报完毕，清空GPS数据
    //    }
    //        //使NB模块向服务器 发送数据
    //        //NB_AT_CMD_Send();
    //         NBTimeOutFlag = RESET;
    //        //User_StartClock(&NBReportclock);
    //    }
}

//GPS数据采集
void User_GPS_TaskProcess(void)
{
    if(currentMode == POSITIONING_MODE)
    {
        if(GPSTimeOutFlag == SET)
        {
            //Open_GPS();
            GPSCollectionclockProcess();
            User_StartClock(&GPSCollectionclock);
            GPSTimeOutFlag = RESET;
        }
    }
}

//使能串口接收
void LPUART1_Rx_Enable(void)
{
    //    HAL_UART_AbortReceive(&hlpuart1);
    //    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_IDLE);//使能 串口 空闲中断
    //    nbRxMessage = (uint8_t *)malloc(USER_BUFF_SIZE);
    //    printf("Rx_En");
    //    HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)nbRxMessage, USER_BUFF_SIZE);
    //    memset(nbRxMessage,0,1);
    HAL_UART_Receive_DMA(&hlpuart1, UsartType.RX_pData, RX_LEN);  
    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_IDLE);
}
//关闭串口接收
HAL_StatusTypeDef LPUART1_Rx_Disable(void)
{
    __HAL_LOCK(&hlpuart1);
    HAL_UART_AbortReceive(&hlpuart1);
    __HAL_UART_DISABLE_IT(&hlpuart1, UART_IT_IDLE);
    __HAL_DMA_DISABLE(hlpuart1.hdmarx);
    hlpuart1.RxXferSize = 0;
    hlpuart1.hdmarx->Instance->CNDTR = 0;
    __HAL_DMA_ENABLE(hlpuart1.hdmarx);
    __HAL_UNLOCK(&hlpuart1);
    return HAL_OK;
}

//使能串口接收
void UART1_Rx_Enable(void)
{
//    HAL_UART_AbortReceive(&huart1);
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//使能 串口 空闲中断
//    gpsRxMessage = (uint8_t *)malloc(USER_BUFF_SIZE);
//    HAL_UART_Receive_DMA(&huart1, (uint8_t *)gpsRxMessage, USER_BUFF_SIZE);
//    memset(gpsRxMessage,0,1);
#warning 修改171229    
    __HAL_DMA_ENABLE(huart1.hdmarx);
    HAL_UART_Receive_DMA(&huart1, UsartType.RX_pData, RX_LEN);  
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}
//关闭串口接收
HAL_StatusTypeDef UART1_Rx_Disable(void)
{
    __HAL_LOCK(&huart1);
    HAL_UART_AbortReceive(&huart1);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_DMA_DISABLE(huart1.hdmarx);
    huart1.RxXferSize = 0;
    huart1.hdmarx->Instance->CNDTR = 0;
#warning 修改171229
    //__HAL_DMA_ENABLE(huart1.hdmarx);
    __HAL_UNLOCK(&huart1);
    return HAL_OK;
}

//串口发送
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == hlpuart1.Instance)
    {
        TxCompleteFlag = SET;
        //printf("T\r\n");        
        //        free(at_cmd_data_p);
        //        at_cmd_data_p = NULL;
        //        if(curAT_RSP_ID == AT_DATA_RSP)
        //        {
        //            printf("RxEn");
        //            LPUART1_Rx_Enable();
        //        }
    }
}
//串口接收
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == hlpuart1.Instance)
    {
        asm("nop");
        //        printf("R\r\n");        
        //        //      测试AT指令用
        //        if(tempLen != 0)
        //        {
        //            __disable_irq();   // 关闭总中断
        //            LPUART1_Rx_Disable();
        //            Handel_NB_AT_RSP(nbRxMessage);
        //            free(nbRxMessage);
        //            //处理完成后重开
        //            if(curAT_RSP_ID != AT_NRB_RESULT)
        //            {
        //                LPUART1_Rx_Enable();
        //            }
        //            __enable_irq();    // 开启总中断
        //        }
    }
    else if(huart->Instance == huart1.Instance)
    {
        //        //关闭接收
        //        UART1_Rx_Disable();
        //        if(tempLen != 0)
        //        {
        //            //处理GPS数据
        //            GPSInfoProcess();
        //            printf("GPSData\r\n");
        //        }
        //        free(gpsRxMessage);
        //        UART1_Rx_Enable();
    }
}

void UserIOInit(void)
{
    //    GPIO_InitTypeDef GPIO_InitStruct;
    //    
    //    /* GPIO Ports Clock Enable */
    //    __HAL_RCC_GPIOA_CLK_ENABLE();
    //    __HAL_RCC_GPIOB_CLK_ENABLE();
    //      
    //    /*Configure GPIO pins : PAPin PAPin PAPin */
    //    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    //    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    //    GPIO_InitStruct.Pull = GPIO_NOPULL;
    //    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //    
    //    /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
    //    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    //    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    //    GPIO_InitStruct.Pull = GPIO_NOPULL;
    //    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    //    
    //    /*Configure GPIO pins : PAPin PAPin PAPin */
    //    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    //    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    //    GPIO_InitStruct.Pull = GPIO_NOPULL;
    //    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    //关闭所有灯
    closeAllLed();
    //关闭,NB,GPS
    HAL_GPIO_WritePin(GPS_SW_GPIO_Port, GPS_SW_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NB_SW_GPIO_Port, NB_SW_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GPIOB, NB_RESET_Pin, GPIO_PIN_SET);
}


void BoardSleep(void)
{
    //closeAllLed();
    //    User_StopClock(&LEDclock);
    //    BoardStart = RESET;
    //    NB_InNetWork_Flag = RESET;
    //    RegistFlag = RESET;
    //    RegistConfirmFlag = RESET;
    //    LED_Init_Blink_Finish = RESET;
    //    NB_NetWork_First_Init = SET;
    //    curAT_RSP_ID = AT_CFUN0;
    //    currentMode = REGIST_MODE;
    //    LED_Init_Blink_Count = 0;
    //    Close_NB();
    //    //LPUART1_Rx_Disable();
    //    Close_GPS();
    //    printf("Boardsleep");
    //    __set_PRIMASK(1);
    //    NVIC_SystemReset();
            //发送完成后清除GPS信息
    if(RegistFlag == RESET)
    {
        NVIC_SystemReset();
    }
    else
    {
        if(currentMode == POSITIONING_MODE)
        {
            //GPS_Info_Delete();
        }
        User_StopClock(&ATTimeoutclock);
        User_StopClock(&DataRptclock);
        User_StopClock(&DataRspclock);
        User_StopClock(&InNetTimeoutclock);
        
        Schedule_StateStartFlag = RESET;
        Normal_StateFlag = RESET;
        if(move_alarm_enable_Flag == SET)
        {
            move_alarm_Init_Flag = SET;
            Locator_Data.sensorFlag = 0x02;
        }
        else
        {
            Locator_Data.sensorFlag = 0x00;
        }
        printf("sleep5\r\n");
        //upload_OK_Flag = RESET;
        cmd_OK_Flag = RESET;
        Close_NB();
        wakeupCounter = 0;
        SleepStatus = SLEEP;
    }
}


/**
* @brief  System Power Configuration
*         The system Power is configured as follow : 
*            + Regulator in LP mode
*            + VREFINT OFF, with fast wakeup enabled
*            + HSI as SysClk after Wake Up
*            + No IWDG
*            + Wakeup using EXTI Line (Key Button PC.13)
* @param  None
* @retval None
*/
static void SystemPower_Config(void)
{
    //  GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Enable Ultra low power mode */
    HAL_PWREx_EnableUltraLowPower();
    
    /* Enable the fast wake up from Ultra low power mode */
    HAL_PWREx_EnableFastWakeUp();
    
    /* Select HSI as system clock source after Wake Up from Stop mode */
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
    
    //  /* Enable GPIOs clock */
    //  __HAL_RCC_GPIOA_CLK_ENABLE();
    //  __HAL_RCC_GPIOB_CLK_ENABLE();
    //  __HAL_RCC_GPIOC_CLK_ENABLE();
    //  __HAL_RCC_GPIOD_CLK_ENABLE();
    //  __HAL_RCC_GPIOH_CLK_ENABLE();
    //
    //  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
    //  GPIO_InitStructure.Pin = GPIO_PIN_All;
    //  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    //  GPIO_InitStructure.Pull = GPIO_NOPULL;
    //  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
    //  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    //  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    //  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
    //  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
    //
    //  /* Disable GPIOs clock */
    //  __HAL_RCC_GPIOA_CLK_DISABLE();
    //  __HAL_RCC_GPIOB_CLK_DISABLE();
    //  __HAL_RCC_GPIOC_CLK_DISABLE();
    //  __HAL_RCC_GPIOD_CLK_DISABLE();
    //  __HAL_RCC_GPIOH_CLK_DISABLE();
    
}
//按键初始化
void KEY_GPIO_Init(void)
{
    
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = KEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/**
* @brief  Configures system clock after wake-up from STOP: enable HSI, PLL
*         and select PLL as system clock source.
* @param  None
* @retval None
*/
static void SystemClockConfig_STOP(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    
    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();
    
    /* The voltage scaling allows optimizing the power consumption when the device is 
    clocked below the maximum system frequency, to update the voltage scaling value 
    regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    /* Get the Oscillators configuration according to the internal RCC registers */
    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
    
    /* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    //RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_4;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
    clocks dividers */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
* @brief  RTC_Timer_Start
* @param  None
* @retval None
*/
void RTC_Timer_Start(void)
{
    /* Disable Wakeup Counter */
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    
    /*## Setting the Wake up time ############################################*/
    /*  RTC Wakeup Interrupt Generation:
    Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
    Wakeup Time = Wakeup Time Base * WakeUpCounter 
    = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
    ==> WakeUpCounter = Wakeup Time / Wakeup Time Base
    
    To configure the wake up timer to 4s the WakeUpCounter is set to 0x1FFF:
    RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
    Wakeup Time Base = 16 /(~39.000KHz) = ~0,410 ms
    Wakeup Time = ~4s = 0,410ms  * WakeUpCounter
    ==> WakeUpCounter = ~4s/0,410ms = 9750 = 0x2616 */
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 40960, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}

/**
* @brief  RTC Wake Up callback
* @param  None
* @retval None
*/
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    /* Clear Wake Up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    //单位20s ,60s/20s = 3
    if(RegistFlag == SET)
    {     
        wakeupCounter++;
        //uint16_t dataCycle_T = 0;
        //单位--分钟
#warning 添加schedule模式180323
        if(move_alarm_Flag == SET)
        {
            currentMode = POSITIONING_MODE;
            dataCycle = 5;
        }
        else
        {
            if(Schedule_ModeFlag == RESET)
            {
                if(currentMode == NORMAL_MODE)
                {
                    dataCycle = Locator_Data.FindnowCycle;
                }
                else if(currentMode == POSITIONING_MODE)
                {
#warning  添加171229
                    //gps保持计数2h
                    GPSKeepCounter++;
                    if(GPSKeepCounter >= GPS_KEEP_MIN*3)
                    {
                        GPSKeepCounter = 0;
                        GPSKeepTimeoutFlag = SET;
                    }
                    dataCycle = Locator_Data.TrackerCycle;
                }
            }
            else if(Schedule_ModeFlag == SET)
            {
                if(move_alarm_enable_Flag == SET)
                {
                    //移动报警结束，恢复schedule
                    if(schedule_recoverFlag == RESET)
                    {
                        schedule_recoverFlag = SET;
                        ScheduleParmSet(CurrentDayInTheLoop);
                    }
                }
                if(currentMode == POSITIONING_MODE)
                {
                    GPSKeepCounter++;
                    if(GPSKeepCounter >= GPS_KEEP_MIN*3)
                    {
                        GPSKeepCounter = 0;
                        GPSKeepTimeoutFlag = SET;
                    }
                }
                if(Time_syning_Flag == RESET)
                {
                    //否计算出下次上报时间
                    ScheduleModeProcess();
                }
            }
        }
        if(dataCycle>0)
        {
            if(wakeupCounter >= (dataCycle*3))
            {
                wakeupCounter = 0;
                if(gps_open_Flag == RESET && NB_open_Flag == RESET)
                {
                    if(move_alarm_enable_Flag == SET)
                    {
                        //5分钟上报完，清空移动标志位
                        if(move_alarm_Flag == SET)
                        {
                            move_alarm_Flag = RESET;
                            move_continue_Flag = SET;
                            currentMode = MoveAlarmPreMode;
                        }
                    }
                }
                Locator_Data.userAlarmFlag = RESET;
                Schedule_Data_upload.scheduleID = RESET;
                if(gps_open_Flag == RESET && NB_open_Flag == RESET)
                {
                    NBTimeOutFlag = SET;
                }
                SleepStatus = AWAKE;
                //printf("AWAKE");
                return;
            }
        }
        if(gps_open_Flag == RESET && NB_open_Flag == RESET)
        {
            if(adc_timeoutFlag == RESET)
            {
                adc_counter++;
                if(adc_counter >= ADC_20S_NUM)
                {
                    adc_counter = 0;
                    adc_timeoutFlag = SET;
                    SleepStatus = AWAKE;
                }
                //printf("ADC:%d", adc_counter);
            }
        }
    }
}
/**
* @brief  User_Event_Process
* @param  None
* @retval None
*/
uint16_t User_Event_Process(void)
{
    if(event & KEY_PRESS_EVENT)
    {
        keyFlag = SET;
        //if(SleepStatus == AWAKE)
        //{
            //printf("keyPress!\r\n");
            if(KeyAction.keyPressFlag != SET)
            {
                //标记按键已按下
                KeyAction.keyPressFlag = SET;
                //判断按鍵定时器状态，开启按鍵定时器
                if(KeyPressTimerStartFlag == RESET )
                {
                    KeyPressTimerStartFlag = SET;
                    //默认此次为长按
                    KeyAction.keyLongPressFlag = SET;
                    User_StartClock(&KeyPressclock);
                    printf("KeyTimerStart\r\n");
                }
                //在按键定时器开启状态下，判断消抖定时器状态，开启消抖定时器
                if( (KeyPressTimerStartFlag == SET) && (AntiShakeTimerStartFlag == RESET) )
                {
                    AntiShakeTimerStartFlag = SET;
                    HAL_TIM_OnePulse_Init(&htim21, TIM_OPMODE_SINGLE);
                    __HAL_TIM_CLEAR_IT(&htim21, TIM_IT_UPDATE);
                    HAL_TIM_Base_Start_IT(&htim21);
                }
            }
        //}
        event = event ^ KEY_PRESS_EVENT;
        //return (event);
    }
    
    if(event & KEY_RELEASE_EVENT)
    {
        //按下一次且未抬起则判断为长按，否则为短按
        if((KeyPressTimerStartFlag == SET) /*&& (KeyAction.keyConut > 0)*/)
        {
            printf("short");
            KeyAction.keyLongPressFlag = RESET;
        }
        event = event ^ KEY_RELEASE_EVENT;
        //return (event);
    }
    if(event & TIM2_TIMEOUT_EVENT)
    {
        //主定时器
        User_TaskRemarks();
        event = event ^ TIM2_TIMEOUT_EVENT;
        //return (event); 
    }
    if(event & TIM21_TIMEOUT_EVENT)
    {
        if(KeyAction.keyPressFlag == SET)
        {
            //防抖
            if( HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
            {
                //按键有效
                KeyAction.keyConut++;
                //                if(KeyAction.keyConut>7)
                //               {
                //                    KeyAction.keyConut = 0;
                //                    NVIC_SystemReset();
                //                }
                //printf("keyCount:%d\r\n", KeyAction.keyConut);
                //注册完成
                if(RegistFlag == SET)
                {
                    //printf("EPStartFlag:%d\r\n", EvenPressTimerStartFlag);
                    //按键正常按下
                    if(KeyPressTimerStartFlag == SET)
                    {
                        //连按未开启
                        if(EvenPressTimerStartFlag == RESET && KeyAction.keyConut == 1)
                        {
                            //不在切换模式
                            if(ModeSwitchFlag == RESET)
                            {
                                //开启连按定时
                                EvenPressTimerStartFlag = SET;
                                //KeyAction.keyConut = 0;
                                //printf("EvenPress");
                                User_StartClock(&EvenPressclock);
                            }
                        }
                    }
                }
                
#warning    短按5下切换模式
                if(KeyAction.keyConut == ENTER_CONFIG_MODE_KEYNUM)
                {
                    if(PT_flag[0] == RESET)
                    {
                        printf("StartPT");
                        Device_Open_NB();
                        User_StartClock(&LEDclock);
                    }
                    else
                    {
                        if(RegistFlag == SET)
                        {
                            if(Schedule_ModeFlag == RESET)
                            {
                                User_StopClock(&KeyPressclock);
                                KeyPressTimerStartFlag = RESET;
                                
                                if(gps_open_Flag == SET)
                                {
                                    User_StopClock(&GPSTimeOutclock);
                                    DataUpFlag = RESET;
                                    printf("5GPS\r\n");
                                    Device_Close_GPS();
                                }
                                if(NB_open_Flag == SET)
                                {
                                    User_StopClock(&ATTimeoutclock);
                                    User_StopClock(&InNetTimeoutclock);
                                    User_StopClock(&DataRptclock);
                                    User_StopClock(&DataRspclock);
                                    User_StopClock(&BoardCloseTimeoutclock);                               

                                    printf("5NB\r\n");
                                    curAT_RSP_ID = AT_NO;
                                    DataUpFlag = RESET;
                                    NB_InNetWork_Flag = RESET;
                                    NB_open_Flag = RESET;
                                    Close_NB();
                                }
                                KeyAction.keyConut = 0;
                                //按键切换模式标记
                                ModeSwitchFlag = SET;
                                if(currentMode == NORMAL_MODE)
                                {
                                    currentMode = POSITIONING_MODE;
                                }
                                else if(currentMode == POSITIONING_MODE)
                                {
                                    currentMode = NORMAL_MODE;
                                    
                                    GPS_Info.stores_num = 0;
                                    Locator_Data.GPS_List_num = GPS_Info.stores_num;
                                    Locator_Data.GPS_data_len = Locator_Data.GPS_List_num*sizeof(GPS_List_t)+1;
                                }
                                
                                //#warning 测试用！！！！！！
                                //                        Schedule_Data_upload.GPS_List_num = 0;
                                //                        Schedule_Data_upload.GPS_data_len = 1;
                                //                        printf("HH,%d" ,Schedule_Data_upload.GPS_data_len);
                                
                                //Locator_Data.userAlarmFlag = SET;
                                Locator_Data.mode = currentMode;
                                LED_Init_Blink_Finish = RESET;
                                LED_Init_Blink_Count = 0;
                                LEDclock.timeOut = MODE_SWICTH_LED_TIMEOUT;
                                User_StartClock(&LEDclock);
                            }
                        }
                    }
                }
            }
            KeyAction.keyPressFlag = RESET;
        }
        AntiShakeTimerStartFlag = RESET;
        
        event = event ^ TIM21_TIMEOUT_EVENT;
        //return (event); 
    }
    if(event & TIM22_TIMEOUT_EVENT)
    {
        //处理LED闪烁问题
        Handel_LED();
        
        event = event ^ TIM22_TIMEOUT_EVENT;
        //return (event); 
    }
    //移动报警
    if(event & MOVE_ALARM_EVENT)
    {
        //app已下发移动报警配置
        if(move_alarm_enable_Flag == SET)
        {
            if(move_alarm_Flag == RESET)
            {
                move_alarm_Flag = SET;
                if(move_alarm_Init_Flag == RESET)
                {
                    MoveAlarmPreMode = currentMode;
                    //初次移动
                    //move_alarm_Init_Flag = SET;
                }
                else
                {
                    if(gps_open_Flag == SET)
                    {
                        User_StopClock(&GPSTimeOutclock);
                        DataUpFlag = RESET;
                        printf("MGPS\r\n");
                        Device_Close_GPS();
                    }
                    if(NB_open_Flag == SET)
                    {
                        User_StopClock(&ATTimeoutclock);
                        User_StopClock(&InNetTimeoutclock);
                        User_StopClock(&DataRptclock);
                        User_StopClock(&DataRspclock);
                        User_StopClock(&BoardCloseTimeoutclock);                               
                        
                        printf("MNB\r\n");
                        curAT_RSP_ID = AT_NO;
                        DataUpFlag = RESET;
                        NB_InNetWork_Flag = RESET;
                        NB_open_Flag = RESET;
                        Close_NB();
                    }
                    //if(NB_open_Flag == RESET && gps_open_Flag == RESET)
                    {
                        printf("ALARM\r\n");
                        schedule_recoverFlag = RESET;
                        Locator_Data.sensorFlag = 0x01;
                        
                        currentMode = POSITIONING_MODE;
                        wakeupCounter = 0;
                        dataCycle = 5;
                        NBTimeOutFlag = SET;
                    }  
                }
            }
            else
            {
                Locator_Data.sensorFlag = 0x01;
                //已在移动
                if(NB_open_Flag == RESET && gps_open_Flag == RESET)
                {
                    if(keyFlag == RESET )
                    {
                        SleepStatus = SLEEP;
                    }
                }
            }
        }
        else
        {
            //未配置过移动报警
            if(NB_open_Flag == RESET && gps_open_Flag == RESET)
            {
                if(keyFlag == RESET )
                {
                    SleepStatus = SLEEP;
                }
            }
        }
        event = event ^ MOVE_ALARM_EVENT;
        //return (event);
    }
    
    
    
    return 0;
}
/**
* @brief  UserStartLedClock,开启LED定时器
* @param  None
* @retval None
*/
void UserStartLedClock(void)
{
    HAL_TIM_OnePulse_Init(&htim22, TIM_OPMODE_SINGLE);
    __HAL_TIM_CLEAR_IT(&htim22, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim22);
}

void UserStopLedClock(void)
{
    __HAL_TIM_CLEAR_IT(&htim22, TIM_IT_UPDATE);
    HAL_TIM_Base_Stop_IT(&htim22);
}

/**
* @brief  ModeDisplayCb,模式显示回调
* @param  None
* @retval None
*/
void ModeDisplayCb(void)
{
    //if(chargeEnable == RESET)
    //{
    closeAllLed();
    if(chargeEnable == SET && chargeFinish == RESET)
    {
        //红灯亮
        HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
    }
    else if(chargeEnable == SET && chargeFinish == SET)
    {
        //充电完成
        HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
    }
#warning 修改0105        
    if(ExtraDataupLEDFlag == SET)
    {
        ExtraDataupLEDFlag = RESET;
        wakeupCounter = 0;
        NBTimeOutFlag = SET;
        SleepStatus = AWAKE;
        printf("xxx\r\n");
    }
    else if(Bat_Level <= BATTERY_LOW_LV)
    {    
        if(ModeDisplayCount < 1)
        {    
            printf("modeDis\r\n");
            HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
            ModeDisplayCount++;
            User_StartClock(&ModeDisplayclock);
        }
        else
        {
            ModeDisplayCount = 0;
            //closeAllLed();
            if(NB_open_Flag == RESET && gps_open_Flag == RESET)
            {
                printf("Sleep8\r\n");
                SleepStatus = SLEEP;
            }
        }
    }
    else
    {
        if(NB_open_Flag == RESET && gps_open_Flag == RESET)
        {
            printf("Sleep9\r\n");
            SleepStatus = SLEEP;
        }
        printf("NOSleep\r\n");
    }
    //}
}
/**
* @brief  InNetCb,入网超时
* @param  None
* @retval None
*/
void InNetCb(void)
{
    if(InNetCounter == 0)
    {
        InNetCounter++;
        User_StartClock(&InNetTimeoutclock);
    }
    else
    {
        InNetCounter = 0;
        if(RegistFlag == RESET)
        {
            if(PT_flag[0] == RESET)
            {
                printf("PT_NB_ERROR\r\n");
                User_StopClock(&LEDclock);
                User_StopClock(&LEDBlinkclock);
                UserStopLedClock();
                HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
                Testclock.timeOut = 5000;
                User_StartClock(&Testclock);
            }
            else
            {
                NVIC_SystemReset();
            }
        }
        else if(RegistFlag == SET)
        {
            printf("InnetTimeOut");
            InnetTimeOutFlag = SET;
            Close_NB();
            wakeupCounter = 0;
            SleepStatus = SLEEP;
        }
    }
}

/**
* @brief  UnbindCb
* @param  None
* @retval None
*/
void UnbindCb(void)
{
    NVIC_SystemReset();    
}

/**
* @brief  EvenPressCb
* @param  None
* @retval None
*/
void EvenPressCb(void)
{
    EvenPressTimerStartFlag = RESET;
    printf("...");
    if(KeyAction.keyLongPressFlag == RESET)
    {
        printf("EventKeyCount:%d\r\n", KeyAction.keyConut);
        if(KeyAction.keyConut == 1)
        {
            printf("!");
            KeyAction.keyConut = 0;
            User_StopClock(&KeyPressclock);
            KeyPressTimerStartFlag = RESET;
            EvenPressFlag = SET;           
            LEDclock.timeOut = MODE_SWICTH_LED_TIMEOUT;
            User_StartClock(&LEDclock);
        }
    }
}
/**
* @brief  GPSKeepCb
* @param  None
* @retval None
*/
void GPSKeepCb(void)
{
    /*
    如果开启了保持定时器 等到定时器结束后再保存最后一个
    有效的gps点指针 GPS_TO_NB_Data
    */
    
    GPS_Info_Store(GPS_TO_NB_Data);
    
    GPS_TO_NB_Data = NULL;
    GPSKeepTimerStartFlag = RESET;
    //若为保持20秒，则时间到后清除标志位
    if(GPSKeepclock.timeOut == GPS_KEEP_TINEOUT_20S)
    {
        GPSKeepTimeoutFlag = RESET;
    }
    
    printf("GPS_SUCCESS\r\n");
    
    Device_Close_GPS();
    
    if( Schedule_ModeFlag == RESET )
    {
        Locator_Data.GPS_List_num = GPS_Info.stores_num;
        Locator_Data.GPS_data_len = Locator_Data.GPS_List_num*sizeof(GPS_List_t)+1;
        NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
    }
    else
    {
        Schedule_Data_upload.GPS_List_num = GPS_Info_schedule.stores_num;
        Schedule_Data_upload.GPS_data_len = Schedule_Data_upload.GPS_List_num*sizeof(GPS_List_t);
        NB_Report_to_ServerData.cmdlen = sizeof(NB_Report_to_ServerData_t)+sizeof(Schedule_Data_upload_t)+ Schedule_Data_upload.GPS_List_num * sizeof(GPS_List_t) + CHECK_SUM_LEN + END_NUM_LEN;
    }
    Device_Open_NB();
}

/**
* @brief  GPSRMCCb
* @param  None
* @retval None
*/
void GPSRMCCb(void)
{
#warning 添加171230
        GPSRMCCounter = SET;
        printf("RMC_Change\r\n");
        UART1_Rx_Enable();
}
/**
* @brief  UserAdcProcess
* @param  None
* @retval None
*/
void User_AdcProcess(void)
{
    //采集时间到
    if(adc_timeoutFlag == SET)
    {
        //uint8_t i = 0;
#warning 修改1230        
        // 关闭总中断
        //__disable_irq();
        //uint8_t adcokCounter = 0;
        AD_ValueSum = 0;
        //for(uint8_t i=0 ;i<10; i++)
        //{
            HAL_ADC_Start(&hadc);
            HAL_ADC_PollForConversion(&hadc, 250);
            if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc), HAL_ADC_STATE_REG_EOC))
            {
                AD_Value = HAL_ADC_GetValue(&hadc);
                //AD_ValueSum = AD_ValueSum + AD_Value;
                //adcokCounter++;
            }
        //}
        //AD_ValueSum = AD_ValueSum/adcokCounter;
        //采样电压值
        AD_Value  = (AD_Value*3200) >> 12;
        //AD_ValueSum  = (AD_ValueSum*3200) >> 12;
        //printf("ADC:%d", AD_Value);
        //锂电池电压值
        //Bat_Value = AD_ValueSum*(R_GND + R_VCC)/R_GND;
        Bat_Value = AD_Value*(R_GND + R_VCC)/R_GND;
        printf("BAT:%d\r\n", Bat_Value);
                
        //电量值
        Bat_Level = BatValueConvert(/*AD_Value*/Bat_Value, Pre_Bat_Level, chargeEnable, RESET);
        Locator_Data.batteryLv = Bat_Level;
        Schedule_Data_upload.battery_Lv = Bat_Level;
        
        Pre_Bat_Level = Bat_Level;
        printf("BAT:%d\t\n", Bat_Level);
        // 开启总中断
        //__enable_irq();
        //处理完成后休眠
        adc_timeoutFlag = RESET;
        if(NB_open_Flag == RESET && gps_open_Flag == RESET)
        {
            printf("Adc_Sleep\r\n");
            SleepStatus = SLEEP;
        }
    }
    //if(Bat_Value < 3180 || testFlag == SET)
    if(Bat_Value < 3200)
    {
        if(BoardStart == SET)
        {
            if(gps_open_Flag == RESET && NB_open_Flag == RESET)
            {
                printf("KK\r\n");
                currentMode = NORMAL_MODE;
                //testFlag = RESET;
                testFlag2 = SET;
                BoardStart = RESET;
                NBTimeOutFlag = SET;
                DataUpFlag = RESET;
                SleepStatus = AWAKE;
                //NVIC_SystemReset();
            }
        }
        else
        {
            if(testFlag3 == RESET && testFlag2 == RESET)
            {
                printf("SL");
                adc_timeoutFlag = SET;
                Bat_OK_Flag = RESET;
                SleepStatus = SLEEP;
                StopMode_Measure();
            }
        }
    }
    else
    {
        Bat_OK_Flag = SET;
    }
}

/**
* @brief  User_UartProcess
* @param  None
* @retval None
*/
void User_UartProcess(void)
{
    if(UsartType.RX_flag)    	// Receive flag
    {  
        UsartType.RX_flag=0;	// clean flag
        //未开启GPS,开启了NB
        if(gps_open_Flag != SET && NB_open_Flag == SET)
        {
            LPUART1_Rx_Disable();
            //__disable_irq();
            Handel_NB_AT_RSP(UsartType.RX_pData);
            //__enable_irq();
            LPUART1_Rx_Enable();
        }
        //开启了GPS
        else if(gps_open_Flag == SET)
        {   
            if(GPSRMCCounter == SET)
            {
                GPSRMCCounter = RESET;
                //HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ConfigRMC, sizeof(ConfigRMC));
                memset(UsartType.RX_pData, 0, RX_LEN);
            }
            else
            {
                //关闭接收
                UART1_Rx_Disable();
                //__disable_irq();   // 关闭总中断  
                GPSInfoProcess(UsartType.RX_pData);
                //__enable_irq();    // 开启总中断
                UART1_Rx_Enable();
            }
        }
    }
    
//#warning 测试！！！！！！
//    if(UsartType2.RX_flag)    	// Receive flag
//    {  
//        UsartType2.RX_flag=0;	// clean flag
//        //UART2_Rx_Disable();
//        
//        if(UsartType2.RX_Size != 0)
//        {
//            GPSInfoProcess(UsartType2.RX_pData);
//        }
//        //UART2_Rx_Enable();
//    }
    
}


/**
* @brief  huart1_to_IOEXIT
* @param  None
* @retval None
*/
void huart1_to_IOEXIT(void)
{
    HAL_UART_MspDeInit(&huart1);
    
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
* @brief  IOEXIT_to_huart1
* @param  None
* @retval None
*/
//void IOEXIT_to_huart1(void)
//{
//    //__disable_irq();   // 关闭总中断
//    printf("!!!\r\n");
//    //if(gps_counter > 0)
//    //{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    
//    GPIO_InitStruct.Pin = GPIO_PIN_10;
//    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_DeInit(GPIOA, GPIO_InitStruct.Pin);
//    HAL_UART_MspInit(&huart1);
//    UART1_Rx_Enable();
//    //}
//    //__enable_irq();    // 开启总中断
//}
/**
* @brief  Device_Open_NB
* @param  None
* @retval None
*/
void Device_Open_NB(void)
{
    //HAL_UART_MspInit(&huart1);
    LPUART1_Rx_Enable();
    NB_open_Flag = SET;
    if(RegistFlag == RESET)
    {
        Open_NB();
        NB_NetWork_Init();
        User_StartClock(&InNetTimeoutclock);
    }
    else
    {
        //测试用
        if( hlpuart1.gState == HAL_UART_STATE_BUSY_TX )
        {
            hlpuart1.gState = HAL_UART_STATE_READY;
            printf("UART_BUSY_TX");
        }
        if(InnetTimeOutFlag == SET)
        {
            printf("NN\r\n");
            User_StartClock(&NBResetclock);
        }
        else
        {
            //testFlag = SET;
            memset(UsartType.RX_pData, 0, RX_LEN);
            
            //curAT_RSP_ID = AT_CLEAR_RESERVE;
            //NB_AT_CMD_Send(AT_NSORF);
            
            curAT_RSP_ID = AT_NUESTATS_NORMAL;
            NB_AT_CMD_Send(AT_NUESTATS);
            
            //NB_InNetWork_Flag = SET;
            BoardCloseTimeoutclock.timeOut = AT_BOARD_CLOSE_TIMEOUT;
            User_StartClock(&BoardCloseTimeoutclock);
        }
    }
    printf("openNB\r\n");
}

/**
* @brief  Device_Open_GPS
* @param  None
* @retval None
*/
void Device_Open_GPS(void)
{   
    memset(&nb_gps_data, 0, sizeof(GPS_TO_NB_Data_t));
    //memset(UsartType.RX_pData, 0, RX_LEN);
    
    //UART1_Rx_Enable();
    Open_GPS();
    gps_open_Flag = SET;
    gps_serch_counter = 0;
    GPSSerchTimerStartFlag = SET;
    User_StartClock(&GPSRMCclock);
    User_StartClock(&GPSTimeOutclock);
    User_StopClock(&BoardCloseTimeoutclock);
    
    GPSRMCCounter = RESET;
    eff_satCounter = 0;     
    printf("openGPS\r\n");
}
/**
* @brief  Device_Close_GPS
* @param  None
* @retval None
*/
void Device_Close_GPS(void)
{
    UART1_Rx_Disable();
    
    Close_GPS();
    //huart1_to_IOEXIT();
    gps_open_Flag = RESET;
    //gps_serch_counter = 0;
    
    memset(UsartType.RX_pData, 0, RX_LEN);    
    printf("closeGPS\r\n");
}
/**
* @brief  ScheduleModeProcess
* @param  None
* @retval None
*/
void ScheduleModeProcess(void)
{
    switchToToday_t switchToTodayFlag = {0};
    //判断23:59:40s天数++，12点更新同步
    switchToTodayFlag = switchToToday(Schedule_CMD_State.configMode);
    
    //    if(Time_synFlag == SET)
    //    {
    //        if(switchToTodayFlag != 0xFF)
    //        {
    //            CurrentDayInTheLoop = switchToTodayFlag;
    //        }
    
    //修改为12点同步时间戳！！
    if(switchToTodayFlag.TimeSynFlag == SET)
    {
        Time_synFlag = RESET;
        //CurrentDayInTheLoop = switchToTodayFlag.CurrentDayInTheLoop;
        
//        //同步时间戳
//        if(Time_syning_Flag == RESET)
//        {
//            Time_syning_Flag = SET;
//            currentMode = NORMAL_MODE;
//            
//            //if(Time_syning_Flag == SET)
//            //{
//            //printf("@@@\r\n");
//            //User_StopClock(&BoardCloseTimeoutclock);
//            //            if(gps_open_Flag == SET)
//            //            {
//            //                User_StopClock(&GPSTimeOutclock);
//            //                Device_Close_GPS();
//            //            }
//            //            if(NB_open_Flag == SET)
//            //            {
//            //                User_StopClock(&ATTimeoutclock);
//            //                User_StopClock(&InNetTimeoutclock);
//            //                NB_open_Flag = RESET;
//            //                Close_NB();
//            //            }
//            //}
//            
//            //唤醒
//            wakeupCounter = 0;
//            Locator_Data.userAlarmFlag = RESET;
//            NBTimeOutFlag = SET;
//            SleepStatus = AWAKE;
//        } 
    }
    if(switchToTodayFlag.DaySwitchFlag == SET)
    {
        //if(cycleStartFlag == SET)
        //{
        //    //配置schedule参数
            //setRTC_Date_Time(18, 5, 3, 23, 53, 40, 4);
            //CurrentDayInTheLoop = switchToTodayFlag.CurrentDayInTheLoop;
            ScheduleParmSet(CurrentDayInTheLoop);
            wakeupCounter = 0;
            SleepStatus = SLEEP;
        //}
    }
}

/**
* @brief  DataRptCb
* @param  None
* @retval None
*/
void DataRptCb(void)
{
    DataReport();
}

/**
* @brief  NB_RESETCb
* @param  None
* @retval None
*/
void NB_RESETCb(void)
{
    //HAL_GPIO_WritePin(GPIOB, NB_RESET_Pin, GPIO_PIN_RESET);
    //NB_NetWork_Init();
    
    AT_Retry_Num = 0;
    Retry_state = RESET;    
    //1.关闭射频功能
    NB_AT_CMD_Send(AT_CFUN0);
    //开启超时定时器，超时重发，重发次数到后 返回失败。       
    User_StartClock(&ATTimeoutclock);
    curAT_RSP_ID = AT_CFUN0;
    User_StartClock(&InNetTimeoutclock);
}


/**
* @brief  DataReport
* @param  None
* @retval None
*/
void DataReport(void)
{
    NB_open_Flag = SET;
    if(RegistFlag == SET || RegistStateFlag == SET)
    {
        curAT_RSP_ID = AT_DATA_RSP;
        
        //时间到，未收到正确的回复，重发
        if(Retry_state == RESET)
        {
            //超时重发
            if(AT_Retry_Num < 5)
            {
                AT_Retry_Num++;
                
                //if(Time_synFlag == SET)
                //{
                if(move_alarm_Flag == SET || move_continue_Flag == SET)
                {
                    curAT_RSP_ID = AT_DATA_RSP;
                    NB_Report_to_ServerData.cmd = SERVER_ID_DATA_REPOET;
                    NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
                    printf("MOVE_DATA_UP: %d", NB_Report_to_ServerData.cmdlen);
                    NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
                    
                }
                else if(Schedule_ModeFlag == SET)
                {
                    curAT_RSP_ID = AT_DATA_RSP;
                    if(Locator_Data.userAlarmFlag == SET)
                    {
                        Schedule_Data_upload.scheduleID = SET;
                    }
                    
                    Schedule_Data_upload.Mode = currentMode;
                    NB_Report_to_ServerData.cmd = SCHEDULE_UPLOAD_ID;
                    NB_Report_to_ServerData.cmdlen = sizeof(NB_Report_to_ServerData_t)+sizeof(Schedule_Data_upload_t)+ Schedule_Data_upload.GPS_List_num * sizeof(GPS_List_t) + CHECK_SUM_LEN + END_NUM_LEN;
                    Schedule_Data_Report(NB_Report_to_ServerData, Schedule_Data_upload, GPS_Info_schedule);
                    printf("Schedule_DATA_UP2: %d", NB_Report_to_ServerData.cmdlen);                        
                }
                //}
                //else
                //{
                else if(Schedule_ModeFlag == RESET)
                {
                    curAT_RSP_ID = AT_DATA_RSP;
                    //NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
                    printf("NB_DATA_UP2: %d", NB_Report_to_ServerData.cmdlen);
                    NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
                }                 
                //}
                User_StartClock(&DataRptclock);
            }
            //重发次数到
            else
            {
                Retry_state = RESET;
                AT_Retry_Num = 0;
                
                //恢复为原模式
                if(ExtraDataupFlag == SET)
                {
                    ExtraDataupFlag = RESET;
                    if(Schedule_ModeFlag == RESET)
                    {
                        currentMode = ExtraDataupPreMode;
                        Locator_Data.mode = currentMode;
                    }
                }
            }
        }
    }
}

/**
* @brief  SleepIO_Config
* @param  None
* @retval None
*/
void SleepIO_Config(void)
{
    
    /**LPUART1 GPIO Configuration    
    PB11     ------> LPUART1_RX
    PB10     ------> LPUART1_TX 
    */
    GPIO_InitTypeDef GPIO_InitStruct; 
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, NB_RI_Pin|GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7|GPIO_PIN_6, GPIO_PIN_SET);
    
//    GPIO_InitStruct.Pin = NB_RI_Pin|GPIO_PIN_11|GPIO_PIN_10;
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
       
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
       
    GPIO_InitStruct.Pin = NB_RI_Pin|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
}

void TSETCb(void)
{
    //User_StartClock(&Testclock);
    NVIC_SystemReset();
    //printf("T1\r\n");
}


//使能串口接收
void UART2_Rx_Enable(void)
{
    HAL_UART_Receive_DMA(&huart2, UsartType2.RX_pData, RX_LEN);  
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    //NVIC_SystemReset();
    //printf("Decice_ERROR");
    /* User can add his own implementation to report the HAL error return state */
    while(1) 
    {
    }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
