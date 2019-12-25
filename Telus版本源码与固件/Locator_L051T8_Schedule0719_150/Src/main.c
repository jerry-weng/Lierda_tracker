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
#include "stm32l0xx_it.h"
#include "tools.h"
#include "gps.h"
#include "nb.h"
#include "schedule.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//��ʱ���¼�
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
      

//��ʱʱ��
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
      
//ģʽ�л�LED��˸����*2
#define LED_BLINK_NUM               8

//����
#define CHARGE_COMPIETE_BATTERY_LV  100      
#define BATTERY_LOW_LV              20
#define BATTERY_VERY_LOW_LV         5

//��ѹ������ֵ
#define R_GND                       100
#define R_VCC                       68//33

#define ENTER_CONFIG_MODE_KEYNUM    5
#define REGIST_RETRYNUM_MAX         5
//ATָ�� ����ط�����
#define AT_RETRY_NUM_MAX            60      

#define KEY_PRESS_EVENT             0x0001
#define KEY_RELEASE_EVENT           0x0002
#define TIM2_TIMEOUT_EVENT          0x0004
#define TIM21_TIMEOUT_EVENT         0x0008
#define TIM22_TIMEOUT_EVENT         0x0010



#define ADC_20S_NUM                 90//30min

//��������
typedef struct
{
    uint8_t keyConut;
    uint8_t keyPressFlag;
    uint8_t keyLongPressFlag;    
} KeyAction_t;
KeyAction_t KeyAction = {0};

//����
uint32_t event;

//��ʱ��������־λ
uint8_t KeyPressTimerStartFlag = RESET;
uint8_t AntiShakeTimerStartFlag = RESET;
uint8_t SleepTimerStartFlag = RESET;
uint8_t GPSSerchTimerStartFlag = RESET;
uint8_t EvenPressTimerStartFlag = RESET;

uint8_t GPSKeepTimerStartFlag = RESET;
uint8_t GPSKeepTimeoutFlag = RESET;
//�������
uint8_t EvenPressFlag = RESET;
//��ʱ��
Clock_Struct KeyPressclock;
Clock_Struct LEDBlinkclock;
Clock_Struct ADCConverclock;
Clock_Struct OneTimeoutclock;
//Clock_Struct DeviceRegistclock;
Clock_Struct ModeDisplayclock;
Clock_Struct APPUnbindclock;
//�ж�����
Clock_Struct EvenPressclock;
//GPS���ֶ�ʱ��
Clock_Struct GPSKeepclock;
//GPS��ʾ��ʽת��
Clock_Struct GPSRMCclock;

//NB��λ
Clock_Struct NBResetclock;

Clock_Struct Testclock;

//GPS�����ָ��
GPS_TO_NB_Data_t* GPS_TO_NB_Data = NULL;

static uint32_t TimeOutInit = 0xFFFF;
//��ʼ����˸5��
static uint8_t LED_Init_Blink_Count = 0;
//��˸
static uint8_t LED_Blink_Count = 0;
//�����Ѳ����־
static uint8_t chargeEnable = RESET;
//�����ɱ�־
uint8_t chargeFinish = RESET;

//����
//uint32_t event = 0;
//�Ƿ񿪻�
uint8_t BoardStart = RESET;
//����ע�ᶯ��
uint8_t RegistActionFlag = RESET;
//GPS�ɼ����ڵ�
uint8_t GPSTimeOutFlag = RESET;
//NB�ϱ����ڵ�
uint8_t NBTimeOutFlag = RESET;

uint8_t NB_ERROR_counter = 0;

//��������NBģ��
uint8_t NB_NetWork_First_Init = SET;

uint8_t NSORF_counter = 0;
uint8_t Schedule_Mode_GPSFlag = RESET;

//ע���������
uint8_t RegistCount = 0;
//ģʽ��ʾ����
uint8_t ModeDisplayCount = 0;

//ģʽ�л���־
uint8_t ModeSwitchFlag = RESET;

//�����ϱ����
//uint8_t ExtraDataupFlag = RESET;
uint8_t ExtraDataupLEDFlag = RESET;
uint8_t ExtraDataupTimerFlag = RESET;
uint8_t ExtraDataupPreMode; 

uint8_t adc_timeoutFlag = SET;
//��ǰ��ص���
uint32_t Bat_Value;
//ǰһ�βɼ��ĵ�ص���
uint8_t Pre_Bat_Level = 100;

//��ص���
uint8_t Bat_Level = 100;

uint16_t regist_counter = 0;
uint16_t adc_counter = 0;
//��ʱ��-Ӧ�ò�ת��Ϊ����
uint16_t GPS_min_counter = 0;
uint16_t gps_serch_counter = 0;
uint16_t sleep_counter = 0;    
uint8_t curAT_RSP_ID_t;

//ADC
uint32_t AD_Value = 0;
//GPS���ֶ�ʱ��
uint32_t GPSKeepCounter = 0;
//ת��Ϊֻ��ʾRMC
//static char ConfigRMC[28] = "$PCAS03,0,0,0,0,1,0,0,0*03\r\n";
uint8_t GPSRMCCounter = RESET;

uint8_t InitFlag = RESET;
uint8_t InitFlag_c = RESET;

uint8_t InNetCounter = 0;
uint8_t BoardSleepCounter = 0;
uint8_t RegistStateCounter = 0;

//�������޷�������־
uint8_t Bat_OK_Flag = RESET;

//�ر�edrx
static char CloseEdrx[28] = "AT+NPTWEDRXS=0,5,0111,0010\r\n";
//����PSM
static char OpenPsm[32] = "AT+CPSMS=1,,,00000001,00000001\r\n";

//���С��Ƶ��
static char ClearFreq[14] = "AT+NCSEARFCN\r\n";
//����С����ѡ
static char CellReselection[35] = "AT+NCONFIG=CELL_RESELECTION, TRUE\r\n";
//cops
static char Cops[11] = "AT+COPS=0\r\n";
//Edrx
static char EdrxCMD[22] = "AT+CEDRXS=2,5,\"0011\"\r\n";

//Close lwm2m
static char Close_LWM2M[19] = "AT+MLWM2MENABLE=0\r\n";
//Close LCTREG
static char Close_LCTREG[13] = "AT+LCTREG=5\r\n";
//Band 5
static char SetBand5[12] = "AT+NBAND=5\r\n";



uint8_t cmeErrorCounter = 0;
uint8_t InnetTimeOutFlag = RESET;

uint8_t  PT_flag[1];

uint32_t Systime;

//�����ã�����
uint8_t  testCounter  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//��ʼ��
void UserDataInit(void);
void UserTimerInit(void);
void NB_NetWork_Init(void);
//��ʱ���ص�
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
//���������ݴ���
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
//�������
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
//�͹���
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

#warning �����־λ  
   EEPROM_ReadBytes(0, PT_flag, 1);
   printf("PT_flag, %d\r\n", PT_flag[0]);
   
#warning �����ã����γ������!!   
   PT_flag[0] = SET;
  //User_StartClock(&Testclock);
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
        
        //�͵���������
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
            //����״̬����
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
                //LED��ʾ���������� NB��ʱ�ϱ���GPS��ʱ�ϱ���
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
//��ʼ������
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
//�û���ʱ����ʼ��
void UserTimerInit(void)
{
    MX_TIM2_Init(TimeOutInit);
    //��������ʱ��
    KeyPressclock.eventID = KEYPRESS_EVENT;
    KeyPressclock.timeOut = KEYPRESS_TIMEOUT;
    KeyPressclock.TaskHook = KeyPressTimerCb;
    //LEDʱ��
    LEDclock.eventID = LED_EVENT;
    LEDclock.timeOut = MODE_SWICTH_LED_TIMEOUT;
    LEDclock.TaskHook = LEDCb;
    //LED��˸
    LEDBlinkclock.eventID = LED_BLINK_EVENT;
    LEDBlinkclock.timeOut = LEDBLINK_TIMEOUT;
    LEDBlinkclock.TaskHook = LEDBlinkCb;
    //ADC����ʱ��
    ADCConverclock.eventID = ADC_CONV_EVENT;
    ADCConverclock.timeOut = ADC_CONVERT_TIMEOUT;
    ADCConverclock.TaskHook = ADCTimerCb;
    //���������ݻظ�ʱ��
    DataRspclock.eventID = DATA_RSP_EVENT;
    DataRspclock.timeOut = DATA_RSP_TIMEOUT; 
    DataRspclock.TaskHook = DataRspTimerCb;
    
    //GPS����ʱ��
    GPSTimeOutclock.eventID = GPS_TIMEOUT_EVENT;
    GPSTimeOutclock.timeOut = GPS_SERCH_TIMEOUT_DEFAULT;
    GPSTimeOutclock.TaskHook = GPS_SerchTimeoutCb;
    
    //ATָ��ͳ�ʱʱ��
    ATTimeoutclock.eventID = AT_CMD_TIMEOUT_EVENT;
    ATTimeoutclock.timeOut = AT_CMD_CFUN0_TIMEOUT;
    ATTimeoutclock.TaskHook = ATTimeoutCb;
    
    //����ʱ��
    OneTimeoutclock.eventID = ONE_TIMEOUT_EVENT;
    OneTimeoutclock.timeOut = ONE_TIMEOUT;
    OneTimeoutclock.TaskHook = OneTimeoutCb;
    
    //����
    BoardCloseTimeoutclock.eventID = BOARD_CLOSE_EVENT;
    BoardCloseTimeoutclock.timeOut = AT_BOARD_CLOSE_TIMEOUT;
    BoardCloseTimeoutclock.TaskHook = BoardCloseCb;
    
    //ע�������ط���ʱ��
    DeviceRegistclock.eventID = DEVICE_REGIST_EVENT;
    DeviceRegistclock.timeOut = DEVICE_REGIST_TIMEOUT;   
    DeviceRegistclock.TaskHook = DeviceRegistCb;
    
    //��һ�°�����һ�ε���ʾģʽ
    ModeDisplayclock.eventID = MODE_DISPLAY_EVENT;
    ModeDisplayclock.timeOut = MODE_DISPLAY_TIMEOUT;
    ModeDisplayclock.TaskHook = ModeDisplayCb;
    
    //������ʱ
    InNetTimeoutclock.eventID = IN_NET_EVENT;
    InNetTimeoutclock.timeOut = IN_NET_TIMEOUT;
    InNetTimeoutclock.TaskHook = InNetCb;
    
    //�ػ���ʱ
    APPUnbindclock.eventID = UNBIND_EVENT;
    APPUnbindclock.timeOut = UNBIND_TIMEOUT;
    APPUnbindclock.TaskHook = UnbindCb;
    
    EvenPressclock.eventID = EVEN_PRESS_EVENT;
    EvenPressclock.timeOut = EVEN_PRESS_TIMEOUT;
    EvenPressclock.TaskHook = EvenPressCb;
    
    //GPS����
    GPSKeepclock.eventID = GPS_KEEP_EVENT;
    GPSKeepclock.timeOut = GPS_KEEP_TINEOUT_20S;
    GPSKeepclock.TaskHook = GPSKeepCb;

    //RMC��ʽ�л�
    GPSRMCclock.eventID = GPS_RMC_FORMAT_EVENT;
    GPSRMCclock.timeOut = GPS_RMC_FORMAT_TIMEOUT;
    GPSRMCclock.TaskHook = GPSRMCCb;

    //�����ط�
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
////���ǳ�ʱ��ʱ��
//void GPSSerchTimerInit(uint8_t timeout)
//{
//    GPSTimeOutclock.eventID = GPS_TIMEOUT_EVENT;
//    GPSTimeOutclock.timeOut = GPS_SERCH_TIMEOUT_DEFAULT;
//    GPSTimeOutclock.TaskHook = GPS_SerchTimeoutCb;
//}
//IO�ж�
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#warning ��һ������������
    HandelKeyInterrupt(GPIO_Pin);
    /* NOTE: This function Should not be modified, when the callback is needed,
    the HAL_GPIO_EXTI_Callback could be implemented in the user file
    */ 
}
//ʱ��
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    //ʱ�ӻ���
    //SleepStatus = AWAKE;
    if (htim->Instance == htim2.Instance)
    {
        event = TIM2_TIMEOUT_EVENT;
    }
    if (htim->Instance == htim21.Instance)
    {
        event = TIM21_TIMEOUT_EVENT;
    }
    if (htim->Instance == htim22.Instance)
    {
        event = TIM22_TIMEOUT_EVENT;
    }
}

//��������
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

//����STOP
void StopMode_Measure(void)
{
    /* Enter Stop Mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    
    /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
    PLL as system clock source (HSE and PLL are disabled in STOP mode) */
    SystemClockConfig_STOP();
}
//��������
void KeyPressTimerCb(void)
{
    //�жϳ�����̰�
    if(KeyAction.keyLongPressFlag == RESET)
    {
        if(BoardStart == RESET)
        {
            //����δ����״̬���̰�������Ч�����½�������
            //printf("sleep\r\n"); 
            if(PT_flag[0] == SET)
            {
                SleepStatus = SLEEP;
            }
            else if(KeyAction.keyConut < 5)
            {
                //δ��������̰��������㣬����
                SleepStatus = SLEEP;
            }
        }
        else
        {
//            //ע��ɹ��󰴼���ʾģʽ
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
#warning �ڶ�������������󿪻�
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
            //�����󣬿���LED��ʱ��
            if(PT_flag[0] == SET)
            {
                User_StartClock(&LEDclock);
            }
        }
#warning ���0105        
        else if(BoardStart == SET)
        {
            //�����󳤰��������ϱ�һ��
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
                //�����ϱ����
                ExtraDataupFlag = SET;
                ExtraDataupLEDFlag = SET;
                ExtraDataupTimerFlag = SET;
                
                Locator_Data.userAlarmFlag = SET;
#warning   ע��˴�ģʽ�ֶ�
                ExtraDataupPreMode = currentMode;
                currentMode = POSITIONING_MODE;
                Locator_Data.mode = POSITIONING_MODE;
                LEDclock.timeOut = 100;
                User_StartClock(&LEDclock);
            }
            
        }        
    }
    //��ռ���
    KeyAction.keyConut = 0;
    //printf("KeyTimeOut\r\n");
    KeyPressTimerStartFlag = RESET;
    //�����󰴼��������ж��Ƿ���Ҫ��������
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

//LED��ʱ���ص�
void LEDCb(void)
{
    Handel_LED();
}
/*
* @fn      LEDBlinkCb
* @brief   LED��˸��ʱ��
* @param   
* @return   
*/
void LEDBlinkCb(void)
{
    closeAllLed();
    if(LED_Blink_Count < 2)
    {
        LED_Blink_Count++;
        //δע���
        if(RegistFlag != SET)
        {
            //δ�����������
            if(NB_InNetWork_Flag == RESET)
            {
                HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
            }
            //���������̵�
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
//ADC�ɼ�
void ADCTimerCb(void)
{

}

void DataRspTimerCb(void)
{
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
                SleepStatus = SLEEP;
            }
        }
        else
        {
            User_StartClock(&DataRspclock);
        }
    }
    else
    {
        if(NB_min_counter == 0)
        {
            printf("confirm\r\n");
            curAT_RSP_ID = AT_DATA_RSP;
            NB_AT_CMD_Send(AT_NSOST, LOCATOR_CMD_CONFIRM);
            DataRspclock.timeOut = DATA_RSP_TIMEOUT_MAX;
            User_StartClock(&DataRspclock);
            data_confirmFlag = SET;
            //NB_min_counter = 1;
        }
        else if(NB_min_counter == 1)
        {
            printf("State\r\n");
            if(Locator_pram.enterSleep == SET)
            {
                Locator_pram.enterSleep = RESET;
#warning ���0104
                if(RegistFlag == SET)
                {
                    User_StartClock(&APPUnbindclock);
                    return;
                }
                else
                {
                    NVIC_SystemReset();
                }
            }
#warning �޸�01223        
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
            if(NB_StateSend_counter > 4)
            {
                NB_StateSend_counter = 0;
                if(RegistFlag == RESET)
                {
                    RegistStateCounter++;
                    if(RegistStateCounter > 4)
                    {
                        RegistStateCounter = 0;
                        NVIC_SystemReset();
                    }
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
                    SleepStatus = SLEEP;
                }
            }
            else
            {
                User_StartClock(&DataRspclock);
            }
        }
    }
}

//GPS��Ϣ����
void GPSInfoProcess(uint8_t *gpsRxMessage)
{   
    //��������Ϣ�����Ҫ��NB�ϱ�����Ϣ�ֶ�
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
                RegistFlag = SET;
                GPS_TO_NB_Data->State = INFO_STATE_UNAVILIABLE;
                //GPSKeepTimerStartFlag = SET;
                printf("PT_GPS_OK2\r\n");
                PT_flag[0] = SET;
                EEPROM_WriteBytes(0, PT_flag, 1);
                
                User_StopClock(&LEDclock);
                User_StopClock(&LEDBlinkclock);
                User_StopClock(&GPSTimeOutclock);
                UserStopLedClock();
                
                HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
                User_StartClock(&Testclock);
                return;
            }
        }
    }
    printf("GPSSTATE: %d", GPS_TO_NB_Data->State);

    //������ǳɹ�
    if(GPS_TO_NB_Data->State == INFO_STATE_AVILIABLE)
    {
        //HAL_UART_Transmit_DMA(&huart2, "GPS_OK", 6);
        //�˴����ǳɹ���ֹͣ���ǳ�ʱ��ʱ 
        //eff_satCounter = 0;
        if(GPSSerchTimerStartFlag == SET)
        {
            User_StopClock(&GPSTimeOutclock);
            GPSSerchTimerStartFlag = RESET;
        }
#warning ���171229        
        /*
        ���ǳɹ���
        ������Ǽ��2Сʱ(120����) ��
        ÿ��2Сʱ����һ�ζ���ά��20�����ǣ��Ա�֤�������ٶ�λ
        ��Ȼ����ά��3�룬��֤�����ȶ�
        */
        if(GPSKeepTimerStartFlag == RESET)
        {
            GPSKeepTimerStartFlag = SET;
            if(Schedule_ModeFlag == RESET)
            {
                if(Locator_Data.TrackerCycle <= GPS_KEEP_MIN )
                {
                    //����������·�ʱ�����л�������2Сʱʱ�䵽
                    if(GPSKeepTimeoutFlag == SET || TrackerCycle_short_Flag == SET)
                    {
                        //��־λ���󱣳�20s
                        TrackerCycle_short_Flag = RESET;
                        GPSKeepclock.timeOut = GPS_KEEP_TINEOUT_20S;
                    }
                    else
                    {
                        //��Ȼ����6s
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

//GPS���ǳ�ʱ
void GPS_SerchTimeoutCb(void)
{
    uint8_t gps_counter = 0;
    if(GPSKeepTimeoutFlag == SET || TrackerCycle_short_Flag == SET)
    {
        //5����
        gps_counter = 6;
        //gps_counter = 3;
        
        //GPSKeepTimeoutFlag = RESET;
        //TrackerCycle_short_Flag = RESET;
    }
    else
    {
        //1�ְ�
        gps_counter = 3;
        //gps_counter = 1;
        
        
    }
    gps_serch_counter++;
    if(gps_serch_counter == 2)
    {
        //�����ڻ����ź���
        if(eff_satCounter <1)
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
            //���������ʧ������
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

//GPS��ʱ
void GPSCollectionTimerCb(void)
{
    
}

void OneTimeoutCb(void)
{
    //��ʱ���ؿ�����
    LPUART1_Rx_Enable();
}

void BoardCloseCb(void)
{
    BoardSleep();
}

//ע�������ط���ʱ
void DeviceRegistCb(void)
{
    if(RegistFlag == RESET)
    {
        regist_counter++;
        //1min ��ʱ
        if(regist_counter < 12)
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
//�����жϴ���
void HandelKeyInterrupt(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == KEY_Pin)
    {   
        //printf("K:%d", Bat_OK_Flag);
        if(InitFlag == SET)
        {
            //if(NB_open_Flag == RESET && gps_open_Flag == RESET)
            //{
            //��ȡIO��ƽ,Ϊ�ͱ�ʾ����
            if( HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
            {
                event = KEY_PRESS_EVENT;
                //printf("keyPress!\r\n");
                SleepStatus = AWAKE;
            }
            else if( HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
            {
                event = KEY_RELEASE_EVENT;
                //printf("keyRelease!\r\n");
            }
        }
    //}
    }
    //���ָʾ
    if(GPIO_Pin == NCHG_Pin )
    {
        //SleepStatus = AWAKE;
        //closeAllLed();
        //User_StopClock(&LEDclock);
        //��-���ڳ��
        if(HAL_GPIO_ReadPin(NCHG_GPIO_Port, NCHG_Pin) == GPIO_PIN_RESET)
        {
            chargeEnable = SET;
            //�����
            HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
        }
        else if(HAL_GPIO_ReadPin(NCHG_GPIO_Port, NCHG_Pin) == GPIO_PIN_SET)
        {
            chargeEnable = RESET;
            HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_SET);
//            if(chargeEnable == SET)
//            {
//                chargeFinish = SET;
//                //������
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
    //�ⲿ��Դ����ָʾ
    if(GPIO_Pin == NPPR_Pin)
    {
//        //SleepStatus = AWAKE;
//        //�ͱ�ʾ�ⲿ�е�Դ����
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
//            //ת���ɴ��ڹ��ܣ�����
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
    
}
/*
* @fn      Handel_LED
* @brief   LED��ʾ
* @param   
* @return
*/
void Handel_LED(void)
{
    if(LED_Init_Blink_Count <= LED_BLINK_NUM && LED_Init_Blink_Finish == RESET)
    {
#warning ���������̵ƿ���5�� 
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
* @brief   ��������ʱ���豸״̬
* @param   
* @return   
*/
void State_Init(void)
{
    closeAllLed();
    if(HAL_GPIO_ReadPin(NCHG_GPIO_Port, NCHG_Pin) == GPIO_PIN_RESET)
    {
        chargeEnable = SET;
        //�����
        HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
    }
    else if(chargeEnable == SET && chargeFinish == SET)
    {
        //������
        HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
    }
    
    //memset(UsartType.RX_pData, 0, RX_LEN);            
    //��λ--����
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
    printf("Rev01_180811\r\n");
    
    
    curAT_RSP_ID = AT_INIT_SUCCESS;
    SleepStatus = SLEEP;
    DataUpFlag = RESET;
    NB_InNetWork_Flag = RESET;
    NBTimeOutFlag = RESET;
    NB_open_Flag = RESET;
    KeyAction.keyConut = 0;
    KeyAction.keyPressFlag = RESET;
    EvenPressTimerStartFlag = RESET;
    Time_syning_Flag = RESET;
    Schedule_State = RESET;
    cycleStartFlag = RESET;
    data_confirmFlag = RESET;
    D4Flag = RESET;
    ExtraDataupFlag = RESET;
    packageStartFlag = RESET;
    
    Schedule_StateSend_counter = 0;
    NB_StateSend_counter = 0;
    NB_ERROR_counter = 0;
    cmeErrorCounter = 0;
    NB_min_counter = 0;
    NB_StateSend_counter = 0;
    InNetCounter = 0;   
}

/*
* @fn      LEDProcess
* @brief   ����LED״̬
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

        //��ص�����
        //if(Bat_Level < BATTERY_LOW_LV)
        //{
        //    LEDclock.timeOut = BATTERY_LV_LOW_LED_TIMEOUT;
        //    UserStartLedClock();
        //    User_StartClock(&LEDBlinkclock);
        //}
        //else
        //{
            //δ��ʼ��ע���
            if(RegistFlag == RESET)
            {
                LEDclock.timeOut = SERCHING_NET_TIMEOUT;
                UserStartLedClock();
                User_StartClock(&LEDBlinkclock);
            }
            else
            {
                //�����ϱ�
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
* @brief   �ر� ����LED
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
    //�����ɹ�
    if(NB_InNetWork_Flag == SET)
    {
        //NB_InNetWork_Flag = RESET;
        //��ע��
        if(RegistFlag == SET)
        {
#warning �����������ڵ����ϱ�����   
            if(DataUpFlag == RESET)
            {
                DataUpFlag = SET;
                if(Time_synFlag == SET)
                {
                    if(Schedule_ModeFlag == SET)
                    {
                        curAT_RSP_ID = AT_DATA_RSP;
                        
                        //cycleStartFlag = SET;
                        //ScheduleModeProcess();
                        
                        //ScheduleParmSet(CurrentDayInTheLoop); 
                        
                        if(Locator_Data.userAlarmFlag == SET)
                        {
                            Schedule_Data_upload.Mode = 0x02;
                        }
                        else
                        {
                            Schedule_Data_upload.Mode = currentMode;
                        }
                        NB_Report_to_ServerData.cmd = SCHEDULE_UPLOAD_ID;
                        NB_Report_to_ServerData.cmdlen = sizeof(NB_Report_to_ServerData_t)+sizeof(Schedule_Data_upload_t)+ Schedule_Data_upload.GPS_List_num * sizeof(GPS_List_t) + CHECK_SUM_LEN + END_NUM_LEN;
                        Schedule_Data_Report(NB_Report_to_ServerData, Schedule_Data_upload, GPS_Info_schedule);
                        printf("Schedule_DATA_UP: %d", NB_Report_to_ServerData.cmdlen);
                        
                        ScheduleParmSet(CurrentDayInTheLoop);
                        if(Schedule_Mode_Data.config_cycle == 0)
                        {
                            Schedule_State = SET;
                        }
                        //testFlag = SET;
                        Retry_state = RESET;
                        AT_Retry_Num = 0;
                        User_StartClock(&DataRptclock);
                    }
                }
                else
                {
                    if(Schedule_ModeFlag == RESET)
                    {
                        curAT_RSP_ID = AT_DATA_RSP;
                        NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
                        printf("NB_DATA_UP: %d", NB_Report_to_ServerData.cmdlen);
                        NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
                        
                        Retry_state = RESET;
                        AT_Retry_Num = 0;
                        User_StartClock(&DataRptclock);
                    }
                    else
                    {
                        //ͬ��ʱ���
                        printf("D5\r\n");
                        curAT_RSP_ID = AT_DATA_RSP;
                        Schedule_Device_Confirm_Report(TIME_SYN_REQ_ID, 0, Schedule_CMD_State.packetNum, 0xFF, 0);
                    }                    
                }
                
//                //��
//                Retry_state = RESET;
//                AT_Retry_Num = 0;
//                User_StartClock(&DataRptclock); 
                
                //User_StartClock(&BoardCloseTimeoutclock);
                
            }
        }
    }
}

/*
����ATָ��   --->  �ȴ����ڻظ�
���ڻظ���ʱ --->  �ط�
�ط�������   --->  ʧ��
*/

//ATָ�� ���ͳ�ʱ
void ATTimeoutCb(void)
{
    //ʱ�䵽��δ�յ���ȷ�Ļظ����ط�
    if(Retry_state != SET)
    {
        //��ʱ�ط�
        if(AT_Retry_Num < AT_RETRY_NUM_MAX-1)
        {
            AT_Retry_Num++;
            //��
            if(curAT_RSP_ID == AT_CFUN0)
            {   
                //�ط�
                NB_AT_CMD_Send(AT_CFUN0);
                ATTimeoutclock.timeOut = AT_CMD_CFUN0_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
                printf("AT_CFUN0");
            }
            //��
            else if(curAT_RSP_ID == AT_CFUN1)
            {
                //�ط�
                NB_AT_CMD_Send(AT_CFUN1);
                ATTimeoutclock.timeOut = AT_CMD_CFUN1_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
                printf("AT_CFUN1");
            }
            //��IP
            else if(curAT_RSP_ID == AT_CGPADDR)
            {
                //�ط�
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
                //�ط�
                NB_AT_CMD_Send(AT_NRB);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
                printf("NRB");
            }
            else if(curAT_RSP_ID == AT_CGATT)
            {
                NB_AT_CMD_Send(AT_CGATT);
                //ATTimeoutclock.timeOut = AT_CMD_CGDCONT_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            else if(curAT_RSP_ID == AT_CGDCONT)
            {
                NB_AT_CMD_Send(AT_CGDCONT);
                //ATTimeoutclock.timeOut = AT_CMD_CGDCONT_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //����Socket
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
            else if(curAT_RSP_ID == AT_DATA_RSP)
            {
                if(D4Flag == SET)
                {
                    printf("D4flag\r\n");
                    Schedule_Device_Confirm_Report(SCHEDULE_CONFIG_ID, 1, Schedule_CMD_State.packetNum,0xFF, 0);
                    ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                    //User_StartClock(&ATTimeoutclock);
                }
            }
			//С����ѡ
            else if(curAT_RSP_ID == AT_CELL_RESELECT)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)CellReselection, 35);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //���Ƶ��
            else if(curAT_RSP_ID == AT_CLEAR_FREQ)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)ClearFreq, 14); 
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //cops
            else if(curAT_RSP_ID == AT_COPS)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)Cops, 11); 
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            //CMEE
            else if(curAT_RSP_ID == AT_NNMI)
            {
                NB_AT_CMD_Send(AT_CMEE);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            else if(curAT_RSP_ID == AT_CIMI)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, "AT+CIMI\r\n", 9);
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            else if(curAT_RSP_ID == AT_EDRX)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)EdrxCMD, 22); 
                //ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
                //User_StartClock(&ATTimeoutclock);
            }
            else if(curAT_RSP_ID == AT_CLOSE_LWM2M)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)Close_LWM2M, 19);
            }
            else if(curAT_RSP_ID == AT_CLOSE_LCTREG)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)Close_LCTREG, 13);
            }
            else if(curAT_RSP_ID == AT_BAND5)
            {
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)SetBand5, 12);
            }
            User_StartClock(&ATTimeoutclock);
        }
        //�ط�������
        else
        {
            Retry_state = RESET;
            AT_Retry_Num = 0;
        }
    }
}

/*
* @fn      NB_NetWork_Init
* @brief   ��ʼ��NBģ�飬��������  
* @param   
* @return   
*/
void NB_NetWork_Init(void)
{      
    //NB_NetWork_First_Init�ֶ���ҪΪ�˻�ȡIMEI����NB�ͺ�
    if(NB_NetWork_First_Init == SET)
    {
        AT_Retry_Num = 0;
        Retry_state = RESET;    
        //1.�ر���Ƶ����
        NB_AT_CMD_Send(AT_CFUN0);
        //������ʱ��ʱ������ʱ�ط����ط��������� ����ʧ�ܡ�       
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
* @brief   ����NBģ��ظ�  
* @param   ��ǰ �ȴ��ظ� ��ָ��ID
* @return   
*/
void Handel_NB_AT_RSP(uint8_t *atMessage)
{
    /*
    �ȴ����ݻظ���Ҫ��ʱ�ط�����
    */
    //�����ر�Э��ջ���ط�
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
                //�رճ�ʱ��ʱ��
                User_StopClock(&ATTimeoutclock);
                
                printf("Clear_Freq\r\n");
                //���Ƶ��
                curAT_RSP_ID = AT_CLEAR_FREQ;
                HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)ClearFreq, 14);
                
                Retry_state = RESET;
                AT_Retry_Num = 0;
                ATTimeoutclock.timeOut = 3000;
                User_StartClock(&ATTimeoutclock);
                
                //��һ����ѯ�汾��               
                //NB_AT_CMD_Send(AT_CGMR);
                //curAT_RSP_ID = AT_CGMR;
            }
            else if(curAT_RSP_ID == AT_CFUN1)
            {
                //�رճ�ʱ��ʱ��
                User_StopClock(&ATTimeoutclock);
                //��һ������������ʾ
                //curAT_RSP_ID = AT_COPS;
                //HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)Cops, 11);
                //User_StartClock(&ATTimeoutclock);
                NB_AT_CMD_Send(AT_CMEE);
                curAT_RSP_ID = AT_CMEE;
                User_StartClock(&ATTimeoutclock);
            }            
        }
    }
    else if(curAT_RSP_ID == AT_COPS)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            AT_Retry_Num = 0;
            User_StopClock(&ATTimeoutclock);
            printf("AT_COPS_ok\r\n");
            //��һ������������ʾ
            NB_AT_CMD_Send(AT_CMEE);
            curAT_RSP_ID = AT_CMEE;
            User_StartClock(&ATTimeoutclock);
        }
    }
    //���Ƶ��
    else if(curAT_RSP_ID == AT_CLEAR_FREQ)
    {
        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        //if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            AT_Retry_Num = 0;
            User_StopClock(&ATTimeoutclock);
            printf("CELL_RESELECT\r\n");
            //���Ƶ��
            curAT_RSP_ID = AT_CELL_RESELECT;
            HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)CellReselection, 35);
            User_StartClock(&ATTimeoutclock);
        }
        
    }
    //С����ѡ
    else if(curAT_RSP_ID == AT_CELL_RESELECT)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            User_StopClock(&ATTimeoutclock);
            //��һ����EdrxCMD
            AT_Retry_Num = 0;
            Retry_state = RESET;
            curAT_RSP_ID = AT_EDRX;
            HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)EdrxCMD, 22);
            User_StartClock(&ATTimeoutclock);
            printf("EdrxCMD\r\n");
        }
    }
    else if(curAT_RSP_ID == AT_EDRX)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            User_StopClock(&ATTimeoutclock);
            //��һ����
            AT_Retry_Num = 0;
            Retry_state = RESET;
            curAT_RSP_ID = AT_CLOSE_LWM2M;
            HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)Close_LWM2M, 19);
            User_StartClock(&ATTimeoutclock);
            printf("Close_LWM2M\r\n");

        }
    }
    //�ر�LWM2M
    else if(curAT_RSP_ID == AT_CLOSE_LWM2M)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            User_StopClock(&ATTimeoutclock);
            //��һ����
            AT_Retry_Num = 0;
            Retry_state = RESET;
            curAT_RSP_ID = AT_CLOSE_LCTREG;
            HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)Close_LCTREG, 13);
            User_StartClock(&ATTimeoutclock);
            printf("Close_LCTREG\r\n");
        }
    }
    //�ر���ע��
    else if(curAT_RSP_ID == AT_CLOSE_LCTREG)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            User_StopClock(&ATTimeoutclock);
            //��һ����
            AT_Retry_Num = 0;
            Retry_state = RESET;
            curAT_RSP_ID = AT_BAND5;
            HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)SetBand5, 12);
            User_StartClock(&ATTimeoutclock);
            printf("SetBand5\r\n");
        }
    }
    //����Band5
    else if(curAT_RSP_ID == AT_BAND5)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            User_StopClock(&ATTimeoutclock);
            //��һ������ѯIMEI��
            AT_Retry_Num = 0;
            Retry_state = RESET;
            NB_AT_CMD_Send(AT_CGSN);
            curAT_RSP_ID = AT_CGSN;
            User_StartClock(&ATTimeoutclock);
            printf("IMEI\r\n");
        }
    }
    
//    //�汾��
//    else if(curAT_RSP_ID == AT_CGMR)
//    {
//        char *str;
//        //\r\nSECURITY, ����Ϊ 11
//        str = strtok((char*)atMessage+11, AT_END); 
//        NB_version_len = strlen(str);      
//        tool_memcpy(NB_version, str, NB_version_len);
//        //��һ������ѯIMEI��
//        AT_Retry_Num = 0;
//        Retry_state = RESET;
//        NB_AT_CMD_Send(AT_CGSN);
//        curAT_RSP_ID = AT_CGSN;
//        User_StartClock(&ATTimeoutclock);
//        printf("IMEI\r\n");
//    }
    //��ѯIMEI
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
                //����IMEI��
                tool_memcpy(NB_Report_to_ServerData.IMEI_num, str, IMEI_NUM_LEN); 
                
//                //��һ������IOTƽ̨��ַ
//                NB_AT_CMD_Send(AT_NCDP);
//                curAT_RSP_ID = AT_NCDP;
//                printf("IOT\r\n");
                
                //if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
                //{
                    //��һ��������
                    AT_Retry_Num = 0;
                    Retry_state = RESET;
                    NB_AT_CMD_Send(AT_NRB);
                    curAT_RSP_ID = AT_NRB;
                    User_StartClock(&ATTimeoutclock);
                    //��ʼ���ɹ�
                    NB_NetWork_First_Init = RESET;
                    printf("NRB\r\n");
                //}
            }
        }
    }
//    //����IOTƽ̨��ַ
//    else if(curAT_RSP_ID == AT_NCDP)
//    {
//        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
//        //�ظ���ΪOK
//        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
//        {
//            //��һ��������
//            AT_Retry_Num = 0;
//            Retry_state = RESET;
//            NB_AT_CMD_Send(AT_NRB);
//            curAT_RSP_ID = AT_NRB;
//            User_StartClock(&ATTimeoutclock);
//            //��ʼ���ɹ�
//            NB_NetWork_First_Init = RESET;
//            printf("NRB\r\n");
//        }
//    }
    //������
    else if(curAT_RSP_ID == AT_NRB)
    {
        if(strncmp((char*)atMessage, AT_NRB_RSP, AT_NRB_RSP_LEN) == STR_CMP_TRUE)
        {
            //��һ���ȴ��������
            curAT_RSP_ID = AT_NRB_RESULT;
            printf("REBOOTING\r\n");
            LPUART1_Rx_Disable();
            User_StartClock(&OneTimeoutclock);
        }
    }
    //�������
    else if(curAT_RSP_ID == AT_NRB_RESULT)
    {
        char* str;
        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        str = strstr((char*)atMessage, AT_NRB_RESULT_RSP);
        if(str != NULL)
        {
            //��һ������Э��ջ
            //NB_AT_CMD_Send(AT_CFUN1);
            curAT_RSP_ID = AT_CFUN1;
            Retry_state = RESET;
            AT_Retry_Num = 0;
            ATTimeoutclock.timeOut = AT_CMD_CFUN1_TIMEOUT;
            User_StartClock(&ATTimeoutclock);
            printf("CFUN1");
        }
    }
    //����������ʾ->���û�վ����֪ͨ����������ظ�OK
    else if(curAT_RSP_ID == AT_CMEE)
    {
//        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
//        {
//            curAT_RSP_ID = AT_CGDCONT;
//            NB_AT_CMD_Send(AT_CGDCONT);
//            printf("AT_CGDCONT\r\n");
//            
//            Retry_state = RESET;
//            AT_Retry_Num = 0;
//            ATTimeoutclock.timeOut = AT_CMD_CGDCONT_TIMEOUT;
//            User_StartClock(&ATTimeoutclock);
//        }
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            curAT_RSP_ID = AT_CIMI;
            HAL_UART_Transmit_DMA(&hlpuart1, "AT+CIMI\r\n", 9);
            
            Retry_state = RESET;
            AT_Retry_Num = 0;
            ATTimeoutclock.timeOut = AT_CMD_CGDCONT_TIMEOUT;
            User_StartClock(&ATTimeoutclock);
        }    
    }
    else if(curAT_RSP_ID == AT_CIMI)
    {
        HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        //if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
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
//    else if(curAT_RSP_ID == AT_CGDCONT)
//    {
//        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
//        {
//            Retry_state = SET;
//            AT_Retry_Num = 0;
//            User_StopClock(&ATTimeoutclock);
//            curAT_RSP_ID = AT_NNMI;
//            NB_AT_CMD_Send(curAT_RSP_ID);
//            printf("AT_NNMI\r\n");
//        }
//    }
    else if(curAT_RSP_ID == AT_NNMI)
    {
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            User_StopClock(&ATTimeoutclock);
            Retry_state = RESET;
            AT_Retry_Num = 0;
            User_StartClock(&ATTimeoutclock);
            
            curAT_RSP_ID = AT_CGATT;
            NB_AT_CMD_Send(curAT_RSP_ID);
            printf("AT_CGATT\r\n");
        }
    }
    else if(curAT_RSP_ID == AT_CGATT)
    {
        //if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        //{
        //curAT_RSP_ID = AT_CSCON;
        //NB_AT_CMD_Send(curAT_RSP_ID);
        //}
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            User_StopClock(&ATTimeoutclock);
            Retry_state = RESET;
            AT_Retry_Num = 0;
            User_StartClock(&ATTimeoutclock);
            
            curAT_RSP_ID = AT_CGPADDR;
            NB_AT_CMD_Send(curAT_RSP_ID);
            printf("AT_CGPADDR\r\n");  
        }
    }
    //    else if(curAT_RSP_ID == AT_CSCON)
    //    {
    //        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
    //        {
    //            curAT_RSP_ID = AT_CGPADDR;
    //            NB_AT_CMD_Send(curAT_RSP_ID);
    //            
    //            Retry_state = RESET;
    //            AT_Retry_Num = 0;
    //            ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
    //            User_StartClock(&ATTimeoutclock);
    //        }
    //    }
    //��ѯ�����IP��ַ
    else if(curAT_RSP_ID == AT_CGPADDR)
    {
        //����
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
            //printf("gettingIP\r\n");
            HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
            //printf("ID1: %d", *(atMessage+AT_NETWORK_STATE_LEN));
            //printf("ID2: %d", *(atMessage+AT_NETWORK_STATE_LEN*2));
            //��ʽ��ȷ+�����ݻظ�
            if((strncmp((char*)atMessage+AT_NETWORK_STATE_LEN, AT_COMMA, AT_COMMA_LEN) == STR_CMP_TRUE) || 
               (strncmp((char*)atMessage+AT_NETWORK_STATE_LEN*2, AT_COMMA, AT_COMMA_LEN) == STR_CMP_TRUE))
            {
                printf("gotIP\r\n");
                User_StopClock(&ATTimeoutclock);
                Retry_state = RESET;
                AT_Retry_Num = 0;
                
                //��һ����ѯ����״̬
                curAT_RSP_ID = AT_NUESTATS;
                NB_AT_CMD_Send(AT_NUESTATS);
                
                //User_StartClock(&ATTimeoutclock);
                //curAT_RSP_ID = AT_CLOSE_EDRX;
                
                //HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)CloseEdrx, 28/*sizeof(CloseEdrx)*/);
            }
        //}       
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
//            //��һ����ѯ����״̬
//            curAT_RSP_ID = AT_NUESTATS;
//            NB_AT_CMD_Send(AT_NUESTATS);
//            //User_StartClock(&ATTimeoutclock);
//            printf("OPEN_PSM\r\n");
//        }
//    }
    //��ѯ����״̬������
    else if(curAT_RSP_ID == AT_NUESTATS)
    {
        Retry_state = SET;
        AT_Retry_Num = 0;
        User_StopClock(&ATTimeoutclock);
        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        
        uint8_t curState = ERROR;
        curState = NueStatesProcess(atMessage);
        //��������ȷ
        //��һ�� ����Socket
        if(curState == SUCCESS)
        {
            User_StopClock(&ATTimeoutclock);
            curAT_RSP_ID = AT_NSOCR;
            NB_AT_CMD_Send(AT_NSOCR);
            Retry_state = RESET;
            AT_Retry_Num = 0;
            ATTimeoutclock.timeOut = AT_CMD_ADDR_TIMEOUT;
            User_StartClock(&ATTimeoutclock);
        }
    }
    //����Socket
    else if(curAT_RSP_ID == AT_NSOCR)
    {
        if(strncmp((char*)atMessage, AT_PRAM_SOCKET_RSP, AT_PRAM_SOCKET_RSP_LEN) == STR_CMP_TRUE)
        {
            printf("Innet!\r\n");
            curAT_RSP_ID = AT_INIT_SUCCESS;
            Retry_state = SET;
            AT_Retry_Num = 0;
            User_StopClock(&ATTimeoutclock);
            User_StopClock(&InNetTimeoutclock);           
            InnetTimeOutFlag = RESET;
#warning �����ɹ� �ȴ�����������
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
#warning ���Ĳ�����ע������
                curAT_RSP_ID = AT_DATA_RSP; 
                
                NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) 
                    + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);                
                NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);             
                DeviceRegistclock.timeOut = 4000;
                User_StartClock(&DeviceRegistclock);
                BoardCloseTimeoutclock.timeOut = 65000;
                User_StartClock(&BoardCloseTimeoutclock);
                printf("registReq\r\n");
            }
            else if(RegistFlag == SET)
            {
                //�޸�0703
                printf("reConnect\r\n");
                User_StartClock(&BoardCloseTimeoutclock);
            }
        }
    }
    else if(curAT_RSP_ID == AT_DATA_RSP)
    {
        printf("DR\r\n");
        HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        char *str = NULL;
        char *str1 = NULL;
        
        //str = strstr((char*)atMessage, AT_RSP_OK);
        str = strstr((char*)atMessage, AT_CME);
        str1 = strstr((char*)atMessage, AT_PRAM_NSONMI);
        
        if(str1 != NULL)
        {
            NB_AT_CMD_Send(AT_NSORF);
            curAT_RSP_ID = AT_NSORF;
        }
        else
        {
            if((strncmp((char*)atMessage, AT_PRAM_CME_159, AT_PRAM_CME_159_LEN) == STR_CMP_TRUE) || (strncmp((char*)atMessage, AT_PRAM_CME_4, AT_PRAM_CME_4_LEN) == STR_CMP_TRUE))
            //if(testFlag == SET)
            {
                cmeErrorCounter++;
                if(cmeErrorCounter < 3)
                {
                    //testFlag = RESET;
                    curAT_RSP_ID = AT_NO;
                    NB_InNetWork_Flag = RESET;
                    DataUpFlag = RESET;
                    
                    //��λNBģ����������
                    User_StopClock(&ATTimeoutclock);
                    User_StopClock(&DataRptclock);
                    User_StopClock(&BoardCloseTimeoutclock);
                    User_StartClock(&NBResetclock);
                }
                else
                {
                    //testFlag = RESET;
                    cmeErrorCounter = 0;
                    User_StopClock(&ATTimeoutclock);
                    User_StopClock(&BoardCloseTimeoutclock);
                    User_StopClock(&InNetTimeoutclock);
                    User_StopClock(&DataRptclock);
                    User_StopClock(&DataRspclock);
                    Close_NB();
                    wakeupCounter = 0;
                    SleepStatus = SLEEP;
                }
            }
            else
            {
                //��ȡ���һ֡��Ч����
                if(str == NULL)
                {
                    printf("S\r\n");
                    
                    Retry_state = SET;
                    AT_Retry_Num = 0;
                    User_StopClock(&DataRptclock);
                    
                    if(RegistFlag == SET)
                    {        
                        //�ָ�Ϊԭģʽ
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
                    if(data_confirmFlag == SET)
                    {
                        printf("cfm\r\n");
                        if(NB_Confirm_counter>0)
                        {
                            NB_Confirm_counter = 0;
                            if(Locator_pram.newMode == SCHEDULE_MODE)
                            {
                                Schedule_StateStartFlag = SET;
                            }
                            else
                            {
                                NB_min_counter = 1;
                            }
                            User_StartClock(&DataRspclock);
                        }
                        else
                        {
                            //data_confirmFlag = RESET;
                            //User_StartClock(&DataRspclock);
                            if(Locator_pram.newMode == SCHEDULE_MODE)
                            {
                                Schedule_StateStartFlag = SET;
                            }
                            else
                            {
                                NB_min_counter = 1;
                            }
                        }
                    }
                }
                else
                {
                    printf("cmdError\r\n");
                    if(data_confirmFlag == SET)
                    {
                        //User_StopClock(&DataRspclock);
                        NB_Confirm_counter++;
                        //NB_AT_CMD_Send(AT_NSOST, LOCATOR_CMD_CONFIRM);
                        if(NB_Confirm_counter >= 3)
                        {
                            NB_Confirm_counter = 0;
                            if(RegistFlag == RESET)
                            {
                                NVIC_SystemReset();
                            }
                            else if(RegistFlag == SET)
                            {
                                printf("ConfirmTimeOut");
                                
                                User_StopClock(&ATTimeoutclock);
                                User_StopClock(&InNetTimeoutclock);
                                User_StopClock(&DataRptclock);
                                User_StopClock(&DataRspclock);
                                User_StopClock(&BoardCloseTimeoutclock);
                                Close_NB();
                                wakeupCounter = 0;
                                SleepStatus = SLEEP;
                            }
                        } 
                    }              
                }
                curAT_RSP_ID = AT_NSONMI;
            }
        }
    }
    //�յ�����
    else if(curAT_RSP_ID == AT_NSONMI)
    {
        if(strncmp((char*)atMessage, AT_PRAM_NSONMI, AT_PRAM_NSONMI_LEN) == STR_CMP_TRUE)
        {
#warning ���0323
            packageFinishFlag = RESET;
                
            printf("NSONMI\r\n");
            if(schedule_congif_Flag == SET)
            {
                User_StopClock(&TimeSynclock);
            }
            if(Schedule_StateStartFlag == SET)
            {
                User_StopClock(&DataRspclock);
            }
            //if(RegistFlag == RESET)
            //{
            //    User_StopClock(&DeviceRegistclock);
            //}
            //���յ����ݣ����Ͷ�ȡ����
            NB_AT_CMD_Send(AT_NSORF);
            curAT_RSP_ID = AT_NSORF;
            
//            //���ճɹ����++
//            msg_seq_num++;
//            NB_Report_to_ServerData.msg_seq_num = BigtoLittle16(msg_seq_num);
        }
    }
    //��ȡ��������    
#warning ���0323    
    else if(curAT_RSP_ID == AT_NSORF)
    {         
        char *str = NULL;
        char *str1 = NULL;
        char *CME_ERROR_Flag = NULL;
        
        str = strstr((char*)atMessage, AT_PRAM_NSONMI_RSP_OK);
        str1 = strstr((char*)atMessage, AT_PRAM_NSONMI_RSP_OK2);
        CME_ERROR_Flag = strstr((char*)atMessage, AT_CME);
        
        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        //�ظ������޴���
        if(CME_ERROR_Flag == NULL)
        {
            //��ȡ���һ֡��Ч����
            if(str == NULL && str1!= NULL )
            {
                packageFinishFlag = SET;
                
                printf("pEND\r\n");
                if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
                {
                    printf("r_end\r\n");
                }
                else
                {
                    printf("process_data\r\n");
                    //CMD_packageNum++;
                    //LPUART1_Rx_Disable();
                    Handel_ServerAT_CMD(atMessage+2);
                    
//                    if(testFlag == SET)
//                    {
//                        testFlag = RESET;
//                        HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
//                    }
                    
//                    if(TxCompleteFlag == SET)
//                    {
//                        TxCompleteFlag = RESET;                   
//                        printf("SR2\r\n");
                          memset(UsartType.RX_pData, 0, RX_LEN);
//                        NB_AT_CMD_Send(AT_NSORF);
//                        curAT_RSP_ID = AT_NSONMI;
//                    }
                }
                //���һ֡�������������յ������ļ���
                CMD_packageNum = 0;
            }
            else if(str != NULL)
            {
                CMD_packageNum++;
                printf("process_dataMix\r\n");
                LPUART1_Rx_Disable();
                Handel_ServerAT_CMD(atMessage+2);
                
                //str = strstr((char*)atMessage, AT_RSP_OK);
                //if(str != NULL)
                if(TxCompleteFlag == SET)
                {
                    TxCompleteFlag = RESET;
                    printf("SR1\r\n");
                    NB_AT_CMD_Send(AT_NSORF);
                    curAT_RSP_ID = AT_NSORF;
                    //memset(UsartType.RX_pData, 0, RX_LEN);
                }
            }
            else if(str1 == NULL)
            {
                NB_AT_CMD_Send(AT_NSORF);
                curAT_RSP_ID = AT_NSORF;
            }       
        }
        else
        {
            printf("CME\r\n");
            HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
            NB_AT_CMD_Send(AT_NSORF);
            curAT_RSP_ID = AT_NSORF;
        }
    }
    //�����������
    else if(curAT_RSP_ID == AT_CLEAR_RESERVE)
    {
        //HAL_UART_Transmit_DMA(&huart2, UsartType.RX_pData, UsartType.RX_Size);
        if(strncmp((char*)atMessage, AT_RSP_OK, AT_RSP_OK_LEN) == STR_CMP_TRUE)
        {
            printf("Clear\r\n");
            NSORF_counter = 0;
            
            curAT_RSP_ID = AT_NUESTATS_NORMAL;
            NB_AT_CMD_Send(AT_NUESTATS); 
            //curAT_RSP_ID = AT_DATA_RSP;
            //NB_InNetWork_Flag = SET;
        }
        else
        {
            if(strncmp((char*)atMessage, AT_RSP_ERROR, AT_RSP_ERROR_LEN) == STR_CMP_TRUE)
            {
                //ģ���쳣
                printf("NB_ERROR\r\n");
                curAT_RSP_ID = AT_NO;

                //if(NB_RESET_Flag == RESET)
                //{
                //    NB_RESET_Flag = SET;
                    User_StopClock(&BoardCloseTimeoutclock);
                    User_StopClock(&DataRptclock);
                    User_StopClock(&ATTimeoutclock);
                    
                    DataUpFlag = RESET;
                    NB_InNetWork_Flag = RESET;
                    NB_ERROR_counter++;
                    if(NB_ERROR_counter > 2)
                    {
                        NB_ERROR_counter = 0;
                        wakeupCounter = 0;
                        SleepStatus = SLEEP;
                    }
                    HAL_GPIO_WritePin(GPIOB, NB_RESET_Pin, GPIO_PIN_SET);
                    User_StartClock(&NBResetclock);
                //} 
            }
            else
            {
                NSORF_counter++;
                if(NSORF_counter < 3)
                {
                    printf("NSORF\r\n");
                    NB_AT_CMD_Send(AT_NSORF);
                    curAT_RSP_ID = AT_CLEAR_RESERVE;
                }
                else
                {
                    NSORF_counter = 0;
                    
                    curAT_RSP_ID = AT_NUESTATS_NORMAL;
                    NB_AT_CMD_Send(AT_NUESTATS);
                    //curAT_RSP_ID = AT_DATA_RSP;
                    //NB_InNetWork_Flag = SET;
                }
            }
        }                
    }
    //�ź�ǿ�Ȳ�ѯ
    else if(curAT_RSP_ID == AT_NUESTATS_NORMAL)
    {
        uint8_t curState = ERROR;
        curState = NueStatesProcess(atMessage);
        //��������ȷ
        //��һ�� ��������
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
            //wakeupCounter = 0;
            //SleepStatus = SLEEP;
        }
    }
}

/*
* @fn      NueStatesProcess
* @brief   ��������״̬֡ 
* @param   ��������
* @return   
*/
uint8_t NueStatesProcess(uint8_t *atMessage)
{
//    /*
//    ����
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
            //�ź�ǿ��
            if(strncmp(str, AT_NUESTATS_RSPR, AT_NUESTATS_RSPR_LEN) == STR_CMP_TRUE)
            {
                uint16_t rspr = 0;
                rspr = atoi(str+AT_NUESTATS_RSPR_LEN)/10;
                Signal_Level.RSPR = BigtoLittle16(rspr);
                dataCounter++;
            }
            //С��ID
            else if(strncmp(str, AT_NUESTATS_CELL_ID, AT_NUESTATS_CELL_ID_LEN) == STR_CMP_TRUE)
            {
                uint32_t cell_id = 0;
                cell_id = atoi(str+AT_NUESTATS_CELL_ID_LEN);
                Signal_Level.cellID = cell_id;
                dataCounter++;
            }
            //���ǵȼ�
            else if(strncmp(str, AT_NUESTATS_ECL, AT_NUESTATS_ECL_LEN) == STR_CMP_TRUE)
            {
                uint8_t ecl = 0;
                ecl = atoi(str+AT_NUESTATS_ECL_LEN);
                Signal_Level.ECL =  ecl;
                dataCounter++;
            }
            //�����
            else if(strncmp(str, AT_NUESTATS_SNR, AT_NUESTATS_SNR_LEN) == STR_CMP_TRUE)
            {
                uint16_t snr = 0;
                snr = atoi(str+AT_NUESTATS_SNR_LEN)/10;
                Signal_Level.SINR = BigtoLittle16(snr);
                dataCounter++;
            }
            //�ź�����
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

//�ϱ�����
void NB_Data_Report(void)
{
    //    if(NB_Init_Flag != SUCCESS)
    //    {
    //        Open_NB();
    //        NB_NetWork_Init();
    //    }
    //    //�����ɹ�
    //    else if( NB_Init_Flag == SUCCESS)
    //    {
    //    if(currentMode == CONFIG_MODE)
    //    {
//#warning ���  Locator_Data �е�ģʽ��Ϣ     
    //    }
    //    else if(currentMode == NORMAL_MODE)
    //    {
    //        
    //    }
    //    else if(currentMode == POSITIONING_MODE)
    //    {
    //        //�ϱ���ϣ����GPS����
    //    }
    //        //ʹNBģ��������� ��������
    //        //NB_AT_CMD_Send();
    //         NBTimeOutFlag = RESET;
    //        //User_StartClock(&NBReportclock);
    //    }
}

//GPS���ݲɼ�
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

//ʹ�ܴ��ڽ���
void LPUART1_Rx_Enable(void)
{
    //    HAL_UART_AbortReceive(&hlpuart1);
    //    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_IDLE);//ʹ�� ���� �����ж�
    //    nbRxMessage = (uint8_t *)malloc(USER_BUFF_SIZE);
    //    printf("Rx_En");
    //    HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)nbRxMessage, USER_BUFF_SIZE);
    //    memset(nbRxMessage,0,1);
    HAL_UART_Receive_DMA(&hlpuart1, UsartType.RX_pData, RX_LEN);  
    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_IDLE);
}
//�رմ��ڽ���
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

//ʹ�ܴ��ڽ���
void UART1_Rx_Enable(void)
{
//    HAL_UART_AbortReceive(&huart1);
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//ʹ�� ���� �����ж�
//    gpsRxMessage = (uint8_t *)malloc(USER_BUFF_SIZE);
//    HAL_UART_Receive_DMA(&huart1, (uint8_t *)gpsRxMessage, USER_BUFF_SIZE);
//    memset(gpsRxMessage,0,1);
#warning �޸�171229    
    __HAL_DMA_ENABLE(huart1.hdmarx);
    HAL_UART_Receive_DMA(&huart1, UsartType.RX_pData, RX_LEN);  
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}
//�رմ��ڽ���
HAL_StatusTypeDef UART1_Rx_Disable(void)
{
    __HAL_LOCK(&huart1);
    HAL_UART_AbortReceive(&huart1);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_DMA_DISABLE(huart1.hdmarx);
    huart1.RxXferSize = 0;
    huart1.hdmarx->Instance->CNDTR = 0;
#warning �޸�171229
    //__HAL_DMA_ENABLE(huart1.hdmarx);
    __HAL_UNLOCK(&huart1);
    return HAL_OK;
}

//���ڷ���
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
//���ڽ���
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == hlpuart1.Instance)
    {
        asm("nop");
        //        printf("R\r\n");        
        //        //      ����ATָ����
        //        if(tempLen != 0)
        //        {
        //            __disable_irq();   // �ر����ж�
        //            LPUART1_Rx_Disable();
        //            Handel_NB_AT_RSP(nbRxMessage);
        //            free(nbRxMessage);
        //            //������ɺ��ؿ�
        //            if(curAT_RSP_ID != AT_NRB_RESULT)
        //            {
        //                LPUART1_Rx_Enable();
        //            }
        //            __enable_irq();    // �������ж�
        //        }
    }
    else if(huart->Instance == huart1.Instance)
    {
        //        //�رս���
        //        UART1_Rx_Disable();
        //        if(tempLen != 0)
        //        {
        //            //����GPS����
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
    
    //�ر����е�
    closeAllLed();
    //�ر�,NB,GPS
    HAL_GPIO_WritePin(GPS_SW_GPIO_Port, GPS_SW_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NB_SW_GPIO_Port, NB_SW_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
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
            //������ɺ����GPS��Ϣ
    if(RegistFlag == RESET)
    {
        if(BoardSleepCounter == 0)
        {
            BoardSleepCounter++;
            User_StartClock(&BoardCloseTimeoutclock);
        }
        else if(BoardSleepCounter == 1)
        {
            BoardSleepCounter = 0;
            NVIC_SystemReset();
        }
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
        printf("sleep5\r\n");
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
//������ʼ��
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
    //HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 9750, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
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
    //��λ20s ,60s/20s = 3
    if(RegistFlag == SET)
    {     
        wakeupCounter++;
        //uint16_t dataCycle_T = 0;
        //��λ--����
#warning ���scheduleģʽ180323   
        if(Schedule_ModeFlag == RESET)
        {
            if(currentMode == NORMAL_MODE)
            {
                dataCycle = Locator_Data.FindnowCycle;
            }
            else if(currentMode == POSITIONING_MODE)
            {
#warning  ���171229
                //gps���ּ���2h
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
                //�������´��ϱ�ʱ��
                ScheduleModeProcess();
            }
        }
        if(dataCycle>0)
        {
            if(wakeupCounter >= (dataCycle*3))
            {
                wakeupCounter = 0;
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
        //if(SleepStatus == AWAKE)
        //{
            //printf("keyPress!\r\n");
            if(KeyAction.keyPressFlag != SET)
            {
                //��ǰ����Ѱ���
                KeyAction.keyPressFlag = SET;
                //�жϰ��I��ʱ��״̬���������I��ʱ��
                if(KeyPressTimerStartFlag == RESET )
                {
                    KeyPressTimerStartFlag = SET;
                    //Ĭ�ϴ˴�Ϊ����
                    KeyAction.keyLongPressFlag = SET;
                    User_StartClock(&KeyPressclock);
                    printf("KeyTimerStart\r\n");
                }
                //�ڰ�����ʱ������״̬�£��ж�������ʱ��״̬������������ʱ��
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
        return (event);
    }
    
    if(event & KEY_RELEASE_EVENT)
    {
        //����һ����δ̧�����ж�Ϊ����������Ϊ�̰�
        if((KeyPressTimerStartFlag == SET) /*&& (KeyAction.keyConut > 0)*/)
        {
            printf("short");
            KeyAction.keyLongPressFlag = RESET;
        }
        event = event ^ KEY_RELEASE_EVENT;
        return (event);
    }
    if(event & TIM2_TIMEOUT_EVENT)
    {
        //����ʱ��
        User_TaskRemarks();
        event = event ^ TIM2_TIMEOUT_EVENT;
        return (event); 
    }
    if(event & TIM21_TIMEOUT_EVENT)
    {
        if(KeyAction.keyPressFlag == SET)
        {
            //����
            if( HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
            {
                //������Ч
                KeyAction.keyConut++;
                //                if(KeyAction.keyConut>7)
                //               {
                //                    KeyAction.keyConut = 0;
                //                    NVIC_SystemReset();
                //                }
                //printf("keyCount:%d\r\n", KeyAction.keyConut);
                //ע�����
                if(RegistFlag == SET)
                {
                    //printf("EPStartFlag:%d\r\n", EvenPressTimerStartFlag);
                    //������������
                    if(KeyPressTimerStartFlag == SET)
                    {
                        //����δ����
                        if(EvenPressTimerStartFlag == RESET && KeyAction.keyConut == 1)
                        {
                            //�����л�ģʽ
                            if(ModeSwitchFlag == RESET)
                            {
                                //����������ʱ
                                EvenPressTimerStartFlag = SET;
                                //KeyAction.keyConut = 0;
                                //printf("EvenPress");
                                User_StartClock(&EvenPressclock);
                            }
                        }
                    }
                }
                
#warning    �̰�5���л�ģʽ
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
#warning ���ԣ�����������
                                    printf("5NB\r\n");
                                    curAT_RSP_ID = AT_NO;
                                    DataUpFlag = RESET;
                                    NB_InNetWork_Flag = RESET;
                                    NB_open_Flag = RESET;
                                    Close_NB();
                                }
                                KeyAction.keyConut = 0;
                                //�����л�ģʽ���
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
                                
                                //#warning �����ã�����������
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
        return (event); 
    }
    if(event & TIM22_TIMEOUT_EVENT)
    {
        //����LED��˸����
        Handel_LED();
        
        event = event ^ TIM22_TIMEOUT_EVENT;
        return (event); 
    }
    return 0;
}
/**
* @brief  UserStartLedClock,����LED��ʱ��
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
* @brief  ModeDisplayCb,ģʽ��ʾ�ص�
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
        //�����
        HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
    }
    else if(chargeEnable == SET && chargeFinish == SET)
    {
        //������
        HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
    }
#warning �޸�0105        
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
* @brief  InNetCb,������ʱ
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
    else if(InNetCounter > 4)
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
    ��������˱��ֶ�ʱ�� �ȵ���ʱ���������ٱ������һ��
    ��Ч��gps��ָ�� GPS_TO_NB_Data
    */
    
    GPS_Info_Store(GPS_TO_NB_Data);
    
    GPS_TO_NB_Data = NULL;
    GPSKeepTimerStartFlag = RESET;
    //��Ϊ����20�룬��ʱ�䵽�������־λ
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
#warning ���171230
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
    //�ɼ�ʱ�䵽
    if(adc_timeoutFlag == SET)
    {
        //uint8_t i = 0;
#warning �޸�1230        
        // �ر����ж�
        //__disable_irq();
        //for( i=0 ;i<3; i++)
        //{
            HAL_ADC_Start(&hadc);
            HAL_ADC_PollForConversion(&hadc, 250);
            if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc), HAL_ADC_STATE_REG_EOC))
            {
                AD_Value = HAL_ADC_GetValue(&hadc);
                //������ѹֵ
                AD_Value  = (AD_Value*3200) >> 12;
                //printf("ADC:%d", AD_Value);
                //﮵�ص�ѹֵ
                Bat_Value = AD_Value*(R_GND + R_VCC)/R_GND;
                printf("BAT:%d\r\n", Bat_Value);
            }
        //}
        //����ֵ
        Bat_Level = BatValueConvert(/*AD_Value*/Bat_Value, Pre_Bat_Level, chargeEnable, RESET);
        Locator_Data.batteryLv = Bat_Level;
        Schedule_Data_upload.battery_Lv = Bat_Level;
        
        Pre_Bat_Level = Bat_Level;
        printf("BAT:%d\t\n", Bat_Level);
        // �������ж�
        //__enable_irq();
        //������ɺ�����
        adc_timeoutFlag = RESET;
        if(NB_open_Flag == RESET && gps_open_Flag == RESET)
        {
            printf("Adc_Sleep\r\n");
            SleepStatus = SLEEP;
        }
    }
    if(Bat_Value < 3000)
    {
        if(BoardStart == SET)
        {
            BoardStart = RESET;
            NVIC_SystemReset();
        }
        else
        {
            printf("SL");
            Bat_OK_Flag = RESET;
            SleepStatus = SLEEP;
            StopMode_Measure();
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
        //δ����GPS,������NB
        if(gps_open_Flag != SET && NB_open_Flag == SET)
        {
            LPUART1_Rx_Disable();
            //__disable_irq();
            Handel_NB_AT_RSP(UsartType.RX_pData);
            //__enable_irq();
            LPUART1_Rx_Enable();
        }
        //������GPS
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
                //�رս���
                UART1_Rx_Disable();
                //__disable_irq();   // �ر����ж�  
                GPSInfoProcess(UsartType.RX_pData);
                //__enable_irq();    // �������ж�
                UART1_Rx_Enable();
            }
        }
    }
    
//#warning ���ԣ�����������
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
//    //__disable_irq();   // �ر����ж�
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
//    //__enable_irq();    // �������ж�
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
        //������
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
            //memset(UsartType.RX_pData, 0, RX_LEN);
            curAT_RSP_ID = AT_CLEAR_RESERVE;
            NB_AT_CMD_Send(AT_NSORF);
            //NB_InNetWork_Flag = SET;
            
            //testCounter++;
            //printf("testCounter:%d", testCounter);
            //if(testCounter == 2)
            //{
                //testCounter = 0;
            //    testFlag = SET;
            //    BoardCloseTimeoutclock.timeOut = 0;
                //event |= TIM2_TIMEOUT_EVENT;
            //}
            //else
            //{
                BoardCloseTimeoutclock.timeOut = AT_BOARD_CLOSE_TIMEOUT;
            //}
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
    
    //memset(UsartType.RX_pData, 0, RX_LEN);    
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
    //�ж�23:59:40s����++��12�����ͬ��
    switchToTodayFlag = switchToToday(Schedule_CMD_State.configMode);
    
    //    if(Time_synFlag == SET)
    //    {
    //        if(switchToTodayFlag != 0xFF)
    //        {
    //            CurrentDayInTheLoop = switchToTodayFlag;
    //        }
  
    
    //�޸�Ϊ12��ͬ��ʱ�������
    if(switchToTodayFlag.TimeSynFlag == SET)
    {
        Time_synFlag = RESET;
        //CurrentDayInTheLoop = switchToTodayFlag.CurrentDayInTheLoop;
        
        //ͬ��ʱ���
        if(Time_syning_Flag == RESET)
        {
            Time_syning_Flag = SET;
            currentMode = NORMAL_MODE;
            
            //if(Time_syning_Flag == SET)
            //{
            //printf("@@@\r\n");
            //User_StopClock(&BoardCloseTimeoutclock);
            //            if(gps_open_Flag == SET)
            //            {
            //                User_StopClock(&GPSTimeOutclock);
            //                Device_Close_GPS();
            //            }
            //            if(NB_open_Flag == SET)
            //            {
            //                User_StopClock(&ATTimeoutclock);
            //                User_StopClock(&InNetTimeoutclock);
            //                NB_open_Flag = RESET;
            //                Close_NB();
            //            }
            //}
            
            //����
            wakeupCounter = 0;
            Locator_Data.userAlarmFlag = RESET;
            NBTimeOutFlag = SET;
            SleepStatus = AWAKE;
        } 
    }
    
    
    if(switchToTodayFlag.DaySwitchFlag == SET)
    {
        //if(cycleStartFlag == SET)
        //{
        //    //����schedule����
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
    HAL_GPIO_WritePin(GPIOB, NB_RESET_Pin, GPIO_PIN_RESET);
    //NB_NetWork_Init();
    
    AT_Retry_Num = 0;
    Retry_state = RESET;    
    //1.�ر���Ƶ����
    NB_AT_CMD_Send(AT_CFUN0);
    //������ʱ��ʱ������ʱ�ط����ط��������� ����ʧ�ܡ�       
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
    if(RegistFlag == SET)
    {
        curAT_RSP_ID = AT_DATA_RSP;
        
        //ʱ�䵽��δ�յ���ȷ�Ļظ����ط�
        if(Retry_state == RESET)
        {
            //��ʱ�ط�
            if(AT_Retry_Num < 5)
            {
                AT_Retry_Num++;
                
                if(Time_synFlag == SET)
                {
                    if(Schedule_ModeFlag == SET)
                    {
                        curAT_RSP_ID = AT_DATA_RSP;
                        if(Locator_Data.userAlarmFlag == SET)
                        {
                            Schedule_Data_upload.Mode = 0x02;
                        }
                        else
                        {
                            Schedule_Data_upload.Mode = currentMode;
                        }
                        NB_Report_to_ServerData.cmd = SCHEDULE_UPLOAD_ID;
                        NB_Report_to_ServerData.cmdlen = sizeof(NB_Report_to_ServerData_t)+sizeof(Schedule_Data_upload_t)+ Schedule_Data_upload.GPS_List_num * sizeof(GPS_List_t) + CHECK_SUM_LEN + END_NUM_LEN;
                        Schedule_Data_Report(NB_Report_to_ServerData, Schedule_Data_upload, GPS_Info_schedule);
                        printf("Schedule_DATA_UP2: %d", NB_Report_to_ServerData.cmdlen);                        
                    }
                }
                else
                {
                    if(Schedule_ModeFlag == RESET)
                    {
                        curAT_RSP_ID = AT_DATA_RSP;
                        //NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
                        printf("NB_DATA_UP2: %d", NB_Report_to_ServerData.cmdlen);
                        NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
                    }                 
                }
                User_StartClock(&DataRptclock);
            }
            //�ط�������
            else
            {
                Retry_state = RESET;
                AT_Retry_Num = 0;
                
                //�ָ�Ϊԭģʽ
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
}

void TSETCb(void)
{
    //User_StartClock(&Testclock);
    NVIC_SystemReset();
    //printf("T1\r\n");
}


//ʹ�ܴ��ڽ���
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
