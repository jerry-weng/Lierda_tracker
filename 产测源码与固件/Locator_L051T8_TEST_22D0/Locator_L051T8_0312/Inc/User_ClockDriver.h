/*******************************************************************************
    Copyright:          Lierda WSN BT team
    Filename:           USER_CLOCK_DRIVER_H
    Description:        USER_CLOCK_DRIVER_H
    FileVersion:        V3.0

    ChangeLog: 

    =========================================
    date:17/3/10
    author:SSS
    log: Create File


*******************************************************************************/

#ifndef USER_CLOCK_DRIVER_H
#define USER_CLOCK_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"
#include <stdlib.h>
#include "stdio.h" 
#include "string.h"
#include "tim.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
typedef struct
{
  uint32_t eventID;
  uint32_t timeOut;
  void (*TaskHook)(void);
}Clock_Struct;  

typedef void (*TaskFunction_t)(void);

typedef struct _ListNode  //List node 类型
{
    Clock_Struct ClockStruct;
    struct _ListNode *next;
}ListNode;
    
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
*  EXTERNAL FUNCTIONS
*/
//使用虚拟定时器之前需要定义如下4个函数
/*
 * @fn      User_Timer_GetCurrentTime
 * @brief   需要外部定义一个获取系统时间的函数，后面用于获取2次时间之间的毫秒差
 * @param   timeOut
 * @return   
 */
extern uint32_t User_Timer_GetCurrentTime();
/*
 * @fn      User_Timer_Diff_ms()
 * @brief   用于获取2次时间之间的毫秒差
 * @param   time1
 * @param   time2
 * @return   
 */
extern uint32_t User_Timer_Diff_ms(uint32_t time1, uint32_t time2);
/*
 * @fn      User_Timer_Start
 * @brief   需要外部定义一个启动定时器函数，可根据mcu不同自己修改
 * @param   timeOut
 * @return   
 */
extern void User_Timer_Start(uint32_t timeOut);
/*
 * @fn      User_Timer_Stop
 * @brief   需要外部定义一个停止定时器函数，可根据mcu不同自己修改
 * @return   
 */
extern void User_Timer_Stop();
/*********************************************************************
 * API FUNCTIONS
 **
 * @fn      CreateClockTask
 * @brief   CreateClockTask 
 * @param   eventID - 事件ID
 * @param   timeOut - 超时
 * @param   loopFlag - 定时器循环标志位
 * @param   clockFunc - 任务实现函数指针
 * @return  
 */
extern uint8_t User_StartClock(Clock_Struct *newClcok);
/**
 * @fn      User_StopClock
 * @brief   User_StopClock 
 * @return   
 */
extern uint8_t User_StopClock(Clock_Struct *pClock); 
/**
 * @fn      User_ResetClock
 * @brief   User_ResetClock 
 * @param   pClock - 需要重启的定时器结构体
 * @return   
 */
extern void User_ResetClock(Clock_Struct *pClock, uint32_t newTimeout);
/**
 * @fn      User_IsActive
 * @brief   User_IsActive 
 * @param   pClock
 * @return  
 */
extern uint8_t User_IsActive(Clock_Struct *pClock);
/**
 * @fn      User_TaskRemarks
 * @brief   User_TaskRemarks 
 * @return   
 */
extern void User_TaskRemarks(void);
/**
 * @fn      User_TaskProcess
 * @brief   User_TaskProcess 
 * @return   
 */
extern void User_Timer_TaskProcess(void);
/*********************************************************************
*********************************************************************/  

#ifdef __cplusplus
}
#endif

#endif /* USER_CLOCK_DRIVER_H */
