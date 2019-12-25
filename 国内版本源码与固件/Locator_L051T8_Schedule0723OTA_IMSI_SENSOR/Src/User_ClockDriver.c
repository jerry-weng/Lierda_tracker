/*******************************************************************************
    Copyright:       Lierda WSN BT team
    Filename:        User_ClockDriver.c
    Description:     User_ClockDriver
    FileVersion:     V3.0

    ChangeLog: 

    =========================================
    date:17/3/10
    author:SSS
    log: Create File
*******************************************************************************/


/*******************************************************************************
* INCLUDES
*/
//#include <stdio.h>
#include <string.h>
#include "User_ClockDriver.h"
#include "tim.h"
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* MACROS
*/
/*********************************************************************
* PUBLIC VARIABLES 
*/
////链表的head
//static ListNode *head = NULL;
////指向链表的最后一个节点
//static Clock_Struct *currentPoint = NULL;
//当前定时器中装载的timeout
uint32_t currentLoadValue = 0xFFFFFFFF;
//最后一次启动定时器时获取的一个系统时钟时间
static uint32_t lastStartTimer_sysT32 = 0;
//定时器任务事件
static uint32_t taskEvents = 0;

extern uint32_t event;
#define TIM2_TIMEOUT_EVENT_T          0x0004

Clock_Struct clock_list[25] = {0};
uint8_t clock_list_idx = 0;
/*********************************************************************
* User Timer API
*/
/*
 * @fn      User_Timer_Start
 * @brief   需要外部定义一个启动定时器函数，可根据mcu不同自己修改
 * @param   timeOut
 * @return   
 */

void User_Timer_Start(uint32_t timeOut)
{
    if( timeOut == 0)
        return;
    else
    {
        //HAL_TIM_Base_MspDeInit(&htim2);
        HAL_TIM_Base_DeInit(&htim2);
        MX_TIM2_Init(timeOut);
        HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE);
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(&htim2);
    }
}
/*
 * @fn      User_Timer_Stop
 * @brief   需要外部定义一个停止定时器函数，可根据mcu不同自己修改
 * @return   
 */
void User_Timer_Stop()
{
    HAL_TIM_Base_Stop_IT(&htim2);
    //HAL_TIM_Base_DeInit(&htim2);
    //HAL_TIM_Base_MspDeInit(&htim2);
}
/*
 * @fn      User_Timer_GetCurrentTime
 * @brief   需要外部定义一个获取系统时间的函数，后面用于获取2次时间之间的毫秒差
 * @param   timeOut
 * @return   
 */
uint32_t User_Timer_GetCurrentTime()
{
    return HAL_GetTick();
}
/*
 * @fn      User_Timer_Diff_ms()
 * @brief   用于获取2次时间之间的毫秒差
 * @param   time1
 * @param   time2
 * @return   
 */
uint32_t User_Timer_Diff_ms(uint32_t time1, uint32_t time2)
{
    uint32_t Diff_Time;
    if(time1 > time2)
    {
        Diff_Time = time1 - time2;
    }
    else
    {
        Diff_Time = time2 - time1;
    }
    return Diff_Time;
}
/*********************************************************************
* LOCAL VARIABLES
*/

/*********************************************************************
* LOCAL FUNCTIONS
*/
//从链表中删除一个节点
static uint8_t TaskDelete(Clock_Struct *pClock);
//从链表中找出timeout最小的节点，并返回
static Clock_Struct* Min();
//将链表中所有节点中的timeout都减去一个差值
static void MinusDiff(uint32_t diff);
//找到链表中的最后一个节点，并返回
//static Clock_Struct* FoundListTail();

static uint8_t ActivedTimerNum(void);

///**
// * @fn      IsFound_Event
// * @brief   找出相应事件的结构体位置，并返回 
// * @param   pClock
// * @return  point
// */
//static ListNode* IsFound_Event(uint32_t eventID)
//{
//    ListNode *p = NULL;
//    p = head;
//    while(p != NULL)
//    {
//        if(p->ClockStruct.eventID == eventID)
//            break;
//        p = p->next;
//    }
//    return p;
//}
/**
 * @fn      IsFound_Event
 * @brief   找出相应事件的结构体位置，并返回 
 * @param   pClock
 * @return  point
 */
static Clock_Struct* IsFound_Event(uint32_t eventID)
{
    Clock_Struct *p = NULL;
    uint8_t i = 0;
    for(i=0;i<clock_list_idx;i++)
    {
        if(clock_list[i].eventID == eventID)
        {
            p = &clock_list[i];
            break;
        }
            
    }
    return p;
}

///**
// * @fn      IsFound
// * @brief   找出相应结构体的位置，并返回 
// * @param   pClock
// * @return  point
// */
//static ListNode* IsFound(Clock_Struct *pClock)
//{
//    ListNode *p = NULL;
//    p = head;
//    while(p != NULL)
//    {
//        if(p->ClockStruct.eventID == pClock->eventID)
//            break;
//        p = p->next;
//    }
//    return p;
//}
/**
 * @fn      IsFound
 * @brief   找出相应结构体的位置，并返回 
 * @param   pClock
 * @return  point
 */
static Clock_Struct* IsFound(Clock_Struct *pClock)
{
    Clock_Struct *p = NULL;
    uint8_t i = 0;
    for(i=0;i<clock_list_idx;i++)
    {
        if(clock_list[i].eventID == pClock->eventID)
        {
            p = &clock_list[i];
            break;
        }
            
    }
    return p;
}
///**
// * @fn      Min
// * @brief   从链表中找出timeout最小的节点，并返回
// * @return  point
// */
//static ListNode* Min()
//{
//    ListNode *p,*q = NULL;
//    p = head;
//    q = head;
//
//    while(q != NULL)//q应该为一个未完成的定时器
//    {
//        if((taskEvents & q->ClockStruct.eventID) == 0)//找到一个未完成的定时器
//        {
//            break;
//        }
//        q = q->next;
//    }
//    p = q;
//    while(p->next != NULL)
//    {
//        if(q->ClockStruct.timeOut > p->next->ClockStruct.timeOut && (taskEvents & p->next->ClockStruct.eventID) == 0)
//        {
//            q = p->next;
//        }
//        p = p->next;
//    }
//    if(q->ClockStruct.timeOut < 2)//防止返回0后导致程序死机
//        q->ClockStruct.timeOut = 2;
//    return q;
//}
/**
 * @fn      Min
 * @brief   从链表中找出timeout最小的节点，并返回
 * @return  point
 */
static Clock_Struct* Min()
{
    uint32_t tmp = 0xFFFFFFFF;
    uint8_t i = 0;
    uint8_t j = 0;
    for(i=0;i<clock_list_idx;i++)
    {
        if((taskEvents & clock_list[i].eventID) == 0)
        {
            if(clock_list[i].timeOut < tmp)
            {
                tmp = clock_list[i].timeOut;
                j = i;
            }
        }
    }
    if(clock_list[j].timeOut < 2)
        clock_list[j].timeOut = 2;
    return &clock_list[j];

}

///**
// * @fn      MinusDiff
// * @brief   将链表中所有节点中的timeout都减去一个差值
// * @param   diff - 需要减去的差值
// * @return  NULL
// */
//static void MinusDiff(uint32_t diff)
//{
//    ListNode *p = head;
//    while(p != NULL)
//    {
//        p->ClockStruct.timeOut = p->ClockStruct.timeOut > diff?p->ClockStruct.timeOut - diff : 0;
//        p = p->next;
//    }
//}
/**
 * @fn      MinusDiff
 * @brief   将链表中所有节点中的timeout都减去一个差值
 * @param   diff - 需要减去的差值
 * @return  NULL
 */
static void MinusDiff(uint32_t diff)
{
    uint8_t i = 0;
    for(i=0;i<clock_list_idx;i++)
    {
        clock_list[i].timeOut = clock_list[i].timeOut > diff ? clock_list[i].timeOut - diff : 0;
    }
}
///**
// * @fn      FoundListTail
// * @brief   找到链表中的最后一个节点，并返回
// * @param   pClock
// * @return  point
// */
//static ListNode* FoundListTail()
//{
//    ListNode *p = head;
//    if(p == NULL)
//      return NULL;
//    
//    while(p->next != NULL)
//    {
//        p = p->next;
//    }
//    return p;
//}
///**
// * @fn      FoundListTail
// * @brief   找到链表中的最后一个节点，并返回
// * @param   pClock
// * @return  point
// */
//static Clock_Struct* FoundListTail()
//{
//    return &clock_list[clock_list_idx-1];
//}

///**
// * @fn      MinusDiff
// * @brief   从链表中删除一个节点 
// * @param   pClock
// * @return  ERROR or SUCCESS
// */
//static uint8_t TaskDelete(Clock_Struct *pClock)
//{
//    ListNode *p,*q = NULL;
//    p = head;
//    if(head == NULL) //如果head为NULL 之间返回错误
//        return ERROR;
//    
//    if(head->ClockStruct.eventID == pClock->eventID)//如果需要删除的就是head的节点
//    {
//        //将head后的下一个节点作为新的head
//        q = head->next; 
//        free(head);
//        head = q;
//        currentPoint = head; //当前节点为head
//        return SUCCESS;
//    }
//      
//    while(p->next != NULL)
//    {
//        if(p->next->ClockStruct.eventID == pClock->eventID)//在链表中找到对应的节点
//        {
//            //将后面的节点提上来
//            q = p->next->next;
//            free(p->next);
//            p->next = q;
//            currentPoint = FoundListTail();//当前节点为链表中的最后一个节点
//            break;
//        }
//        p = p->next;
//    }
//    return SUCCESS;
//}
/**
 * @fn      MinusDiff
 * @brief   从链表中删除一个节点 
 * @param   pClock
 * @return  ERROR or SUCCESS
 */
static uint8_t TaskDelete(Clock_Struct *pClock)
{
    if(clock_list_idx == 0) //如果head为NULL 之间返回错误
        return ERROR;
    
    for(uint8_t i=0;i<clock_list_idx;i++)
    {
        if(clock_list[i].eventID == pClock->eventID)
        {
            if(i == clock_list_idx-1)
            {
                memset(&clock_list[clock_list_idx-1], 0, sizeof(Clock_Struct));
            }
            else
            {
                memcpy(&clock_list[i], &clock_list[i+1], (clock_list_idx-i-1)*sizeof(Clock_Struct));
                memset(&clock_list[clock_list_idx-1], 0, sizeof(Clock_Struct));
            }
            clock_list_idx--;
//            currentPoint = &clock_list[clock_list_idx-1];
            return SUCCESS;
        }
    }
    return ERROR;
}
/*********************************************************************
* PUBLIC FUNCTIONS
*/

///**
// * @fn      User_StopClock
// * @brief   User_StopClock 
// * @param   pClock - 要取消的定时器
// * @return   
// */
//uint8_t User_StopClock(Clock_Struct *pClock)  
//{  
//    ListNode *p = IsFound(pClock);//在链表中先找到对应结构体的指针
//    if(p == NULL)
//    {
//        return ERROR;
//    }
//    else
//    {
//        HAL_TIM_Base_Stop_IT(&htim2);
//        
//        //计算距离上次启动定时器已经过了多久
//        uint32_t diff = User_Timer_Diff_ms(User_Timer_GetCurrentTime(), lastStartTimer_sysT32);
//        diff = diff > 0 ? diff : 0;
//        
//        MinusDiff(diff);//并将链表中的所有节点减去差值
//        
//        TaskDelete(pClock);//删除该节点
//        
//        if(ActivedTimerNum()!= 0)//删除节点以后，head若不为NULL，继续计算最小定时器并启动
//        {
//            currentLoadValue = Min()->ClockStruct.timeOut;
//            lastStartTimer_sysT32 = User_Timer_GetCurrentTime();
//              
//            User_Timer_Start(currentLoadValue);
//        }
//        return SUCCESS;
//    }
//    
//}  
/**
 * @fn      User_StopClock
 * @brief   User_StopClock 
 * @param   pClock - 要取消的定时器
 * @return   
 */
uint8_t User_StopClock(Clock_Struct *pClock)  
{  
    Clock_Struct *p = IsFound(pClock);//在链表中先找到对应结构体的指针
    if(p == NULL)
    {
        return ERROR;
    }
    else
    {
        HAL_TIM_Base_Stop_IT(&htim2);
        
        //计算距离上次启动定时器已经过了多久
        uint32_t diff = User_Timer_Diff_ms(User_Timer_GetCurrentTime(), lastStartTimer_sysT32);
        diff = diff > 0 ? diff : 0;
        
        MinusDiff(diff);//并将链表中的所有节点减去差值
        
        TaskDelete(pClock);//删除该节点
        
        if(ActivedTimerNum()!= 0)//删除节点以后，待完成的定时器数量
        {
            currentLoadValue = Min()->timeOut;
            lastStartTimer_sysT32 = User_Timer_GetCurrentTime();
              
            User_Timer_Start(currentLoadValue);
        }
//        if(!clock_list_idx)
        {
            event = event & (~TIM2_TIMEOUT_EVENT_T);
        }
        return SUCCESS;
    }
    
} 
/**
 * @fn      User_ResetClock
 * @brief   User_ResetClock 
 * @param   pClock - 需要重启的定时器结构体
 * @param   newTimeout - 如果定时器任务超时时间不变 填0或者原数值
 * @return   
 */
void User_ResetClock(Clock_Struct *pClock, uint32_t newTimeout) 
{
    User_StopClock(pClock);
    pClock->timeOut = newTimeout;
    User_StartClock(pClock);
}

///**
// * @fn      User_StartClock
// * @brief   User_StartClock 
// * @param   newClcok
// * @return  
// */
//uint8_t User_StartClock(Clock_Struct *newClock) 
//{  
//    if(newClock->eventID == 0 ||  newClock->timeOut == 0)
//        return ERROR;
//    
//    if(head == NULL)//若链表为空
//    {
//        head = (ListNode*)malloc(sizeof(ListNode)); //先创建头结点
//
//        if(NULL == head) //分配成功后才能对它进行操作
//            return ERROR;
//
//        memcpy(&(head->ClockStruct), newClock , sizeof(Clock_Struct));//将数据填充到节点
//        head->next = NULL;
//        currentPoint = head;//当前节点为head
//        
//        currentLoadValue = head->ClockStruct.timeOut;//当前装载值
//    }
//    else//若链表不为空，即已有定时器运行
//    {
//        ListNode *p = IsFound(newClock);//先寻找是否存在，存在的不处理
//        if(p == NULL)
//        {
//            ListNode *q = (ListNode*)malloc(sizeof(ListNode));
//            
//            //添加一个新节点
//            currentPoint->next = q;
//            currentPoint = currentPoint->next;
//            
//            if(NULL == head) //分配成功后才能对它进行操作
//                return ERROR;
//            
//            memcpy(&(currentPoint->ClockStruct), newClock , sizeof(Clock_Struct));
//            currentPoint->next = NULL;
//            
//            //计算记录上次启动已经过去多少时间，并将链表中的所有成员减去差值
//            uint32_t diff = User_Timer_Diff_ms(User_Timer_GetCurrentTime(), lastStartTimer_sysT32);
//            diff = diff > 0 ? diff : 0;
//            
////            MinusDiff(diff);
//            ListNode *p = head;
//            while(p != NULL)
//            {
//                p->ClockStruct.timeOut = p->ClockStruct.timeOut > diff?p->ClockStruct.timeOut - diff : 0;
//                p = p->next;
//                if(p->next == NULL)
//                    break;
//            }
//        }
//    }
//    
//    currentLoadValue = Min()->timeOut;//计算得最快到达的定时器，如为0也将返回1，防止死机
//    
//    lastStartTimer_sysT32 = User_Timer_GetCurrentTime();
//    
//    User_Timer_Stop();
//    User_Timer_Start(currentLoadValue);
//    return SUCCESS;
//}

/**
 * @fn      User_StartClock
 * @brief   User_StartClock 
 * @param   newClcok
 * @return  
 */
uint8_t User_StartClock(Clock_Struct *newClock) 
{  
    if(newClock->eventID == 0 ||  newClock->timeOut == 0)
        return ERROR;
    

    Clock_Struct *p = IsFound(newClock);//先寻找是否存在，存在的不处理
    if(p == NULL)
    {
        memcpy(&clock_list[clock_list_idx], newClock, sizeof(Clock_Struct));
        clock_list_idx++;
        
        if(clock_list_idx > 1)
        {
            //计算记录上次启动已经过去多少时间，并将链表中的所有成员减去差值
            uint32_t diff = User_Timer_Diff_ms(User_Timer_GetCurrentTime(), lastStartTimer_sysT32);
            diff = diff > 0 ? diff : 0;
            
            //MinusDiff(diff);
            for(uint8_t i=0;i<clock_list_idx-1;i++)
            {
                clock_list[i].timeOut = clock_list[i].timeOut > diff ? clock_list[i].timeOut - diff : 0;
            }
        }
    }
    
    currentLoadValue = Min()->timeOut;//计算得最快到达的定时器，如为0也将返回1，防止死机
    
    lastStartTimer_sysT32 = User_Timer_GetCurrentTime();
    
    User_Timer_Stop();
    User_Timer_Start(currentLoadValue);
    return SUCCESS;
}
/**
 * @fn      User_IsActive
 * @brief   User_IsActive 
 * @param   eventID - The ID to check
 * @return  ENABLE OR DISABLE
 */
uint8_t User_IsActive(Clock_Struct *pClock)  
{  
  return IsFound(pClock) == NULL? ERROR : SUCCESS;
}  


///**
// * @fn      User_TaskRemarks
// * @brief   User_TaskRemarks 
// * @return   
// */
//void User_TaskRemarks(void)  
//{  
//    ListNode *p = head;
//    uint32_t diff = currentLoadValue;
//    
//    while(p != NULL)
//    {
//        if(p->ClockStruct.timeOut)
//        {
//            //所有成员都减去之前的装载值，最小为0
//            p->ClockStruct.timeOut = p->ClockStruct.timeOut > diff ? p->ClockStruct.timeOut - diff : 0;
//            
//            if(p->ClockStruct.timeOut == 0)//若为0，标志该位
//            {
//                taskEvents |= p->ClockStruct.eventID;
//            }
//        }
//        p = p->next;
//    }
//    if(ActivedTimerNum() != 0)
//    {
//        currentLoadValue = /*TimeOut*/Min()->timeOut;
//        lastStartTimer_sysT32 = User_Timer_GetCurrentTime(); 
//        User_Timer_Start(currentLoadValue);
//    }
//    else
//    {
//        currentLoadValue = 0xFFFFFFFF;
//    }
//}  
/**
 * @fn      User_TaskRemarks
 * @brief   User_TaskRemarks 
 * @return   
 */
void User_TaskRemarks(void)  
{  
    uint32_t diff = currentLoadValue;
    
    for(uint8_t i=0;i<clock_list_idx;i++)
    {
        if(clock_list[i].timeOut)
        {
            clock_list[i].timeOut = clock_list[i].timeOut > diff ? clock_list[i].timeOut - diff : 0;
            if(clock_list[i].timeOut == 0)//若为0，标志该位
            {
                taskEvents |= clock_list[i].eventID;
            }
        }
    }
    if(ActivedTimerNum() != 0)
    {
        currentLoadValue = /*TimeOut*/Min()->timeOut;
        lastStartTimer_sysT32 = User_Timer_GetCurrentTime(); 
        User_Timer_Start(currentLoadValue);
    }
    else
    {
        currentLoadValue = 0xFFFFFFFF;
    }
}  
/**
 * @fn      User_TaskProcess
 * @brief   User_TaskProcess 
 * @return   
 */
void User_Timer_TaskProcess(void)  
{  
    if(taskEvents == 0)//若无事件，直接返回
      return;
    
    for(uint8_t i=0;i<32;i++)//循环查询哪个事件已经生成
    {
        if(taskEvents & (1 << i))
        {
            Clock_Struct *p = IsFound_Event(1 << i);//在链表中找到该节点
            
            TaskFunction_t TaskFunction = p->TaskHook;//缓存要执行的回调函数
            
            TaskDelete(p);//删除该节点
                       
            taskEvents &= ~(1 << i);//清除标志
            
            TaskFunction();//执行缓存的回调
        }
    }
}


//static uint8_t ActivedTimerNum(void)
//{
//    ListNode *p = head;
//    uint8_t num = 0;
//    while(p != NULL)
//    {
//        if((taskEvents & p->ClockStruct.eventID) == 0)
//        {
//            num++;
//        }
//        p = p->next;
//    }
//    return num;
//}
static uint8_t ActivedTimerNum(void)
{
    uint8_t num = 0;

    for(uint8_t i=0;i<clock_list_idx;i++)
    {
        if((taskEvents & clock_list[i].eventID) == 0)
        {
            num++;
        }
    }
    return num;
}
/*********************************************************************
*********************************************************************/
