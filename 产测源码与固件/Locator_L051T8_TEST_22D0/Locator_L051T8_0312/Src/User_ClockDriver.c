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
#include <stdio.h>
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
//�����head
static ListNode *head = NULL;
//ָ����������һ���ڵ�
static ListNode *currentPoint = NULL;
//��ǰ��ʱ����װ�ص�timeout
static uint32_t currentLoadValue = 0xFFFFFFFF;
//���һ��������ʱ��ʱ��ȡ��һ��ϵͳʱ��ʱ��
static uint32_t lastStartTimer_sysT32 = 0;
//��ʱ�������¼�
static uint32_t taskEvents = 0;

/*********************************************************************
* User Timer API
*/
/*
 * @fn      User_Timer_Start
 * @brief   ��Ҫ�ⲿ����һ��������ʱ���������ɸ���mcu��ͬ�Լ��޸�
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
 * @brief   ��Ҫ�ⲿ����һ��ֹͣ��ʱ���������ɸ���mcu��ͬ�Լ��޸�
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
 * @brief   ��Ҫ�ⲿ����һ����ȡϵͳʱ��ĺ������������ڻ�ȡ2��ʱ��֮��ĺ����
 * @param   timeOut
 * @return   
 */
uint32_t User_Timer_GetCurrentTime()
{
    return HAL_GetTick();
}
/*
 * @fn      User_Timer_Diff_ms()
 * @brief   ���ڻ�ȡ2��ʱ��֮��ĺ����
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
//��������ɾ��һ���ڵ�
static uint8_t TaskDelete(Clock_Struct *pClock);
//���������ҳ�timeout��С�Ľڵ㣬������
static ListNode* Min();
//�����������нڵ��е�timeout����ȥһ����ֵ
static void MinusDiff(uint32_t diff);
//�ҵ������е����һ���ڵ㣬������
static ListNode* FoundListTail();

static uint8_t ActivedTimerNum(void);

/**
 * @fn      IsFound_Event
 * @brief   �ҳ���Ӧ�¼��Ľṹ��λ�ã������� 
 * @param   pClock
 * @return  point
 */
static ListNode* IsFound_Event(uint32_t eventID)
{
    ListNode *p = NULL;
    p = head;
    while(p != NULL)
    {
        if(p->ClockStruct.eventID == eventID)
            break;
        p = p->next;
    }
    return p;
}
/**
 * @fn      IsFound
 * @brief   �ҳ���Ӧ�ṹ���λ�ã������� 
 * @param   pClock
 * @return  point
 */
static ListNode* IsFound(Clock_Struct *pClock)
{
    ListNode *p = NULL;
    p = head;
    while(p != NULL)
    {
        if(p->ClockStruct.eventID == pClock->eventID)
            break;
        p = p->next;
    }
    return p;
}
///**
// * @fn      Min
// * @brief   ���������ҳ�timeout��С�Ľڵ㣬������
// * @return  point
// */
//static ListNode* Min()
//{
//    ListNode *p,*q = NULL;
//    p = head;
//    q = head;
//    
//    if(p->ClockStruct.timeOut == 0)//�Ƚ�����������0��timeout����Ϊ1����ֹ����0���³�������
//        p->ClockStruct.timeOut = 1;
//#warning ��һ�����ʱ��BUG,��task�����йأ����޳�������¼�    
//    while(p->next != NULL)
//    {
//        if(p->next->ClockStruct.timeOut == 0)
//            p->next->ClockStruct.timeOut = 1;
//        if(p->ClockStruct.timeOut > p->next->ClockStruct.timeOut)
//        {
//            q = p->next;
//        }
//        p = p->next;
//    }
//    return q;
//}

/**
 * @fn      Min
 * @brief   ���������ҳ�timeout��С�Ľڵ㣬������
 * @return  point
 */
static ListNode* Min()
{
    ListNode *p,*q = NULL;
    p = head;
    q = head;

    while(q != NULL)//qӦ��Ϊһ��δ��ɵĶ�ʱ��
    {
        if((taskEvents & q->ClockStruct.eventID) == 0)//�ҵ�һ��δ��ɵĶ�ʱ��
        {
            break;
        }
        q = q->next;
    }
    p = q;
    while(p->next != NULL)
    {
        if(q->ClockStruct.timeOut > p->next->ClockStruct.timeOut && (taskEvents & p->next->ClockStruct.eventID) == 0)
        {
            q = p->next;
        }
        p = p->next;
    }
    if(q->ClockStruct.timeOut == 0)//��ֹ����0���³�������
        q->ClockStruct.timeOut = 1;
    return q;
}

//static ListNode* TimeOutMin()
//{
//    ListNode *p,*q = NULL;
//    p = head;
//    q = head;
//    
//    if(p->ClockStruct.timeOut == 0)//�Ƚ�����������0��timeout����Ϊ1����ֹ����0���³�������
//        p->ClockStruct.timeOut = 1;
//    
//    while(p->next != NULL)
//    {
//        if(p->next->ClockStruct.timeOut == 0)
//            p->next->ClockStruct.timeOut = 1;
//        
//        if(p->ClockStruct.timeOut != 1)
//        {
//            if(p->ClockStruct.timeOut < p->next->ClockStruct.timeOut)
//            {
//                q = p;
//            }
//            else if(p->next->ClockStruct.timeOut != 1)
//            {
//                q = p->next;
//            }
//        }
//        else
//        {
//            q = p->next;
//        }
//        p = p->next;
//    }
//    
//    return q;
//}

/**
 * @fn      MinusDiff
 * @brief   �����������нڵ��е�timeout����ȥһ����ֵ
 * @param   diff - ��Ҫ��ȥ�Ĳ�ֵ
 * @return  NULL
 */
static void MinusDiff(uint32_t diff)
{
    ListNode *p = head;
    while(p != NULL)
    {
        p->ClockStruct.timeOut = p->ClockStruct.timeOut > diff?p->ClockStruct.timeOut - diff : 0;
        p = p->next;
    }
}
/**
 * @fn      FoundListTail
 * @brief   �ҵ������е����һ���ڵ㣬������
 * @param   pClock
 * @return  point
 */
static ListNode* FoundListTail()
{
    ListNode *p = head;
    if(p == NULL)
      return NULL;
    
    while(p->next != NULL)
    {
        p = p->next;
    }
    return p;
}
/**
 * @fn      MinusDiff
 * @brief   ��������ɾ��һ���ڵ� 
 * @param   pClock
 * @return  ERROR or SUCCESS
 */
static uint8_t TaskDelete(Clock_Struct *pClock)
{
    ListNode *p,*q = NULL;
    p = head;
    if(head == NULL) //���headΪNULL ֮�䷵�ش���
        return ERROR;
    
    if(head->ClockStruct.eventID == pClock->eventID)//�����Ҫɾ���ľ���head�Ľڵ�
    {
        //��head�����һ���ڵ���Ϊ�µ�head
        q = head->next; 
        free(head);
        head = q;
        currentPoint = head; //��ǰ�ڵ�Ϊhead
        return SUCCESS;
    }
      
    while(p->next != NULL)
    {
        if(p->next->ClockStruct.eventID == pClock->eventID)//���������ҵ���Ӧ�Ľڵ�
        {
            //������Ľڵ�������
            q = p->next->next;
            free(p->next);
            p->next = q;
            currentPoint = FoundListTail();//��ǰ�ڵ�Ϊ�����е����һ���ڵ�
            break;
        }
        p = p->next;
    }
    return SUCCESS;
}
/*********************************************************************
* PUBLIC FUNCTIONS
*/

/**
 * @fn      User_StopClock
 * @brief   User_StopClock 
 * @param   pClock - Ҫȡ���Ķ�ʱ��
 * @return   
 */
uint8_t User_StopClock(Clock_Struct *pClock)  
{  
    ListNode *p = IsFound(pClock);//�����������ҵ���Ӧ�ṹ���ָ��
    if(p == NULL)
    {
        return ERROR;
    }
    else
    {
        HAL_TIM_Base_Stop_IT(&htim2);
        
        //��������ϴ�������ʱ���Ѿ����˶��
        uint32_t diff = User_Timer_Diff_ms(User_Timer_GetCurrentTime(), lastStartTimer_sysT32);
        diff = diff > 0 ? diff : 0;
        
        MinusDiff(diff);//���������е����нڵ��ȥ��ֵ
        
        TaskDelete(pClock);//ɾ���ýڵ�
        
        if(ActivedTimerNum()!= 0)//ɾ���ڵ��Ժ�head����ΪNULL������������С��ʱ��������
        {
            currentLoadValue = Min()->ClockStruct.timeOut;
            lastStartTimer_sysT32 = User_Timer_GetCurrentTime();
              
            User_Timer_Start(currentLoadValue);
        }
        return SUCCESS;
    }
    
}  

/**
 * @fn      User_ResetClock
 * @brief   User_ResetClock 
 * @param   pClock - ��Ҫ�����Ķ�ʱ���ṹ��
 * @param   newTimeout - �����ʱ������ʱʱ�䲻�� ��0����ԭ��ֵ
 * @return   
 */
void User_ResetClock(Clock_Struct *pClock, uint32_t newTimeout) 
{
    User_StopClock(pClock);
    pClock->timeOut = newTimeout;
    User_StartClock(pClock);
}

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
    
    if(head == NULL)//������Ϊ��
    {
        head = (ListNode*)malloc(sizeof(ListNode)); //�ȴ���ͷ���

        if(NULL == head) //����ɹ�����ܶ������в���
            return ERROR;

        memcpy(&(head->ClockStruct), newClock , sizeof(Clock_Struct));//��������䵽�ڵ�
        head->next = NULL;
        currentPoint = head;//��ǰ�ڵ�Ϊhead
        
        currentLoadValue = head->ClockStruct.timeOut;//��ǰװ��ֵ
    }
    else//������Ϊ�գ������ж�ʱ������
    {
        ListNode *p = IsFound(newClock);//��Ѱ���Ƿ���ڣ����ڵĲ�����
        if(p == NULL)
        {
            ListNode *q = (ListNode*)malloc(sizeof(ListNode));
            
            //���һ���½ڵ�
            currentPoint->next = q;
            currentPoint = currentPoint->next;
            
            if(NULL == head) //����ɹ�����ܶ������в���
                return ERROR;
            
            memcpy(&(currentPoint->ClockStruct), newClock , sizeof(Clock_Struct));
            currentPoint->next = NULL;
            
            //�����¼�ϴ������Ѿ���ȥ����ʱ�䣬���������е����г�Ա��ȥ��ֵ
            uint32_t diff = User_Timer_Diff_ms(User_Timer_GetCurrentTime(), lastStartTimer_sysT32);
            diff = diff > 0 ? diff : 0;
            
//            MinusDiff(diff);
            ListNode *p = head;
            while(p != NULL)
            {
                p->ClockStruct.timeOut = p->ClockStruct.timeOut > diff?p->ClockStruct.timeOut - diff : 0;
                p = p->next;
                if(p->next == NULL)
                    break;
            }
        }
    }
    
    currentLoadValue = Min()->ClockStruct.timeOut;//�������쵽��Ķ�ʱ������Ϊ0Ҳ������1����ֹ����
    
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


/**
 * @fn      User_TaskRemarks
 * @brief   User_TaskRemarks 
 * @return   
 */
void User_TaskRemarks(void)  
{  
    ListNode *p = head;
    uint32_t diff = currentLoadValue;
    
    while(p != NULL)
    {
        if(p->ClockStruct.timeOut)
        {
            //���г�Ա����ȥ֮ǰ��װ��ֵ����СΪ0
            p->ClockStruct.timeOut = p->ClockStruct.timeOut > diff ? p->ClockStruct.timeOut - diff : 0;
            
            if(p->ClockStruct.timeOut == 0)//��Ϊ0����־��λ
            {
                taskEvents |= p->ClockStruct.eventID;
            }
        }
        p = p->next;
    }
    if(ActivedTimerNum() != 0)
    {
        currentLoadValue = /*TimeOut*/Min()->ClockStruct.timeOut;
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
    if(taskEvents == 0)//�����¼���ֱ�ӷ���
      return;
    
    for(uint8_t i=0;i<32;i++)//ѭ����ѯ�ĸ��¼��Ѿ�����
    {
        if(taskEvents & (1 << i))
        {
            ListNode *p = IsFound_Event(1 << i);//���������ҵ��ýڵ�
            
            TaskFunction_t TaskFunction = p->ClockStruct.TaskHook;//����Ҫִ�еĻص�����
            
            TaskDelete(&(p->ClockStruct));//ɾ���ýڵ�
                       
            taskEvents &= ~(1 << i);//�����־
            
            TaskFunction();//ִ�л���Ļص�
        }
    }
}


static uint8_t ActivedTimerNum(void)
{
    ListNode *p = head;
    uint8_t num = 0;
    while(p != NULL)
    {
        if((taskEvents & p->ClockStruct.eventID) == 0)
        {
            num++;
        }
        p = p->next;
    }
    return num;
}
/*********************************************************************
*********************************************************************/
