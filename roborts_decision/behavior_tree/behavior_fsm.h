//
// Created by ClearLoveX on 2020-02-08.
//

#ifndef ROBORTS_BEHAVIOR_FSM_H
#define ROBORTS_BEHAVIOR_FSM_H

#include "Blackboard.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <fstream>
#include <string>
#include <sstream>
#include "behavior_node.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"

// 事件枚举
enum Event{
    event0 = 0,
    event1 = 1,
    event2 = 2,
    event3 = 3,
    event4 = 4,
    event5 = 5,
    event7 = 7,
};

// 状态枚举
enum State{
    state0 = 0,
    state1 = 1,
    state2 = 2,
    state3 = 3,
    state4 = 4,
};

auto chassis_executor = new roborts_decision::ChassisExecutor;
auto gimbal_executor = new roborts_decision::GimbalExecutor;

//行为节点
//search节点
std::shared_ptr<SearchBehavior> search_behavior_1(new SearchBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<SearchBehavior> search_behavior_2(new SearchBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<SearchBehavior> search_behavior_3(new SearchBehavior(chassis_executor, blackboard, full_path));
//chase节点
std::shared_ptr<ChaseBehavior> chase_behavior_1(new ChaseBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<ChaseBehavior> chase_behavior_2(new ChaseBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<ChaseBehavior> chase_behavior_3(new ChaseBehavior(chassis_executor, blackboard, full_path));
//escape节点
std::shared_ptr<EscapeBehavior> escape_behavior_1(new EscapeBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<EscapeBehavior> escape_behavior_2(new EscapeBehavior(chassis_executor, blackboard, full_path));
//gotobuff节点
std::shared_ptr<GoToBuffBehavior> go_to_buff_behavior_1(new GoToBuffBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<GoToBuffBehavior> go_to_buff_behavior_2(new GoToBuffBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<GoToBuffBehavior> go_to_buff_behavior_3(new GoToBuffBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<GoToBuffBehavior> go_to_buff_behavior_4(new GoToBuffBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<GoToBuffBehavior> go_to_buff_behavior_5(new GoToBuffBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<GoToBuffBehavior> go_to_buff_behavior_6(new GoToBuffBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<GoToBuffBehavior> go_to_buff_behavior_7(new GoToBuffBehavior(chassis_executor, blackboard, full_path));
std::shared_ptr<GoToBuffBehavior> go_to_buff_behavior_8(new GoToBuffBehavior(chassis_executor, blackboard, full_path));
//
std::shared_ptr<RotateBehavior> rotate_behavior(new RotateBehavior(chassis_executor, blackboard, full_path));

//组合节点
//select节点
std::shared_ptr<SelectorNode> S1(new SelectorNode("S1", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S2(new SelectorNode("S2", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S3(new SelectorNode("S3", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S4(new SelectorNode("S4", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S5(new SelectorNode("S5", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S6(new SelectorNode("S6", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S7(new SelectorNode("S7", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S8(new SelectorNode("S8", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S9(new SelectorNode("S9", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S10(new SelectorNode("S10", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S11(new SelectorNode("S11", pointer2ptr(blackboard)));
std::shared_ptr<SelectorNode> S12(new SelectorNode("S12", pointer2ptr(blackboard)));

//先决条件节点
//precondition节点
std::shared_ptr<PreconditionNode> Pre1(new PreconditionNode("Pre1", pointer2ptr(blackboard), blackboard.get_buff, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre2(new PreconditionNode("Pre2", pointer2ptr(blackboard), blackboard.get_bullet, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre3(new PreconditionNode("Pre3", pointer2ptr(blackboard), blackboard.get_blood, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre4(new PreconditionNode("Pre4", pointer2ptr(blackboard), blackboard.get_buff, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre5(new PreconditionNode("Pre5", pointer2ptr(blackboard), blackboard.get_bullet, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre6(new PreconditionNode("Pre6", pointer2ptr(blackboard), blackboard.get_blood, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre7(new PreconditionNode("Pre7", pointer2ptr(blackboard), blackboard.beyond_threat, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre8(new PreconditionNode("Pre8", pointer2ptr(blackboard), blackboard.beyond_threat_difference, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre9(new PreconditionNode("Pre9", pointer2ptr(blackboard), blackboard.get_buff, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre10(new PreconditionNode("Pre10", pointer2ptr(blackboard), blackboard.get_bullet, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre11(new PreconditionNode("Pre11", pointer2ptr(blackboard), blackboard.get_blood, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre12(new PreconditionNode("Pre12", pointer2ptr(blackboard), blackboard.beyond_threat, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre13(new PreconditionNode("Pre13", pointer2ptr(blackboard), blackboard.beyond_threat_difference, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre14(new PreconditionNode("Pre14", pointer2ptr(blackboard), blackboard.get_buff, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre15(new PreconditionNode("Pre15", pointer2ptr(blackboard), blackboard.get_bullet, AbortType::SELF));
std::shared_ptr<PreconditionNode> Pre16(new PreconditionNode("Pre16", pointer2ptr(blackboard), blackboard.get_blood, AbortType::SELF));

typedef struct ACT_TABLE_T
{
    int Trans_Id;
    void (*Func)(void *);
}ACT_TABLE_T;

typedef struct STATE_TABLE_T
{
    int State_Id; // 0-防御1 1-攻击 2-搜寻 3-防御2 4-旋转跳跃
    int Trans;
    ACT_TABLE_T *Trans_Table;
    void (*Fun_Start)(void); // 状态对应行为树执行函数
    void (*Fun_Cancel)(void); //状态对应行为树终止函数
}STATE_TABLE_T;

typedef struct FSM_T
{
    int Cur_State_Id; //当前状态id
    int State_Num; //状态的数量
    STATE_TABLE_T* FsmTable; //状态机对应的状态表
    int State_Table_Size; //状态表的大小
}FSM_T;

void FSM_MoveState(FSM_T* pFsm,int state)
{
    pFsm->Cur_State_Id = state;
    return;
}
/* 状态处理函数
   event:
   0 - 我方机器人威胁值低于或等于敌方机器人
   1 - 我方机器人威胁值高于敌方机器人
   2 - 未检测/观测到敌方机器人
   3 - 检测/观测到敌方机器人
   4 - 未受到攻击
   5 - 受到攻击
*/
void state0_event1_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 1);
}

void state0_event2_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 3);
}

void state1_event0_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 0);
}

void state1_event2_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 2);
}

void state2_event0_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 3);
}

void state2_event3_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 1);
}

void state2_event5_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 4);
}

void state3_event3_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 0);
}

void state3_event5_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 4);
}

void state4_event3_fun(void *pFsm)
{
    FSM_MoveState((FSM_T*) pFsm, 0);
}

// 状态对应行为函数
void state0_behavior()
{
    //调用该状态下行为树的start节点
    S3->Run();
}

void state1_behavior()
{
    S5->Run();
}

void state2_behavior()
{
    S9->Run();
}

void state3_behavior()
{
    S1->Run();
}

void state4_behavior()
{
    rotate_behavior->Run();
}

// 状态下行为树取消函数
void state0_cancel()
{
    //停止该状态下行为树的执行节点

}

void state1_cancel()
{

}

void state2_cancel()
{

}

void state3_cancel()
{

}

void state4_cancel()
{

}

// 状态动作表 0-4
ACT_TABLE_T State0_Act_Table[] = {
        {event1, state0_event1_fun},
        {event2, state0_event2_fun}
};

ACT_TABLE_T State1_Act_Table[] = {
        {event0, state1_event0_fun},
        {event2, state1_event2_fun}
};

ACT_TABLE_T State2_Act_Table[] = {
        {event0, state2_event0_fun},
        {event3, state2_event3_fun},
        {event5, state2_event5_fun}
};

ACT_TABLE_T State3_Act_Table[] = {
        {event3, state3_event3_fun},
        {event5, state3_event5_fun}
};

ACT_TABLE_T State4_Act_Table[] = {
        {event3, state4_event3_fun}
};

// 状态表
STATE_TABLE_T Fsm_Table[] = {
        {state0, 2, State0_Act_Table, state0_behavior, state0_cancel},
        {state1, 2, State1_Act_Table, state1_behavior, state1_cancel},
        {state2, 3, State2_Act_Table, state2_behavior, state2_cancel},
        {state3, 2, State3_Act_Table, state3_behavior, state3_cancel},
        {state4, 1, State4_Act_Table, state4_behavior, state4_cancel}
};

// 状态表注册函数
void FSM_Regist(FSM_T* pFsm, STATE_TABLE_T* pStateTable, int state_num)
{
    pFsm->FsmTable =  pStateTable;
    pFsm->State_Table_Size = sizeof(*pStateTable)/sizeof(STATE_TABLE_T);
    pFsm->State_Num = state_num;
    return;
}

// 返回当前状态动作表
/*ACT_TABLE_T* FSM_Get_Act_Table(FSM_T* pFsm)
{
    return pFsm->FsmTable[pFsm->Cur_State_Id].Trans_Table;
}*/

typedef void (*ActFun) (void*);

// 事件处理函数
void FSM_Event_Handle(FSM_T* pFsm, int event, int i)
{
    // 执行行为
    if(event != 7)
    { //跳转条件满足
        pFsm->FsmTable[pFsm->Cur_State_Id].Fun_Cancel();
        pFsm->FsmTable[pFsm->Cur_State_Id].Trans_Table[i].Func(pFsm);
    }
    else
    { //跳转条件不满足
        pFsm->FsmTable[pFsm->Cur_State_Id].Fun_Start();
    }
}

// Begin函数开启状态机流程
void FSM_Begin(FSM_T* pFsm, int cur_state_id)
{
    pFsm->Cur_State_Id = cur_state_id;
}

/* event判断函数
 * 状态处理函数
 * event:
 * 0 - 我方机器人威胁值低于或等于敌方机器人
 * 1 - 我方机器人威胁值高于敌方机器人
 * 2 - 未检测/观测到敌方机器人
 * 3 - 检测/观测到敌方机器人
 * 4 - 未受到攻击
 * 5 - 受到攻击
 * 7 - 不用跳转
*/

// 我方机器人的威胁值是否高于敌方机器人，是则返回事件1；不是则返回事件0
int event_judge_threat(Blackboard blackboard_)
{
    if(blackboard_.get_threat() <= blackboard_.get_enemy_threat())
    {
        return 0;
    }
    else
        return 1;
}

// 主视野是否观测到敌方机器人，观测到返回事件3，未观测到返回事件2
int event_judge_detect(Blackboard blackboard_)
{
    if(blackboard_.enemy_detect)
        return 3;
    return 2;
}

// 机器人是否受到攻击，受到攻击返回事件5，没有受到攻击返回事件4
int event_judge_attack(Blackboard blackboard_)
{
    if(blackboard_.under_attack)
        return 5;
    return 4;
}

// event判断函数指针数组
int (*judge_fun[3])(Blackboard)={event_judge_detect, event_judge_attack, event_judge_threat};

// event判断函数，返回值为event
int event_judge(FSM_T pFsm, Blackboard blackboard_, int* j)
{
    int event = 0;
    int i = 0;
    for((*j)=0; i < 3; i++)
    {
        event = judge_fun[i](blackboard_); //第i个判断函数的判断结果
        for(; (*j) < pFsm.FsmTable[pFsm.Cur_State_Id].Trans; (*j)+=1 )
        {//该状态的行为表的第j行的转换事件id是否和判断结果相同
            if(event == pFsm.FsmTable[pFsm.Cur_State_Id].Trans_Table[*j].Trans_Id)
                return event;
            else
                return event7;
        }
    }
}

// 定时执行函数
void timer_start(std::function<void(FSM_T*, Blackboard, int*)> func, unsigned int interval, FSM_T* pfsm, Blackboard blackboard_, int* i)
{
    std::thread([func, interval, pfsm, blackboard_, i]() {
        while (true)
        {
            func(pfsm, blackboard_, i);
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        }
    }).detach();
}

// event处理函数
void handle(FSM_T* pfsm, Blackboard blackboard_, int* j)
{
    int event = event_judge(*pfsm, blackboard_, j);
    FSM_Event_Handle(pfsm, event, *j);
}

//读取参数文件函数
void read_param(std::ifstream &file,Blackboard* blackboard_)
{
    std::string strLine;
    std::string param[6];
    getline(file,strLine);
    std::istringstream is(strLine);
    is >> param[0] >> param[1] >> param[2] >> param[3] >> param[4] >> param[5];
    blackboard_->bullet = atoi(param[0].c_str());
    blackboard_->enemy_bullet = atoi(param[1].c_str());
    blackboard_->blood = atoi(param[2].c_str());
    blackboard_->enemy_blood = atoi(param[3].c_str());
    blackboard_->enemy_detect = atoi(param[4].c_str());
    blackboard_->under_attack = atoi(param[5].c_str());
}

void timer_start_read(std::function<void(std::ifstream &, Blackboard*)> func, unsigned int interval, std::ifstream &file, Blackboard* blackboard_)
{
    std::thread([func, interval, &file, blackboard_]() {
        while (true)
        {
            func(file, blackboard_);
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        }
    }).detach();
}

#endif //ROBORTS_BEHAVIOR_FSM_H
