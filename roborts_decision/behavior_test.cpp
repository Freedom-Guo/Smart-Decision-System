#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/rotate_behavior.h"
#include "example_behavior/go_to_buff_behavior.h"
#include "behavior_tree/behavior_fsm.h"

using namespace std;
using namespace roborts_decision;

Blackboard::Ptr pointer2ptr(roborts_decision::Blackboard* &blackboard){
    roborts_decision::Blackboard::Ptr blackboard_ptr;
    blackboard_ptr.reset(blackboard);
    return blackboard_ptr;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
  blackboard = new roborts_decision::Blackboard(full_path);

  //子节点列表
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S1_child_node_ptr_list = {Pre1, escape_behavior_1};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S2_child_node_ptr_list = {Pre2, Pre3};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S3_child_node_ptr_list = {Pre4, escape_behavior_2};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S4_child_node_ptr_list = {Pre5, Pre6};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S5_child_node_ptr_list = {Pre7, S6};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S6_child_node_ptr_list = {Pre8, S7};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S7_child_node_ptr_list = {Pre9, chase_behavior_3};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S8_child_node_ptr_list = {Pre10, Pre11};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S9_child_node_ptr_list = {Pre12, S10};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S10_child_node_ptr_list = {Pre13, S11};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S11_child_node_ptr_list = {Pre14, search_behavior_3};
  std::initializer_list<roborts_decision::BehaviorNode::Ptr> S12_child_node_ptr_list = {Pre15, Pre16};

  S1->AddChildren(S1_child_node_ptr_list);
  Pre1->SetChild(S2);
  S2->AddChildren(S2_child_node_ptr_list);
  Pre2->SetChild(go_to_buff_behavior_1);
  Pre3->SetChild(go_to_buff_behavior_2);

  S3->AddChildren(S3_child_node_ptr_list);
  Pre4->SetChild(S4);
  S4->AddChildren(S4_child_node_ptr_list);
  Pre5->SetChild(go_to_buff_behavior_3);
  Pre6->SetChild(go_to_buff_behavior_4);

  S5->AddChildren(S5_child_node_ptr_list);
  Pre7->SetChild(chase_behavior_1);
  S6->AddChildren(S6_child_node_ptr_list);
  Pre8->SetChild(chase_behavior_2);
  S7->AddChildren(S7_child_node_ptr_list);
  Pre9->SetChild(S8);
  S9->AddChildren(S9_child_node_ptr_list);
  Pre10->AddChildren(go_to_buff_behavior_5);
  Pre11->AddChildren(go_to_buff_behavior_6);

  S9->AddChildren(S9_child_node_ptr_list);
  Pre12->SetChild(search_behavior_1);
  S10->AddChildren(S10_child_node_ptr_list);
  Pre13->SetChild(search_behavior_2);
  S11->AddChildren(S11_child_node_ptr_list);
  Pre14->SetChild(S12);
  S12->AddChildren(S12_child_node_ptr_list);
  Pre15->SetChild(go_to_buff_behavior_7);
  Pre16->SetChild(go_to_buff_behavior_8);

  return 0;
}

