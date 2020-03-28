//
// Created by ClearLoveX on 2020-02-08.
//

#ifndef ROBORTS_DECISION_GO_TO_BUFF_BEHAVIOR_H
#define ROBORTS_DECISION_GO_TO_BUFF_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "../behavior_tree/behavior_node.h"

#include "line_iterator.h"

roborts_decision::Blackboard::Ptr transptr5(roborts_decision::Blackboard* &blackboard){
    roborts_decision::Blackboard::Ptr blackboard_ptr;
    blackboard_ptr.reset(blackboard);
    return blackboard_ptr;
}

namespace roborts_decision {
    class GoToBuffBehavior:public ActionNode {
    public:
        GoToBuffBehavior(ChassisExecutor* &chassis_executor,
                         Blackboard* &blackboard,
                         const std::string & proto_file_path) : ActionNode("go_to_buff", transptr5(blackboard)), chassis_executor_(chassis_executor),
                                                                blackboard_(blackboard) {

            buff_count_ = 0;
            buff_size_ = 0;

            if (!LoadParam(proto_file_path)) {
                ROS_ERROR("%s can't open file", __FUNCTION__);
            }

        }

        void Run() {

            auto executor_state = Update();

            std::cout << "state:" << (int)(executor_state) << std::endl;

            if (executor_state != BehaviorState::RUNNING) {

                if (buff_goals_.empty()) {
                    ROS_ERROR("buff goal is empty");
                    return;
                }

                std::cout << "send goal" << std::endl;
                chassis_executor_->Execute(buff_goals_[buff_count_]);
                buff_count_ = ++buff_count_ % buff_size_;

            }
        }

        void Cancel() {
            chassis_executor_->Cancel();
        }

        BehaviorState Update() {
            return chassis_executor_->Update();
        }

        bool LoadParam(const std::string &proto_file_path) {
            roborts_decision::DecisionConfig decision_config;
            if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
                return false;
            }

            buff_size_ = (unsigned int)(decision_config.buff_point().size());
            buff_goals_.resize(buff_size_);
            for (int i = 0; i != buff_size_; i++) {//Ω´…Ë÷√∫√µƒº∏∏ˆµ„£¨◊™ªª≥…msg
                buff_goals_[i].header.frame_id = "map";
                buff_goals_[i].pose.position.x = decision_config.buff_point(i).x();
                buff_goals_[i].pose.position.y = decision_config.buff_point(i).y();
                buff_goals_[i].pose.position.z = decision_config.buff_point(i).z();

                tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.buff_point(i).roll(),
                                                                        decision_config.buff_point(i).pitch(),
                                                                        decision_config.buff_point(i).yaw());

                buff_goals_[i].pose.orientation.x = quaternion.x();
                buff_goals_[i].pose.orientation.y = quaternion.y();
                buff_goals_[i].pose.orientation.z = quaternion.z();
                buff_goals_[i].pose.orientation.w = quaternion.w();
            }

            return true;
        }

        ~GoToBuffBehavior() = default;

    private:
        //! executor
        ChassisExecutor* const chassis_executor_;

        //! perception information
        Blackboard* const blackboard_;

        //! buff buffer
        std::vector<geometry_msgs::PoseStamped> buff_goals_;
        unsigned int buff_count_;
        unsigned int buff_size_;

    };
}
#endif //ROBORTS_DECISION_GO_TO_BUFF_BEHAVIOR_H


