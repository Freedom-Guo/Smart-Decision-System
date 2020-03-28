#ifndef ROBORTS_DECISION_ROTATE_BEHAVIOR_H
#define ROBORTS_DECISION_ROTATE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "../behavior_tree/behavior_node.h"

#include "line_iterator.h"

#include "stdio.h"
#include "time.h"

#define R_LIM -70*3.1415926/180
#define L_LIM 70*3.1415926/180

roborts_decision::Blackboard::Ptr transptr6(roborts_decision::Blackboard* &blackboard){
    roborts_decision::Blackboard::Ptr blackboard_ptr;
    blackboard_ptr.reset(blackboard);
    return blackboard_ptr;
}

namespace roborts_decision {
    class RotateBehavior : public ActionNode {
    public:
        RotateBehavior(ChassisExecutor*& chassis_executor,
                       Blackboard*& blackboard,
                       const std::string& proto_file_path) :ActionNode("rotate", transptr6(blackboard))chassis_executor_(chassis_executor),
                                                            blackboard_(blackboard) {

            cur_angle = 0;
            next_agl = 0;

            if (!LoadParam(proto_file_path)) {
                ROS_ERROR("%s can't open file", __FUNCTION__);//ªπ «“ª∏ˆœﬂ≥ÃŒ™ø’æÕ±®¥Ì
            }
        }
        void Run() {

            auto executor_state = Update();

            srand((unsigned)time(NULL));
            int num2;
            num2 = pow(-1,rand() % 2 + 1);
            num1 = num1 + num2;

            rotate_accel_.twist.angular.x = 0;
            rotate_accel_.twist.angular.y = 0;
            rotate_accel_.twist.angular.z = 5 *  num2;
            chassis_executor_->Execute(rotate_accel_);
        }

        void Cancel() {
            chassis_executor_->Cancel();
        }

        BehaviorState Update() {
            return chassis_executor_->Update();
        }

        bool LoadParam(const std::string& proto_file_path) {
            roborts_decision::DecisionConfig decision_config;
            if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
                return false;
            }
            return true;
        }

        ~RotateBehavior() = default;
    private:
        //! executor
        ChassisExecutor* const chassis_executor_;

        //! perception information
        Blackboard* const blackboard_;

        //! chase goal
        geometry_msgs::PoseStamped rotate_goal_;
        roborts_msgs::TwistAccel rotate_accel_;
        //! cancel flag
        bool cancel_goal_;

        double cur_angle, next_agl;
    };
}
#endif //ROBORTS_DECISION_ROTATE_BEHAVIOR_H
