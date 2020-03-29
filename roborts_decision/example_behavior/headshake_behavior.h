#ifndef ROBORTS_DECISION_HEADSHAKE_BEHAVIOR_H
#define ROBORTS_DECISION_HEADSHAKE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "../behavior_tree/behavior_node.h"
#include "../executor/gimbal_executor.h"


#include<roborts_msgs/GimbalAngle.h>

#include "line_iterator.h"

#define R_LIM -65*3.1415926/180
#define L_LIM 65*3.1415926/180
#define STEP 3*3.1415926/180

namespace roborts_decision {
class HeadshakeBehavior :public ActionNode {
 public:
	HeadshakeBehavior(GimbalExecutor*& gimbal_executor,
	Blackboard* &blackboard,
	const std::string & proto_file_path) :ActionNode("Headshake", pointer2ptr(blackboard)), blackboard_(blackboard), gimbal_executor_(gimbal_executor)
	{ 
    	cur_angle = 0;
		next_agl = 0;
		gimbal_angle_.yaw_mode=0;
		gimbal_angle_.pitch_mode=1;
		gimbal_angle_.yaw_angle=0;
		gimbal_angle_.pitch_angle=0.1;

	//ROS_INFO("-----------------%d-----------------",patrol_count_);

  }

  void Cancel()
  {
	    gimbal_executor_->Cancel();
  }

  virtual BehaviorState Update()
  {
	    return gimbal_executor_->Update();
  }

  virtual BehaviorState Run() {
	ros::NodeHandle nh_;
	ros::Publisher publisher = nh_.advertise<roborts_msgs::GimbalAngle>("/cmd_gimbal_angle",20);
	cur_angle = gimbal_angle_.yaw_angle;
	ROS_INFO("**%lf***********%lf**",step,next_agl);
	
		        if(cur_angle >= L_LIM)
			{
			    step = -1*STEP;
			}
			if(cur_angle <= R_LIM)
			{
			    step = 1*STEP;
			}
	next_agl = cur_angle + step;
	while ( next_agl >= L_LIM || next_agl <= R_LIM )
	{
	next_agl = next_agl + step;
	ROS_ERROR("**%lf***********%lf**",step,next_agl);
	break;
	}
	//ROS_INFO("**%lf***********%lf**",step,next_agl);
	gimbal_angle_.yaw_angle = next_agl;
	gimbal_executor_->Execute(gimbal_angle_);
	//publisher.publish(gimbal_angle_);
	return Update();
	}

 ~HeadshakeBehavior() = default;

private:
	GimbalExecutor* const gimbal_executor_;
	Blackboard* const blackboard_;

	double step=STEP;
	double cur_angle, next_agl;
	roborts_msgs::GimbalAngle gimbal_angle_;
};
}
#endif
