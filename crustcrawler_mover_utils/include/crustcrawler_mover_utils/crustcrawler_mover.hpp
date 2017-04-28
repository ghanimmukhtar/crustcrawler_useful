#ifndef _CRUSTCRAWLER_MOVER_HPP
#define _CRUSTCRAWLER_MOVER_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <memory>
#include <boost/bind.hpp>

#include <crustcrawler_mover_utils/parameters.hpp>
#include <crustcrawler_mover_utils/helpers_methods.hpp>

namespace crustcrawler_mover {
class CRUSTCRAWLER_Mover {
public:
    typedef std::shared_ptr<CRUSTCRAWLER_Mover> Ptr;
    typedef const std::shared_ptr<CRUSTCRAWLER_Mover> ConstPtr;

    CRUSTCRAWLER_Mover(ros::NodeHandle& nh){
        //_nh.reset(new ros::NodeHandle(nh, "crustcrawler_mover"));
        _nh.reset(new ros::NodeHandle("/"));
        init(*_nh);

    }

    void init(ros::NodeHandle& nh);
    bool move_crustcrawler_arm_cb(crustcrawler_mover_utils::move_crustcrawler_arm::Request& req,
                                  crustcrawler_mover_utils::move_crustcrawler_arm::Response& res);

    //call back that register crustcrawler end effector pose and rearrange the orientation in RPY
    void eef_Callback(const crustcrawler_core_msgs::EndpointState::ConstPtr& eef_feedback){
        crustcrawler_helpers_methods::locate_eef_pose(eef_feedback->pose, global_parameters);
    }

    //call back that register crustcrawler joint states
    void joint_state_Callback(const sensor_msgs::JointState::ConstPtr& joint_state_feedback){
        global_parameters.set_joint_state(joint_state_feedback);
    }

    //get the clear octomap service client
    std::unique_ptr<ros::ServiceClient>& get_clear_octomap_service_client(){
        return _clear_octomap;
    }

    Data_config global_parameters;
    std::shared_ptr<moveit::planning_interface::MoveGroup> group;

private:
    std::unique_ptr<ros::AsyncSpinner> _my_spinner;
    std::unique_ptr<ros::ServiceServer> _crustcrawler_mover;
    std::unique_ptr<ros::ServiceClient> _get_motion_plan;
    std::unique_ptr<ros::ServiceClient> _execute_motion_plan;
    std::unique_ptr<ros::ServiceClient> _clear_octomap;
    std::unique_ptr<ros::Subscriber> _sub_eef_msg;
    std::unique_ptr<ros::Subscriber> _sub_joint_state_msg;

    std::string _planner_id;
    XmlRpc::XmlRpcValue _planner_parameters;

    ros::NodeHandlePtr _nh;
};


}

#endif //_CRUSTCRAWLER_MOVER_HPP
