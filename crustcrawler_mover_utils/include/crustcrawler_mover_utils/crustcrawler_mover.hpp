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
        //nh_.reset(new ros::NodeHandle(nh, "crustcrawler_mover"));
        init(nh);

    }

    void init(ros::NodeHandle& nh);
    bool move_crustcrawler_arm_cb(crustcrawler_mover_utils::move_crustcrawler_arm::Request& req,
                                  crustcrawler_mover_utils::move_crustcrawler_arm::Response& res);

    //call back that register baxter left end effector pose and rearrange the orientation in RPY
    void eef_Callback(const sensor_msgs::JointState::ConstPtr& eef_feedback){
    }

private:
    Data_config _global_parameters;
    std::unique_ptr<ros::AsyncSpinner> _my_spinner;
    std::unique_ptr<ros::ServiceServer> _crustcrawler_mover;
    std::unique_ptr<ros::ServiceClient> _get_motion_plan;
    std::unique_ptr<ros::ServiceClient> _execute_motion_plan;
    std::unique_ptr<ros::ServiceClient> _clear_octomap;
    std::shared_ptr<moveit::planning_interface::MoveGroup> _group;
    std::unique_ptr<ros::Subscriber> _sub_eef_msg;
    std::string _planner_id;
    XmlRpc::XmlRpcValue _planner_parameters;

    ros::NodeHandlePtr _nh;
};


}

#endif //_CRUSTCRAWLER_MOVER_HPP
