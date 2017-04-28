#include <ros/ros.h>

#include <crustcrawler_core_msgs/EndpointState.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <moveit/move_group/move_group_context.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <crustcrawler_mover_utils/parameters.hpp>
#include <crustcrawler_mover_utils/move_crustcrawler_arm.h>

namespace crustcrawler_helpers_methods{
/*get crustcrawler eef pose with orientation expressed as RPY
 * input: pose as geometry msgs of the baxter eef (including the ), and the name of the gripper
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_eef_pose(geometry_msgs::Pose eef_feedback, Data_config& parameters);

/* set the motion plan request as delivered from any node wants to use the class BAXTER_mover
 * input: request and response to the service baxter_mover_utils::move_baxter_arm, and Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void prepare_motion_request(crustcrawler_mover_utils::move_crustcrawler_arm::Request &req,
                            crustcrawler_mover_utils::move_crustcrawler_arm::Response &res,
                            Data_config& params);

}
