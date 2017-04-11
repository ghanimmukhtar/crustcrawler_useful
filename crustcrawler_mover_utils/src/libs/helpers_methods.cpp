#include <crustcrawler_mover_utils/helpers_methods.hpp>


void crustcrawler_helpers_methods::prepare_motion_request(crustcrawler_mover_utils::move_crustcrawler_arm::Request &req,
                                                          crustcrawler_mover_utils::move_crustcrawler_arm::Response &res,
                                                          Data_config &params){

    ROS_INFO("I am at the service to move the arm");
    params.get_motion_request().motion_plan_request.goal_constraints.clear();
    std::string eef_name = "the_gripper";
    geometry_msgs::PointStamped position_target;
    geometry_msgs::PoseStamped pose_target;
    std::vector<double> tolerance_pose(3,  std::stod(params.get_planner_parameters()["tolerance_pose"]));
    std::vector<double> tolerance_angle(3, std::stod(params.get_planner_parameters()["tolerance_angle"]));

    params.get_motion_request().motion_plan_request.planner_id = static_cast<std::string>(params.get_planner_parameters()["planner_id"]);
    params.get_motion_request().motion_plan_request.num_planning_attempts = std::stoi(params.get_planner_parameters()["planning_attempts"]);
    params.get_motion_request().motion_plan_request.allowed_planning_time = std::stod(params.get_planner_parameters()["planning_time"]);

    ROS_INFO_STREAM("planner id is: " << params.get_motion_request().motion_plan_request.planner_id);
    ROS_INFO_STREAM("planner attempts is: " << params.get_motion_request().motion_plan_request.num_planning_attempts);
    ROS_INFO_STREAM("planner time is: " << params.get_motion_request().motion_plan_request.allowed_planning_time);


    params.get_motion_request().motion_plan_request.group_name = "crustcrawler_arm";
    eef_name = "the_gripper";


    if(strcmp(req.type.c_str(), "position") == 0){
        ROS_INFO("building the position item regardless of orientation");
        position_target.header.frame_id = "/world_base";
        position_target.point.x = req.goal[0];
        position_target.point.y = req.goal[1];
        position_target.point.z = req.goal[2];
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(eef_name, position_target);
        ROS_WARN_STREAM("the goal is: "
                        << req.goal[0] << ", "
                                       << req.goal[1] << ", "
                                       << req.goal[2]);
        params.get_motion_request().motion_plan_request.goal_constraints.push_back(pose_goal);

    }
    else if(strcmp(req.type.c_str(), "pose") == 0){
        ROS_INFO("building the pose item");
        pose_target.header.frame_id = "/world_base";
        pose_target.pose.position.x = req.goal[0];
        pose_target.pose.position.y = req.goal[1];
        pose_target.pose.position.z = req.goal[2];

        params.set_roll(atan2(pose_target.pose.position.y, pose_target.pose.position.x));
        params.get_quat_angles().setRPY(params.get_roll(), params.get_pitch(), params.get_yaw());

        pose_target.pose.orientation.x = params.get_quat_angles().getX();
        pose_target.pose.orientation.y = params.get_quat_angles().getY();
        pose_target.pose.orientation.z = params.get_quat_angles().getZ();
        pose_target.pose.orientation.w = params.get_quat_angles().getW();

        params.set_pose_target(pose_target);

        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(eef_name, pose_target, tolerance_pose, tolerance_angle);
        params.get_motion_request().motion_plan_request.goal_constraints.push_back(pose_goal);
        ROS_WARN_STREAM("the goal is: "
                        << req.goal[0] << ", "
                                       << req.goal[1] << ", "
                                       << req.goal[2] << ", "
                                       << pose_target.pose.orientation.w << ", "
                                       << pose_target.pose.orientation.x << ", "
                                       << pose_target.pose.orientation.y << ", "
                                       << pose_target.pose.orientation.z << ", ");
    }
    else
        ROS_INFO("determine if the target is pose, position or position with same orientation as start");
}
