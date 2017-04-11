#include <crustcrawler_mover_utils/crustcrawler_mover.hpp>

using namespace crustcrawler_mover;

void CRUSTCRAWLER_Mover::init(ros::NodeHandle& nh){
    _crustcrawler_mover.reset(new ros::ServiceServer(nh.advertiseService("move_crustcrawler_arm", &CRUSTCRAWLER_Mover::move_crustcrawler_arm_cb, this)));
    _get_motion_plan.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path", 1)));
    _execute_motion_plan.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path", 1)));
    _clear_octomap.reset(new ros::ServiceClient(nh.serviceClient<std_srvs::Empty>("/clear_octomap", 1)));
    _sub_eef_msg.reset(new ros::Subscriber(nh.subscribe("/crustcrawler/joint_states", 10, &CRUSTCRAWLER_Mover::eef_Callback, this)));

    ROS_ERROR_STREAM("MY NAME SPACE IS: " << nh.getNamespace() << " /////////////////////////!!!!!!!!!!!!!!");
    nh.getParam(nh.getNamespace() + "/planner_parameters", _global_parameters.get_planner_parameters());
    //nh.getParam("planner_id", _planner_id);
    _my_spinner.reset(new ros::AsyncSpinner(1));
    _my_spinner->start();

}

bool CRUSTCRAWLER_Mover::move_crustcrawler_arm_cb(crustcrawler_mover_utils::move_crustcrawler_arm::Request &req,
                                                  crustcrawler_mover_utils::move_crustcrawler_arm::Response &res){
    crustcrawler_helpers_methods::prepare_motion_request(req, res, _global_parameters);

    bool plan_success = false, execute_success = false;
    if(!req.collision_detection){
        _clear_octomap->call(_global_parameters.get_empty_octomap_request(), _global_parameters.get_empty_octomap_response());
        if(_get_motion_plan->call(_global_parameters.get_motion_request(), _global_parameters.get_motion_response()))
            plan_success = true;
        //if didn't work first time try 10 more times
        /*else{
            for(int i = 0; i < 10 || !plan_success; i++){
                ROS_WARN("trying to move without checking collision :):):):)");
                _clear_octomap->call(_global_parameters.get_empty_octomap_request(), _global_parameters.get_empty_octomap_response());
                if(_get_motion_plan->call(_global_parameters.get_motion_request(), _global_parameters.get_motion_response()))
                    plan_success = true;
            }
        }*/
    }
    else if(_get_motion_plan->call(_global_parameters.get_motion_request(), _global_parameters.get_motion_response())){
        plan_success = true;

    }
    else
        plan_success = false;

    if(plan_success){
        _global_parameters.get_motion_execute_request().trajectory = _global_parameters.get_motion_response().motion_plan_response.trajectory;
        ROS_ERROR_STREAM("size of trajectory to be executed is: "
                         << _global_parameters.get_motion_execute_request().trajectory.joint_trajectory.points.size());
        _global_parameters.get_motion_execute_request().wait_for_execution = true;
        if(_execute_motion_plan->call(_global_parameters.get_motion_execute_request(), _global_parameters.get_motion_execute_response()))
            execute_success = true;
    }

    ROS_INFO_STREAM("the planning response error_code is: " << _global_parameters.get_motion_response().motion_plan_response.error_code);
    ROS_INFO_STREAM("and planning success is: " << plan_success);
    ROS_INFO_STREAM("and executing success is: " << execute_success);

    return execute_success;
}