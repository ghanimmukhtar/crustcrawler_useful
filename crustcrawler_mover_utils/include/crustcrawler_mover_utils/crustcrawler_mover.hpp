#ifndef _CRUSTCRAWLER_MOVER_HPP
#define _CRUSTCRAWLER_MOVER_HPP

#include <crustcrawler_mover_utils/helpers_methods.hpp>

namespace crustcrawler_mover {
class CRUSTCRAWLER_Mover {
public:
    typedef std::shared_ptr<CRUSTCRAWLER_Mover> Ptr;
    typedef const std::shared_ptr<CRUSTCRAWLER_Mover> ConstPtr;

    CRUSTCRAWLER_Mover(ros::NodeHandle& nh){
        //_nh.reset(new ros::NodeHandle(nh, "crustcrawler_mover"));
        init(nh);

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

    void call_service_get_ps(){
        global_parameters.get_ps_request().components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        _get_planning_scene->call(global_parameters.get_ps_request(), global_parameters.get_ps_response());
        if(global_parameters.get_adding_octomap_to_acm()){
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_names.push_back("<octomap>");
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_values.push_back(true);
        }
        else{
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_names.clear();
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_values.clear();
        }
    }

    void publish_psm_msg(){
        global_parameters.get_ps_msg().is_diff = true;
        global_parameters.get_ps_msg().allowed_collision_matrix = global_parameters.get_ps_response().scene.allowed_collision_matrix;
        _psm_pub->publish(global_parameters.get_ps_msg());
    }

    void publish_psm_msg(moveit_msgs::PlanningScene the_scene){
        _psm_pub->publish(the_scene);
    }

    void remove_world_object(std::string object_name){
            _co.operation = moveit_msgs::CollisionObject::REMOVE;
            _co.id = object_name;
            _pub_co->publish(_co);
        }

    void add_world_object(std::string object_name,
                                         geometry_msgs::PoseStamped object_pose,
                                         std::vector<double> object_dimensions){
            _co.header.stamp = ros::Time::now();
            _co.header.frame_id = group->getPlanningFrame();

            _co.primitives.resize(1);
            _co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
            _co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
            _co.primitive_poses.resize(1);

            _co.id = object_name;
            _co.operation = moveit_msgs::CollisionObject::ADD;
            _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = object_dimensions[0];
            _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = object_dimensions[1];
            _co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = object_dimensions[2];

            _co.primitive_poses[0].position = object_pose.pose.position;
            _pub_co->publish(_co);
        }
    Data_config global_parameters;
    std::shared_ptr<moveit::planning_interface::MoveGroup> group;

private:
    std::unique_ptr<ros::AsyncSpinner> _my_spinner;
    std::unique_ptr<ros::ServiceServer> _crustcrawler_mover;
    std::unique_ptr<ros::ServiceClient> _get_motion_plan;
    std::unique_ptr<ros::ServiceClient> _execute_motion_plan;
    std::unique_ptr<ros::ServiceClient> _clear_octomap;
    std::unique_ptr<ros::ServiceClient> _get_planning_scene;
    std::unique_ptr<ros::Publisher> _psm_pub;
    std::unique_ptr<ros::Publisher> _pub_co;
    std::unique_ptr<ros::Subscriber> _sub_eef_msg;
    std::unique_ptr<ros::Subscriber> _sub_joint_state_msg;

    moveit_msgs::CollisionObject _co, _collision_object;
    std::string _planner_id;
    XmlRpc::XmlRpcValue _planner_parameters;

    ros::NodeHandlePtr _nh;

};


}

#endif //_CRUSTCRAWLER_MOVER_HPP
