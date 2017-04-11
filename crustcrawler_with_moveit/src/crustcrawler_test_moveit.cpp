#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>


class crustcrawler_with_moveit
{
public:
    crustcrawler_with_moveit(std::string name){
        jo_state_sub_ = nh_.subscribe("/crustcrawler/joint_states", 1, &crustcrawler_with_moveit::joint_states_cb, this);
        get_motion_plan_ = nh_.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path", 1);
        execute_trajectory_ = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path", 1);
        display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, this);
        counter_ = 0;

    }

    ~crustcrawler_with_moveit(void)
    {
    }

    void joint_states_cb(const sensor_msgs::JointState::ConstPtr& jo_state)
    {
        crustcrawler_joint_names_ = jo_state->name;
        joints_values_ = jo_state->position;
    }

    void several_movement(){
        ros::WallDuration sleep_time(5.0);
        nh_.getParam("/number_of_trials", number_of_trials_);
        while(ros::ok() && counter_ < number_of_trials_){
            std::cin.ignore();
            std::cin >> x_ >> y_ >> z_;

            yaw_ = atan2(y_, x_);
            pitch_ = 2.1;
            roll_ = 0.0;
            my_angles_.setRPY(roll_, pitch_, yaw_);

            pose_.header.frame_id = "/world_base";
            pose_.pose.position.x = x_;
            pose_.pose.position.y = y_;
            pose_.pose.position.z = z_;
            pose_.pose.orientation.w = my_angles_.getW();
            pose_.pose.orientation.x = my_angles_.getX();
            pose_.pose.orientation.y = my_angles_.getY();
            pose_.pose.orientation.z = my_angles_.getZ();

            req2_.group_name = "crustcrawler_arm";
            req2_.planner_id = "RRTConnectkConfigDefault";

            req2_.num_planning_attempts = 2;
            req2_.allowed_planning_time = 2.0;
            moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("the_gripper", pose_);

            req2_.goal_constraints.push_back(pose_goal);

            req3_.motion_plan_request = req2_;
            get_motion_plan_.call(req3_, res3_);


            /* Visualize the trajectory*/
            ROS_INFO("Visualizing the trajectory");

            display_trajectory_.trajectory_start = res3_.motion_plan_response.trajectory_start;
            display_trajectory_.trajectory.push_back(res3_.motion_plan_response.trajectory);
            display_publisher_.publish(display_trajectory_);

            sleep_time.sleep();

            traj_req_.trajectory = res3_.motion_plan_response.trajectory;
            traj_req_.wait_for_execution = true;

            execute_trajectory_.call(traj_req_, traj_res_);

            counter_+=1;
        }
    }

protected:
    ros::NodeHandle nh_;
    ros::Subscriber jo_state_sub_;
    ros::Publisher display_publisher_;
    ros::ServiceClient get_motion_plan_, execute_trajectory_;
    std::vector<std::string> crustcrawler_joint_names_;
    std::vector<double> joints_values_;
    int number_of_trials_, counter_ = 0;
    double x_, y_, z_, roll_, pitch_, yaw_;
    moveit_msgs::MotionPlanRequest req2_;
    moveit_msgs::GetMotionPlanRequest req3_;
    moveit_msgs::GetMotionPlanResponse res3_;

    moveit_msgs::ExecuteKnownTrajectoryRequest traj_req_;
    moveit_msgs::ExecuteKnownTrajectoryResponse traj_res_;
    moveit_msgs::DisplayTrajectory display_trajectory_;

    geometry_msgs::PoseStamped pose_;
    tf::Quaternion my_angles_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "crustcrawler_with_moveit");

    crustcrawler_with_moveit crustcrawler_robot(ros::this_node::getName());

    crustcrawler_robot.several_movement();

    ros::spin();
    return 0;
}
