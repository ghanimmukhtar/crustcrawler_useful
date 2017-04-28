#include <crustcrawler_mover_utils/crustcrawler_mover_utils/crustcrawler_mover.hpp>
#include <crustcrawler_mover_utils/crustcrawler_mover_utils/parameters.hpp>

using namespace crustcrawler_mover;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_crustcrawler_utils");
    ros::NodeHandle node;
    CRUSTCRAWLER_Mover::Ptr _my_test;
    _my_test.reset(new CRUSTCRAWLER_Mover(node));

    //ros::AsyncSpinner my_spinner(1);
    //my_spinner.start();

    /*
    std::map<std::string, double> _home_variable_values;
    moveit::planning_interface::MoveGroup::Plan _group_plan;

    _home_variable_values.insert ( std::pair<std::string, double>("joint_1", -1.3) );
    _home_variable_values.insert ( std::pair<std::string, double>("joint_2",  0.3) );
    _home_variable_values.insert ( std::pair<std::string, double>("joint_3", -1.1) );
    _home_variable_values.insert ( std::pair<std::string, double>("joint_4",  0.0) );
    _home_variable_values.insert ( std::pair<std::string, double>("joint_5", -0.5) );
    _home_variable_values.insert ( std::pair<std::string, double>("joint_6",  0.0) );

    _my_test->group->setPlannerId(static_cast<std::string>(_my_test->global_parameters.get_planner_parameters()["planner_id"]));
    _my_test->group->setPlanningTime(std::stod(_my_test->global_parameters.get_planner_parameters()["planning_time"]));
    //_my_test->group->setJointValueTarget(_home_variable_values);
    _my_test->group->setPositionTarget(0.3, 0.3, 0.2);
    if(_my_test->group->plan(_group_plan))
        _my_test->group->execute(_group_plan);*/

    ros::waitForShutdown();
    return 0;
}
