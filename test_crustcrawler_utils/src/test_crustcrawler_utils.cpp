#include <crustcrawler_mover_utils/crustcrawler_mover.hpp>

using namespace crustcrawler_mover;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_crustcrawler_utils");
    ros::NodeHandle node;
    CRUSTCRAWLER_Mover::Ptr _my_test;
    _my_test.reset(new CRUSTCRAWLER_Mover(node));

    //ros::AsyncSpinner my_spinner(1);
    //my_spinner.start();

    ros::waitForShutdown();
    return 0;
}
