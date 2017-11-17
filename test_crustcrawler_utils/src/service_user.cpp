#include <crustcrawler_mover_utils/crustcrawler_mover.hpp>
#include <crustcrawler_mover_utils/move_crustcrawler_arm.h>

using namespace crustcrawler_mover;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_user");
    ros::NodeHandle node;

    ros::ServiceClient mover = node.serviceClient<crustcrawler_mover_utils::move_crustcrawler_arm>("move_crustcrawler_arm");

    //ros::AsyncSpinner my_spinner(1);
    //my_spinner.start();



    crustcrawler_mover_utils::move_crustcrawler_arm::Request request;
    crustcrawler_mover_utils::move_crustcrawler_arm::Response response;
    request.type = "position";
    request.goal = {0.3, -0.3, 0.0};
    request.collision_detection = false;

    mover.call(request, response);



    return 0;
}
