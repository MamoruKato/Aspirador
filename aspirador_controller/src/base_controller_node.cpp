#include <nox/base_controller.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_controller_node");
    BaseController a_base_controller;

    a_base_controller.PublishTF();

    return 0;
}