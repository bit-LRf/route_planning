#include    <ctime>
#include    "route_planning/head_plan.h"

int main(int argc, char*  argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc,argv, "main_route_planning");

    ROS_INFO("\033[1;32m---->\033[0m global path find Started.");

    ros::NodeHandle nh;

    class_plan plan(nh);

    ros::Rate rate(10);

    clock_t startTime,endTime;

    while (ros::ok())
    {
        ros::spinOnce();

        plan.updateHandler();

        rate.sleep();
    }
}
