#include    "std_msgs/builtin_double.h"
#include     <iostream>
#include     <ctime>
#include    "ros/ros.h"

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL," ");//这条是防止日志输出中文为乱码的

    ros::init(argc,argv,"checktime");//初始化ROS节点

    ros::NodeHandle nh;//创建节点句柄

    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/checktime",20);//创建发布者对象

    ros::Time rostime;

    double now;

    ros::Rate rate(1000);//设置频率

    // ROS_INFO("check point 1");

    while (ros::ok())//节点没死就一直循环
    {
        // ROS_INFO("check point 2");

        rate.sleep();

        // ROS_INFO("check point 3");

        try
        {
            rostime = ros::Time::now();

            now = rostime.toSec();

            pub.publish(now);

            ROS_INFO("still working...");
        }
        catch(const std::exception& e)
        {
            ROS_INFO("worng!");
            std::cerr << e.what() << '\n';
        }
    }   

    return 0;
}
