#include     <iostream>
#include     <ctime>
#include    "ros/ros.h"
#include    "tf/transform_datatypes.h"
#include    "tf2_ros/buffer.h"
#include    "tf2_ros/transform_listener.h"
// #include    "tf2/LinearMath/Quaternion.h"
// #include    "tf2/LinearMath/Vector3.h"
#include    "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include    "geometry_msgs/PoseStamped.h"
#include    "geometry_msgs/TransformStamped.h"
#include    "geometry_msgs/PointStamped.h"
#include    "geometry_msgs/Twist.h"
#include    "nav_msgs/Path.h"
#include    "astarfun/path_ok.h"


using namespace std;


int main(int argc, char  *argv[])
{
    setlocale(LC_ALL," ");//这条是防止日志输出中文为乱码的

    ros::init(argc,argv,"followThePath");//初始化ROS节点

    ros::NodeHandle nh;//创建节点句柄

    ros::Rate rate(20);//设置频率

    cout<<"step 1 finished, waiting..."<<endl;

    ros::Duration(2.0).sleep();//等待2秒

    while (ros::ok())
    {
        rate.sleep();

        ROS_INFO("0");
    }
    
    return 0;
}

