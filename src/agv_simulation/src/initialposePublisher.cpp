#include    "ros/ros.h"
#include    "tf2_ros/transform_listener.h"
#include    "tf2/LinearMath/Quaternion.h"
#include    "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include    "geometry_msgs/TransformStamped.h"
#include    "geometry_msgs/PointStamped.h"
#include    "tf2_ros/buffer.h"


int main(int argc, char  *argv[])
{
    setlocale(LC_ALL," ");//这条是防止日志输出中文为乱码的

    ros::init(argc,argv,"initialposePublisher");//初始化ROS节点

    ros::NodeHandle nh;//创建节点句柄

    tf2_ros::Buffer buffer;

    tf2_ros::TransformListener  listener(buffer);

    geometry_msgs::TransformStamped tfs;

    ros::Rate rate(200);//设置频率

    ROS_INFO("waitting for 3 sec...");

    ros::Duration(3.0).sleep();//等待3秒

    geometry_msgs::PoseWithCovarianceStamped initial_pose;//创建发布小车坐标的消息载体

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",20);//创建发布者对象


    while (ros::ok())//节点没死就一直循环
    {
        rate.sleep();//频率

        try
        {
            initial_pose.header.stamp = ros::Time::now();

            tfs = buffer.lookupTransform("odom","base_footprint",ros::Time(0));//接收订阅的坐标变换
            
            //给发布的消息载体赋值
            initial_pose.pose.pose.position.x = tfs.transform.translation.x;
            initial_pose.pose.pose.position.y = tfs.transform.translation.y;
            initial_pose.pose.pose.orientation = tfs.transform.rotation;
            initial_pose.header.frame_id = "/odom";
            initial_pose.header.stamp.sec = ros::Time::now().toSec();
            initial_pose.header.stamp.nsec = ros::Time::now().toNSec();

            pub.publish(initial_pose);//发布消息

            // ROS_INFO("the carPosition has been pubed ");
            // ROS_INFO("pose x is:%.2f",initial_pose.pose.pose.position.x);
            // ROS_INFO("pose y is:%.2f",initial_pose.pose.pose.position.y);

           std:: cout<<initial_pose.pose.pose.position.x<<" "<<initial_pose.pose.pose.position.y <<"  "<<ros::Time::now()<<std::endl;
        }
        catch(const std::exception& e)
        {
            ROS_INFO("worng!");
            std::cerr << e.what() << '\n';
        }
        ros::spinOnce();
    }   
    return 0;
}
