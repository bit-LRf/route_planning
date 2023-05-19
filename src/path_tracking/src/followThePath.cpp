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

nav_msgs::Path path_msg;

astarfun::path_ok path_ok;

geometry_msgs::Twist cmd_vel;

geometry_msgs::PointStamped goal_position;

geometry_msgs::PointStamped car_position;

geometry_msgs::PointStamped intertim_point;

clock_t contTime, nowTime;

void get_search_path(const nav_msgs::Path::ConstPtr& msg)
{
    // cout<<"get the path_ok"<<endl;

    path_msg.poses = msg->poses;
}
void get_path_ok(const astarfun::path_ok::ConstPtr& msg)
{
    // cout<<"get the path_msg"<<endl;

    path_ok.path_ok = msg->path_ok;
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL," ");//这条是防止日志输出中文为乱码的

    ros::init(argc,argv,"followThePath");//初始化ROS节点

    ros::NodeHandle nh;//创建节点句柄

    tf2_ros::Buffer buffer;

    tf2_ros::TransformListener  listener(buffer);

    tf::Quaternion qtn;

    // tf2::Vector3 v3;

    ros::Subscriber sub_search_path;

    ros::Subscriber sub_path_ok;

    geometry_msgs::TransformStamped tfs;

    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",50);

    ros::Rate rate(20);//设置频率

    cout<<"step 1 finished, waiting..."<<endl;

    ros::Duration(2.0).sleep();//等待2秒

    double s;

    double car_angular;

    while (ros::ok())
    {
        rate.sleep();

        // cout<<"start try"<<endl;

        try
        {
            // cout<<"now try to serch the path"<<endl;

            sub_path_ok = nh.subscribe<astarfun::path_ok>("path_ok",10,get_path_ok);//接收消息

            tfs = buffer.lookupTransform("map","base_footprint",ros::Time(0));//接收订阅的坐标变换

            // cout<<"----------------------------------------"<<endl;

            sub_search_path  = nh.subscribe<nav_msgs::Path>("/search_path",10,get_search_path); //接收路径消息

            if (path_ok.path_ok)//当路径可用时，继续
            {
                car_position.point.x = tfs.transform.translation.x; //获取小车坐标点和方向（在map坐标系下）
                car_position.point.y = tfs.transform.translation.y;
                tf::quaternionMsgToTF(tfs.transform.rotation,qtn);
                double roll,pitch,yaw;
                tf::Matrix3x3(qtn).getRPY(roll,pitch,yaw);
                car_angular = yaw;

                
                for (auto  i : path_msg.poses)
                {
                    // rate.sleep();//调试用的，运行时忽略这条

                    intertim_point.point.x = i.pose.position.x;//把当前路径点放入临时点
                    intertim_point.point.y = i.pose.position.y;

                    // cout<<i.pose.position.x<<"___________"<<i.pose.position.y<<endl;

                    s = sqrt(pow(i.pose.position.x - car_position.point.x,2)+pow(i.pose.position.y - car_position.point.y,2));//计算小车到当前点的距离

                    if (s >= 1)//设置最大预描距离
                    {
                       break;
                    }
                }

                cout<<"s = "<<s<<endl;

                if (s>=0.25)
                {
                    goal_position.point.x = intertim_point.point.x;//将最后的预描点赋值给目标点
                    goal_position.point.y = intertim_point.point.y;

                    double c2g = atan2(goal_position.point.y - car_position.point.y , goal_position.point.x - car_position.point.x);
                    double theta = abs(c2g - car_angular)>3.14? -(c2g - car_angular)/abs(c2g - car_angular) * (6.28-abs(c2g-car_angular)) : (c2g-car_angular);

                    cout<<c2g<<"      "<<car_angular<<"      "<<theta<<endl;

                    cmd_vel.angular.z = theta==0? 0 : theta/abs(theta)*(abs(theta)+1)/(abs(theta)+2.5) ;   //设置角速度
                    cmd_vel.linear.x = abs(theta)<=0.2? 0.2 : 0  ;     //设置线速度

                    cout<<"angular = "<<cmd_vel.angular.z<<"___"<<"linear = "<<cmd_vel.linear.x<<endl;

                    pub_cmd_vel.publish(cmd_vel);//发布运动控制消息

                    cout<<"cmd_vel has been pubed"<<endl;
                }
                else
                {
                    cmd_vel.linear.x = 0;//停车
                    cmd_vel.angular.z = 0;

                    pub_cmd_vel.publish(cmd_vel);//发布运动控制消息

                    cout<<"stop"<<endl;
                }
            }
            else
            {
                // contTime = clock();

                // while ((double)(clock() - contTime) / CLOCKS_PER_SEC<=0.8)//等待0.8s，在此期间继续行驶
                // {
                //     sub_path_ok = nh.subscribe<astarfun::path_ok>("path_ok",10,get_path_ok);//接收消息
                    
                //     if(path_ok.path_ok)
                //     {
                //         break;
                //     }
                // }
                
                if (!path_ok.path_ok)//路径不可用
                {
                    cmd_vel.linear.x = 0;//停车
                    cmd_vel.angular.z = 0;

                    pub_cmd_vel.publish(cmd_vel);//发布运动控制消息

                    cout<<"stop"<<endl;
                }  
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        ros::spinOnce();
    }
    
    return 0;
}

