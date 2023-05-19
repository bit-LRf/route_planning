#include     <iostream>
#include     <ctime>
#include    "ros/ros.h"
#include    "tf/transform_datatypes.h"
#include    "tf2/LinearMath/Quaternion.h"
#include    "tf2/LinearMath/Vector3.h"
#include    "nav_msgs/Path.h"
#include    "/home/lrf/ROS/route_planning/src/reeds_shepp/include/reeds_shepp/rs_head.h"
#include    "/home/lrf/ROS/route_planning/src/reeds_shepp/src/rs_fun.cpp"

using namespace std;

nav_msgs::Path path_msg;//储存接收的路径消息

bool path_flag;//判断是否收到路径消息

vector<point> path_points;//接收当前生成的RS曲线点集

nav_msgs::Path rs_path;//储存发布的路径

point start_position;//起点信息

point end_position;//终点信息

point startPoint;//用于生成RS曲线的起点

point endPoint;//用于生成RS曲线的终点

clock_t contTime, nowTime;

//phi四元数转欧拉角
double getPhi(geometry_msgs::Pose pose)
{
    double roll , pitch , yaw;
    tf::Quaternion qtn;

    tf::quaternionMsgToTF(pose.orientation,qtn);
    tf::Matrix3x3(qtn).getRPY(roll,pitch,yaw);
    return yaw;
}

//phi欧拉角转四元数
void getQtn(double phi , geometry_msgs::Pose &pose)
{
    tf::Quaternion qtn;
    qtn.setRPY(0 , 0 , phi);
    pose.orientation.w = qtn.getW();
    pose.orientation.x = qtn.getX();
    pose.orientation.y = qtn.getY();
    pose.orientation.z = qtn.getZ();
}

void get_searchedpath(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("get searchedpath");

    path_flag = true;

    path_msg.poses = msg->poses;
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL," ");//这条是防止日志输出中文为乱码的

    ros::init(argc,argv,"path_smoothing");//初始化ROS节点

    ros::NodeHandle nh;//创建节点句柄

    tf::Quaternion qtn;

    ros::Subscriber sub_searchedpath;

    ros::Publisher pub_rs_path = nh.advertise<nav_msgs::Path>("/rs_path",50,true);

    ros::Rate rate(20);//设置频率

    ros::Duration(2.0).sleep();//等待2秒

    int sp_distance = 10;//采样距离10，单位：个路径点

    int sp_num;//采样点个数

    int rs_length = 60;//每次生成的rs曲线点的总个数60   (需要是60的倍数)

    double roll,pitch,yaw;

    double radius = 0.5;//最小转向半径0.5

    while (ros::ok())
    {
        rate.sleep();

        path_flag = false;
        
        ros::spinOnce();

        try
        {
            sub_searchedpath  = nh.subscribe<nav_msgs::Path>("/searchedpath",10,get_searchedpath); //接收路径消息

            //如果没有接收到路径消息就跳过
            if(path_flag)
            {
                //获取起点坐标点和方向（在map坐标系下）
                start_position.x = path_msg.poses[0].pose.position.x; 
                start_position.y = path_msg.poses[0].pose.position.y;
                start_position.phi = getPhi(path_msg.poses[0].pose);

                //获取终点的坐标和方向
                end_position.x = path_msg.poses.back().pose.position.x;
                end_position.y = path_msg.poses.back().pose.position.y;
                end_position.phi = getPhi(path_msg.poses.back().pose);

                int size = path_msg.poses.size();

                //计算采样点个数
                int l = size / sp_distance;
                if(size - l * sp_distance >= sp_distance/2)
                {
                    sp_num = l + 1;
                }
                else
                {
                    sp_num = l;
                }

                vector<point> sp_points;//采样点向量

                //设置采样点向量的长度
                sp_points.resize(sp_num);

                //提取采样点
                for (size_t i = 0; i < sp_num-1; i++)
                {
                    sp_points[i].x = path_msg.poses[(i + 1) * sp_distance].pose.position.x;
                    sp_points[i].y = path_msg.poses[(i + 1) * sp_distance].pose.position.y;
                    sp_points[i].phi = atan2(sp_points[i].y - path_msg.poses[(i + 1) * sp_distance - 1].pose.position.y ,
                                                                    sp_points[i].x - path_msg.poses[(i + 1) * sp_distance - 1].pose.position.x);
                }
                //最后一个采样点是终点
                sp_points[sp_num - 1].x = path_msg.poses.back().pose.position.x;
                sp_points[sp_num - 1].y = path_msg.poses.back().pose.position.y;
                sp_points[sp_num - 1].phi = getPhi(path_msg.poses.back().pose);
                
                //设置path_points的长度
                path_points.resize(rs_length);

                //设置rs_path的长度
                rs_path.poses.resize(sp_num * rs_length);

                //开始RS曲线平滑
                int rs_now = 0;//记录当前的rs_path位置
                startPoint = start_position;
                for (size_t i = 0; i < sp_num; i++)
                {
                    //更改终点
                    endPoint = sp_points[i];

                    //生成rs曲线
                    rs_class rs(startPoint , endPoint , radius , rs_length);

                    //将生成的曲线点储存在临时容器内
                    path_points = rs.points ;

                    //将该容器的信息插入到rs_path中
                    for (size_t j = 0; j < rs_length; j++)
                    {
                        rs_path.poses[rs_now].pose.position.x = path_points[j].x;
                        rs_path.poses[rs_now].pose.position.y = path_points[j].y;
                        getQtn(path_points[j].phi , rs_path.poses[rs_now].pose);

                        rs_now++;
                    }

                    //更改起始点
                    startPoint = endPoint;
                }

                //编辑rs_path的header
                rs_path.header.frame_id = "map";
                rs_path.header.stamp = ros::Time::now();

                //发布rs_path
                pub_rs_path.publish(rs_path);

                ROS_INFO("pub rs_path done");
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    
    return 0;
}
