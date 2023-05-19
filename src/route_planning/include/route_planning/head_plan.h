#ifndef         head_plan
#define        head_plan
#include    <iostream>
#include    <ros/ros.h>
#include    <geometry_msgs/PoseWithCovarianceStamped.h>
#include    <nav_msgs/OccupancyGrid.h>
#include    <nav_msgs/Path.h>
#include    <geometry_msgs/PoseStamped.h>
#include    <opencv2/core.hpp>
#include    <opencv2/imgcodecs.hpp>
#include    <opencv2/highgui.hpp>
#include    <cv_bridge/cv_bridge.h>
#include    <vector>
#include    <queue>
#include    <nav_msgs/Odometry.h>

using namespace std;
using namespace cv;

struct node//每个节点的数据都会放在这
{
    int x,y;//节点在mapList下的坐标

    int version;//节点的版本，随地图的更新而增大

    double G;//到终点的代价（累加出来的）
    double Rhs;//一个变量，指示节点的变化情况
    double H;//节点到起点的启发值
    double Key1;//总代价1
    double Key2;//总代价2

    double riskGrade;//风险系数，越高发生干涉的概率越大

    bool queue_flag;//是否在队列

    bool forced_update_flag = false;//强制更新

    bool operator<(const node& other) const
    {
        if (Key1 > other.Key1)
        {
            return true;
        }
        else if (Key1 < other.Key1)
        {
            return false;
        }
        else
        {
            return Key2 > other.Key2;
        }
    }
};

//________________________________________________________________

class class_plan//定义类
{
    public:

    class_plan(ros::NodeHandle& nh);//构造函数，初始化

    void publisher(void);//话题发布

    void queue_clean(void);//清空队列

    void updateHandler(void);//更新处理

    bool path_flag;//指示路径是否可用
    bool map_flag;//指示地图是否可用
    bool initialPose_flag;//指示初始点是否可用
    bool endPose_flag;//指示目标点是否可用
    bool time_flag;//指示是否在规定时间内

    clock_t countTime, nowTime;

    ros::Publisher pubPath;

    ros::Publisher pubMap;

//________________________分割线_________________________

    private:

    ros::NodeHandle nh;

    ros::Subscriber subInitialPose;
    void subInitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    ros::Subscriber subEndPose;
    void subEndPoseCallback(const geometry_msgs::PoseStamped& msg);

    ros::Subscriber subMap;
    void subMapCallback(const nav_msgs::OccupancyGrid& msg);

//--------------------------------------------节点------------------------------------------------------

    geometry_msgs::PoseWithCovarianceStamped  initialPose;//起点
    geometry_msgs::PoseWithCovarianceStamped  initialPose_old;//上一个起点
    geometry_msgs::PoseStamped endPose;//终点
    geometry_msgs::PoseStamped endPose_old;//上一个终点

    node* start_node;//起点的指针
    node* end_node;//终点的指针
    node node_tmp;//一个临时的节点

    void updateVertex(int x, int y);//处理节点

    void calculateKey(int x, int y);//计算key值
    
    bool compareKey(double key11,double key12 , double key21,double key22);//比较节点的key值大小

    double calculateDistance(int x1, int x2, int y1, int y2);//计算两点的代价距离

    double realDistance(int x1, int x2, int y1, int y2);//计算两点的实际距离

    double epsilon;//Anytime Repairing A* 的膨胀系数

//-------------------------------------------地图----------------------------------------------------------

    void CVmap_generator(const nav_msgs::OccupancyGrid& msg);//生成距离位图

    void serchedmap_clean(void);//重置serchedmap

    double calculateRiskGrade(int x , int y);//计算危险等级

    nav_msgs::OccupancyGrid map;//地图
    nav_msgs::OccupancyGrid map_old;//旧地图
    nav_msgs::OccupancyGrid serchedmap;//扩展地图（每次扩展都会使该点颜色更亮，这个不会影响程序运行


    vector<vector<node>> mapList;//地图上的点的信息的容器
    priority_queue<node> queue;//优先队列

    int col;//地图的列数，单位：像素点
    int row;//地图的行数，单位：像素点

    int inf;//定义地图可能的最大代价

    vector<vector<int>> grid_map;//地图本身信息的容器
    double dis2wall;//与障碍物的距离
    double origin_x;//地图原点的x坐标
    double origin_y;//地图原点的y坐标
    double map_resolution;//地图的分辨率，单位：米/像素
    double map_version;//地图的版本；

    cv::Mat image_dis;//二值化地图
    cv::Mat image_map;    

//------------------------------------------路径---------------------------------------------------

    void path_calculator(void);//计算路径

    void path_generator(void);//生成路径

    nav_msgs::Path path_msg;//要发布的原始路径消息
    nav_msgs::Path path_msg2;//要发布的初次平滑路径消息
    nav_msgs::Path tempPath;//临时的路径消息
    geometry_msgs::PoseStamped POSE;//用于生成路径的中间变量
};

#endif