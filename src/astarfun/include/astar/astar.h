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
#include    "astarfun/path_ok.h"
using namespace std;
using namespace cv;

const  int INF=9999;
const  int MAX_IT=10;

struct node//单个节点（节点就是栅格地图的一个栅格），包括xy，代价，是否找到最短路径，父节点
{
    int x,y;//x,y；
    float F,G,H;
    bool open_list=false;
    bool close_list=false;
    bool pathNood = false;

    node* parent_node;

    bool operator<(const node& other) const//与优先队列相关，将cost设置为排序标准
    {
        return F> other.F; 
    }
};

class astar//定义类
{
    public:

    int old_x,old_y;//储存上次的小车位姿

    astar(ros::NodeHandle& nh);

    bool start(void);

    void astarextend(void);

    void findPath(void);

    void reset(void);

    bool pubPath(void);

    float getH( node* path);

    //发布路径
    bool path_flag;//记录是否已经生成了路径
    bool map_flag;//记录是否收到了地图
    bool update_flag;//记录是否需要更新
    bool map_update_flag;//记录地图是否徐啊哟更新
    bool pub_flag;//记录是否需要重新发布路径
    bool time_flag;//记录运行时间是否超时

    clock_t contTime, nowTime;

    ros::Publisher pubThePath;
    
    ros::Publisher pubMap;

    ros::Publisher pub_path_ok;

    astarfun::path_ok path_ok;//记录路径是否生效
    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid old_map;

//__________________________________________________________

    private:

    ros::NodeHandle nh;

    ros::Subscriber subInitialpose;
    void subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    ros::Subscriber subEndpose;
    void subendPoseCallback(const geometry_msgs::PoseStamped& msg);

    ros::Subscriber subMap;
    void subMapCallback(const nav_msgs::OccupancyGrid& msg);

    geometry_msgs::PoseWithCovarianceStamped initial_pose;//起点
    geometry_msgs::PoseStamped end_pose;//终点

    node* start_node;//起点的指针
    node* end_node;//终点的指针
    node* finall_path;//指向当前路径点

    vector<vector<node>> maplist;
    priority_queue<node> node_queue;//优先队列，会将代价最小的节点放到最前面。


    bool initial_pose_flag;
    bool end_pose_flag;

    //地图
    int col;//列
    int row;//行
    vector<vector<int>> grid_map;//地图的二维容器
    float dis2wall;//与障碍物的距离
    float origin_x;//原点x
    float origin_y;//原点y
    float map_resolution;//地图分辨率
    cv::Mat image_dis;//二值化地图
    std::string image_path;
    Mat img;
    cv::Mat image_map;    
    
    //Astar
    node tmp_node;


    //寻路
    node* parent_path;
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped POSE_msg;
};

