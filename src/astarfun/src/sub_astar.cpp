 #include"astar/astar.h"

astar::astar(ros::NodeHandle& nh)//构造函数，创建对象时运行
{
    subMap = nh.subscribe("/map", 1000, &astar::subMapCallback,this);

    subInitialpose = nh.subscribe("/initialpose", 1000, &astar::subPoseCallback,this);

    subEndpose = nh.subscribe("/move_base_simple/goal", 1000, &astar::subendPoseCallback,this);

    pubThePath = nh.advertise<nav_msgs::Path>("/searchedpath", 1000,true);

    pubMap = nh.advertise<nav_msgs::OccupancyGrid>("/rastermap",2);

    pub_path_ok = nh.advertise<astarfun::path_ok>("/path_ok",10,true);

    map_flag = false;
    initial_pose_flag = false;
    end_pose_flag = false;
    path_flag = false;
    update_flag = false;
    map_update_flag = false;
    pub_flag = false;
    path_ok.path_ok = false;
    time_flag = true;

}

void astar::subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    initial_pose = msg; 

    if(!map_flag) //如果没有得到栅格地图，则不进行
    return;

    int x=(initial_pose.pose.pose.position.x - origin_x) / map_resolution;

    int y=(initial_pose.pose.pose.position.y - origin_y) / map_resolution;//获得起点的栅格位置

    
    if(grid_map[x][y] == 100 || image_dis.at<float>(y, x) < dis2wall/2)//如果将起点设置在障碍物上或离障碍物太近，返回(这里设置的比其他距离小，保证程序不会卡死)
    {
        cout << "please set the right initial position" << endl;

        return;
    }

    if((initial_pose.pose.pose.position.x != 0||initial_pose.pose.pose.position.y != 0))
    {
        start_node = &maplist[x][y];//设置开始点的指针

        initial_pose_flag = true;   

        if (path_flag)//如果已经生成了路径，再次收到就进行更新
        {
            double distans = sqrt(pow((old_x-x),2)+pow(old_y-y,2));
            if (distans*map_resolution>=0.01)//如果距离上个位姿太近就不更新
            {
                cout<<"update robot position"<<endl;
                update_flag = true;
                pub_flag = true;
                // path_ok.path_ok = false;//路径禁用
            }          
        }
        else
        {
            cout << "robot has been loaded , waitting for goal" << endl;
        }
        
        // cout << "start x=" << start_node->x << "  start y=" << start_node-> y << endl;

        old_x = x;//将上次起始点坐标储存起来
        old_y = y;

        return;
    }
}

void astar::subendPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    end_pose = msg;

    if(!map_flag)
    return;

    int x = (end_pose.pose.position.x - origin_x) / map_resolution;

    int y = (end_pose.pose.position.y - origin_y) / map_resolution;

    if(grid_map[x][y] == 100||image_dis.at<float>(y, x) < dis2wall)
    {
        cout << "please set the right end goal" << endl;

        return;
    }

    if((end_pose.pose.position.x != 0||end_pose.pose.position.y != 0))
    {
       
        end_node = &maplist[x][y];

        end_pose_flag = true; 

        if (path_flag)//如果已经生成了路径，再次收到就进行更新
        {
            update_flag = true;
            pub_flag = true;
            path_ok.path_ok = false;//路径禁用
        }

        cout << "get end goal   " << endl;

        // cout << "end x=" << end_node->x << "  end y=" << end_node->y << endl;

        return;
    }
}

void astar::subMapCallback(const nav_msgs::OccupancyGrid& msg)
{
    if(map_flag == false)
    {
       pub_flag = true;

        old_map.data = msg.data;//将地图数据储存起来
        old_map.info.height = msg.info.height;//栅格地图的行数(高度)
        old_map.info.width = msg.info.width;//栅格地图的行数(高度)

        map_resolution = msg.info.resolution;//栅格地图的分辨率

        dis2wall = 0.45 / map_resolution;//生成路径与障碍物距离,当前为0.45m

        row = msg.info.height;//栅格地图的行数(高度)

        col = msg.info.width;//栅格地图的列数(宽度)

        origin_x = msg.info.origin.position.x ;//栅格地图原点，原为收到地图的左下角的点

        origin_y = msg.info.origin.position.y ;

        grid_map.resize(col);//设置地图列数
        
        for (size_t i = 0; i < grid_map.size(); i++)
        {
            grid_map[i].resize(row);//设置地图行数
        }//设置地图大小
        
        // cout << "col=" << col <<"  row=" << row << endl;

//--------------------------------生成距离位图--------------------------------------
        image_map.create(row, col, CV_8UC1);    

        for(int i = 0; i < row; i++)
        {
            for(int j = 0; j < col; j++)
            image_map.at<uchar>(i, j) = 255; //opencv中255为白色，0为黑色
        }

        for(int j = 0;j < col;j++)
        {
            for(int i = 0;i < row;i++)
            {
                grid_map[j][i] = msg.data[i*col+j];//对二维地图进行构建  grid_map[列][行] ，OccupancyGrid地图中0为白色，100为黑色

                if(grid_map[j][i] == 100)
                {
                    image_map.at<uchar>(i, j) = 0;
                }
            }
        }

        image_dis.create(row, col, CV_8UC1);    

        distanceTransform(image_map, image_dis, CV_DIST_L2, 5, CV_32FC1);//设置距离位图  ，可以直接通过image_dis.at<float>(y, x)得到当前节点离最近障碍物的距离。
    //-------------------------------------------------------------------------------------------------


    //------------------------用于生成发布的地图------------------------
    //有必要这一步马？

        map.header.frame_id="map";
        map.header.stamp = ros::Time::now(); 
        map.info.origin.position.x = origin_x;
        map.info.origin.position.y = origin_y;
        map.info.origin.position.z = 0;
        map.info.resolution = map_resolution;    
        map.info.width = col;   
        map.info.height = row; 
        map.data.resize(col*row);

        for( int i = 0; i < row; ++i)
        {
            for ( int j = 0; j < col; ++j)
            {
                if(grid_map[col-1-j][i] == 100)
                map.data[i*col+col-1-j] = 100;

                else
                map.data[i*col+col-1-j] = 0;
            }
        }


//-------------------------------------------------------------------------------------------------

        map_flag = true;

        cout << "get the map  " << endl;

//---------------------------地图队列初始化-------------------------

        maplist.resize(image_dis.cols);
        for (size_t i = 0; i < image_dis.cols; i++)
        {
            maplist[i].resize(image_dis.rows);
        }

        for (size_t i = 0; i < image_dis.rows; i++)
        {
            for (size_t j = 0; j < image_dis.cols; j++)
            {
                maplist[ j][i].y = i;
                maplist[ j][i].x = j;
                maplist[ j][i].G = col*col+row*row;    //将代价定义的足够大      
                maplist[ j][i].H = col*col+row*row;    //将代价定义的足够大      
                maplist[ j][i].F = col*col+row*row;    //将代价定义的足够大      
            }
        }
    }//初始化
    else
    {
        for(int j = 0;j < col;j++)
        {
            for(int i = 0;i < row;i++)
            {
                if(fabs(msg.data[i*col+j] - old_map.data[i*col+j])>=100)//判断是否出现新的障碍物
                {
                    image_map.at<uchar>(i, j) = 0;
                    map_update_flag = true;
                }
            }
        }

        if (map_update_flag == true&&path_flag == true )
        {
            image_dis.create(row, col, CV_8UC1);    

            distanceTransform(image_map, image_dis, CV_DIST_L2, 5, CV_32FC1);//更新距离位图

            finall_path = end_node;

            map_update_flag = false;

            // cout<<"map update done"<<endl;

            while(finall_path)//检查现有路径是否依旧能够同行，如果不能则更新
            {
                if(image_dis.at<float>(finall_path->y, finall_path->x) < dis2wall){
                pub_flag = true;
                update_flag = true;
                path_ok.path_ok = false;//路径禁用
                cout<<"obstacles  on the path, need update the path"<<endl;
                break;
                }

                finall_path = finall_path->parent_node;
            }
        }
    }
}
