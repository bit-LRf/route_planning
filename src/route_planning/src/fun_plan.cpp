#include "route_planning/head_plan.h"
#include    <ctime>


//构造函数，创建对象时运行
class_plan::class_plan(ros::NodeHandle& nh)
{
    subMap = nh.subscribe("/map",1,&class_plan::subMapCallback,this);

    subInitialPose = nh.subscribe("/initialpose",5,&class_plan::subInitialPoseCallback,this);

    subEndPose = nh.subscribe("/move_base_simple/goal",1,&class_plan::subEndPoseCallback,this);
    
    pubPath = nh.advertise<nav_msgs::Path>("/searchedpath",2,true);

    pubMap = nh.advertise<nav_msgs::OccupancyGrid>("/serchedmap",2,true);

    //设置epsilon的初始值
    epsilon = 2.5;

    map_flag = false;
    initialPose_flag = false;
    endPose_flag = false;
    path_flag = false;
    time_flag = false;

    ROS_INFO("Planning function has been launched");
}


//处理起始点话题的回调函数
void class_plan::subInitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    initialPose = msg;

    //如果没有地图，则不进行
    if (!map_flag)
    {
        return;
    }
    
    //获得起点的栅格位置
    int x = (initialPose.pose.pose.position.x - origin_x)/map_resolution;
    int y = (initialPose.pose.pose.position.y - origin_y)/map_resolution;

    //获得上一个起点的栅格位置
    int x_old = (initialPose_old.pose.pose.position.x - origin_x)/map_resolution;
    int y_old = (initialPose_old.pose.pose.position.y - origin_y)/map_resolution;

    //如果起点离障碍物太近，则报错
    if (grid_map[x][y] == 100 || mapList[x][y].riskGrade >= 2)
    {
        ROS_ERROR("initial pose is too close to the wall !");

        //将初始点设置为禁用
        initialPose_flag = false;

        return;
    }
    
    if(initialPose.pose.pose.position.x != 0 || initialPose.pose.pose.position.y != 0)
    {
        //更新起点的版本
        mapList[x][y].version = map_version;

        //设置起始点的指针
        start_node = &mapList[x][y];

        //如果存在可用初始点
        if(initialPose_flag)
        {
            double distans = sqrt(pow((x_old - x),2)+pow((y_old - y),2));//初始点移动的距离

            //距离太近就不更新
            if (distans * map_resolution <= 0.1)
            {
                return;
            }
            
            ROS_INFO("update robot initial pose");

            //计算新起点的key
            calculateKey(x,y);

            //判断新初始点的版本，如果版本不同则重置该初始点
            if (start_node->version != map_version)
            {
                //重置起点
                start_node->G = inf;
                start_node->Rhs = inf;
                start_node->Key1 = inf;
                start_node->Key2 = inf;
                start_node->version = map_version;
            }
        }
        else
        {
            ROS_INFO("get the initial pose");
        }

        cout<<"起点x , y:"<<x<<" , "<<y<<endl;
        cout<<"riskgrade:"<<start_node->riskGrade<<endl;
        
        //把本次初始点储存为旧初始点
        initialPose_old = initialPose;

        //将初始点设置为可用
        initialPose_flag = true;

        //重新设置epsilon(不是1就行)
        epsilon = 1.5;

        //更新处理
        updateHandler();
    }
}


//处理终点话题的回调函数
void class_plan::subEndPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    endPose = msg;

    if(!map_flag)//如果没有接收到地图就不进行
    {
        return;
    }

    //得到终点在栅格地图里的位置
    int x = (endPose.pose.position.x - origin_x)/map_resolution;
    int y = (endPose.pose.position.y - origin_y)/map_resolution;

    //获得上一个起点的栅格位置
    int x_old = (endPose_old.pose.position.x- origin_x)/map_resolution;
    int y_old = (endPose_old.pose.position.y - origin_y)/map_resolution;

    //如果终点离障碍物太近，则报错
    if(grid_map[x][y] == 100 || mapList[x][y].riskGrade >= 1)
    {
        ROS_WARN("end pose is too close to the wall !");

        //将终点设置为禁用
        endPose_flag = false;

        return;
    }

    if(endPose.pose.position.x != 0 || endPose.pose.position.y != 0)
    {
        double distans = sqrt(pow((x_old - x),2)+pow((y_old - y),2));//移动的距离
        

        //距离太近就不更新
        if (distans * map_resolution <= 0.1)
        {
            return;
        }

        //如果已经存在终点
        if(endPose_flag)
        {
            //清空队列
            queue_clean();

            //清空serchedmap
            serchedmap_clean();

            //提高地图版本
            map_version = map_version + 1;

            //重新设置epsilon
            epsilon = 2.5;

            //重置起点
            start_node->G = inf;
            start_node->Rhs = inf;
            start_node->Key1 = inf;
            start_node->Key2 = inf;
            start_node->version = map_version;
        }

        //更新终点版本
        mapList[x][y].version = map_version;

        //设置终点指针
        end_node = &mapList[x][y];

        ROS_INFO("get the end pose");

        cout<<"终点x , y:"<<x<<" , "<<y<<endl;
        cout<<"riskgrade:"<<end_node->riskGrade<<endl;

        //将终点设置为可用
        endPose_flag = true;

        //储存本次的终点
        endPose_old = endPose;

        //更新处理
        updateHandler();
    }
}


//处理地图话题的回调函数。说明：地图构建的数据格式为map[列][行]
void class_plan::subMapCallback(const nav_msgs::OccupancyGrid& msg)
{
    //将地图数据储存为旧的地图数据
    map_old.data = msg.data;
    map_old.info.height = msg.info.height;
    map_old.info.width = msg.info.width;

    //第一次收到地图
    if(map_flag == false)
    {
        //设置地图初始版本
        map_version = 1;

        map_resolution = msg.info.resolution;//计算栅格地图的分辨率

        dis2wall = 0.6 / map_resolution;//设置小车和障碍物的最小距离

        row = msg.info.height;//栅格地图的行数（高度y）

        col = msg.info.width;//栅格地图的列数（宽度x）

        inf = row  * col;//保证这个数永远大于其他可能出现的数值
        cout<<"inf = "<<inf<<endl;

        //栅格地图坐标系的原点
        origin_x = msg.info.origin.position.x;
        origin_y = msg.info.origin.position.y;

        grid_map.resize(col);//设置地图列数

        for (size_t i = 0; i < grid_map.size(); i++)
        {
            grid_map[i].resize(row);//设置地图行数
        }
        
        //生成距离位图
        CVmap_generator(msg);

        //------------------------用于生成发布的地图------------------------

        serchedmap.header.frame_id="serchedmap";
        serchedmap.header.stamp = ros::Time::now(); 
        serchedmap.info.origin.position.x = origin_x;
        serchedmap.info.origin.position.y = origin_y;
        serchedmap.info.origin.position.z = 0;
        serchedmap.info.resolution = map_resolution;    
        serchedmap.info.width = col;   
        serchedmap.info.height = row; 
        serchedmap.data.resize(col*row);

        for( int j = 0; j < row; j++)
        {
            for ( int i = 0; i < col; i++)
            {
                serchedmap.data[j * col + i] = 0;
            }
        }

        //-------------------------------------------------------------------------------------------------

        map_flag = true;

        ROS_INFO("get the map");

        //地图队列初始化
        mapList.resize(col);

        for (size_t i = 0; i < col; i++)
        {
            mapList[i].resize(row);
        }
        
        for (size_t j = 0; j < row; j++)
        {
            for (size_t i = 0; i < col; i++)
            {
                mapList[i][j].x = i; 
                mapList[i][j].y = j; 
                mapList[i][j].G = inf;
                mapList[i][j].Rhs = inf;
                mapList[i][j].H = inf;
                mapList[i][j].Key1 = inf;
                mapList[i][j].Key2 = inf;
                mapList[i][j].queue_flag = false;
                mapList[i][j].forced_update_flag = false;
                mapList[i][j].version = 0;
                mapList[i][j].riskGrade = calculateRiskGrade(i , j);
            }
        }
    }
    else//不是第一次收到地图
    {
        //如果还没有路径，则直接返回
        if (path_flag == false)
        {
            return;
        }

        bool map_updateflag = false;

        //更新距离位图
        CVmap_generator(msg);

        //检查终点和障碍物的距离
        if (image_dis.at<float>(end_node->x,end_node->y) < dis2wall)
        {
            ROS_WARN("终点距离障碍物太近了，需要重新设置");

            return;
        }

        //给一个临时的G表示最大值
        double G_max = 2 * inf;
        
        //确认路径是否受到影响
        for (size_t j = 0; j < row; j++)
        {
            for (size_t i = 0; i < col; i++)
            {
                //暂时储存旧的危险等级
                double riskGrade_old = mapList[i][j].riskGrade;

                //计算新的危险等级
                double riskGrade_new = calculateRiskGrade(i,j);

                //检查所有现版本的节点
                if (mapList[i][j].version == map_version)
                {
                    //跳过终点
                    if (i == end_node->x && j == end_node->y)
                    {
                        continue;
                    }
                    
                    //如果新的障碍物影响了该点
                    if (riskGrade_old < 0 && riskGrade_new > 0)
                    {
                        map_updateflag = true;

                        //如果该点的G小于G_max，则更新G_max
                        if (mapList[i][j].G < G_max)
                        {
                            G_max = mapList[i][j].G;
                        }
                    }

                    //如果消失的障碍物影响了该点
                    if (riskGrade_old > 0 && mapList[i][j].riskGrade < 0)
                    {
                        map_updateflag = true;
                    }
                }
            }
        }

        if (map_updateflag == true)
        {
            ROS_INFO("地图更新,G_max = %.2f",G_max);

            //清空队列
            queue_clean();

            //重新设置epsilon
            epsilon = 2.5;

            //重置起点
            start_node->G = inf;
            start_node->Rhs = inf;
            start_node->Key1 = inf;
            start_node->Key2 = inf;
            start_node->version = map_version;

            ros::Rate rate(5000);

            //更新地图各点的危险等级
            for (size_t j = 0; j < row; j++)
            {
                for (size_t i = 0; i < col; i++)
                {
                    //暂时储存旧的危险等级
                    double riskGrade_old = mapList[i][j].riskGrade;

                    //更新新的危险等级
                    mapList[i][j].riskGrade = calculateRiskGrade(i,j);

                    //检查所有现版本的节点
                    if (mapList[i][j].version == map_version)
                    {   
                        //如果该点的G>G_max
                        if (mapList[i][j].G >= G_max)
                        {
                            //将该点版本减小
                            mapList[i][j].version -= 1;

                            //重置该节点
                            mapList[i][j].G = inf;
                            mapList[i][j].Rhs = inf;
                            mapList[i][j].Key1 = inf;
                            mapList[i][j].Key2 = inf;

                            serchedmap.data[j * col + i] = 0;

                            // pubMap.publish(serchedmap);//发布搜索图

                            // rate.sleep();

                            //如果周围有G<G_max的点，则将周围点设置为局部过一致并更新
                            for (size_t n = j-1; n <= j+1; n++)
                            {
                                for (size_t m = i-1; m <= i+1; m++)
                                {
                                    if (mapList[m][n].G < G_max && mapList[m][n].version == map_version)
                                    {
                                        mapList[m][n].forced_update_flag = true;

                                        serchedmap.data[j * col + i] = 0;

                                        //将节点放进队列中
                                        mapList[m][n].queue_flag = true;
                                        queue.push(mapList[m][n]);
                                    }
                                }
                            }

                            //直接开始下一个点
                            continue;
                        }
                        
                        //消失的障碍物影响的点和G = G_max的点
                        if (riskGrade_old > 0 && mapList[i][j].riskGrade < 0)
                        {
                            double Rhs_old = inf;

                            //为该点寻找新的最优successor
                            for (size_t n = j-1; n <= j+1; n++)
                            {
                                for (size_t m = i-1; m <= i+1; m++)
                                {
                                    //不考虑不同版本的点作为successor
                                    if (mapList[m][n].version != map_version)
                                    {
                                        continue;
                                    }
                                    
                                    if (calculateDistance(m,i,n,j) + mapList[m][n].G <= Rhs_old)
                                    {
                                        mapList[i][j].Rhs = calculateDistance(m,i,n,j) + mapList[m][n].G;

                                        Rhs_old = mapList[i][j].Rhs;
                                    }            
                                }
                            }

                            //更新该节点
                            updateVertex(i,j);
                        }
                    }
                }
            }

            pubMap.publish(serchedmap);//发布搜索图

            //更新处理
            updateHandler();
        }
    }
}


//更新处理
void class_plan::updateHandler()
{
    //判断是否满足路径生成或者更新条件
    if (!map_flag || !initialPose_flag || !endPose_flag)
    {
        return;//如果不满足，则返回
    }
    //如果满足条件，则继续

    // //如果epsilon已经是1了，说明已经是最优路径，就不用再找了
    if (epsilon <= 1)
    {
        return;
    }

    ROS_INFO("准备更新");
    
    //初始化终点并放入队列
    mapList[end_node->x][end_node->y].version = map_version;
    mapList[end_node->x][end_node->y].G = 0.01;
    mapList[end_node->x][end_node->y].Rhs = 0;
    updateVertex(end_node->x,end_node->y);

    //更新epsilon
    double a = start_node->Key1 / (queue.top().Key1);

    //设置一个最小下降速度
    double b = (epsilon - a) > 0.2 ? (epsilon - a) : 0.2;

    epsilon -= b;
    
    //epsilon不能小于1
    if (epsilon < 1)
    {
        epsilon = 1;
    }

    cout<<"updateHandler: epsilon = "<<epsilon<<endl;

    //更新队列中的所有Key值
    priority_queue<node> queue_temp;//临时的优先队列
    while (queue.empty() == false)
    {
        //取出优先队列的top
        int x = queue.top().x;
        int y = queue.top().y;

        //如果发现地图上该结点并不在队列中，则弹出并跳过
        if (mapList[x][y].queue_flag == false)
        {
            queue.pop();

            continue;
        }

        //重新计算Key
        calculateKey(x,y);

        //将该节点放进一个临时的队列中
        queue.pop();
        queue_temp.push(mapList[x][y]);
    }
    //交换
    swap(queue,queue_temp);
    //---------------------------------------------------------
    
    //计算路径
    path_calculator();

    //生成路径
    path_generator();

    //发布路径
    publisher();
}


//计算路径
void class_plan::path_calculator()
{
    //设置频率
    ros::Rate rate(5000);

    ROS_INFO("开始计算路径");

    while (compareKey(start_node->Key1,start_node->Key2 , mapList[queue.top().x][queue.top().y].Key1,mapList[queue.top().x][queue.top().y].Key2 ) || start_node->Rhs != start_node->G)
    {
        // rate.sleep();

        pubMap.publish(serchedmap);//发布搜索图

        //复制栈顶的值
        int x_tmp = queue.top().x;
        int y_tmp = queue.top().y;

        // cout<<"栈顶的x和y是:"<<x_tmp<<" , "<<y_tmp<<endl;
        // cout<<"栈顶的riskgrade是:"<<mapList[x_tmp][y_tmp].riskGrade<<endl;
        // cout<<"栈顶的G是:"<<mapList[x_tmp][y_tmp].G<<endl;
        // cout<<"栈顶的Key1是:"<<mapList[x_tmp][y_tmp].Key1<<endl;

        //对于非强制更新节点
        if (mapList[x_tmp][y_tmp].forced_update_flag == false)
        {
            //储存旧的key值
            double key1_old = mapList[x_tmp][y_tmp].Key1;
            double key2_old = mapList[x_tmp][y_tmp].Key2;

            //cout<<"旧key是:"<<key1_old<<endl;

            calculateKey(x_tmp, y_tmp);//计算新的key值

            //如果队列中该节点的key值需要更新
            if (key1_old != mapList[x_tmp][y_tmp].Key1 || key2_old != mapList[x_tmp][y_tmp].Key2)
            {
                // cout<<"key 值需要更新"<<endl;

                //cout<<"新key值是:"<<mapList[x_tmp][y_tmp].Key1<<endl;

                //更新队列中该结点的key
                queue.pop();
                queue.push(mapList[x_tmp][y_tmp]);

                continue;
            }

            //如果该点rhs = g,则直接pop并进入下一循环,这条必须放在验证Key有效之后
            if(mapList[x_tmp][y_tmp].G == mapList[x_tmp][y_tmp].Rhs)
            {
                // cout<<"该点局部一致,踢出队列"<<endl;

                mapList[x_tmp][y_tmp].queue_flag = false;
                queue.pop();

                // cout<<"栈顶的x和y是:"<<queue.top().x<<" , "<<queue.top().y<<endl;
                // cout<<"该点是否在队列中："<<mapList[queue.top().x][queue.top().y].queue_flag<<endl;

                //如果是非正常结束循环，则声明路径未生成
                if (queue.empty() == true)
                {
                    path_flag = false;

                    return;
                }

                continue;
            }
        }

        //顺利取出该点后将该点踢出队列
        mapList[x_tmp][y_tmp].queue_flag = false;
        queue.pop();
        
        //局部过一致或者强制更新
        if (mapList[x_tmp][y_tmp].G > mapList[x_tmp][y_tmp].Rhs || mapList[x_tmp][y_tmp].forced_update_flag == true)
        {
            // cout<<"该点局部过一致"<<endl;

            mapList[x_tmp][y_tmp].G = mapList[x_tmp][y_tmp].Rhs;

            // cout<<"更新后的G:"<<mapList[x_tmp][y_tmp].G<<endl;

            //遍历周围的点
            for (size_t j = y_tmp-1; j <= y_tmp+1; j++)
            {
                for (size_t i = x_tmp-1; i <= x_tmp+1; i++)
                {
                    // rate.sleep();

                    //如果该点是终点，跳过
                    if (i == end_node->x && j == end_node->y)
                    {
                        continue;
                    }

                    //如果该点离障碍物太近，跳过
                    if (mapList[i][j].riskGrade >= 2)
                    {
                        mapList[i][j].G = inf;//保险

                        continue;
                    }

                    //如果该点超出地图边界，跳过
                    if(i >= col|| i <= 0|| j >= row || j <= 0)
                    continue;

                    //跳过本身
                    if (i == x_tmp && j == y_tmp)
                    {
                        continue;
                    }

                    //计算该点到周围点的总代价
                    double GpC = mapList[x_tmp][y_tmp].G + calculateDistance(i,x_tmp,j,y_tmp);

                    //如果周围点的版本不同，则强制更新周围点
                    if (mapList[i][j].version != map_version)
                    {
                        mapList[i][j].Rhs = GpC;
                        mapList[i][j].G = inf;
                        mapList[i][j].version = map_version;
                    }
                    //否则如果该点是周围点更好的successor，则更改
                    else
                    {
                        mapList[i][j].Rhs = mapList[i][j].Rhs < GpC ? mapList[i][j].Rhs : GpC;

                        //cout<<"Rhs-----GpC:"<< mapList[i][j].Rhs<<"-----"<<GpC<<endl;
                    }

                    updateVertex(i, j);//更新该节点
                }
            }

            mapList[x_tmp][y_tmp].forced_update_flag = false;
        }
        //局部欠一致
        else
        {
            // cout<<"该点局部欠一致"<<endl;

            double G_old = mapList[x_tmp][y_tmp].G;

            mapList[x_tmp][y_tmp].G = inf;

            // cout<<"更新后的G:"<<mapList[x_tmp][y_tmp].G<<endl;

            //遍历周围所有的点
            for (size_t j = y_tmp-1; j <= y_tmp+1; j++)
            {
                for (size_t i = x_tmp-1; i <= x_tmp+1; i++)
                {
                    // rate.sleep();
                    
                    //跳过终点
                    if (i == end_node->x && j == end_node->y)
                    {
                        continue;
                    }

                    //跳过本身
                    if (i == x_tmp && j == y_tmp)
                    {
                        continue;
                    }

                    //如果该点超出地图边界，跳过
                    if(i >= col|| i <= 0|| j >= row || j <= 0)
                    continue;

                    //如果周围点版本不同，将周围点的G和Rhs设置为inf
                    if (mapList[i][j].version != map_version)
                    {
                        mapList[i][j].G = inf;

                        mapList[i][j].Rhs = inf;
                    }
                    
                    //如果该点是周围点的successor,则需要重新寻找新的successor
                    if (mapList[i][j].Rhs <= calculateDistance(i,x_tmp,j,y_tmp) + G_old)
                    {
                        double Rhs_old = inf;

                        for (size_t n = j-1; n <= j+1; n++)
                        {
                            for (size_t m = i-1; m <= i+1; m++)
                            {
                                //不考虑不同版本的点作为successor
                                if (mapList[m][n].version != map_version)
                                {
                                    continue;
                                }

                                //如果该点超出地图边界，跳过
                                if(m >= col|| m <= 0|| n >= row || n <= 0)
                                continue;
                                
                                if (calculateDistance(m,i,n,j) + mapList[m][n].G < Rhs_old)
                                {
                                    mapList[i][j].Rhs = calculateDistance(m,i,n,j) + mapList[m][n].G;

                                    Rhs_old = mapList[i][j].Rhs;
                                }            
                            }
                        }

                        //更新周围点的版本
                        mapList[i][j].version = map_version;
                    }

                    updateVertex(i,j);
                }
            }
        }
    }

    ROS_INFO("计算完成");
    path_flag = true;
}


//生成路径
void class_plan::path_generator()
{
    ros::Rate rate(100);

    if (path_flag == false)
    {
        return;
    }

    //清空路径消息
    path_msg.poses.clear();

    //把终点重新设置为连续（这一步是为了防止程序死循环
    end_node->G = 0;
    end_node->Rhs = 0;
    
    //放一个临时节点，从起点开始储存
    node_tmp = mapList[start_node->x][start_node->y];

    ROS_INFO("开始生成路径");
    // cout<<"起点："<<start_node->x<<" , "<<start_node->y<<"          Key = "<<start_node->Key1<<endl;
    // cout<<"终点："<<end_node->x<<" , "<<end_node->y<<"          Key = "<<end_node->Key1<<endl;

    //寻找当前最优路径
    while (node_tmp.x != end_node->x || node_tmp.y != end_node->y)
    {
        // cout<<"寻找当前最优路径点:"<<node_tmp.x<<" , "<<node_tmp.y<<"    G:"<<mapList[node_tmp.x][node_tmp.y].G
        // <<"  key:"<<mapList[node_tmp.x][node_tmp.y].Key1<<endl;

        // rate.sleep();

        pubPath.publish(path_msg);//发布路径消息

        POSE.pose.position.x = node_tmp.x * map_resolution + origin_x;
        POSE.pose.position.y = node_tmp.y * map_resolution + origin_y;

        path_msg.poses.insert(path_msg.poses.begin(),POSE);

        double G = inf;
        int x,y;

        //寻找该路径点周围G最小的点
        for (size_t j = node_tmp.y - 1; j <= node_tmp.y + 1; j++)
        {
            for (size_t i = node_tmp.x - 1; i <= node_tmp.x + 1; i++)
            {
                //跳过局部不一致的点
                if (mapList[i][j].G != mapList[i][j].Rhs)
                {
                    // cout<<"该点的周围(局部不一致，跳过):"<<i<<" , "<<j<<"  flag:"<<mapList[i][j].queue_flag<<"  key:"<<mapList[i][j].Key1<<endl;

                    continue;
                }
                
                //跳过不同版本的点
                if (mapList[i][j].version != map_version)
                {
                    // cout<<"该点的周围(低版本，跳过):"<<i<<" , "<<j<<"  flag:"<<mapList[i][j].queue_flag<<"  key:"<<mapList[i][j].Key1<<endl;

                    continue;
                }

                // cout<<"该点的周围："<<i<<" , "<<j<<"  G:"<<mapList[i][j].G<<"  key:"<<mapList[i][j].Key1<<endl;
                
                if (G > mapList[i][j].G)
                {
                    G = mapList[i][j].G;
                    x = i;
                    y = j;
                }
            }
        }

        node_tmp.x = x;
        node_tmp.y = y;
    }

    //把终点也放进路径中
    POSE.pose.position.x = node_tmp.x * map_resolution + origin_x;
    POSE.pose.position.y = node_tmp.y * map_resolution + origin_y;
    POSE.pose.orientation = endPose.pose.orientation;

    // cout<<"到达终点:"<<node_tmp.x<<" , "<<node_tmp.y<<endl;

    //生成的路径是从终点开始描述的
    path_msg.poses.insert(path_msg.poses.begin(),POSE);

    //将生成的路径反过来，从起点开始描述
    tempPath.poses.clear();
    for (auto i : path_msg.poses)
    {
        POSE.pose = i.pose;
        tempPath.poses.insert(tempPath.poses.begin(),POSE);
    }
    path_msg.poses = tempPath.poses;

    //把起点的位姿放进起点
    path_msg.poses[0].pose.orientation = initialPose.pose.pose.orientation;

    //给路径消息的header赋值
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    ROS_INFO("路径生成完毕");
}


//发布
void class_plan::publisher()
{
    if (path_flag == true)
    {
        pubPath.publish(path_msg);//发布路径消息

        pubMap.publish(serchedmap);//发布搜索图

        ROS_INFO("规划完成,路径已发布");
    }
    else if(initialPose_flag && endPose_flag && map_flag)
    {
        ROS_WARN("未生成路径，请检查起点到终点是否存在路径");
    }
    cout<<"-------------------------分割线-----------------------------"<<endl;
}


//更新节点
void class_plan::updateVertex(int x, int y)
{
    // cout<<"updateVertex once"<<endl;

    // serchedmap.data[y * col + x ] = 100;
    
    //储存旧的key值
    double key1_old = mapList[x][y].Key1;
    double key2_old = mapList[x][y].Key2;

    //cout<<"旧key是:"<<key1_old<<endl;

    calculateKey(x , y);//计算新的key值

    //如果该点在队列中且键值需要更新，因为优先队列没法检索，所以只能重新push一个进去
    if ((key1_old != mapList[x][y].Key1 || key2_old != mapList[x][y].Key2) &&  mapList[x][y].queue_flag == true)
    {
        serchedmap.data[y * col + x ] = 50;
        queue.push(mapList[x][y]);
    }
    
    //如果rhs != g且该结点不再队列中，将该节点push进队列
    if ( mapList[x][y].G !=  mapList[x][y].Rhs &&  mapList[x][y].queue_flag == false)
    {
        // cout<<"push node to queue once . x , y:"<<x<<" , "<<y<<endl;

        serchedmap.data[y * col + x ] = 50;

        //将节点放进队列中
        mapList[x][y].queue_flag = true;
        queue.push(mapList[x][y]);
    }
    
    //队列处理的其他情况放进path_calculator函数里进行
}


//生成距离位图
void class_plan::CVmap_generator(const nav_msgs::OccupancyGrid& msg)
{
    image_map.create(row, col, CV_8UC1);    

    for(int j = 0; j < col; j++)
    {
        for(int i = 0; i < row; i++)
        image_map.at<uchar>(i, j) = 255; //opencv中255为白色，0为黑色
    }

    for(int j = 0;j < row;j++)
    {
        for(int i = 0;i < col;i++)
        {
            grid_map[i][j] = msg.data[j*col+i];//对二维地图进行构建  grid_map[行][列] ，OccupancyGrid地图中0为白色，100为黑色

            if(grid_map[i][j] == 100)
            {
                image_map.at<uchar>(i, j) = 0;
            }
        }
    }

    image_dis.create(row, col, CV_8UC1);    

    //设置距离位图  ，可以直接通过 image_dis.at<float>(y, x) 得到当前节点离最近障碍物的距离。(暂时别用double，会出错)
    distanceTransform(image_map, image_dis, CV_DIST_L2, 5, CV_32FC1);
}


//计算key值
void class_plan::calculateKey(int x, int y)
{
    double G = mapList[x][y].G;
    double Rhs =  mapList[x][y].Rhs;
    double min = G > Rhs? Rhs:G;
    mapList[x][y].H = epsilon * realDistance(x,start_node->x , y,start_node->y);
    mapList[x][y].Key1 = min +  mapList[x][y].H;
    mapList[x][y].Key2 = min;

    // cout<<"计算"<<x<<" , "<<y<<"的key值:"<<min<<"+"<<mapList[x][y].H<<"="<<mapList[x][y].Key1<<endl;
}


//两个节点的key值比较
bool class_plan::compareKey(double key11,double key12 , double key21,double key22)
{
    if (key11 > key21)
    {
        return true;
    }
    else if (key11 < key21)
    {
        return false;
    }
    else
    {
        return key12 > key22;
    }
}


//计算危险等级
double class_plan::calculateRiskGrade(int x , int y)
{
    double dis = image_dis.at<float>(x,y);

    if (dis > dis2wall)
    {
        return  -1;
    }
    else if (dis > dis2wall/2)
    {
        return dis2wall / dis;
    }
    else
    {
        return 3;
    }
}


//计算两点代价距离
double class_plan::calculateDistance(int x1, int x2, int y1, int y2)
{
    double d;

    if (mapList[x1][y1].riskGrade <= 1 && mapList[x2][y2].riskGrade <= 1)
    {
        d = 0;
    }
    else if(mapList[x1][y1].riskGrade <= 2 && mapList[x2][y2].riskGrade <= 2)
    {
        d = mapList[x1][y1].riskGrade > mapList[x2][y2].riskGrade ? mapList[x1][y1].riskGrade : mapList[x2][y2].riskGrade;
        d = (d - 0.5) * 10;//后面乘的那个系数越大，起点在障碍物附近的情况需要搜索的节点越多
    }
    else
    {
        d = inf;
    }

    return realDistance(x1,x2,y1,y2) + d;
}


//计算两点实际距离
double class_plan::realDistance(int x1, int x2, int y1, int y2)
{
    // return sqrt(pow(x1-x2, 2)+pow(y1-y2, 2)) ;//欧式距离
    return fabs(x1-x2)+fabs(y1-y2) ;//曼哈顿距离
    // return fabs(x1-x2)>fabs(y1-y2) ? fabs(x1-x2):fabs(y1-y2) ;//切比雪夫距离
}


//清空队列
void class_plan::queue_clean()
{
    while (queue.empty() == false)
    {
        mapList[queue.top().x][queue.top().y].queue_flag = false;
        queue.pop();
    }
}


//清空serchedmap
void class_plan::serchedmap_clean()
{
    for( int j = 0; j < row; j++)
    {
        for ( int i = 0; i < col; i++)
        {
            serchedmap.data[j * col + i] = 0;
        }
    }
}