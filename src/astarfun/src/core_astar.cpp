#include"astar/astar.h"
#include <ctime>

bool astar::start()
{
    return(map_flag && end_pose_flag && initial_pose_flag);
}

void astar::astarextend()
{
    // ros::Rate rate(10);//调试建议频率1～10hz

    for (size_t i = 0; i < image_dis.rows; i++)//重置地图（保存了父节点）
    {
        for (size_t j = 0; j < image_dis.cols; j++)
        {
            maplist[ j][i].y = i;
            maplist[ j][i].x = j;       
            maplist[ j][i].F = sqrt(col*col+row*row);
            maplist[ j][i].G = sqrt(col*col+row*row);
            maplist[ j][i].H = sqrt(col*col+row*row);
            maplist[ j][i].close_list = false;
            maplist[ j][i].open_list = false;
            maplist[ j][i].parent_node = NULL;
            maplist[j][i].pathNood = false;
        }
    }

    // cout<<"map reset"<<endl;

    // while (!node_queue.empty()) node_queue.pop();//清空队列
    priority_queue<node>empty;//也是清空队列
    swap(empty,node_queue);

    // cout<<"node_queue.empty is empty"<<endl;

    maplist[start_node->x][start_node->y].F = 0;
    maplist[start_node->x][start_node->y].G = 0;

    node_queue.push(maplist[start_node->x][start_node->y]);//把开始节点压入队列
    // cout<<"start_node x is:"<<start_node->x<<"____"<<"start_node y is:"<<start_node->y<<endl;
    // cout<<"end_node x is:"<<end_node->x<<"______"<<"end_node y is:"<<end_node->y<<endl;

    tmp_node.x = start_node->x;//初始化缓冲节点
    tmp_node.y = start_node->y;

    contTime = clock();

    while(tmp_node.x!=end_node->x||tmp_node.y!=end_node->y)//每个栅格为一个节点，探索完所有节点后停止
    {
        nowTime = clock();
        if ( (double)(nowTime - contTime) / CLOCKS_PER_SEC>=5.0)//如果超时了就停止搜索路径
        {
           ROS_ERROR("time's out");
           ROS_ERROR("Please reset the start or end point");
           
           time_flag = false;
           return;
        }
        
        // rate.sleep();
        tmp_node = node_queue.top();//找到队列中代价最小的节点作为父节点
        // cout<<"tmp_node x is:"<<tmp_node.x<<"____"<<"tmp_nood y is:"<<tmp_node.y<<endl;
        // cout<<"tmp_node F is:"<<tmp_node.F<<"____"<<"tmp_nood G is:"<<tmp_node.G<<"____"<<"tmp_nood H is:"<<tmp_node.H<<endl;
        node_queue.pop();//将代价最小的节点弹出（此节点最短路径已经找到）
        maplist[tmp_node.x][tmp_node.y].close_list = true;//加入close_list

        for(int i = -1;i<= 1;i++)
        {
            for(int j =-1;j <= 1;j++)
            {
                if(j == 0 && i == 0)//跳过本身
                continue;

                if(tmp_node.x + i >= col||tmp_node.x + i <= 0||tmp_node.y + j >= row || tmp_node.y + j <= 0)//跳过超过地图的点
                continue;

                if(image_dis.at<float>(tmp_node.y + j, tmp_node.x + i) < dis2wall/2)//跳过离障碍物太近的点
                continue;

                if(maplist[tmp_node.x + i][tmp_node.y + j].close_list)//跳过已放入close_list的点
                continue;
                
                if(maplist[tmp_node.x + i][tmp_node.y + j].open_list == false){

                    maplist[tmp_node.x + i][tmp_node.y + j].open_list = true;

                    maplist[tmp_node.x + i][tmp_node.y + j].G = maplist[tmp_node.x][tmp_node.y].G + ((fabs(i)+fabs(j)) > 1 ? 1.414:1);//更新G
                    
                    //更新H,欧拉距离
                    maplist[tmp_node.x + i][tmp_node.y + j].H = sqrt(pow(end_node->x-(tmp_node.x + i),2)+pow(end_node->y-(tmp_node.y + j),2));
                    //更新H,曼哈顿距离
                    // maplist[tmp_node.x + i][tmp_node.y + j].H = fabs(end_node->x-(tmp_node.x + i))+fabs(end_node->y-(tmp_node.y + j));
                    //更新H,切比雪夫距离
                    // maplist[tmp_node.x + i][tmp_node.y + j].H =(fabs(end_node->x-(tmp_node.x + i))>fabs(end_node->y-(tmp_node.y + j)) ? fabs(end_node->x-(tmp_node.x + i)):fabs(end_node->y-(tmp_node.y + j)));

                    //更新F,如果离障碍物太近就给极大的代价,而且越近越大
                    maplist[tmp_node.x + i][tmp_node.y + j].F = 
                    maplist[tmp_node.x + i][tmp_node.y + j].G 
                    + maplist[tmp_node.x + i][tmp_node.y + j].H 
                    + (image_dis.at<float>(tmp_node.y + j, tmp_node.x + i) < dis2wall?
                    (dis2wall - (image_dis.at<float>(tmp_node.y + j, tmp_node.x + i))) * sqrt(col*col+row*row):0);

                    maplist[tmp_node.x + i][tmp_node.y + j].parent_node = &maplist[tmp_node.x][tmp_node.y];//更新父节点

                    node_queue.push(maplist[tmp_node.x + i][tmp_node.y + j]);//将新节点压入队列
                 }
                 else if(maplist[tmp_node.x + i][tmp_node.y + j].G > maplist[tmp_node.x][tmp_node.y].G+((fabs(i)+fabs(j)) >1 ?1.414:1))//该节点是否有更小的G
                {
                    maplist[tmp_node.x + i][tmp_node.y + j].G = maplist[tmp_node.x][tmp_node.y].G + ((fabs(i)+fabs(j)) > 1 ? 1.4:1);//更新G

                    //更新F,如果离障碍物太近就给极大的代价,而且越近越大           
                    maplist[tmp_node.x + i][tmp_node.y + j].F = 
                    maplist[tmp_node.x + i][tmp_node.y + j].G 
                    + maplist[tmp_node.x + i][tmp_node.y + j].H 
                    + (image_dis.at<float>(tmp_node.y + j, tmp_node.x + i) < dis2wall?
                    (dis2wall - (image_dis.at<float>(tmp_node.y + j, tmp_node.x + i))) * sqrt(col*col+row*row):0);

                    maplist[tmp_node.x + i][tmp_node.y + j].parent_node = &maplist[tmp_node.x][tmp_node.y];//更新父节点
                }
            }
        }
    }
}


void astar::findPath()
{
    if (!time_flag)//如果超时了，就不建立路径
    {
       return;
    }

    cout<<"start find path"<<endl;
    
    finall_path = end_node;

    path_msg.poses.clear();

    while(finall_path)//将起点到终点的父节点依次连接，就得到了最终路径
    {
        // cout<<"building the path"<<endl;
        POSE_msg.pose.position.x = (finall_path->x)*map_resolution + origin_x;
        POSE_msg.pose.position.y = (finall_path->y)*map_resolution + origin_y;

        maplist[finall_path->x][finall_path->y].pathNood = true;//这条目前没啥用

        finall_path = finall_path->parent_node;

        path_msg.poses.insert(path_msg.poses.begin(), POSE_msg);
    }

    path_msg.header.stamp = ros::Time::now();

    path_msg.header.frame_id = "map";

    path_flag = true;

    path_ok.path_ok = true;//路径启用

    cout<<"path has been built and enabled"<<endl;

    cout<<"The distans is :"<<end_node->G<<endl;
}

void astar::reset( )
{
    initial_pose_flag = false;
    end_pose_flag = false;
    update_flag = false;
    
    cout<<"reset done"<<endl;
    cout<<"---------------------------------------------"<<endl;
}

