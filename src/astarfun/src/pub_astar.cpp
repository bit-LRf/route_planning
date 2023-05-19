#include"astar/astar.h"

bool astar::pubPath()
{
    // cout<<"update_flag is"<<update_flag<<endl;
    // cout<<"initial_pose_flag is"<<initial_pose_flag<<endl;
    // cout<<"end_pose_flag is"<<end_pose_flag<<endl;
    // cout<<"map_flag is"<<map_flag<<endl;
    // cout<<"map_update_flag is"<<map_update_flag<<endl;

    // if(map_flag)
    // {
    //     cout<<"start pub the map"<<endl;
    //     pubMap.publish(map);
    // }

    pub_path_ok.publish(path_ok);//发布路径启用情况
    
    if (!path_ok.path_ok&&path_flag&&!time_flag)
    {
        cout<<"path has been disabled"<<endl;
    }
    
//判断是否完成了路径生成
//1.啥也没收到：返回false ；
//2.部分收到了并且还没有发布路径：返回false；
//3.部分收到了并且已经发布了路径：返回true；
//4.全部收到了但是还没有生成路径：返回true；
//5.全部收到并生成了路径：发布路径并返回false；
    if(!start()&&!update_flag)                 
    {                                   
        if(pub_flag&&path_flag&&path_ok.path_ok)//生成了有效路径并且需要更新时       
        {
            pubThePath.publish(path_msg);//发布路径

            cout<<"now pub the path"<<endl;
            cout<<"---------------------------------------------"<<endl;

            pub_flag = false;//重置地图更新记号
        }

        time_flag = true;//重置时间记号

        return false;
    }

    time_flag = true;//重置时间记号

    return true;
}
