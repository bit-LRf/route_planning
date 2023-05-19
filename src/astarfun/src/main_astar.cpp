#include    "astar/astar.h"
#include    <ctime>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "main_astar");

    ROS_INFO("\033[1;32m---->\033[0m global path find Started.");

    ros::NodeHandle nh;

    astar Astar(nh);

    ros::Rate rate(20);//调试建议频率0.25hz，运行建议频率20hz
    
    clock_t startTime, endTime;

    while(ros::ok())
    {
        // cout<<"------------------------------------"<<endl;
        // cout << ros::Time::now()<< endl;
        // cout<<"mainfun is running"<<endl;

        ros::spinOnce();

        if(!Astar.pubPath())
        {
            // rate.sleep();
            
            continue;
        }
        startTime = clock();
        Astar.astarextend();
        Astar.findPath();
        endTime = clock();
        cout << "Building path cost: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
        Astar.reset();

        rate.sleep();
    }
}
