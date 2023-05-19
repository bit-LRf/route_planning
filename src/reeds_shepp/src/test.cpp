#include    "/home/lrf/ROS/route_planning/src/reeds_shepp/include/reeds_shepp/rs_head.h"

int main(int argc, char* argv[])
{
    point startPoint;

    startPoint.phi = 0;
    startPoint.x = 1;
    startPoint.y = 3;

    point endPoint;

    endPoint.phi = -pi/4;
    endPoint.x = 3;
    endPoint.y = 5;

    double radius = 0.6;

    double step = 60;

    rs_class rs(startPoint , endPoint , radius , step);

    path_msg  pathMsg;

    vector<point> pathPoints;

    pathMsg = rs.path;

    pathPoints = rs.points;

    string type = pathMsg.type;


    //-------------------------------------------------------------------------------------------------------------------------------------------
    //-------------------------------------以下是用来测试的部分，可以直接注释掉--------------------------------------------------
    //-------------------------------------------------------------------------------------------------------------------------------------------
    cout << pathMsg.type <<" : "<< pathMsg.t<<" / " << pathMsg.u <<" / "<< pathMsg.v <<" / "
    << pathMsg.w<<" / " << pathMsg.x <<"    length =  "<<pathMsg.totalLength * radius<<endl;
    

    // int k = rs.k;
    // int col = pathPoints.size() / k;
    // for (size_t i = 0; i < k; i++)
    // {
    //     for (size_t j = 0; j < col; j++)
    //     {
    //         cout<<pathPoints[i * col + j].x<<endl;
    //     }
    // }
    // cout<<"--------------------------------------------------------------"<<endl;
    // for (size_t i = 0; i < k; i++)
    // {
    //     for (size_t j = 0; j < col; j++)
    //     {
    //         cout<<pathPoints[i * col + j].y<<endl;
    //     }
    // }
    // cout<<"--------------------------------------------------------------"<<endl;
    // for (size_t i = 0; i < k; i++)
    // {
    //     for (size_t j = 0; j < col; j++)
    //     {
    //         cout<<pathPoints[i * col + j].phi<<endl;
    //     }
    // }

    return 0;
}
