#ifndef rs_head
#define rs_head
#include    <iostream>
#include    <vector>
#include    <cmath>
#include    <string>
#include    <Eigen/Dense> 
#include    "tf/transform_datatypes.h"


using namespace std;

const double pi = 3.1415926535;
const double inf = 9999;


//定义结构
struct path_msg
{
    string type;

    double t;

    double u;

    double v;
    
    double w;

    double x;

    double totalLength;
};

struct point
{
    double x;

    double y;

    double phi;
};

class rs_class
{
    public:

    Eigen::MatrixXd rotateMatrix{2,1};//旋转矩阵
    Eigen::MatrixXd translationMatrix{2,2};//平移向量

    int step;//每个弧的步数
    int k;//共有k段弧

    double radius;//最小转向半径

    point startPoint;//起始点
    point endPoint;//终止点
    point stadardizedPoint;//规范化的终止点

    vector<point> points;//路径点的向量，从起点开始到终点

    path_msg path;//路径信息

    rs_class(point startPoint , point endPoint , double radius , int step);//构造函数

    bool LpSpLp(double x , double y , double phi , double &t , double &u , double &v);

    bool LpSpRp(double x , double y , double phi , double &t , double &u , double &v);

    bool CSC(double x , double y , double phi , path_msg &path);

    bool LpRmL(double x , double y , double phi , double &t , double &u , double &v);

    bool CCC(double x , double y , double phi , path_msg &path);

    bool LpRupLumRm(double x , double y , double phi , double &t , double &u , double &v);

    bool LpRumLumRp(double x , double y , double phi , double &t , double &u , double &v);

    bool CCCC(double x , double y , double phi , path_msg &path);

    bool LpRmSmLm(double x , double y , double phi , double &t , double &u , double &v);

    bool LpRmSmRm(double x , double y , double phi , double &t , double &u , double &v);

    bool CCSC(double x , double y , double phi , path_msg &path);

    bool LpRmSLmRp(double x , double y , double phi , double &t , double &u , double &v);

    bool CCSCC(double x , double y , double phi , path_msg &path);

    void pointPath_generator(point startPoint , point endPoint , double radius , path_msg &path , double step);//用路径信息生成路径点集

    private:

    void creatTypes();//建立表格

    void coordStandardize(point startPoint , point endPoint , double radius);//坐标标准化

    double mod2pi(double x);//角度规范化，[-pi,pi]

    void polar(double x , double y , double &r , double &theta);//直角坐标转极坐标

    void tauOmega(double u , double v , double xi , double eta , double phi , double &tau , double &omega);

    void stepFoward(point start , point &end , double step , double radius , char type);//步进

    string type;//行进类型

    vector<string> types;//所有的行进类型

    path_msg path1;//路径信息1
    path_msg path2;//路径信息2
    path_msg path3;//路径信息3
    path_msg path4;//路径信息4
    path_msg path5;//路径信息5

    point point_now;//当前点
    point point_befor;//上一个点

    double x;//x轴坐标
    double y;//y轴坐标
    double alpha;//角度
    double phi;//角度
    double t;//第一个弧长
    double u;//第二个弧长
    double v;//第三个弧长
    double tau;
    double omega;
    double theta;//角度
};


#endif