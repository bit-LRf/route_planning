#include    "/home/lrf/ROS/route_planning/src/reeds_shepp/include/reeds_shepp/rs_head.h"

rs_class::rs_class(point startPoint , point endPoint , double radius , int step)
{
    creatTypes();

    type = "NNNNN";
    path.type = type;
    path.t = 0;
    path.u = 0;
    path.v = 0;
    path.w = 0;
    path.x = 0;
    path.totalLength = inf;

    coordStandardize(startPoint , endPoint , radius);//将终点标准化
    
    // cout<<startPoint.x<<endl;
    // cout<<startPoint.y<<endl;
    // cout<<startPoint.phi<<endl;
    // cout<<endPoint.x<<endl;
    // cout<<endPoint.y<<endl;
    // cout<<endPoint.phi<<endl;
    // cout<<stadardizedPoint.x<<endl;
    // cout<<stadardizedPoint.y<<endl;
    // cout<<stadardizedPoint.phi<<endl;

    bool csc = CSC(stadardizedPoint.x , stadardizedPoint.y , stadardizedPoint.phi , path1);
    if(csc && path1.totalLength < path.totalLength)
    {
        path = path1;
    }
    bool ccc = CCC(stadardizedPoint.x , stadardizedPoint.y , stadardizedPoint.phi , path2);
    if(ccc && path2.totalLength < path.totalLength)
    {
        path = path2;
    }
    bool cccc = CCCC(stadardizedPoint.x , stadardizedPoint.y , stadardizedPoint.phi , path3);
    if(cccc && path3.totalLength < path.totalLength)
    {
        path = path3;
    }
    bool ccsc = CCSC(stadardizedPoint.x , stadardizedPoint.y , stadardizedPoint.phi , path4);
    if(ccsc && path4.totalLength < path.totalLength)
    {
        path = path4;
    }
    bool ccscc = CCSCC(stadardizedPoint.x , stadardizedPoint.y , stadardizedPoint.phi , path5);
    if(ccscc && path5.totalLength < path.totalLength)
    {
        path = path5;
    }
    
    pointPath_generator(startPoint , endPoint , radius , path , step);
}

void rs_class::creatTypes()
{
    types.resize(19);

    int i = 0;

   //0
    type = "NNNNN";
    types[i] = type;
    i++;

    //1
    type = "LRLNN";
    types[i] = type;
    i++;

    //2
    type = "RLRNN";
    types[i] = type;
    i++;

    //3
    type = "LRLRN";
    types[i] = type;
    i++;

    //4
    type = "RLRLN";
    types[i] = type;
    i++;

    //5
    type = "LRSLN";
    types[i] = type;
    i++;

    //6
    type = "RLSRN";
    types[i] = type;
    i++;

    //7
    type = "LSRLN";
    types[i] = type;
    i++;

    //8
    type = "RSLRN";
    types[i] = type;
    i++;

    //9
    type = "LRSRN";
    types[i] = type;
    i++;

    //10
    type = "RLSLN";
    types[i] = type;
    i++;

    //11
    type = "RSRLN";
    types[i] = type;
    i++;

    //12
    type = "LSLRN";
    types[i] = type;
    i++;

    //13
    type = "LSRNN";
    types[i] = type;
    i++;

    //14
    type = "RSLNN";
    types[i] = type;
    i++;

    //15
    type = "LSLNN";
    types[i] = type;
    i++;

    //16
    type = "RSRNN";
    types[i] = type;
    i++;

    //17
    type = "LRSLR";
    types[i] = type;
    i++;

    //18
    type = "RLSRL";
    types[i] = type;
}

void rs_class::coordStandardize(point startPoint , point endPoint , double radius)
{
    translationMatrix.resize(2,1);

    rotateMatrix.resize(2,2);

    //计算平移向量
    translationMatrix << 
    -startPoint.x,
    -startPoint.y;

    // cout<<translationMatrix(0)<<endl;
    // cout<<translationMatrix(1)<<endl;

    //计算旋转矩阵
    alpha = -startPoint.phi;

    rotateMatrix << 
    cos(alpha) , -sin(alpha),
    sin(alpha) , cos(alpha);

    //将坐标规范化,因为起始点在原点就不计算了
    //平移
    endPoint.x = endPoint.x + translationMatrix(0);
    endPoint.y = endPoint.y + translationMatrix(1);

    //旋转
    stadardizedPoint.phi = endPoint.phi + alpha;
    stadardizedPoint.x = endPoint.x*rotateMatrix(0,0) + endPoint.y*rotateMatrix(0,1);
    stadardizedPoint.y = endPoint.x*rotateMatrix(1,0) + endPoint.y*rotateMatrix(1,1);

    //缩放
    stadardizedPoint.x = stadardizedPoint.x / radius;
    stadardizedPoint.y = stadardizedPoint.y / radius;

    //把endPoint还原
    endPoint.x = endPoint.x - translationMatrix(0);
    endPoint.y = endPoint.y - translationMatrix(1);
}

double rs_class::mod2pi(double x)
{
    double v = fmod(x, 2*pi);
    if (v < -pi)
        v += 2*pi;
    else if (v > pi)
        v -= 2*pi;
    return v;
}

void rs_class::polar(double x , double y , double &r , double &theta)
{
    r = sqrt(x*x + y*y);

    theta = atan2(y , x);
}

void rs_class::tauOmega(double u , double v , double xi , double eta , double phi , double &tau , double &omega)
{
    double delta = mod2pi(u - v);
    double A = sin(u) - sin(delta);
    double B = cos(u) - cos(delta) - 1;
    double t1 = atan2(eta*A - xi*B , xi*A + eta*B);
    double t2 = 2*(cos(delta) - cos(v) - cos(u)) + 3;
    tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
    omega = mod2pi(tau - u + v - phi);
}

bool rs_class::LpSpLp(double x , double y , double phi , double &t , double &u , double &v)
{
    polar(x - sin(phi), y - 1. + cos(phi), u, t);

    if (t >= 0)
    {
        v = mod2pi(phi - t);

        if (v >= 0)
        {
            return true;
        }
    }

    return false;
}

bool rs_class::LpSpRp(double x , double y , double phi , double &t , double &u , double &v)
{
    double t1, u1;

    polar(x + sin(phi), y - 1. - cos(phi), u1, t1);

    u1 = u1 * u1;

    if (u1 >= 4.)
    {
        double theta;

        u = sqrt(u1 - 4.);

        theta = atan2(2., u);

        t = mod2pi(t1 + theta);
        
        v = mod2pi(t - phi);

        return t >= 0 && v >= 0;
    }
    return false;
}

bool rs_class::CSC(double x , double y , double phi , path_msg &path)
{
    double Lmin = inf;
    double t , u , v , L;
    type = "NNNNN";
    path.type = type;
    path.t = 0;
    path.u = 0;
    path.v = 0;
    path.w = 0;
    path.x = 0;
    path.totalLength = 0;

    if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
    {
        Lmin = L;
        path.type = types[15];
        path.t = t;
        path.u = u;
        path.v = v;
        path.w = 0;
        path.x = 0;
    }

    if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//timeflip
    {
        Lmin = L;
        path.type = types[15];
        path.t = -t;
        path.u = -u;
        path.v = -v;
        path.w = 0;
        path.x = 0;
    }

    if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//reflect
    {
        Lmin = L;
        path.type = types[16];
        path.t = t;
        path.u = u;
        path.v = v;
        path.w = 0;
        path.x = 0;
    }

    if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//timeflp,reflect
    {
        Lmin = L;
        path.type = types[16];
        path.t = -t;
        path.u = -u;
        path.v = -v;
        path.w = 0;
        path.x = 0;
    }

    if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
    {
        Lmin = L;
        path.type = types[13];
        path.t = t;
        path.u = u;
        path.v = v;
        path.w = 0;
        path.x = 0;
    }

    if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//timeflip
    {
        Lmin = L;
        path.type = types[13];
        path.t = -t;
        path.u = -u;
        path.v = -v;
        path.w = 0;
        path.x = 0;
    }

    if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//reflect
    {
        Lmin = L;
        path.type = types[14];
        path.t = t;
        path.u = u;
        path.v = v;
        path.w = 0;
        path.x = 0;
    }

    if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//timeflip,reflect
    {
        Lmin = L;
        path.type = types[14];
        path.t = -t;
        path.u = -u;
        path.v = -v;
        path.w = 0;
        path.x = 0;
    }

    if(Lmin == inf)
    {
        return false;
    }
    else
    {
        path.totalLength = Lmin;

        return true;
    }
}

bool rs_class::LpRmL(double x , double y , double phi , double &t , double &u , double &v)
{
    double xi = x - sin(phi);
    double eta = y - 1. + cos(phi);
    double u1;
    double theta;

    polar(xi, eta, u1, theta);
    
    if (u1 <= 4.)
    {
        u = -2. * asin(.25 * u1);

        t = mod2pi(theta + .5 * u + pi);

        v = mod2pi(phi - t + u);

        return t >= 0 && u <=0;
    }

    return false;
}

bool rs_class::CCC(double x , double y , double phi , path_msg &path)
{
    double Lmin = inf;
    double t , u , v , L;
    type = "NNNNN";
    path.type = type;
    path.t = 0;
    path.u = 0;
    path.v = 0;
    path.w = 0;
    path.x = 0;
    path.totalLength = 0;

    if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
    {
        Lmin = L;
        path.type = types[1];
        path.t = t;
        path.u = u;
        path.v = v;
        path.w = 0;
        path.x = 0;
    }

    if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//timeflip
    {
        Lmin = L;
        path.type = types[1];
        path.t = -t;
        path.u = -u;
        path.v = -v;
        path.w = 0;
        path.x = 0;
    }

    if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//reflect
    {
        Lmin = L;
        path.type = types[2];
        path.t = t;
        path.u = u;
        path.v = v;
        path.w = 0;
        path.x = 0;
    }

    if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//timeflip,reflect
    {
        Lmin = L;
        path.type = types[2];
        path.t = -t;
        path.u = -u;
        path.v = -v;
        path.w = 0;
        path.x = 0;
    }

    //backwards
    double xb = x * cos(phi) + y * sin(phi);
    double yb = x * sin(phi) - y * cos(phi);

    if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
    {
        Lmin = L;
        path.type = types[1];
        path.t = v;
        path.u = u;
        path.v = t;
        path.w = 0;
        path.x = 0;
    }

    if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//timeflip
    {
        Lmin = L;
        path.type = types[1];
        path.t = -v;
        path.u = -u;
        path.v = -t;
        path.w = 0;
        path.x = 0;
    }

    if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//reflect
    {
        Lmin = L;
        path.type = types[2];
        path.t = v;
        path.u = u;
        path.v = t;
        path.w = 0;
        path.x = 0;
    }

    if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))//timeflip,reflect
    {
        Lmin = L;
        path.type = types[2];
        path.t = -v;
        path.u = -u;
        path.v = -t;
        path.w = 0;
        path.x = 0;
    }

    if(Lmin == inf)
    {
        return false;
    }
    else
    {
        path.totalLength = Lmin;

        return true;
    }
}

bool rs_class::LpRupLumRm(double x , double y , double phi , double &t , double &u , double &v)
{
    double xi = x + sin(phi);
    double eta = y - 1. - cos(phi);
    double rho = .25 * (2. + sqrt(xi * xi + eta * eta));

    if (rho <= 1.)
    {
        u = acos(rho);

        tauOmega(u, -u, xi, eta, phi, t, v);

        return t >= 0 && v <= 0;
    }

    return false;
}

bool rs_class::LpRumLumRp(double x , double y , double phi , double &t , double &u , double &v)
{
    double xi = x + sin(phi);
    double eta = y - 1. - cos(phi);
    double rho = (20. - xi * xi - eta * eta) / 16.;

    if (rho >= 0 && rho <= 1)
    {
        u = -acos(rho);

        if (u >= -0.5 * pi)
        {
            tauOmega(u, u, xi, eta, phi, t, v);

            return t >= 0 && v >= 0;
        }
    }

    return false;
}

bool rs_class::CCCC(double x , double y , double phi , path_msg &path)
{
    double Lmin = inf;
    double t , u , v , L;
    type = "NNNNN";
    path.type = type;
    path.t = 0;
    path.u = 0;
    path.v = 0;
    path.w = 0;
    path.x = 0;
    path.totalLength = 0;
    
    if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
    {
        Lmin = L;
        path.type = types[3];
        path.t = t;
        path.u = u;
        path.v = -u;
        path.w = v;
        path.x = 0;
    }

    if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
    {
        Lmin = L;
        path.type = types[3];
        path.t = -t;
        path.u = -u;
        path.v = u;
        path.w = -v;
        path.x = 0;
    }

    if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
    {
        Lmin = L;
        path.type = types[4];
        path.t = t;
        path.u = u;
        path.v = -u;
        path.w = v;
        path.x = 0;
    }
    if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
    {
        Lmin = L;
        path.type = types[4];
        path.t = -t;
        path.u = -u;
        path.v = u;
        path.w = -v;
        path.x = 0;
    }

    if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
    {
        Lmin = L;
        path.type = types[3];
        path.t = t;
        path.u = u;
        path.v = u;
        path.w = v;
        path.x = 0;
    }

    if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
    {
        Lmin = L;
        path.type = types[3];
        path.t = -t;
        path.u = -u;
        path.v = -u;
        path.w = -v;
        path.x = 0;
    }

    if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
    {
        Lmin = L;
        path.type = types[4];
        path.t = t;
        path.u = u;
        path.v = u;
        path.w = v;
        path.x = 0;
    }

    if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
    {
        Lmin = L;
        path.type = types[4];
        path.t = -t;
        path.u = -u;
        path.v = -u;
        path.w = -v;
        path.x = 0;
    }

    if(Lmin == inf)
    {
        return false;
    }
    else
    {
        path.totalLength = Lmin;

        return true;
    }
}

bool rs_class::LpRmSmLm(double x , double y , double phi , double &t , double &u , double &v)
{
    double xi = x - sin(phi);
    double eta = y - 1. + cos(phi);
    double rho;
    double theta;

    polar(xi, eta, rho, theta);

    if (rho >= 2.)
    {
        double r = sqrt(rho * rho - 4.);

        u = 2. - r;

        t = mod2pi(theta + atan2(r, -2.));

        v = mod2pi(phi - .5 * pi - t);

        return t >= 0 && u <= 0 && v <= 0;
    }
    return false;
}

bool rs_class::LpRmSmRm(double x , double y , double phi , double &t , double &u , double &v)
{
    double xi = x + sin(phi);
    double eta = y - 1. - cos(phi);
    double rho;
    double theta;

    polar(-eta, xi, rho, theta);

    if (rho >= 2.)
    {
        t = theta;

        u = 2. - rho;

        v = mod2pi(t + .5 * pi - phi);

        return t >= 0 && u <= 0 && v <= 0;
    }
    return false;
}

bool rs_class::CCSC(double x , double y , double phi , path_msg &path)
{
    double Lmin = inf;
    double t , u , v , L;
    type = "NNNNN";
    path.type = type;
    path.t = 0;
    path.u = 0;
    path.v = 0;
    path.w = 0;
    path.x = 0;
    path.totalLength = 0;

    if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))
    {
        Lmin = L;
        path.type = types[5];
        path.t = t;
        path.u = -0.5*pi;
        path.v = u;
        path.w = v;
        path.x = 0;
    }

    if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // timeflip
    {
        Lmin = L;
        path.type = types[5];
        path.t = -t;
        path.u = 0.5*pi;
        path.v = -u;
        path.w = -v;
        path.x = 0;
    }

    if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // reflect
    {
        Lmin = L;
        path.type = types[6];
        path.t = t;
        path.u = -0.5*pi;
        path.v = u;
        path.w = v;
        path.x = 0;
    }

    if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // timeflip + reflect
    {
        Lmin = L;
        path.type = types[6];
        path.t = -t;
        path.u = 0.5*pi;
        path.v = -u;
        path.w = -v;
        path.x = 0;
    }

    if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))
    {
        Lmin = L;
        path.type = types[9];
        path.t = t;
        path.u = -0.5*pi;
        path.v = u;
        path.w = v;
        path.x = 0;
    }

    if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // timeflip
    {
        Lmin = L;
        path.type = types[9];
        path.t = -t;
        path.u = 0.5*pi;
        path.v = -u;
        path.w = -v;
        path.x = 0;
    }

    if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // reflect
    {
        Lmin = L;
        path.type = types[10];
        path.t = t;
        path.u = -0.5*pi;
        path.v = u;
        path.w = v;
        path.x = 0;
    }
    if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // timeflip + reflect
    {
        Lmin = L;
        path.type = types[10];
        path.t = -t;
        path.u = 0.5*pi;
        path.v = -u;
        path.w = -v;
        path.x = 0;
    }

    // backwards
    double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
    if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))
    {
        Lmin = L;
        path.type = types[7];
        path.t = v;
        path.u = u;
        path.v = -0.5*pi;
        path.w = t;
        path.x = 0;
    }

    if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // timeflip
    {
        Lmin = L;
        path.type = types[7];
        path.t = -v;
        path.u = -u;
        path.v = 0.5*pi;
        path.w = -t;
        path.x = 0;
    }

    if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // reflect
    {
        Lmin = L;
        path.type = types[8];
        path.t = v;
        path.u = u;
        path.v = -0.5*pi;
        path.w = t;
        path.x = 0;
    }
    if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // timeflip + reflect
    {
        Lmin = L;
        path.type = types[8];
        path.t = -v;
        path.u = -u;
        path.v = 0.5*pi;
        path.w = -t;
        path.x = 0;
    }

    if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))
    {
        Lmin = L;
        path.type = types[11];
        path.t = v;
        path.u = u;
        path.v = -0.5*pi;
        path.w = t;
        path.x = 0;
    }

    if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // timeflip
    {
        Lmin = L;
        path.type = types[11];
        path.t = -v;
        path.u = -u;
        path.v = 0.5*pi;
        path.w = -t;
        path.x = 0;
    }

    if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // reflect
    {
        Lmin = L;
        path.type = types[12];
        path.t = v;
        path.u = u;
        path.v = -0.5*pi;
        path.w = t;
        path.x = 0;
    }

    if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + 0.5*pi))  // timeflip + reflect
    {
        Lmin = L;
        path.type = types[12];
        path.t = -v;
        path.u = -u;
        path.v = 0.5*pi;
        path.w = -t;
        path.x = 0;
    }

    if(Lmin == inf)
    {
        return false;
    }
    else
    {
        path.totalLength = Lmin;

        return true;
    }
}

bool rs_class::LpRmSLmRp(double x , double y , double phi , double &t , double &u , double &v)
{
    double xi = x + sin(phi);
    double eta = y - 1. - cos(phi);
    double rho;
    double theta;

    polar(xi, eta, rho, theta);

    if (rho >= 2.)
    {
        u = 4. - sqrt(rho * rho - 4.);

        if (u <= 0)
        {
            t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));

            v = mod2pi(t - phi);

            return t >= 0 && v >= 0;
        }
    }

    return false;
}

bool rs_class::CCSCC(double x , double y , double phi , path_msg &path)
{
    double Lmin = inf;
    double t , u , v , L;
    type = "NNNNN";
    path.type = type;
    path.t = 0;
    path.u = 0;
    path.v = 0;
    path.w = 0;
    path.x = 0;
    path.totalLength = 0;

    if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + pi))
    {
        Lmin = L;
        path.type = types[17];
        path.t = t;
        path.u = -0.5*pi;
        path.v = u;
        path.w = -0.5*pi;
        path.x = v;
    }

    if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + pi))  // timeflip
    {
        Lmin = L;
        path.type = types[17];
        path.t = -t;
        path.u = 0.5*pi;
        path.v = -u;
        path.w = 0.5*pi;
        path.x = v;
    }
    
    if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + pi))  // reflect
    {
        Lmin = L;
        path.type = types[18];
        path.t = t;
        path.u = -0.5*pi;
        path.v = u;
        path.w = -0.5*pi;
        path.x = v;
    }

    if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v) + pi))  // timeflip + reflect
    {
        Lmin = L;
        path.type = types[18];
        path.t = -t;
        path.u = 0.5*pi;
        path.v = -u;
        path.w = 0.5*pi;
        path.x = -v;
    }

    if(Lmin == inf)
    {
        return false;
    }
    else
    {
        path.totalLength = Lmin;

        return true;
    }
}

void rs_class::pointPath_generator(point startPoint , point endPoint , double radius , path_msg &path , double step)
{
    point_befor = startPoint;

    //读取路径类型并开始生成路径点
    type = path.type;
    
    //计算一共有几段曲线
    k = 0;
    while(type[k] != 'N')
    {
        k = k+1;
    }
    
    int col = step / k;//每段的点个数

    points.resize(col * k);//设置路径点向量长度
    
    vector<double> ss;
    ss.resize(5);
    ss[0] = path.t;
    ss[1] = path.u;
    ss[2] = path.v;
    ss[3] = path.w;
    ss[4] = path.x;

    for (size_t i = 0; i < k; i++)
    {
        //计算当前曲线点间距(弧度)
        double d = ss[i];
        d = d / col;

        for (size_t j = 0; j < col; j++)
        {
            stepFoward(point_befor , point_now , d , radius , type[i]);

            points[i * col + j] = point_now;

            point_befor = point_now;
        }
    }
}

void rs_class::stepFoward(point start , point &end , double step , double radius , char type)//步进
{
    if (type == 'L')
    {
        end.phi = start.phi + step;

        end.x = start.x + radius*sin(end.phi) - radius*sin(start.phi);

        end.y = start.y - radius*cos(end.phi) + radius*cos(start.phi);
    }
    else if (type == 'R')
    {
        end.phi = start.phi - step;

        end.x = start.x - radius*sin(end.phi) + radius*sin(start.phi);

        end.y = start.y + radius*cos(end.phi) - radius*cos(start.phi);
    }
    else if (type == 'S')
    {
        end.phi = start.phi;

        end.x = start.x + radius*step*cos(end.phi);

        end.y = start.y + radius*step*sin(end.phi);
    }
    else if(type == 'N')
    {
        return;
    }
}
