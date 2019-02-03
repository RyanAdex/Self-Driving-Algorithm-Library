#ifndef FRENET_H
#define FRENET_H
#endif
#include <iostream>
#include <vector>
#include <opencv.hpp>
#include <math.h>
#include <./cubicSplines.h>
using namespace std;
#define MAX_SPEED 50.0/3.6  //maximum speed [m/s]
#define MAX_ACCEL 2.0  //maximum acceleration [m/ss]
#define MAX_CURVATURE 1.0  //maximum curvature [1/m]
#define MAX_ROAD_WIDTH 7.0  //maximum road width [m]
#define D_ROAD_W 1.0  //road width sampling length [m]
#define DT 0.2  //time tick [s]
#define MAXT 5.0  //max prediction time [m]
#define MINT 4.0  //min prediction time [m]
#define TARGET_SPEED 30.0 / 3.6  //target speed [m/s]
#define D_T_S 5.0 / 3.6  //target speed sampling length [m/s]
#define N_S_SAMPLE 1  //sampling number of target speed
#define ROBOT_RADIUS 2.0  //robot radius [m]
//cost weights
#define KJ 0.1
#define KT 0.1
#define KD 1.0
#define KLAT 1.0
#define KLON 1.0

struct Frenet{
    vector<double> t,d,d_d,d_dd,d_ddd,s,s_d,s_dd,s_ddd,x,y,yaw,ds,c;
    double cd,cv,cf;
};
class quintic_polynomial{
private:
    double a0,a1,a2,a3,a4,a5;
public:
    quintic_polynomial(const double xs[3],const double xe[3],const double& T);
    double calc_point(double& t);
    double calc_first_derivative(double& t);
    double calc_second_derivative(double& t);
    double calc_third_derivative(double& t);
};

class quartic_polynomial{
private:
    double a0,a1,a2,a3,a4;
public:
    quartic_polynomial(const double xs[3],const double xe[3],const double& T);
    double calc_point(double& t);
    double calc_first_derivative(double& t);
    double calc_second_derivative(double& t);
    double calc_third_derivative(double& t);
};
//计算所有局部路径
vector<Frenet> calc_frenet_paths(const double& c_speed,const double c[3],const double& s0);

void calc_global_paths(const vector<Frenet>& fplist,const vector<cv::Point>& csp);

bool check_collision(const Frenet& fp,const vector<cv::Point2d>& ob);

vector<Frenet> check_paths(const vector<Frenet>& fplist,const vector<cv::Point2d>& ob);

