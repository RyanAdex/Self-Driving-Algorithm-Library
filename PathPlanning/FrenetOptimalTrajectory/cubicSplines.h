#ifndef SPLINES_H
#define SPLINES_H
#endif

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <iterator>
#include <opencv.hpp>

using namespace std;

struct cSpline{
    cv::Mat_<double> a,b,c,d,h,x,y;
};

class cubicSpline{
private:
    cSpline spl;
    int ptsnum,splnum;
    cv::Mat calc_A(){
        cv::Mat_<double> A=cv::Mat_<double>::zeros(ptsnum,ptsnum);
        for(auto i=0;i<ptsnum;i++){
            if(i==0||i==splnum){
                A(i,i)=1;
                continue;
            }
            A(i,i-1)=spl.h(i-1);
            A(i,i)=2*(spl.h(i-1)+spl.h(i));
            A(i,i+1)=spl.h(i);
        }
        return A;
    }
    cv::Mat calc_B(){
        cv::Mat_<double> B=cv::Mat_<double>::zeros(ptsnum,1);
        for(auto i=0;i<ptsnum-2;i++){
            B(i+1,1)=3*(spl.a(i+2)-spl.a(i+1))/spl.h(i+1)-3*(spl.a(i+1)-spl.a(i))/spl.h(i);
            return B;
        }
    }
    int search_index(const double& x){
        auto lower = std::lower_bound(spl.x.begin(),spl.x.end(), x);
        return std::distance(spl.x.begin(),lower)
    }
public:
    void Spline(const cv::Mat_<double>& x,const cv::Mat_<double>& y){
        spl.x=x;
        spl.y=y;
        ptsnum=x.rows;
        splnum=ptsnum-1;
        spl.a=spl.y;
        for(auto i=0;i<splnum;i++){
            spl.h.push_back(spl.x(i+1)-spl.x(i));
        }
        cv::solve(calc_A(),calc_B(),spl.c,cv::DECOMP_LU);
        for(auto i=0;i<splnum;i++){
            spl.d.push_back(spl.c(i+1)-spl.c(i)/(3*spl.h(i));
            spl.b.push_back(spl.a(i+1)-spl.a(i)/spl.h(i)-spl.h(i)*(spl.c(i+1)+2*spl.c(i)/3));
        }
    }
    double calc(const double& t){
        if(t<spl.x(0)||t>spl.x(spl.x.rows-1))return NULL;
        int i=search_index(t);
        double dx=t-spl.x(i);
        return spl.a(i)+spl.b(i)*dx+spl.c(i)*pow(dx,2)+spl.d(i)*pow(dx,3);
    }
    double calc_d(const double& t){
        if(t<spl.x(0)||t>spl.x(spl.x.rows-1))return NULL;
        int i=search_index(t);
        double dx=t-spl.x(i);
        return spl.b(i)+2*spl.c(i)*dx+3*spl.d(i)*pow(dx,2);
    }
    double calc_dd(const double& t){
        if(t<spl.x(0)||t>spl.x(spl.x.rows-1))return NULL;
        int i=search_index(t);
        double dx=t-spl.x(i);
        return 2*spl.c(i)+6*spl.d(i)*dx;
    }
};

class cubicSpline2D{
private:
    cv::Mat_<double> s;
    cubicSpline sx,sy;
    int ptsnum,splnum;
    cv::Mat_<double> calc_s(const cv::Mat_<double>& x,const cv::Mat_<double>& y){
        cv::Mat_<double> dx,dy,ds,ss;
        double sum;
        for(auto i=0;i<splnum;i++)dx.push_back(x(i+1)-x(i));
        for(auto i=0;i<splnum;i++)dy.push_back(y(i+1)-y(i));
        cv::pow(dx.mul(dx)+dy.mul(dy),0.5,ds);
        for(auto i:ds){
            sum+=i;
            ss.push_back(sum);
        }
        return ss;
    }
public:
    cubicSpline2D(const cv::Mat_<double>& x,const cv::Mat_<double>& y){
        ptsnum=x.rows;
        splnum=ptsnum-1;
        s=calc_s(x,y);
        sx.Spline(s,x);
        sy.Spline(s,y);
    }
    cv::Point2d calc_position(const double& s){
        cv::Point2d p;
        p.x=sx.calc(s);
        p.y=sy.calc(s);
        return p;
    }

    double calc_curvature(const double& s){
        double dx=sx.calc(s);
        double ddx=sx.calc_dd(s);
        double dy=sy.calc_d(s);
        double ddy=sy.calc(s);
        return (ddy*dx-ddx*dy)/(pow(dx,2)+pow(dy,2));
    }

    double calc_yaw(const double& s){
        double dx=sx.calc_d(s);
        double dy=sy.calc_d(s);
        return atan2(dy,dx);
    }

    
};