#include <Frenet.h>

quintic_polynomial::quintic_polynomial(const double xs[3],const double xe[3],const double& T){
    a0 = xs[0];
    a1 = xs[1];
    a2 = xs[2]/2;
    cv::Mat_<double> A = (cv::Mat_<double>(3,3) <<
        3 * pow(T,2), 4 * pow(T,3), 5 * pow(T,4),
        pow(T,3), pow(T,4), pow(T,5),
        6 * T, 12 * pow(T,2), 20 * pow(T,3));
    cv::Mat_<double> b = (cv::Mat_<double>(3,1) <<
        xe[0]-a0-a1*T-a2*pow(T,2),
        xe[1]-a1-2*a2*T,
        xe[2]-2*a2);
    cv::Mat_<double> x;
    cv::solve(A,b,x,cv::DECOMP_LU);
    a3 = x(0);
    a4 = x(1);
    a5 = x(2);
}
double quintic_polynomial::calc_point(double& t){
    return a0+a1*t+a2*pow(t,2)+a3*pow(t,3)+a4*pow(t,4)+a5*pow(t,5);
}
double quintic_polynomial::calc_first_derivative(double& t){
    return a1+2*a2*t+3*a3*pow(t,2)+4*a4*pow(t,3)+5*a5*pow(t,4);
}
double quintic_polynomial::calc_second_derivative(double& t){
    return 2*a2+6*a3*t+12*a4*pow(t,2)+20*a5*pow(t,3);
}
double quintic_polynomial::calc_third_derivative(double& t){
    return 6*a3+24*a4*t+60*a5*pow(t,2);
}
//------------------------------
quartic_polynomial::quartic_polynomial(const double xs[3],const double xe[2],const double& T){
    a0 = xs[0];
    a1 = xs[1];
    a2 = xs[2]/2;
    cv::Mat_<double> A = (cv::Mat_<double>(2,2) <<
        3 * pow(T,2), 4 * pow(T,3),
        6 * T, 12 * pow(T,2));
    cv::Mat_<double> b = (cv::Mat_<double>(2,1) <<
        xe[0]-a1-2*a2*T,
        xe[1]-2*a2);
    cv::Mat_<double> x;
    cv::solve(A,b,x,cv::DECOMP_LU);
    a3 = x(0);
    a4 = x(1);
}
double quartic_polynomial::calc_point(double& t){
    return a0+a1*t+a2*pow(t,2)+a3*pow(t,3)+a4*pow(t,4);
}
double quartic_polynomial::calc_first_derivative(double& t){
    return a1+2*a2*t+3*a3*pow(t,2)+4*a4*pow(t,3);
}
double quartic_polynomial::calc_second_derivative(double& t){
    return 2*a2+6*a3*t+12*a4*pow(t,2);
}
double quartic_polynomial::calc_third_derivative(double& t){
    return 6*a3+24*a4*t;
}
//计算基于Frenet坐标系s0点所有轨迹的集合
vector<Frenet> calc_frenet_paths(const double& c_speed,const double c[3],const double& s0){
    vector<Frenet> frenet_paths;
    double d[3]={0,0,0};
    double s[3]={s0,c_speed,0};
    double stv[2]={0,0};
    for(auto di=-MAX_ROAD_WIDTH;di<=MAX_ROAD_WIDTH;di+=D_ROAD_W){
        d[0]=di;
        for(auto Ti=MINT;Ti<=MAXT;Ti+=DT){
            Frenet fp;//深拷贝
            quintic_polynomial lat_qp(c,d,Ti);
            //TODO：可将时间复杂度简化
            for(auto t=0;t<=Ti;t+=DT)fp.t.push_back(t);
            for(auto& t:fp.t){
                fp.d.push_back(lat_qp.calc_point(t));
                fp.d_d.push_back(lat_qp.calc_first_derivative(t));
                fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
            }
            for(auto tv=TARGET_SPEED - D_T_S * N_S_SAMPLE;tv<=TARGET_SPEED + D_T_S * N_S_SAMPLE;tv+=D_T_S){
                Frenet tfp(fp);
                stv[0]=tv;
                quartic_polynomial lon_qp(s,stv,Ti);
                for(auto& t:fp.t){
                    tfp.s.push_back(lon_qp.calc_point(t));
                    tfp.s_d.push_back(lon_qp.calc_first_derivative(t));
                    tfp.s_dd.push_back(lon_qp.calc_second_derivative(t));
                    tfp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
                }
                vector<double> sojp,sojs;
                cv::pow(tfp.d_ddd,2,sojp);
                cv::pow(tfp.s_ddd,2,sojs);
                auto Jp=cv::sum(sojp).val[0];
                auto Js=cv::sum(sojs).val[0];
                auto ds=pow(TARGET_SPEED-tfp.s_d.back(),2);
                tfp.cd=KJ*Jp+KT*Ti+KD*pow(tfp.d.back(),2);
                tfp.cv=KJ*Js+KT*Ti+KD*ds;
                tfp.cf=KLAT*tfp.cd+KLON*tfp.cv;

                frenet_paths.push_back(tfp);
            }
        }
    }
    return frenet_paths;
}

void calc_global_paths(vector<Frenet>& fplist,cubicSpline2D& csp){
    for(auto& fp:fplist){
        //计算全局坐标
        for(int i=0;i<fp.s.size();i++){
            cv::Point2d ip=csp.calc_position(fp.s[i]);
            if(ip.x==NULL)break;
            double iyaw=csp.calc_yaw(fp.s[i]);
            double di=fp.d[i];
            fp.x.push_back(ip.x+di*cos(iyaw+CV_PI/2));
            fp.y.push_back(ip.y+di*sin(iyaw+CV_PI/2));
        }
        //计算ds和yaw
        for(int i=0;i<fp.x.size()-1;i++){
            double dx=fp.x[i+1]-fp.x[i];
            double dy=fp.y[i+1]-fp.y[i];
            fp.yaw.push_back(atan2(dy,dx));
            fp.ds.push_back(sqrt(pow(dx,2)+pow(dy,2)));
        }
        fp.yaw.push_back(fp.yaw.back());
        fp.ds.push_back(fp.ds.back());

        //计算曲率
        for(int i=0;i<fp.yaw.size()-1;i++){
            fp.c.push_back(fp.yaw[i+1]-fp.yaw[i]/fp.ds[i]);
        }
    }
}

bool check_collision(const Frenet& fp,const vector<cv::Point2d>& ob){
    for(auto& obp:ob){
        for(auto i=0;i<fp.x.size();i++){
            if(pow(fp.x[i]-obp.x,2)+pow(fp.y[i]-obp.y,2)<=pow(ROBOT_RADIUS,2))return false;
        }
    }
    return true;
}
//过滤掉不合格的轨迹
vector<Frenet> check_paths(const vector<Frenet>& fplist,const vector<cv::Point2d>& ob){
    vector<Frenet> goodfplist;
    for(auto& fp:fplist){
        for(auto& v:fp.s_d)if(v>MAX_SPEED)continue;
        for(auto& a:fp.s_dd)if(abs(a)>MAX_ACCEL)continue;
        for(auto& c:fp.c)if(abs(c)>MAX_CURVATURE)continue;
        if(!check_collision(fp,ob))continue;

        goodfplist.push_back(fp);
    }
    return goodfplist;
}
