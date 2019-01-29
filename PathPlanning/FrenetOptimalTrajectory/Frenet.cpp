#include <Frenet.h>

quintic_polynomial::quintic_polynomial(const float xs[3],const float xe[3],const float& T){
    a0 = xs[0];
    a1 = xs[1];
    a2 = xs[2]/2;
    cv::Mat A = (cv::Mat_<float>(3,3) <<
        3 * pow(T,2), 4 * pow(T,3), 5 * pow(T,4),
        pow(T,3), pow(T,4), pow(T,5),
        6 * T, 12 * pow(T,2), 20 * pow(T,3));
    cv::Mat b = (cv::Mat_<float>(3,1) <<
        xe[0]-a0-a1*T-a2*pow(T,2),
        xe[1]-a1-2*a2*T,
        xe[2]-2*a2);
    cv::Mat x;
    cv::solve(A,b,x,cv::DECOMP_LU);
    a3 = x.at<float>(0);
    a4 = x.at<float>(1);
    a5 = x.at<float>(2);
}
float quintic_polynomial::calc_point(float& t){
    return a0+a1*t+a2*pow(t,2)+a3*pow(t,3)+a4*pow(t,4)+a5*pow(t,5);
}
float quintic_polynomial::calc_first_derivative(float& t){
    return a1+2*a2*t+3*a3*pow(t,2)+4*a4*pow(t,3)+5*a5*pow(t,4);
}
float quintic_polynomial::calc_second_derivative(float& t){
    return 2*a2+6*a3*t+12*a4*pow(t,2)+20*a5*pow(t,3);
}
float quintic_polynomial::calc_third_derivative(float& t){
    return 6*a3+24*a4*t+60*a5*pow(t,2);
}
//------------------------------
quartic_polynomial::quartic_polynomial(const float xs[3],const float xe[2],const float& T){
    a0 = xs[0];
    a1 = xs[1];
    a2 = xs[2]/2;
    cv::Mat A = (cv::Mat_<float>(2,2) <<
        3 * pow(T,2), 4 * pow(T,3),
        6 * T, 12 * pow(T,2));
    cv::Mat b = (cv::Mat_<float>(2,1) <<
        xe[0]-a1-2*a2*T,
        xe[1]-2*a2);
    cv::Mat x;
    cv::solve(A,b,x,cv::DECOMP_LU);
    a3 = x.at<float>(0);
    a4 = x.at<float>(1);
}
float quartic_polynomial::calc_point(float& t){
    return a0+a1*t+a2*pow(t,2)+a3*pow(t,3)+a4*pow(t,4);
}
float quartic_polynomial::calc_first_derivative(float& t){
    return a1+2*a2*t+3*a3*pow(t,2)+4*a4*pow(t,3);
}
float quartic_polynomial::calc_second_derivative(float& t){
    return 2*a2+6*a3*t+12*a4*pow(t,2);
}
float quartic_polynomial::calc_third_derivative(float& t){
    return 6*a3+24*a4*t;
}

vector<Frenet> calc_frenet_paths(const float& c_speed,const float c[3],const float& s0){
    vector<Frenet> frenet_paths;
    float d[3]={0,0,0};
    float s[3]={s0,c_speed,0};
    float stv[2]={0,0};
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
                vector<float> sojp,sojs;
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

vector<Frenet> calc_global_paths(const vector<Frenet>& fplist,const vector<cv::Point>& csp){
    for(auto& fp:fplist){
        for(int i=0;i<fp.s.size();i++){
            
        }
    }
}