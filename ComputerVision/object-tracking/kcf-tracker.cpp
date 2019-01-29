
#include <iostream>
#include <string>
#include <opencv.hpp>
#include <tracking.hpp>

using namespace std;

int main(int argc, char const *argv[])
{
    cv::Ptr<cv::Tracker> tracker;//跟踪器
    cv::Mat frame;//保存每帧图像
    char key;//快捷键
    tracker = cv::TrackerKCF::create();
    cv::Rect2d initBB;//初始目标坐标
    //Read video
    // cv::VideoCapture cap("D:\\Code\\C++\\YOLO3-detecction\\run.mp4");
    cv::VideoCapture cap(0);
    //创建窗口
    static const string kWinName = "object tracking in OpenCV";
    cv::namedWindow(kWinName,cv::WINDOW_AUTOSIZE);
    if(!cap.isOpened()){
        cout <<"Could not read video file"<<endl;
        return -1;
    }
    while(key=cv::waitKey(1)){
        //读取frame
        cap>>frame;
        cv::resize(frame,frame,cv::Size(frame.cols/2,frame.rows/2),0,0,cv::INTER_LINEAR);
        //如果播放完则停止
        if(frame.empty()){
            break;
        }
        
        if(key=='s'){
            if(!tracker->empty()){
                tracker.release();
                tracker = cv::TrackerKCF::create();
            }
            
            //选择初始目标区域
            initBB=cv::selectROI(kWinName,frame,true,false);
            tracker->init(frame,initBB);
        }
        if(tracker->update(frame,initBB)){
            cv::rectangle(frame,initBB,cv::Scalar(255,0,0),3);
        }
        cv::imshow(kWinName,frame);
    }
    tracker.release();
    return 0;
}
