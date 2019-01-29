> 参考：https://www.pyimagesearch.com/2018/07/30/opencv-object-tracking/

# 引言
Opencv作为图像处理开源库包含了Object Tracking目标追踪的一些API，使用Opencv能够方便快捷的编写目标追踪程序。

Opencv4.0目前包含了8种目标追踪算法：

* **`Boosting`**：基于在线的AdaBoost, 这个分类器需要对对象的正、负例进行训练。用户提供的初始化框(或通过其它算法检测到对象，必须MOG2，KNN检测到小车)来作为对象的正例，并将边界框外的图像块作为背景(负类) 
优点：没有。这个算法已经有10年的历史了，找不到一个很好的理由去使用它，特别是当其他基于类似原理的高级跟踪器(MIL, KCF)也可用的时候。 
* **`CSRT`**：判别性相关滤波器。
优点：精确度比KCF稍高。
缺点：速度不如KCF块。
* **`GOTURN`**：在跟踪器类的所有跟踪算法中，这是唯一基于卷积神经网络(CNN)的算法。也是唯一一个使用离线训练的模型，因此它比其他跟踪器更快。从opencv文档可以看出该算法对视角变化、光照、变形都具有很好的鲁棒性，但是对于遮挡性能较差。 
* **`KCF`**：这个跟踪器基于前面两个跟踪器中提出的想法。该跟踪器在MIL跟踪器中使用的多个正样本具有较大的重叠区域。 
优点：精度和速度都比MIL好，建议在大多数应用程序中使用该算法。 
缺点：还是完全遮挡 
* **`MedianFlow`**：经过测试中，发现这个跟踪器在小运动情况下表现最好。不像其他跟踪器，即使跟踪失败了还继续跟踪，这个跟踪器知道什么时候失败。 
优点：跟踪失败报告，小运动下表现好 
缺点：大运动，该算法失灵 
* **`MIL`**：这个跟踪器与上面描述的boost跟踪器类似。最大的区别是，它不是只考虑对象的当前位置作为正类，还考虑当前位置邻域范围的潜在位置作为正类。
优点： 性能很好。它不boosting跟踪器那样，在部分遮挡下依然表现挺佳。但效果不如KCF好。
缺点：跟踪失败没有可靠的报告。完全遮挡的话性能差 
* **`MOSSE`**：如果对速度要求非常高，MOSSE可能是你的更好选择。
优点：速度比CSRT和KCF都要快。
缺点：精度没有CSRT和KCF高。
* **`TLD`**：TLD stands for Tracking, learning and detection. 如果有一个视频序列，对象隐藏在另一个对象后面，这个跟踪器可能是一个不错的选择。
优点：在多帧的情况下，在遮挡的情况下工作最好。此外，该算法能很好的应对尺度变化。  
缺点：大量的假阳性使得其几乎无法使用。
	
	
**我的个人建议是：**

* 当您需要**更高的目标跟踪精度并可承受较慢的fps吞吐量时**，请使用CSRT
* 当您需要**更快的fps吞吐量**，但可以**处理稍低的对象跟踪精度时**，请使用KCF
* 当您需要**纯速度时**使用MOSSE

# 使用Opencv进行对象跟踪
确保电脑已经安装了opencv4.0。
要使用Opencv执行对象跟踪，请引入以下头文件：
```cpp
#include <iostream>
#include <string>
#include <opencv.hpp>
#include <tracking.hpp>
```
首先要创建一个跟踪器并实例化，本文仅使用KCF跟踪器作为示例：
```cpp
cv::Ptr<cv::Tracker> tracker;//跟踪器
tracker = cv::TrackerKCF::create();
```
接下来，我们初始化视频流，并定义显示窗口：
```cpp
cv::Mat frame;//保存每帧图像
cv::VideoCapture cap("run.mp4");
//创建窗口
static const string kWinName = "object tracking in OpenCV";
cv::namedWindow(kWinName,cv::WINDOW_AUTOSIZE);
if(!cap.isOpened()){
cout <<"Could not read video file"<<endl;
return -1;
}
```
为了能够在视频中选取跟踪目标，需要定义一个初始化ROI区域：
```cpp
cv::Rect2d initBB;//初始目标坐标
```
从视频流循环帧分析：
```cpp
while(key=cv::waitKey(1)){
	//读取frame
	cap>>frame;
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
```
为了更快的处理数据，我们可以将读取的frame进行cv::resize（）操作调整图片大小，该用法文章不做说明。因为处理的数据越少，我们的对象跟踪管道的运行速度就越快。
在跟踪前我们需要使用快捷键s来对跟踪目标进行选取并初始化，考虑到在视频流中可能进行多次跟踪目标初始化，所以需要加入：
```cpp
if(!tracker->empty()){
	tracker.release();
	tracker = cv::TrackerKCF::create();
}
```
以确保跟踪器是新的。
如果选择了某个对象，我们需要跟新该对象的位置，因为每一帧对于跟踪器都会重新计算跟踪目标的位置。使用update（）方法将定位对象的新位置，并返回`true`和边界box对象。

![](https://s3-us-west-2.amazonaws.com/static.pyimagesearch.com/opencv-object-tracking/opencv_object_tracking_selection.gif)
