# 1. 引言
YOLO3能够快速识别图片和视频中的80种物体，而且实时性强，准确度接近SSD。
Opencv是目前最流行的开源图像处理库，使用Opencv能够非常方便的对图像进行处理。
Opencv4.0已经包含DNN相关的库函数，可以非常方便的调用训练好的YOLO3模型使用。
# 2. 采用YOLO3目标检测
## 第一步：下载模型
使用wget下载训练号的模型与COCO数据库。
COCO数据库包含了识别的类型名。
```shell
wget https://pjreddie.com/media/files/yolov3.weights
wget https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg?raw=true -O ./yolov3.cfg
wget https://github.com/pjreddie/darknet/blob/master/data/coco.names?raw=true -O ./coco.names
```
## 第二步：初始化参数
YOLOv3算法的预测结果就是边界框。每一个边界框都旁随着一个置信值。第一阶段中，全部低于置信度阀值的都会排除掉。
对剩余的边界框执行非最大抑制算法，以去除重叠的边界框。非最大抑制由一个参数nmsThrehold控制。读者可以尝试改变这个数值，观察输出的边界框的改变。
接下来，设置输入图片的宽度（inpWidth）和高度（inpHeight）。我们设置他们为416，以便对比YOLOv3作者提供的Darknets的C代码。如果想要更快的速度，读者可以把宽度和高度设置为320。如果想要更准确的结果，改变他们到608。
```cpp
float confThreshold = 0.5;//置信度阈值
float nmsThreshold = 0.4;//非最大抑制阈值
int inpWidth = 416;//网络输入图片宽度
int inpHeight = 416;//网络输入图片高度
```
## 第三步：读取模型和COCO数据库
接下来我们读入COCO数据库并将类名存入vector<string> classes容器。并加载模型与权重文件yolov3.cfg和yolov3.weights。
最后把DNN的后端设置为OpenCV，目标设置CPU。也可以通过cv::dnn::DNN_TARGET_OPENCL设置目标为GPU。
```cpp
//将类名存进容器
string classesFile = "D:\\Code\\C++\\YOLO3-detecction\\coco.names";//coco.names包含80种不同的类名
ifstream ifs(classesFile.c_str());
string line;
while(getline(ifs,line))classes.push_back(line);
//取得模型的配置和权重文件
cv::String modelConfiguration = "D:\\Code\\C++\\YOLO3-detecction\\yolov3.cfg";
cv::String modelWeights = "D:\\Code\\C++\\YOLO3-detecction\\yolov3.weights";
//加载网络
cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration,modelWeights);
net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
net.setPreferableBackend(cv::dnn::DNN_TARGET_OPENCL);
```
## 第四步：读取输入
```cpp
//打开视频文件或者图形文件或者相机数据流
string str, outputFile;
cv::VideoCapture cap("D:\\Code\\C++\\YOLO3-detecction\\run.mp4");
cv::VideoWriter video;
cv::Mat frame,blob;
//开启摄像头
// cv::VideoCapture cap(1);
//创建窗口
static const string kWinName = "Deep learning object detection in OpenCV";
cv::namedWindow(kWinName,cv::WINDOW_AUTOSIZE);
```
## 第五步：处理每一帧
输入到神经网络的图像需要以一种叫bolb的格式保存。
读取了输入图片或者视频流的一帧图像后，这帧图像需要经过bolbFromImage()函数处理为神经网络的输入类型bolb。在这个过程中，图像像素以一个1/255的比例因子，被缩放到0到1之间。同时，图像在不裁剪的情况下，大小调整到416x416。注意我们没有降低图像平均值，因此传递[0,0,0]到函数的平均值输入，保持swapRB参数到默认值1。
输出的bolb传递到网络，经过网络正向处理，网络输出了所预测到的一个边界框清单。这些边界框通过后处理，滤除了低置信值的。我们随后再详细的说明后处理的步骤。我们在每一帧的左上方打印出了推断时间。伴随着最后的边界框的完成，图像保存到硬盘中，之后可以作为图像输入或者通过Videowriter作为视频流输入。
```cpp
while(cv::waitKey(1)<0){
	//取每帧图像
	cap>>frame;
	//如果视频播放完则停止程序
	if(frame.empty()){
		break;
	}
	//在dnn中从磁盘加载图片
	cv::dnn::blobFromImage(frame,blob,1/255.0,cv::Size(inpWidth,inpHeight));
	//设置输入
	net.setInput(blob);
	//设置输出层
	vector<cv::Mat> outs;//储存识别结果
	net.forward(outs,getOutputNames(net));
	//移除低置信度边界框
	postprocess(frame,outs);
	//显示s延时信息并绘制
	vector<double> layersTimes;
	double freq = cv::getTickFrequency()/1000;
	double t=net.getPerfProfile(layersTimes)/freq;
	string label = cv::format("Infercence time for a frame:%.2f ms",t);
	cv::putText(frame,label,cv::Point(0,15),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255));
	//绘制识别框
	cv::imshow(kWinName,frame);
}
```
## 第五a步：得到输出层名字
OpenCV的网络类中的前向功能需要结束层，直到它在网络中运行。因为我们需要运行整个网络，所以我们需要识别网络中的最后一层。我们通过使用getUnconnectedOutLayers()获得未连接的输出层的名字，该层基本就是网络的最后层。然后我们运行前向网络，得到输出，如前面的代码片段（net.forward(getOutputsNames(net))）。
```cpp
//从输出层得到名字
vector<cv::String> getOutputNames(const cv::dnn::Net& net){
	static vector<cv::String> names;
	if(names.empty()){
		//取得输出层指标
		vector<int> outLayers = net.getUnconnectedOutLayers();
		vector<cv::String> layersNames = net.getLayerNames();
		//取得输出层名字
		names.resize(outLayers.size());
		for(size_t i =0;i<outLayers.size();i++){
			names[i] = layersNames[outLayers[i]-1];
		}
	}
	return names;
}
```
## 第五b步：后处理网络输出
网络输出的每个边界框都分别由一个包含着类别名字和5个元素的向量表示。
头四个元素代表center_x, center_y, width和height。第五个元素表示包含着目标的边界框的置信度。
其余的元素是和每个类别（如目标种类）有关的置信度。边界框分配给最高分数对应的那一种类。
一个边界框的最高分数也叫做它的置信度（confidence）。如果边界框的置信度低于规定的阀值，算法上不再处理这个边界框。
置信度大于或等于置信度阀值的边界框，将进行非最大抑制。这会减少重叠的边界框数目。
```cpp
//移除低置信度边界框
void postprocess(cv::Mat& frame,const vector<cv::Mat>& outs){
	vector<int> classIds;//储存识别类的索引
	vector<float> confidences;//储存置信度
	vector<cv::Rect> boxes;//储存边框
	for(size_t i=0;i<outs.size();i++){
	//从网络输出中扫描所有边界框
	//保留高置信度选框
	//目标数据data:x,y,w,h为百分比，x,y为目标中心点坐标
		float* data = (float*)outs[i].data;
		for(int j=0;j<outs[i].rows;j++,data+=outs[i].cols){
			cv::Mat scores = outs[i].row(j).colRange(5,outs[i].cols);
			cv::Point classIdPoint;
			double confidence;//置信度
			//取得最大分数值与索引
			cv::minMaxLoc(scores,0,&confidence,0,&classIdPoint);
			if(confidence>confThreshold){
				int centerX = (int)(data[0]*frame.cols);
				int centerY = (int)(data[1]*frame.rows);
				int width = (int)(data[2]*frame.cols);
				int height = (int)(data[3]*frame.rows);
				int left = centerX-width/2;
				int top = centerY-height/2;
				classIds.push_back(classIdPoint.x);
                       confidences.push_back((float)confidence);
                       boxes.push_back(cv::Rect(left, top, width, height));
			}
		}
	}
	//低置信度
	vector<int> indices;//保存没有重叠边框的索引
	//该函数用于抑制重叠边框
	cv::dnn::NMSBoxes(boxes,confidences,confThreshold,nmsThreshold,indices);
	for(size_t i=0;i<indices.size();i++){
		int idx = indices[i];
		cv::Rect box = boxes[idx];
		drawPred(classIds[idx],confidences[idx],box.x,box.y,
		box.x+box.width,box.y+box.height,frame);
	}
}
	
```
非最大抑制由参数nmsThreshold控制。如果nmsThreshold设置太少，比如0.1，我们可能检测不到相同或不同种类的重叠目标。如果设置得太高，比如1，可能出现一个目标有多个边界框包围。所以我们在上面的代码使用了0.4这个中间的值。


## 第五c步：画出计算得到的边界框
 最后，经过非最大抑制后，得到了边界框。我们把边界框在输入帧上画出，并标出种类名和置信值。
```cpp
//绘制预测边界框
void drawPred(int classId,float conf,int left,int top,int right,int bottom,cv::Mat& frame){
	//绘制边界框
	cv::rectangle(frame,cv::Point(left,top),cv::Point(right,bottom),cv::Scalar(255,178,50),3);
	string label = cv::format("%.2f",conf);
	if(!classes.empty()){
		CV_Assert(classId < (int)classes.size());
		label = classes[classId]+":"+label;//边框上的类别标签与置信度
	}
	//绘制边界框上的标签
	int baseLine;
	cv::Size labelSize = cv::getTextSize(label,cv::FONT_HERSHEY_SIMPLEX,0.5,1,&baseLine);
	top = max(top,labelSize.height);
	cv::rectangle(frame,cv::Point(left,top-round(1.5*labelSize.height)),cv::Point(left+round(1.5*labelSize.width),top+baseLine),cv::Scalar(255,255,255),cv::FILLED);
	cv::putText(frame, label,cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75,cv::Scalar(0, 0, 0), 1);
}
```