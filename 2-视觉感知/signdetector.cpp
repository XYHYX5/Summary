////在编译运行源代码之前，需将87行与214行的输入与输出路径更改为当前电脑图片所在绝对路径
//#include<iostream>  
//#include<opencv2/opencv.hpp>  
//#define PI 3.1415926  
//
//using namespace std;
//using namespace cv;
//
//void RGB2HSI(double red, double green, double blue, double& hue, double& saturation, double& intensity)
//{
//
//	double r, g, b;
//	double h, s, i;
//
//	double sum;
//	double minRGB, maxRGB;
//	double theta;
//
//	r = red / 255.0;
//	g = green / 255.0;
//	b = blue / 255.0;
//
//	minRGB = ((r < g) ? (r) : (g));
//	minRGB = (minRGB < b) ? (minRGB) : (b);
//
//	maxRGB = ((r > g) ? (r) : (g));
//	maxRGB = (maxRGB > b) ? (maxRGB) : (b);
//
//	sum = r + g + b;
//	i = sum / 3.0;
//
//	if (i < 0.001 || maxRGB - minRGB < 0.001)
//	{
//
//		h = 0.0;
//		s = 0.0;
//	}
//	else
//	{
//		s = 1.0 - 3.0 * minRGB / sum;
//		theta = sqrt((r - g) * (r - g) + (r - b) * (g - b));
//		theta = acos((r - g + r - b) * 0.5 / theta);
//		if (b <= g)
//			h = theta;
//		else
//			h = 2 * PI - theta;
//		if (s <= 0.01)
//			h = 0;
//	}
//
//	hue = (int)(h * 180 / PI);
//	saturation = (int)(s * 100);
//	intensity = (int)(i * 100);
//}
//
//void fillHole(const Mat srcBw, Mat& dstBw)
//{
//	Size m_Size = srcBw.size();
//	Mat Temp = Mat::zeros(m_Size.height + 2, m_Size.width + 2, srcBw.type());
//	srcBw.copyTo(Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)));
//
//	cv::floodFill(Temp, Point(0, 0), Scalar(255));
//
//	Mat cutImg;
//	Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)).copyTo(cutImg);
//
//	dstBw = srcBw | (~cutImg);
//}
//
////判断rect1与rect2是否有交集  
//bool isInside(Rect rect1, Rect rect2)
//{
//	Rect t = rect1 & rect2;
//	if (rect1.area() > rect2.area())
//	{
//		return false;
//	}
//	else
//	{
//		if (t.area() != 0)
//			return true;
//	}
//}
//
//int main()
//{
//	Mat srcImg = imread("E:\\图像处理\\learning\\TestOpenCV\\timg.jpg");
//	Mat srcImgCopy;
//	srcImg.copyTo(srcImgCopy);
//
//	//红色色彩分割
//	int width = srcImg.cols;//图像宽度  
//	int height = srcImg.rows;//图像高度  
//	double B = 0.0, G = 0.0, R = 0.0, H = 0.0, S = 0.0, V = 0.0;
//	Mat matRgb = Mat::zeros(srcImg.size(), CV_8UC1);
//	int x, y; //循环  
//	for (y = 0; y < height; y++)
//	{
//		for (x = 0; x < width; x++)
//		{
//			// 获取BGR值  
//			B = srcImg.at<Vec3b>(y, x)[0];
//			G = srcImg.at<Vec3b>(y, x)[1];
//			R = srcImg.at<Vec3b>(y, x)[2];
//			RGB2HSI(R, G, B, H, S, V);
//			//红色范围  
//			//cout << H << '\t' << S<<'\t' << V << endl;
//			if ((H >= 330 && H <= 360 || H >= 0 && H <= 10) && S >= 21 && S <= 100 && V > 16 && V < 99) //H不能低于10，H不能大于344,S不能高于21，V不能变
//			{
//				matRgb.at<uchar>(y, x) = 255;
//			}
//			if (H >= 50 && H <= 60  && S >= 21 && S <= 100 && V > 16 && V < 99) 
//			{
//				matRgb.at<uchar>(y, x) = 255;
//			}
//		}
//	}
//	imshow("hsv", matRgb);
//	
//
//	//// 中值滤波
//	//medianBlur(matRgb, matRgb, 3);
//	//medianBlur(matRgb, matRgb, 5);
//	//imshow("medianBlur", matRgb);
//
//	//Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * 1 + 1, 2 * 1 + 1), Point(1, 1));
//	//Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(2 * 3 + 1, 2 * 3 + 1), Point(3, 3));
//	//erode(matRgb, matRgb, element);//腐蚀    
//	//imshow("erode", matRgb);
//	//dilate(matRgb, matRgb, element1);//膨胀    
//	//imshow("dilate", matRgb);
//	//fillHole(matRgb, matRgb);//填充    
//	//imshow("fillHole", matRgb);
//
//	//Mat matRgbCopy;
//	//matRgb.copyTo(matRgbCopy);
//
//	////找轮廓
//	//vector<vector<Point>>contours; //轮廓    
//	//vector<Vec4i> hierarchy;//分层    
//	//findContours(matRgb, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));//寻找轮廓    
//	//vector<vector<Point>> contours_poly(contours.size());  //近似后的轮廓点集    
//	//vector<Rect> boundRect(contours.size());  //包围点集的最小矩形vector    
//
//	////画轮廓
//	//for (int i = 0; i < contours.size(); i++)
//	//{
//	//	approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); //对多边形曲线做适当近似，contours_poly[i]是输出的近似点集    
//	//	boundRect[i] = boundingRect(Mat(contours_poly[i])); //计算并返回包围轮廓点集的最小矩形       
//	//}
//
//	////对提取出的轮廓进行去噪，筛选出交通标志
//	//Mat drawing = Mat::zeros(matRgb.size(), CV_8UC3);
//	//for (int i = 0; i < contours.size(); i++)
//	//{
//	//	Rect rect = boundRect[i];
//	//	//首先进行一定的限制，筛选出区域
//
//	//	//若轮廓矩形内部还包含着矩形，则将被包含的小矩形取消
//	//	bool inside = false;
//	//	for (int j = 0; j < contours.size(); j++)
//	//	{
//	//		Rect t = boundRect[j];
//	//		if (rect == t)
//	//			continue;
//	//		else if (isInside(rect, t))
//	//		{
//	//			inside = true;
//	//			break;
//	//		}
//	//	}
//	//	if (inside)
//	//		continue;
//
//	//	//高宽比限制  
//	//	float ratio = (float)rect.width / (float)rect.height;
//	//	//轮廓面积限制       
//	//	float Area = (float)rect.width * (float)rect.height;
//	//	float dConArea = (float)contourArea(contours[i]);
//	//	float dConLen = (float)arcLength(contours[i], 1);
//	//	if (dConArea < 700)
//	//		continue;
//	//	if (ratio > 1.3 || ratio < 0.4)
//	//		continue;
//
//	//	//画轮廓识别框
//	//	Scalar color = (0, 0, 255);//蓝色线画轮廓    
//	//	drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
//	//	rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
//	//	rectangle(srcImg, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
//
//	//	imshow("drawing.jpg", drawing);
//
//	//	//Harris角点检测，去除掉不规则轮廓
//	//	Mat grayImg, dstImg, normImg, scaledImg;
//	//	cvtColor(drawing, grayImg, COLOR_BGR2GRAY);
//	//	cornerHarris(grayImg, dstImg, 2, 3, 0.04);
//
//	//	//归一化与转换
//	//	normalize(dstImg, normImg, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
//	//	convertScaleAbs(normImg, scaledImg);
//
//	//	int harrisNum = 0;
//	//	for (int j = 0; j < normImg.rows; j++)
//	//	{
//	//		for (int i = 0; i < normImg.cols; i++)
//	//		{
//	//			if ((int)normImg.at<float>(j, i) > 160)
//	//			{
//	//				circle(scaledImg, Point(i, j), 4, Scalar(0, 10, 255), 2, 8, 0);
//	//				harrisNum++;
//	//			}
//	//		}
//	//	}
//	//	if (harrisNum > 33)//当角点数目大于33，则忽略此轮廓
//	//		continue;
//
//	//	imshow("result.jpg", srcImg);
//	//	imshow("cornerHarris.jpg", scaledImg);
//	//	imwrite("E:\\图像处理\\learning\\TestOpenCV\\result1.jpg", srcImg);//保存最终的检测识别结果  
//	//}
//
//	waitKey(0);
//	return 0;
//}
