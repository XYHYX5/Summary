////�ڱ�������Դ����֮ǰ���轫87����214�е����������·������Ϊ��ǰ����ͼƬ���ھ���·��
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
////�ж�rect1��rect2�Ƿ��н���  
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
//	Mat srcImg = imread("E:\\ͼ����\\learning\\TestOpenCV\\timg.jpg");
//	Mat srcImgCopy;
//	srcImg.copyTo(srcImgCopy);
//
//	//��ɫɫ�ʷָ�
//	int width = srcImg.cols;//ͼ����  
//	int height = srcImg.rows;//ͼ��߶�  
//	double B = 0.0, G = 0.0, R = 0.0, H = 0.0, S = 0.0, V = 0.0;
//	Mat matRgb = Mat::zeros(srcImg.size(), CV_8UC1);
//	int x, y; //ѭ��  
//	for (y = 0; y < height; y++)
//	{
//		for (x = 0; x < width; x++)
//		{
//			// ��ȡBGRֵ  
//			B = srcImg.at<Vec3b>(y, x)[0];
//			G = srcImg.at<Vec3b>(y, x)[1];
//			R = srcImg.at<Vec3b>(y, x)[2];
//			RGB2HSI(R, G, B, H, S, V);
//			//��ɫ��Χ  
//			//cout << H << '\t' << S<<'\t' << V << endl;
//			if ((H >= 330 && H <= 360 || H >= 0 && H <= 10) && S >= 21 && S <= 100 && V > 16 && V < 99) //H���ܵ���10��H���ܴ���344,S���ܸ���21��V���ܱ�
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
//	//// ��ֵ�˲�
//	//medianBlur(matRgb, matRgb, 3);
//	//medianBlur(matRgb, matRgb, 5);
//	//imshow("medianBlur", matRgb);
//
//	//Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * 1 + 1, 2 * 1 + 1), Point(1, 1));
//	//Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(2 * 3 + 1, 2 * 3 + 1), Point(3, 3));
//	//erode(matRgb, matRgb, element);//��ʴ    
//	//imshow("erode", matRgb);
//	//dilate(matRgb, matRgb, element1);//����    
//	//imshow("dilate", matRgb);
//	//fillHole(matRgb, matRgb);//���    
//	//imshow("fillHole", matRgb);
//
//	//Mat matRgbCopy;
//	//matRgb.copyTo(matRgbCopy);
//
//	////������
//	//vector<vector<Point>>contours; //����    
//	//vector<Vec4i> hierarchy;//�ֲ�    
//	//findContours(matRgb, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));//Ѱ������    
//	//vector<vector<Point>> contours_poly(contours.size());  //���ƺ�������㼯    
//	//vector<Rect> boundRect(contours.size());  //��Χ�㼯����С����vector    
//
//	////������
//	//for (int i = 0; i < contours.size(); i++)
//	//{
//	//	approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); //�Զ�����������ʵ����ƣ�contours_poly[i]������Ľ��Ƶ㼯    
//	//	boundRect[i] = boundingRect(Mat(contours_poly[i])); //���㲢���ذ�Χ�����㼯����С����       
//	//}
//
//	////����ȡ������������ȥ�룬ɸѡ����ͨ��־
//	//Mat drawing = Mat::zeros(matRgb.size(), CV_8UC3);
//	//for (int i = 0; i < contours.size(); i++)
//	//{
//	//	Rect rect = boundRect[i];
//	//	//���Ƚ���һ�������ƣ�ɸѡ������
//
//	//	//�����������ڲ��������ž��Σ��򽫱�������С����ȡ��
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
//	//	//�߿������  
//	//	float ratio = (float)rect.width / (float)rect.height;
//	//	//�����������       
//	//	float Area = (float)rect.width * (float)rect.height;
//	//	float dConArea = (float)contourArea(contours[i]);
//	//	float dConLen = (float)arcLength(contours[i], 1);
//	//	if (dConArea < 700)
//	//		continue;
//	//	if (ratio > 1.3 || ratio < 0.4)
//	//		continue;
//
//	//	//������ʶ���
//	//	Scalar color = (0, 0, 255);//��ɫ�߻�����    
//	//	drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
//	//	rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
//	//	rectangle(srcImg, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
//
//	//	imshow("drawing.jpg", drawing);
//
//	//	//Harris�ǵ��⣬ȥ��������������
//	//	Mat grayImg, dstImg, normImg, scaledImg;
//	//	cvtColor(drawing, grayImg, COLOR_BGR2GRAY);
//	//	cornerHarris(grayImg, dstImg, 2, 3, 0.04);
//
//	//	//��һ����ת��
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
//	//	if (harrisNum > 33)//���ǵ���Ŀ����33������Դ�����
//	//		continue;
//
//	//	imshow("result.jpg", srcImg);
//	//	imshow("cornerHarris.jpg", scaledImg);
//	//	imwrite("E:\\ͼ����\\learning\\TestOpenCV\\result1.jpg", srcImg);//�������յļ��ʶ����  
//	//}
//
//	waitKey(0);
//	return 0;
//}
