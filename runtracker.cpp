#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <conio.h>
#include <stdio.h>
#include "fdssttracker.hpp"
#include "CSerialPort.h"

using namespace std;
using namespace cv;

Point previousPoint, currentPoint;
Rect2d bbox;
//动态数组存储坐标点

void draw_rectangle(int event, int x, int y, int flags, void*);
Mat tmp, frame, dst;

bool get_first_bbox(Mat src);

void middle_cut(Mat img, Mat &dst, int minV, int maxV, int maxpixel)
{
	Mat temp1, temp2;
	threshold(img, temp1, minV, maxpixel, THRESH_BINARY);
	threshold(img, temp2, maxV, maxpixel, THRESH_BINARY_INV);
	bitwise_and(temp1, temp2, dst);
}

typedef struct Threshold
{
	int maxT;
	int minT;
}thresh;

void adaptiveThresh(Threshold &thr, Mat himg)
{
	int pixelCount[256];
	//init array
	for (int i = 0; i < 256; i++)	pixelCount[i] = 0;

	////get h in hsv
	//Mat himg;
	//cvtColor(img, himg, COLOR_RGB2HSV);
	//vector<Mat> channels;
	//split(himg, channels);
	//himg = channels.at(0);

	// iterate all pixels
	for (int i = 0; i < himg.rows; i++)
	{
		for (int j = 0; j < himg.cols; j++)
		{
			int pid = himg.at<uchar>(i, j);
			pixelCount[pid] ++;
		}
	}

	// find maxpixel
	int maxP, maxPid;
	for (int i = 0; i < 256; i++)
	{
		if (i == 0)	maxP = pixelCount[0];
		else
		{
			if (pixelCount[i] > maxP)
			{
				maxP = pixelCount[i];
				maxPid = i;
			}
		}
	}

	// Decay and find thresh
	float decayRate = 0.4f;
	bool forwardDone = false;
	bool backwardDone = false;
	int cnt = 1;

	while ((!forwardDone) || (!backwardDone))
	{
		// following 2 cases is impossible to happen
		// but only for programme safty
		if (maxPid - cnt <= 0)
		{
			thr.minT = 0;
			thr.maxT = maxPid + cnt;
			break;
		}
		else if (maxPid + cnt >= 255)
		{
			thr.minT = maxPid - cnt;
			thr.maxT = 255;
			break;
		}

		if (!forwardDone)
		{
			int nextid = maxPid + cnt;
			int lastid = nextid - 1;
			float rate = 1.0f * pixelCount[nextid] / pixelCount[lastid];
			if (rate < decayRate)
			{
				thr.maxT = nextid;
				forwardDone = true;
			}
		}
		if (!backwardDone)
		{
			int nextid = maxPid - cnt;
			int lastid = nextid + 1;
			float rate = 1.0f * pixelCount[nextid] / pixelCount[lastid];
			if (rate < decayRate)
			{
				thr.minT = nextid;
				backwardDone = true;
			}
		}
		cnt++;
	}

}


bool tracking_or_not(Mat dst, float track_rate, Threshold thr)
{
	Mat obj;
	middle_cut(dst, obj, thr.minT, thr.maxT, 1);
	Scalar ss = sum(obj);
	double rate = ss[0] / (obj.rows * obj.cols);
	//cout << rate << endl;
	if (rate > track_rate)
		return true;
	else
		return false;
}

Ptr<BackgroundSubtractorMOG2> mog2 = createBackgroundSubtractorMOG2(130, 50.0, false);
Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
bool get_new_bbox(Mat img, Threshold thr)	
{
	Mat dst;
	middle_cut(img, dst, thr.minT, thr.maxT, 255);
	//检测运动
	Mat fgmask;
	morphologyEx(dst, dst, MORPH_OPEN, element);
	mog2->apply(dst, fgmask);
	dilate(fgmask, fgmask, element);
	moveWindow("dst", 800, 0);
	imshow("dst", fgmask);
	//找轮廓
	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;
	findContours(fgmask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	if (!hierarchy.empty())
	{
		vector<double> area;
		Moments mc;
		double each_area;
		for (int i = 0; i < hierarchy.size(); i++)
		{
			each_area = contourArea(contours[i]);
			if ((each_area < 7000) && (each_area > 2000))
			{
				bbox = boundingRect(contours[i]);
				float bili = 1.0f * bbox.width / bbox.height;
				if ((bili > 0.76) && (bili < 1.3))	return true;
			}
		}
	}
	return false;

}


void rect2Center(char *dat2Send, int len)
{
	int x, y;
	memset(dat2Send, 0, len);
	x = (bbox.x + bbox.width) / 2;
	y = (bbox.y + bbox.height) / 2;
	sprintf_s(dat2Send, len, "(%d,%d)\n", x, y);
}


int main(int argc, char *argv[])
{

	//// firstly init the comm
	//cout << "finding comm ..." << endl;
	//CSerialPort mSerialPort;
	//vector<string> comList;
	//CSerialPort::getComList(comList);
	//if (comList.empty())
	//{
	//	cout << "no comm available!" << endl;
	//	return 0;
	//	system("pause");
	//}
	//for (int i = 0; i < comList.size(); i++)
	//{
	//	cout << comList[i] << endl;
	//}

	//UINT portNo;
	//cout << "please choose the comm number: ";
	//cin >> portNo;

	//if (!mSerialPort.initPort(portNo))
	//{
	//	cout << "init failed! Please check!" << endl;
	//	return 0;
	//	system("pause");
	//}

	//cout << "init success!" << endl;

	vector<Mat> channels;
	VideoCapture cap;
	int count = 1;
	//string filename = "../5.mp4";
	cap.open(0);	//using webcam
	if (!cap.isOpened())
	{
		cout << "fail to open" << endl;
		system("pause");
		return -1;
	}

	// init draw bbox
	while (cap.read(frame))
	{
		imshow("init", frame);
		if (waitKey(5) == 's')	break;
	}

	cvtColor(frame, dst, COLOR_BGR2HSV);
	split(dst, channels);
	dst = channels.at(0);
	setMouseCallback("init", draw_rectangle, 0);
	waitKey(0);

	Threshold thr;
	adaptiveThresh(thr, dst(bbox));

	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool LAB = true;
	// Create DSSTTracker tracker object
	FDSSTTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	tracker.init(bbox, dst);


	bool obj = true;	//假设一开始有目标
	char writeBuff[32];
	memset(writeBuff, 0, sizeof(writeBuff));
	while (cap.read(frame))
	{
		cvtColor(frame, dst, COLOR_BGR2HSV);
		split(dst, channels);
		dst = channels.at(0);
		if (obj)
		{
			bbox = tracker.update(dst);
			rectangle(frame, bbox, Scalar(255, 255, 0), 2, 1);
			if (!tracking_or_not(dst(bbox), 0.60, thr))
			{
				//cout << "obj lost" << endl;
				obj = false;
			}
			else
			{
				rectangle(frame, bbox, Scalar(255, 255, 0), 2, 1);
				rect2Center(writeBuff, sizeof(writeBuff));
				//mSerialPort.writeData((unsigned char*)writeBuff, sizeof(writeBuff));
			}
		}
		else
		{
			obj = get_new_bbox(dst, thr);
			if (obj)
			{
				tracker.init(bbox, dst);
			}
		}

		namedWindow("tracking", 0);
		imshow("tracking", frame);
		if (waitKey(5) == 27)
			break;
	}

	destroyAllWindows();
	system("pause");
	return 0;
}



void draw_rectangle(int event, int x, int y, int flags, void*)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		previousPoint = Point(x, y);
		cout << "(" << previousPoint.x << "," << previousPoint.y << ")" << endl;
	}
	else if (event == EVENT_MOUSEMOVE && (flags&EVENT_FLAG_LBUTTON))
	{
		Mat tmp;
		char str[20];
		frame.copyTo(tmp);
		currentPoint = Point(x, y);
		sprintf_s(str, 20, "(%d,%d)", currentPoint.x, currentPoint.y);
		putText(tmp, str, currentPoint, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 2, 8);
		rectangle(tmp, previousPoint, currentPoint, Scalar(0, 255, 0), 2, 1, 0);
		imshow("init", tmp);
	}
	else if (event == EVENT_LBUTTONUP)
	{
		bbox.x = previousPoint.x;
		bbox.y = previousPoint.y;
		bbox.width = abs(previousPoint.x - currentPoint.x);
		bbox.height = abs(previousPoint.y - currentPoint.y);

	}
	else if (event == EVENT_RBUTTONUP)
		destroyWindow("init");
}

//
//bool get_first_bbox(Mat src)
//{
//	Mat temp = imread("../temp.jpg");
//	if (temp.empty())
//	{
//		cout << "can't load image!" << endl;
//		system("pause");
//		return false;
//	}
//
//	Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();
//	vector<KeyPoint> keyL, keyR;
//	fast->detect(src, keyL);
//	fast->detect(temp, keyR);
//
//	int nfeatures = 10;
//	Ptr<DescriptorExtractor> sift = SIFT::create(nfeatures);
//
//	Mat desL, desR;
//	sift->compute(src, keyL, desL);
//	sift->compute(temp, keyR, desR);
//
//	FlannBasedMatcher matcher;
//	vector<DMatch> matches;
//	matcher.match(desL, desR, matches);
//	if (matches.size() < 100)
//		return false;
//
//	double dist, maxdist = 0;
//	for (int i = 0; i < desL.rows; i++)
//	{
//		if (maxdist < matches[i].distance)
//			maxdist = matches[i].distance;
//	}
//
//	float good_rate = 0.2f;
//	vector<DMatch> good_match;
//	for (int i = 0; i < desL.rows; i++)
//	{
//		if (matches[i].distance < good_rate * maxdist)
//			good_match.push_back(matches[i]);
//	}
//
//	vector<Point> good_points;
//	int pt_id;
//	for (int i = 0; i < good_match.size(); i++)
//	{
//		pt_id = good_match[i].queryIdx;
//		good_points.push_back(keyL[pt_id].pt);
//	}
//
//	bbox = boundingRect(good_points);
//
//	return true;
//}
