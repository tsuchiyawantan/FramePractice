#include <iostream>
#include <sstream>
#include <time.h>
#include "stdafx.h"
#include "KinectControl.h"
#include "Depth.h"
#include "Dot.h"
#include "Bezier.h"
#include "NeonDesign.h"
#include "Log.h"
#include "ArmMovements.h"
#include "CatmullSpline.h"
#include "Gaussian.h"
#define SPACESIZE 10
#define SCALESIZE 1
#define FILTERSIZE 81
#define HUE 60

string hstate[] = { "unknown", "nottracked", "Open", "Closed", "Lasso" };
string hconf[] = { "low", "high" };

string str(pair<int, int> p) {
	stringstream ss;
	ss << hstate[p.first] << ":" << hconf[p.second];
	return ss.str();
}

string str(pair<int, int> left, pair<int, int>right) {
	stringstream ss;
	ss << str(left) << " " << str(right);
	return ss.str();
}

void doArm(cv::Mat &image, Log log){
	ArmMovements arm;
	//arm.drawArmMove(image); 
	arm.drawCurvedArmMove(image);
	//arm.drawLine(image, arm.yx, HUE);
}
void doArm(cv::Mat &src_img, Log log, vector<vector<pair<int, int>>> &forBezier){
	ArmMovements arm;
	vector<vector<pair<int, int>>> armPts;
	armPts.resize(100);
	cv::Mat img = cv::Mat(src_img.rows+300, src_img.cols+300, CV_8UC3, cv::Scalar(0));
	int k = 0;
	for (int i = 0; i < forBezier.size(); i++){
		for (int j = 0; j < forBezier[i].size() - 1; j++)
			arm.drawArmMove(img, armPts[k++], forBezier[i].at(j), forBezier[i].at(j+1));
	}

	for (int i = 0; i < armPts.size(); i++)
		arm.drawLine(img, armPts[i], HUE);
	cv::GaussianBlur(img, img, cv::Size(19, 15), 0, 0);
	for (int i = 0; i < armPts.size(); i++)
		arm.drawInline(img, armPts[i], HUE);
	cv::imshow("arm image", img);
}
void doArm2(cv::Mat &image, Log log, vector<vector<pair<int, int>>> &yx){
	ArmMovements arm;
	//Gaussian gaus;
	//gaus.createKernel(4);
	cv::Mat img = cv::Mat(image.rows + 300, image.cols + 300, CV_8UC3, cv::Scalar(0));
	for (int i = 0; i < yx.size(); i++)
		arm.drawLine(img, yx[i], HUE);
	//cv::GaussianBlur(img, img, cv::Size(19, 15), 0, 0);
	for (int i = 0; i < yx.size(); i++)
		arm.drawInline(img, yx[i], HUE);
	cv::imshow("arm image", img);
}
void doCatmull(cv::Mat &srcImg, vector<vector<pair<int, int>>> &approximationLine){
	cv::Mat resultImg = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC3, cv::Scalar(0, 0, 0));
	CatmullSpline catmull;
	for (int i = 0; i < approximationLine.size(); i++){
		catmull.drawLine(resultImg, approximationLine[i], HUE);
	}
	//SpaceFiltering
	//catmull.exeGaussian(approximationLine, resultImg);
	
	//Opencv Gaussian
	//cv::GaussianBlur(resultImg, resultImg, cv::Size(19, 15), 0, 0);
	catmull.drawInline(resultImg, HUE, FILTERSIZE);
	cv::imshow("Catmull Spline", resultImg);
}

void doDot(cv::Mat &srcImg){
	Dot dot;
	dot.setWhiteDots(srcImg);
	dot.findStart(srcImg);
	dot.makeLine(srcImg);
	dot.makeSpace(SPACESIZE);
	dot.scalable(SCALESIZE);
	doCatmull(srcImg, dot.approximationLine);
}
void main() {
	try {
		Log log;
		log.Initialize("log.txt");
			clock_t start = clock();
			clock_t end = clock();
			cv::Mat src_img = cv::imread("sample.jpg", 0);
			//cv::Mat line_img = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);		
			threshold(src_img, src_img, 150, 255, CV_THRESH_BINARY);
			//threshold(line_img, line_img, 150, 255, CV_THRESH_BINARY);
			cv::imshow("src_img", src_img);
			doDot(src_img);
			//doArm(line_img, log);
			//cv::imshow("complete image", src_img);
			//cv::imshow("line image", line_img);
		//	cv::imwrite("comp.jpg", src_img);
			log.Write("---------------------------------------------");
			auto key = cv::waitKey(200000);

	}
	catch (exception& ex) {
		cout << ex.what() << endl;
		string s;
		cin >> s;
	}
}
