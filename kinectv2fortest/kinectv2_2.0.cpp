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
#include "Gaussian.h"

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
void doBezier(vector<vector<pair<int, int>>> &forBezier, cv::Mat &image, int hue, Log log){
	Bezier bezier;
	clock_t start = clock();
	bezier.bezierLike(forBezier, image);
	clock_t end = clock();
	bezier.drawBezier(forBezier, image, hue);
	cv::imshow("bezier image", image);
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

void doDot(cv::Mat &image, int hue, Log log){
	Dot dot;
	clock_t start = clock();
	dot.setWhiteDots(image, dot.dots);
	clock_t end = clock();
	dot.findStart(image, dot.dots, dot.start);
	dot.makeLine(dot.contours, dot.start, dot.dots, dot.used, image);
	dot.makeSpace(dot.contours, dot.forBezier);
	dot.scalable(dot.forBezier);
	dot.writeDots(dot.forBezier, image, dot.dots);
	
	doBezier(dot.forBezier, image, hue, log);
	//doArm(image, log, dot.forBezier);
	dot.init();
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
			doDot(src_img, 60, log);
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
