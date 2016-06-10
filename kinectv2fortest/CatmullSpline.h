#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include "NeonDesign.h"

using namespace std;

class CatmullSpline{
private:
public:
	vector<pair<int, int>> contour;
	vector<vector<pair<int, int>>> catmullLine;

	CatmullSpline(){}
	~CatmullSpline(){}

	double catmullRom(double p0, double p1, double p2, double p3, double t){
		double v0 = (p2 - p0) / 2;
		double v1 = (p3 - p1) / 2;
		double t2 = t*t;
		double t3 = t2*t;
		return 0.5*((2 * p1) + (p2 - p0)*t + (2 * p0 - 5 * p1 + 4 * p2 - p3)*t2 + (-p0 + 3 * p1 - 3 * p2 + p3)*t3);
	}
	double catmullRomFL(double p0, double p1, double p2, double p3, double t, double){
		double v0 = (p2 - p0) / 2;
		double v1 = (p3 - p1) / 2;
		double t2 = t*t;
		double t3 = t2*t;
		return 0.5*((2 * p1) + (p2 - p0)*t + (2 * p0 - 5 * p1 + 4 * p2 - p3)*t2 + (-p0 + 3 * p1 - 3 * p2 + p3)*t3);
	}
	void adjust(vector<pair<int, int>> &yx){
		int j = yx.size() + (4 - yx.size() % 4);
		while (yx.size() < j){
			yx.push_back(make_pair(yx.back().first, yx.back().second));
		}
	}
	void drawInline(cv::Mat &srcImg, int hue){
		NeonDesign design;
		vector<int> bgr = { 0, 0, 0 };
		design.rgb(hue, 255 - 100, 255, bgr);

		for (int i = 0; i < catmullLine.size(); i++){
			for (int j = 0; j < catmullLine[i].size(); j++){
				int y = catmullLine[i].at(j).first;
				int x = catmullLine[i].at(j).second;
				circle(srcImg, cv::Point(x, y), 0.5, cv::Scalar(bgr.at(0), bgr.at(1), bgr.at(2)), -1, 4);
			}
		}
	}
	void drawLine(cv::Mat &srcImg, vector<pair<int, int>> &contours, int hue){
		NeonDesign design;
		vector<int> bgr = { 0, 0, 0 };
		vector<pair<int, int>> ctr;
		design.rgb(hue, 255, 255 - 100, bgr);
		for (int i = 0; i < contours.size(); i++){
			int y = contours.at(i).first;
			int x = contours.at(i).second;
			if (i >= contours.size() || i + 1 >= contours.size() || i + 2 >= contours.size() || i + 3 >= contours.size()) break;
			for (double t = 0; t <= 1.0; t += 0.009){
				y = catmullRom(contours.at(i).first, contours.at(i + 1).first, contours.at(i + 2).first, contours.at(i + 3).first, t);
				x = catmullRom(contours.at(i).second, contours.at(i + 1).second, contours.at(i + 2).second, contours.at(i + 3).second, t);
				ctr.push_back(make_pair(y, x));
				circle(srcImg, cv::Point(x, y), 2, cv::Scalar(bgr.at(0), bgr.at(1), bgr.at(2)), -1, 4);
			}
		}
		catmullLine.push_back(ctr);
	}
};