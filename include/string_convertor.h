/*************************************************************************
	> File Name: norm_dist.h
	> Author:
	> Mail:
	> Created Time: Mon 20 Feb 2017 09:23:05 PM MST
 ************************************************************************/

// #ifndef _NORM_DIST_H
// #define _NORM_DIST_H
// #endif
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <exception>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
class string_convertor
{
    public:
       string_convertor();
       ~string_convertor();
       static string  d2s(double d);
       static std::vector<float> convert2Float(std::vector<double> v);
       static void printOutStdVector(std::vector<double> v);
       static std::vector< double > fromString2Array(string inStr);
       static std::vector< string > fromString2ArrayStr(string inStr);
       static string constructPubStr(vector< vector<Point> > vps);
       static double pointDistance(Point2f p1, Point2f P2);
       static string constructPubStr2(vector< vector<Point> > vps, int gap);
};
