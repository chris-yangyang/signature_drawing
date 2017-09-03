#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <map>
#include <ctime>
#include <sstream>
#include <algorithm>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "math_helper.h"
#include "string_convertor.h"

using namespace cv;
using namespace std;

int mbd_gt = 0;//mouse button down for finish current shape labeling
int lbd_gt = 0;//mouse middle button dow-L lib -Wl,-rpath,'$$ORIGIN/lib'n for selecting edge fragments
int lbu_gt = 0;

Point pt_lbd_gt, pt_mv_gt, pt_lbu_gt;//left mouse button down point and mouse move points

//mouse event response function
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void DrawButton(Mat &paper,vector<Point> &resetButtonPts, vector<Point> &finishButtonPts);
int k, pk=1;
int pki = 1;

int mbd = 0;//flag of middle button click

int frame_width = 0;
int frame_height = 0;

int drawFlag = 0;
int keyPress;

int resetState = 0;
int finishState = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
   try
    {
       cv::Mat myImage=cv_bridge::toCvShare(msg, "bgr8")->image;//using for simulation display
      cv::imshow("view", myImage);
      cv::waitKey(30);
    }//end of try
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
 }

//===========================MAIN FUNCTION START===========================

int main(int argc, char* argv[]){

  ros::init(argc, argv, "signature_extractor");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/rgb/image_rect_color", 1000, imageCallback);
  //ros::Subscriber subP = nh.subscribe("/chris_tracker/currentPoint", 1000, currentPointCallback);
  //ros::Subscriber subPd = nh.subscribe("/chris_tracker/desiredPoint", 1000, desiredPointCallback);geometry_msgs::Point
  ros::Publisher pubTask = nh.advertise<std_msgs::String>("/chris/targetPoints_2D", 1, true);//task will be only published once

  //ros::ServiceClient Joint_move_client = nh.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");
  cout << "Press any key to continue..." << endl;

    Mat paper(480,640,CV_8UC3,Scalar(255,255,255));

    vector<Point> resetButtonPts;
    resetButtonPts.push_back(Point(170,15));
    resetButtonPts.push_back(Point(270,15));
    resetButtonPts.push_back(Point(270,65));
    resetButtonPts.push_back(Point(170,65));

    vector<Point> finishButtonPts;
    finishButtonPts.push_back(Point(370,15));
    finishButtonPts.push_back(Point(470,15));
    finishButtonPts.push_back(Point(470,65));
    finishButtonPts.push_back(Point(370,65));

    vector<Point> ctrlRegion;
    ctrlRegion.push_back(Point(1,1));
    ctrlRegion.push_back(Point(640,1));
    ctrlRegion.push_back(Point(640,106));
    ctrlRegion.push_back(Point(1,106));

     vector<Point> drawRegion;
     drawRegion.push_back(Point(1,107));
     drawRegion.push_back(Point(640,107));
     drawRegion.push_back(Point(640,480));
     drawRegion.push_back(Point(1,480));

//**************vector for storing signature points*********************
    vector<vector<Point> > vPtSignature;

    vector<Point> vPtSigTmp;
    namedWindow("Signature",CV_WINDOW_NORMAL);

    //int frame_id = 0;
    for(;;){

        Mat tmp_paper = paper.clone();

        DrawButton(paper, resetButtonPts, finishButtonPts);//draw buttons

        setMouseCallback("Signature", CallBackFunc, &pt_lbd_gt);//get mouse events

        if(vPtSignature.size()>0){
            for(int i = 0; i < vPtSignature.size(); i++){
		for(int j = 0; j < vPtSignature[i].size()-1; j++){
	            line(tmp_paper, vPtSignature[i][j], vPtSignature[i][j+1], Scalar(255,0,0), 1, 8);
 	        }
            }
        }

        if(lbu_gt && vPtSigTmp.size()>0){
            vPtSignature.push_back(vPtSigTmp);
            vPtSigTmp.clear();
            vector<Point>().swap(vPtSigTmp);
            lbu_gt = 0;
        }

        if(drawFlag){

        if(pointPolygonTest(drawRegion,Point2f(float(pt_mv_gt.x),float(pt_mv_gt.y)),false) > 0){
            vPtSigTmp.push_back(pt_mv_gt);
        }
            //drawFlag = 0;
            if(vPtSigTmp.size()>1){
                for(int i = 0; i < vPtSigTmp.size()-1; i++){
                    line(tmp_paper, vPtSigTmp[i], vPtSigTmp[i+1], Scalar(255,0,0), 1, 8);
                }
            }
        }


        int num_px = 0;
        for(int i = 0; i < vPtSignature.size(); i++){
            for(int j = 0; j < vPtSignature[i].size();j++){
                num_px++;
            }
        }

        imshow("Signature",tmp_paper);
        keyPress = cvWaitKey(1)&255;

        if(27 == keyPress){//"Esc"
            return 0;
        }else if(32 == keyPress || 1 == mbd_gt || (1 == lbu_gt && pointPolygonTest(resetButtonPts,Point2f(float(pt_lbu_gt.x),float(pt_lbu_gt.y)),false) > 0)){//"Space" for re-sign
            lbu_gt = 0;
            mbd_gt = 0;
            vPtSignature.clear();
            vector<vector<Point> >().swap(vPtSignature);
        }else if(1 == lbu_gt && pointPolygonTest(finishButtonPts,Point2f(float(pt_lbu_gt.x),float(pt_lbu_gt.y)),false) > 0){
            lbu_gt = 0;
            if(vPtSignature.size()==0){
                cout<<" There is no signature to draw...."<<endl;
            }else{
                cout<<" The strokes of signature: "<<vPtSignature.size()<<endl;
                std_msgs::String msg;
                msg.data = string_convertor::constructPubStr(vPtSignature);
              	pubTask.publish(msg);
            }
        }
    }
    return 0;
}


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {

        pt_lbd_gt.x = x;
        pt_lbd_gt.y = y;
        drawFlag = 1;
        lbd_gt=1;

    }
    else if  ( event == EVENT_LBUTTONUP )
    {

        pt_lbu_gt.x = x;
        pt_lbu_gt.y = y;

        drawFlag = 0;
        lbu_gt = 1;

    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {

    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        mbd_gt = 1;//finish one object lableing

    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        pt_mv_gt.x = x;
        pt_mv_gt.y = y;
    }

    return;
}

void DrawButton(Mat &paper,vector<Point> &resetButtonPts, vector<Point> &finishButtonPts){

    const Point* pptRt[1] = {&resetButtonPts[0]};
    int nptRt[] = {resetButtonPts.size()};

    polylines(paper, pptRt, nptRt, 1, 1, Scalar(0,0,255),2,8,0);

    if(pointPolygonTest(resetButtonPts,Point2f(float(pt_mv_gt.x),float(pt_mv_gt.y)),false) > 0){
        fillPoly(paper, pptRt, nptRt, 1, Scalar(0,0,255));
    }else{
        fillPoly(paper, pptRt, nptRt, 1, Scalar(0,255,0));
    }


    const Point* pptFh[1] = {&finishButtonPts[0]};
    int nptFh[] = {finishButtonPts.size()};
    polylines(paper, pptFh, nptFh, 1, 1, Scalar(0,0,255),2,8,0);

     if(pointPolygonTest(finishButtonPts,Point2f(float(pt_mv_gt.x),float(pt_mv_gt.y)),false) > 0){
        fillPoly(paper, pptFh, nptFh, 1, Scalar(0,0,255));
     }else{
        fillPoly(paper, pptFh, nptFh, 1, Scalar(0,255,0));
     }


    putText(paper, "RESET", Point(177,50),  FONT_HERSHEY_COMPLEX, 0.8, Scalar(255,0,0), 1, 8, false);
    putText(paper, "FINISH", Point(377,50), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255,0,0), 1, 8, false);

    putText(paper, "PLEASE SIGN BELLOW", Point(178,100), FONT_HERSHEY_COMPLEX, 0.8, Scalar(0,0,0), 1, 8, false);
    line(paper, Point(1,105),Point(640,105),Scalar(0,0,0),1,8);

}
