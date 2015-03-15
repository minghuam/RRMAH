#include "stdafx.h"
#include "obj_color_feature.h"
#include <opencv2/opencv.hpp>
#include "dir_helper.h"
#include "obj_detector.h"
#include "obj_tracker.h"
#include "kinect2.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include "matt.h"

const int imgWidth = 1920;
const int imgHeight = 1080;
cv::Point mouseDownPt;
cv::Rect imageROI;
Kinect2 kinect;
Matt mat;

int selectStep = 0;

int clamp(int x, int low, int high){
	if(x < low) return low;
	if(x > high) return high;
	return x;
}

cv::Mat estimateCam2Matt(std::vector<cv::Point> colorPts, Kinect2 &kinect){
	
	float x[8] = {0.0f, 0.5f, 1.0f, 1.0f, 1.0f, 0.5f, 0.0f, 0.0f};
	float y[8] = {0.0f, 0.0f, 0.0f, 0.5f, 1.0f, 1.0f, 1.0f, 0.5f};
	float z[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	cv::Mat from(1, colorPts.size(), CV_32FC3);
	cv::Mat to(1, colorPts.size(), CV_32FC3);

	// map color points to camera points
	for(int i = 0; i < colorPts.size(); i++){
		CameraSpacePoint csp = kinect.colorPointToCameraPoint(colorPts[i].x, colorPts[i].y);
		from.ptr<cv::Point3f>()[i] = cv::Point3f(csp.X, csp.Y, csp.Z);
	}
	// mat space points
	for(int i = 0; i < colorPts.size(); i++){
		to.ptr<cv::Point3f>()[i] = cv::Point3f(x[i], y[i], z[i]);
	}

	// estimate 3D affine transformation
	cv::Mat ret;
	std::vector<unsigned char> outliers;
	cv::estimateAffine3D(from, to, ret, outliers);

	return ret;
}

cv::Mat estimateMatt2Color(std::vector<cv::Point> colorPts){
	
	float x[8] = {0.0f, 0.5f, 1.0f, 1.0f, 1.0f, 0.5f, 0.0f, 0.0f};
	float y[8] = {0.0f, 0.0f, 0.0f, 0.5f, 1.0f, 1.0f, 1.0f, 0.5f};

	cv::Mat from(1, colorPts.size(), CV_32FC2);
	cv::Mat to(1, colorPts.size(), CV_32FC2);
	
	// mat space points
	for(int i = 0; i < colorPts.size(); i++){
		from.ptr<cv::Point2f>()[i] = cv::Point2f(x[i], y[i]);
	}
	// color space points
	for(int i = 0; i < colorPts.size(); i++){
		to.ptr<cv::Point2f>()[i] = cv::Point2f(colorPts[i].x, colorPts[i].y);
	}

	// estimate 2D homography
	return cv::findHomography(from, to);
}

cv::Mat estimateColor2Matt(std::vector<cv::Point> colorPts){

	float x[8] = {0.0f, 0.5f, 1.0f, 1.0f, 1.0f, 0.5f, 0.0f, 0.0f};
	float y[8] = {0.0f, 0.0f, 0.0f, 0.5f, 1.0f, 1.0f, 1.0f, 0.5f};

	cv::Mat from(1, colorPts.size(), CV_32FC2);
	cv::Mat to(1, colorPts.size(), CV_32FC2);
	
	// color space points
	for(int i = 0; i < colorPts.size(); i++){
		from.ptr<cv::Point2f>()[i] = cv::Point2f(colorPts[i].x, colorPts[i].y);
	}
	// mat space points
	for(int i = 0; i < colorPts.size(); i++){
		to.ptr<cv::Point2f>()[i] = cv::Point2f(x[i], y[i]);
	}

	// estimate 2D homography
	return cv::findHomography(from, to);
}

void onMouse(int mouseEvent, int x, int y, int flags, void *param){
	
	CameraSpacePoint temp = kinect.colorPointToCameraPoint(x, y);
	cv::Point3f camPt = cv::Point3f(temp.X, temp.Y, temp.Z);

	switch (mouseEvent)
	{
	case CV_EVENT_LBUTTONDOWN:
		mouseDownPt = cv::Point(x, y);
		break;
	case CV_EVENT_LBUTTONUP:
		mat.corners[selectStep] = cv::Point(x, y);
		std::cout << camPt << std::endl;
		// check cam space positions
		if(camPt.x < -2 || camPt.x > 2 || camPt.y < -2 || camPt.y > 2 || camPt.z < 0.8 || camPt.z > 2){
			break;
		}
		selectStep++;
		if(selectStep > NUM_CTRL_PTS - 1){
			selectStep = 0;
			int left = INT_MAX;
			int right = INT_MIN;
			int top = INT_MAX;
			int bottom = INT_MIN;
			for(int i = 0; i < NUM_CTRL_PTS; i++){
				left = min(mat.corners[i].x, left);
				right = max(mat.corners[i].x, right);
				top = min(mat.corners[i].y, top);
				bottom = max(mat.corners[i].y, bottom);
			}
			imageROI = cv::Rect(left, top, right - left, bottom - top);
			mat.cam2Matt = estimateCam2Matt(mat.corners, kinect);
			mat.matt2Color = estimateMatt2Color(mat.corners);
			mat.color2Matt = estimateColor2Matt(mat.corners);
			std::cout << "cam2Matt matrix: " << std::endl;
			std::cout << mat.cam2Matt << std::endl;
			std::cout << "matt2Color matrix: " << std::endl;
			std::cout << mat.matt2Color << std::endl;
			std::cout << "color2Matt matrix: " << std::endl;
			std::cout << mat.color2Matt << std::endl;
		}
		break;
	case CV_EVENT_RBUTTONUP:
		std::cout << "color space: " << cv::Point(x,y) << std::endl;
		std::cout << "cam space: " << camPt << std::endl;
		std::cout << "mat space: " << mat.cameraToMatt(camPt) << std::endl;
		std::cout << "region start: " << mat.hitTestRegion(camPt, REGION_START) << "," << mat.distanceFromRegion(camPt, REGION_START) << std::endl;
		std::cout << "region left: " << mat.hitTestRegion(camPt, REGION_LEFT) << "," << mat.distanceFromRegion(camPt, REGION_LEFT) <<std::endl;
		std::cout << "region right: " << mat.hitTestRegion(camPt, REGION_RIGHT) << "," << mat.distanceFromRegion(camPt, REGION_RIGHT) <<std::endl;
		std::cout << "region body mid: " << mat.hitTestRegion(camPt, REGION_BODY_MIDDLE) << "," << mat.distanceFromRegion(camPt, REGION_BODY_MIDDLE) <<std::endl;
		std::cout << "region ipsilateral: " << mat.hitTestRegion(camPt, REGION_IPSILATERAL) << "," << mat.distanceFromRegion(camPt, REGION_IPSILATERAL) <<std::endl;
		break;
	default:
		break;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	cv::setUseOptimized( true );

	cv::Mat rawImg;
	cv::Mat roiImg;
	cv::Mat depthImg;
	
	// load ROI
	int imgWidth = 384;
	int imgHeight = 216;
	cv::FileStorage fs;
	if(fs.open("roi.xml", cv::FileStorage::READ)){
		fs["roi"] >> imageROI;
		fs.release();
	}
	if(imageROI.width == 0){
		imageROI = cv::Rect(0, 0, imgWidth, imgHeight);
	}

	// load mat configuration
	mat.load("matt.xml");

	// open kinect
	if(kinect.open()){
		std::cerr << "Failed to open kinect." << std::endl;
		return -1;
	}

	std::stringstream ss;
	cv::namedWindow("raw", 0);
	cv::setWindowProperty("raw", CV_WND_PROP_FULLSCREEN, 1 - cv::getWindowProperty("raw", CV_WND_PROP_FULLSCREEN));
	cv::setMouseCallback("raw", onMouse);
	bool running = true;
	cv::Rect instructionRect(50, 50, 200, 50);
	float cx[8] = {50, 150, 250, 250, 250, 150, 50, 50};
	float cy[8] = {50, 50, 50, 75, 100, 100, 100, 75};
	
	cv::Point instructionCorners[4] = {cv::Point(50, 50), cv::Point(350, 50), cv::Point(350, 150), cv::Point(50, 150)};

	while(running){
		// query latest frame
		kinect.update();
		
		// depth image
		if(kinect.getIsDepthFrameNew()){ }

		// color image
		if(kinect.getIsColorFrameNew()){
			rawImg = cv::Mat(kinect.imageHeight, kinect.imageWidth, CV_8UC4, kinect.getImageData());
			cv::cvtColor(rawImg, rawImg, CV_BGRA2BGR);

			roiImg = rawImg(imageROI);
			cv::resize(roiImg, roiImg, cv::Size(roiImg.cols/2, roiImg.rows/2));

			// draw roi rectangle
			cv::rectangle(rawImg, imageROI, cv::Scalar(0, 0, 255));

			// draw regions
			for(int i = 0; i < mat.regions.size(); i++){
				cv::Rect_<float> rect = mat.regions[i];
				cv::Point2f p1 = mat.mattToColor(cv::Point2f(rect.x, rect.y));
				cv::Point2f p2 = mat.mattToColor(cv::Point2f(rect.x + rect.width, rect.y));
				cv::Point2f p3 = mat.mattToColor(cv::Point2f(rect.x + rect.width, rect.y + rect.height));
				cv::Point2f p4 = mat.mattToColor(cv::Point2f(rect.x, rect.y + rect.height));
				cv::line(rawImg, p1, p2, cv::Scalar(0, 255, 0));
				cv::line(rawImg, p2, p3, cv::Scalar(0, 255, 0));
				cv::line(rawImg, p3, p4, cv::Scalar(0, 255, 0));
				cv::line(rawImg, p4, p1, cv::Scalar(0, 255, 0));
			}

			// draw instruction rect
			cv::rectangle(rawImg, instructionRect, cv::Scalar(0, 0, 255));
			cv::circle(rawImg, cv::Point(cx[selectStep], cy[selectStep]), 5, cv::Scalar(0, 255, 0));
			
			cv::imshow("raw", rawImg);
			cv::imshow("roi", roiImg);
		}

		int key = cv::waitKey(30) & 0xFF;

		switch(key){
		case 27:
			running = false;
			break;
		case'f':
			cv::setWindowProperty("raw", CV_WND_PROP_FULLSCREEN, 1 - cv::getWindowProperty("raw", CV_WND_PROP_FULLSCREEN));
			break;
		case 's':
			std::cout << "saving mat configurations..." << std::endl;
			mat.save("matt.xml");
			std::cout << "saving roi..." << std::endl;
			fs.open("roi.xml", cv::FileStorage::WRITE);
			fs << "roi" << imageROI;
			fs.release();
			break;
		case 'p':
			selectStep--;
			if(selectStep < 0) selectStep = NUM_CTRL_PTS - 1;
			break;
		case 'n':
			selectStep++;
			if(selectStep > NUM_CTRL_PTS - 1) selectStep = 0;
			break;
		}
	}

	return 0;
}

