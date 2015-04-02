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
			mat.imageROI = cv::Rect(left, top, right - left, bottom - top);
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

class ObjXCompare{
public:
	bool operator()(const Object &obj1, const Object &obj2){
		return obj1.centroid.x < obj2.centroid.x;
	}
};

class ObjYCompare{
public:
	bool operator()(const Object &obj1, const Object &obj2){
		return obj1.centroid.y < obj2.centroid.y;
	}
};

void getBoxBases(ObjTracker &boxTracker, Matt &mat){
	if(boxTracker.objects.size() == 6){
		// get left 3 objs and right 3 objs
		sort(boxTracker.objects.begin(), boxTracker.objects.end(), ObjXCompare());
		std::vector<Object> left_objs(boxTracker.objects.begin(), boxTracker.objects.begin() + 3);
		std::vector<Object> right_objs(boxTracker.objects.begin() + 3, boxTracker.objects.end());
		sort(left_objs.begin(), left_objs.end(), ObjYCompare());
		sort(right_objs.begin(), right_objs.end(), ObjYCompare());

		// get upper 2 objs and lower 2 objs
		sort(boxTracker.objects.begin(), boxTracker.objects.end(), ObjYCompare());
		std::vector<Object> up_objs(boxTracker.objects.begin(), boxTracker.objects.begin() + 2);
		std::vector<Object> low_objs(boxTracker.objects.begin() + 4, boxTracker.objects.end());
		sort(up_objs.begin(), up_objs.end(), ObjXCompare());
		sort(low_objs.begin(), low_objs.end(), ObjXCompare());
	
		if(left_objs[0].id == up_objs[0].id && right_objs[0].id == up_objs[1].id && \
			left_objs[2].id == low_objs[0].id && right_objs[2].id == low_objs[1].id){
				mat.boxBases[0] = left_objs[0].centroid;
				mat.boxBases[1] = left_objs[1].centroid;
				mat.boxBases[2] = left_objs[2].centroid;
				mat.boxBases[3] = right_objs[0].centroid;
				mat.boxBases[4] = right_objs[1].centroid;
				mat.boxBases[5] = right_objs[2].centroid;
		}
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	cv::setUseOptimized( true );

	if(argc < 2){
		std::cout << "invalid arguments: dataDir" << std::endl;
		return -1;
	}

	// load detector for box
	ObjDetector boxDetector;
	std::string modelDir = DirHelper::combinePath(argv[1], "model");
	if(boxDetector.load(modelDir)){
		std::cerr << "failed to load " + modelDir << std::endl;
		return -1;
	}
	ObjTracker boxTracker(0.4f, 100, 6);

	cv::Mat rawImg;
	cv::Mat roiImg;
	cv::Mat roiMsk;
	cv::Mat boxImg;
	cv::Mat boxMsk;
	cv::Mat depthImg;

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
			
			roiImg = rawImg(mat.imageROI);
			// resize to 1/2
			cv::resize(roiImg, roiImg, cv::Size(roiImg.cols/2, roiImg.rows/2));

			// detect object base
			boxDetector.predict(roiImg, roiMsk, 0.1f, 100);
			cv::resize(roiMsk, roiMsk, cv::Size(roiMsk.cols*2, roiMsk.rows*2));
			cv::Rect boxRoi = mat.getBoxBoundingRectColor();
			cv::Rect boxRoiInRoi = cv::Rect(boxRoi.x - mat.imageROI.x, boxRoi.y - mat.imageROI.y, \
				boxRoi.width, boxRoi.height);
			roiMsk(boxRoiInRoi).copyTo(boxMsk);
			boxTracker.track(boxMsk, boxRoi, kinect);

			// find matches
			getBoxBases(boxTracker, mat);

			// draw object bases
			for(int i = 0; i < (int)boxTracker.objects.size(); i++){
				cv::circle(rawImg, boxTracker.objects[i].centroid, 5, cv::Scalar(0, 255, 0), -1);
			}

			// draw matched bases
			for(int i = 0; i < 6; i++){
				cv::circle(rawImg, mat.boxBases[i], 6, cv::Scalar(0, 0, 255), 2);
			}

			// draw roi rectangle
			cv::rectangle(rawImg, mat.imageROI, cv::Scalar(0, 0, 255));

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

			// draw box bounding box
			cv::rectangle(rawImg, boxRoi, cv::Scalar(0, 0, 255));

			// draw instruction rect
			cv::rectangle(rawImg, instructionRect, cv::Scalar(0, 0, 255));
			cv::circle(rawImg, cv::Point(cx[selectStep], cy[selectStep]), 5, cv::Scalar(0, 255, 0));

			//cv::putText(rawImg, "press m to select mat points", cv::Point(rawImg.cols - 400, 100), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0), 1);
			//cv::putText(rawImg, "press b to select box points", cv::Point(rawImg.cols - 400, 120), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0), 1);

			cv::imshow("roi", roiImg);
			cv::imshow("raw", rawImg);
			cv::imshow("roiMsk", roiMsk);
			cv::imshow("boxMsk", boxMsk);
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

