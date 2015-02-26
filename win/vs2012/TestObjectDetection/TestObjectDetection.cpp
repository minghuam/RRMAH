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

int numObjs = 1;
std::mutex g_mutex;
std::condition_variable g_cv;
int g_count = 3;

void predictAsync(ObjDetector *obj, cv::Mat *Iraw, cv::Mat *Imsk){
	obj->predict(*Iraw, *Imsk, 0.2f, 100);
	std::unique_lock<std::mutex> lck(g_mutex);
	g_count--;
	g_cv.notify_all();
}

int _tmain(int argc, _TCHAR* argv[])
{
	cv::setUseOptimized( true );

	if(argc < 2){
		std::cout << "invalid arguments: dataDir" << std::endl;
		return -1;
	}

	numObjs = argc - 1;

	// load object detectors
	std::vector<ObjDetector*> objDetectors;
	for(int i = 0; i < numObjs; i++){
		ObjDetector *oj = new ObjDetector();
		std::string modelDir = DirHelper::combinePath(argv[i+1], "model");
		if(oj->load(modelDir)){
			std::cerr << "faild to load " + modelDir << std::endl;
			return -1;
		}
		objDetectors.push_back(oj);
	}

	cv::Mat rawImg;
	cv::Mat roiImg;
	cv::Mat depthImg;
	std::vector<ObjTracker*> trackers;
	std::vector<cv::Mat> mskImgs;
	std::vector<std::thread> thrs;
	for(int i = 0; i < numObjs; i++){
		ObjTracker *tr = new ObjTracker();
		tr->numObjs = 2;
		trackers.push_back(tr);
		mskImgs.push_back(cv::Mat());
		thrs.push_back(std::thread());
	}

	// load ROI
	int imgWidth = 384;
	int imgHeight = 216;
	cv::Rect imageROI;
	cv::FileStorage fs;
	fs.open("roi.xml", cv::FileStorage::READ);
	fs["roi"] >> imageROI;
	fs.release();
	if(imageROI.width == 0){
		imageROI = cv::Rect(0, 0, imgWidth, imgHeight);
	}

	// open kinect
	Kinect2 kinect;
	if(kinect.open()){
		std::cerr << "Failed to open kinect." << std::endl;
		return -1;
	}

	std::stringstream ss;

	while(1){
		// query latest frame
		kinect.update();
		
		// depth image
		if(kinect.getIsDepthFrameNew()){
		}

		// color image
		if(kinect.getIsColorFrameNew()){
			rawImg = cv::Mat(kinect.imageHeight, kinect.imageWidth, CV_8UC4, kinect.getImageData());

			roiImg = rawImg(imageROI);
			cv::resize(roiImg, roiImg, cv::Size(roiImg.cols/2, roiImg.rows/2));
			cv::rectangle(rawImg, imageROI, cv::Scalar(0, 0, 255));
			
			g_count = numObjs;
			int64 start = cv::getTickCount();
			for(int i = 0; i < numObjs; i++){
				thrs[i] = std::thread(predictAsync, objDetectors[i], &roiImg, &mskImgs[i]);
			}

			// wait for all threads done
			std::unique_lock<std::mutex> lck(g_mutex);
			while(g_count){
				g_cv.wait(lck);
			}

			for(int i = 0; i < numObjs; i++){
				thrs[i].join();
			}
			
			// track objects
			for(int i = 0; i < numObjs; i++){
				trackers[i]->track(mskImgs[i]);
			}

			// draw tracking results
			std::vector<cv::Scalar> colors;
			colors.push_back(cv::Scalar(0,0,255));
			colors.push_back(cv::Scalar(0,255,0));
			colors.push_back(cv::Scalar(255,0,0));
			for(int i = 0; i < numObjs; i++){
				for(int j = 0; j < (int)trackers[i]->objects.size(); j++){
					Object obj = trackers[i]->objects[j];
					cv::rectangle(roiImg, obj.bbox, colors[i]);
					char text[32];
					sprintf(text, "id=%d", obj.id);
					cv::putText(roiImg, text, obj.centroid, CV_FONT_HERSHEY_PLAIN, 1.0, colors[i]);
				}
			}
			double seconds = (cv::getTickCount() - start)/cv::getTickFrequency();
	
			ss.str("");
			ss << "test time: " << seconds * 1000 << " ms\n";
			std::cout << ss.str();
			for(int i = 0; i < numObjs; i++){
				char buf[32];
				sprintf(buf, "mask %d", i + 1);
				cv::imshow(buf, mskImgs[i]);
			}

			// draw body frame
			if(kinect.getIsBodyFrameNew()){
				ICoordinateMapper *mapper = kinect.getCoordinateMapper();
				Joint *joints = kinect.getJoints();
				// head
				ColorSpacePoint colorPt;
				mapper->MapCameraPointToColorSpace(joints[JointType_Head].Position, &colorPt);
				cv::circle(rawImg, cv::Point((int)colorPt.X, (int)colorPt.Y), 5, cv::Scalar(0, 255, 0), -1);
				// left shoulder
				mapper->MapCameraPointToColorSpace(joints[JointType_ShoulderLeft].Position, &colorPt);
				cv::circle(rawImg, cv::Point((int)colorPt.X, (int)colorPt.Y), 5, cv::Scalar(0, 255, 0), -1);
				// right shoulder
				mapper->MapCameraPointToColorSpace(joints[JointType_ShoulderRight].Position, &colorPt);
				cv::circle(rawImg, cv::Point((int)colorPt.X, (int)colorPt.Y), 5, cv::Scalar(0, 255, 0), -1);
				// sholder spin
				mapper->MapCameraPointToColorSpace(joints[JointType_SpineShoulder].Position, &colorPt);
				cv::circle(rawImg, cv::Point((int)colorPt.X, (int)colorPt.Y), 5, cv::Scalar(0, 255, 0), -1);
			}
			
			cv::resize(rawImg, rawImg, cv::Size(kinect.imageWidth/2, kinect.imageHeight/2));
			cv::imshow("raw", rawImg);
			cv::imshow("roi", roiImg);
		}

		int key = cv::waitKey(30) & 0xFF;
		if(key == 27){
			break;
		}
	}

	for(int i = 0; i < numObjs; i++){
		delete objDetectors[i];
		delete trackers[i];
	}

	return 0;
}

