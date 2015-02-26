#include "obj_color_feature.h"
#include <opencv2/opencv.hpp>
#include "tinylog.hpp"
#include "tinydir.hpp"
#include "obj_detector.h"
#include "obj_tracker.h"
#include "kinect2.h"
#include <thread>
#include <mutex>
#include <condition_variable>

#define NUM_OBJS 3

std::mutex g_mutex;
std::condition_variable g_cv;
int g_count = 3;

void predictAsync(ObjDetector *obj, cv::Mat *Iraw, cv::Mat *Imsk){
	obj->predict(*Iraw, *Imsk, 0.2f, 100);
	std::unique_lock<std::mutex> lck(g_mutex);
	g_count--;
	g_cv.notify_all();
}

int main(int argc, char **argv){

	cv::setUseOptimized( true );

	ObjColorFeature objFeature;
	std::vector<ObjDetector*> objDetectors;
	for(int i = 0; i < NUM_OBJS; i++){
		objDetectors.push_back(new ObjDetector());
	}


	std::vector<std::string> folders;
	folders.push_back(".\\hand3");
	folders.push_back(".\\obj10");
	folders.push_back(".\\obj11");

	std::vector<std::string> trainImgDirs;
	std::vector<std::string> trainMskDirs;
	trainImgDirs.push_back(".\\hand3\\train\\image\\");
	trainMskDirs.push_back(".\\hand3\\train\\mask\\");
	trainImgDirs.push_back(".\\obj10\\train\\image\\");
	trainMskDirs.push_back(".\\obj10\\train\\mask\\");
	trainImgDirs.push_back(".\\obj11\\train\\image\\");
	trainMskDirs.push_back(".\\obj11\\train\\mask\\");

#if 0
	// train
	std::cout << "training..." << std::endl;
	for(int i = 0; i < NUM_OBJS; i++){
		std::string imgDir = folders[i] + "\\train\\image\\";
		std::string mskDir = folders[i] + "\\train\\mask\\";
		std::vector<std::string> trainImgs = list_dir(imgDir.c_str(), "*.jpg");
		std::vector<std::string> trainMsks = list_dir(mskDir.c_str(), "*.jpg");
		assert(trainImgs.size() == trainMsks.size());
		objDetectors[i]->trainBatchAsync(trainImgs, trainMsks, 4);
		while(!objDetectors[i]->isModelReady()){
			std::cout << ".";
			Sleep(1000);
		}
		std::string modelDir = folders[i] + "\\model\\";
		objDetectors[i]->save(modelDir);
	}
#endif

	// load
	for(int i = 0; i < NUM_OBJS; i++){
		std::string modelDir = folders[i] + "\\model\\";
		objDetectors[i]->load(modelDir.c_str());
	}

	cv::Mat rawImg;
	cv::Mat roiImg;
	cv::Mat depthImg;
	std::vector<ObjTracker*> trackers;
	std::vector<cv::Mat> mskImgs;
	std::vector<std::thread> thrs;

	for(int i = 0; i < NUM_OBJS; i++){
		trackers.push_back(new ObjTracker());
		mskImgs.push_back(cv::Mat());
		thrs.push_back(std::thread());
	}
	trackers[0]->numObjs = 2;
	trackers[0]->minArea = 500;
	trackers[1]->numObjs = 2;
	trackers[2]->numObjs = 2;

	Kinect2 kinect;
	if(kinect.open()){
		std::cout << "Failed to open kinect." << std::endl;
	}

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
			
			g_count = 3;
			int64 start = cv::getTickCount();
			for(int i = 0; i < NUM_OBJS; i++){
				thrs[i] = std::thread(predictAsync, objDetectors[i], &roiImg, &mskImgs[i]);
			}

			std::unique_lock<std::mutex> lck(g_mutex);
			while(g_count){
				g_cv.wait(lck);
			}

			for(int i = 0; i < NUM_OBJS; i++){
				thrs[i].join();
			}
			
			for(int i = 0; i < NUM_OBJS; i++){
				trackers[i]->track(mskImgs[i]);
			}

			std::vector<cv::Scalar> colors;
			colors.push_back(cv::Scalar(0,0,255));
			colors.push_back(cv::Scalar(0,255,0));
			colors.push_back(cv::Scalar(255,0,0));
			for(int i = 0; i < NUM_OBJS; i++){
				for(int j = 0; j < (int)trackers[i]->objects.size(); j++){
					Object obj = trackers[i]->objects[j];
					cv::rectangle(roiImg, obj.bbox, colors[i]);
					char text[32];
					sprintf(text, "id=%d", obj.id);
					cv::putText(roiImg, text, obj.centroid, CV_FONT_HERSHEY_PLAIN, 1.0, colors[i]);
				}
			}
			double seconds = (cv::getTickCount() - start)/cv::getTickFrequency();
			
			LOGF("test time: %f ms.", seconds * 1000);
			for(int i = 0; i < NUM_OBJS; i++){
				char buf[32];
				sprintf(buf, "mask %d", i + 1);
				cv::imshow(buf, mskImgs[i]);
			}

			// body frame
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

	for(int i = 0; i < NUM_OBJS; i++){
		delete objDetectors[i];
		delete trackers[i];
	}

	return 0;
}