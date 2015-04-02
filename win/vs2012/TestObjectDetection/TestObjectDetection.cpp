#include "stdafx.h"
#include "obj_color_feature.h"
#include <opencv2/opencv.hpp>
#include "dir_helper.h"
#include "obj_detector.h"
#include "obj_tracker.h"
#include "kinect2.h"
#include "matt.h"
#include "traj_drawer.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <queue>

int numObjs = 1;
int g_count = 1;
std::mutex g_mutex;
std::condition_variable g_cv;

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
	trackers[1]->setMode(1);

	// load mat configuration
	Matt mat;
	mat.load("matt.xml");

	// open kinect
	Kinect2 kinect;
	if(kinect.open()){
		std::cerr << "Failed to open kinect." << std::endl;
		return -1;
	}
	
	cv::namedWindow("raw", 0);
	std::stringstream ss;
	
	std::vector<cv::Scalar> colors;
	colors.push_back(cv::Scalar(0,0,255));
	colors.push_back(cv::Scalar(0,255,0));
	colors.push_back(cv::Scalar(255,0,0));
	colors.push_back(cv::Scalar(255,255,0));
	colors.push_back(cv::Scalar(0,255,255));

	TrajDrawer<cv::Point3f> velDrawer(100, 3);
	TrajDrawer<cv::Point3f> speedDrawer(100, 3);

	bool running = true;

	while(running){
		// query latest frame
		kinect.update();
		
		// depth image
		if(kinect.getIsDepthFrameNew()){
			/*
			cv::Mat depthRaw = cv::Mat(kinect.depthHeight, kinect.depthWidth, CV_16UC1, kinect.getDepthData());
			depthRaw.convertTo(depthImg, CV_8UC1, 255.0/8000.0);
			*/
		}

		// color image
		if(kinect.getIsColorFrameNew()){
			rawImg = cv::Mat(kinect.imageHeight, kinect.imageWidth, CV_8UC4, kinect.getImageData());
			roiImg = rawImg(mat.imageROI);

			cv::resize(roiImg, roiImg, cv::Size(roiImg.cols/2, roiImg.rows/2));
			cv::rectangle(rawImg, mat.imageROI, cv::Scalar(0, 0, 255));
			
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
				trackers[i]->track(mskImgs[i], mat.imageROI, kinect, mat, cv::Size(120, 120));
			}

			double seconds = (cv::getTickCount() - start)/cv::getTickFrequency();

			// draw tracking results
			for(int i = 0; i < numObjs; i++){
				for(int j = 0; j < (int)trackers[i]->objects.size(); j++){
					Object obj = trackers[i]->objects[j];
					std::cout << obj << std::endl;
					std::cout << "mat space: " << mat.cameraToMatt(obj.position) << std::endl;
					cv::rectangle(rawImg, obj.bbox, colors[i]);
					char text[32];
					sprintf(text, "id=%d", obj.id);
					cv::putText(rawImg, text, obj.centroid, CV_FONT_HERSHEY_PLAIN, 2.0, colors[i], 2);

					if(i == 0 && j == 0){
						velDrawer.add(obj.velocity);
						float speed = sqrt(obj.velocity.x * obj.velocity.x + obj.velocity.y * obj.velocity.y + obj.velocity.z * obj.velocity.z);
						float speeFileterd = sqrt(obj.velocityFiltered.x * obj.velocityFiltered.x + obj.velocityFiltered.y * obj.velocityFiltered.y + obj.velocityFiltered.z * obj.velocityFiltered.z);
						float acc = sqrt(obj.acc.x * obj.acc.x + obj.acc.y * obj.acc.y + obj.acc.z * obj.acc.z);
						float speed2D = sqrt(obj.velocity2D.x * obj.velocity2D.x + obj.velocity2D.y * obj.velocity2D.y);
						speedDrawer.add(cv::Point3f(speed, obj.velocity2D.x, obj.velocity2D.y));
					}

				}
			}
	
			ss.str("");
			ss << "test time: " << seconds * 1000 << " ms\n";
			std::cout << ss.str();
			for(int i = 0; i < numObjs; i++){
				char buf[32];
				sprintf(buf, "mask %d", i + 1);
				cv::imshow(buf, mskImgs[i]);
			}

			ICoordinateMapper *mapper = kinect.getCoordinateMapper();
	
			// draw body frame
			if(kinect.getIsBodyFrameNew()){
				Joint *joints = kinect.getJoints();
				// head
				ColorSpacePoint colorPt;
				mapper->MapCameraPointToColorSpace(joints[JointType_Head].Position, &colorPt);
				cv::circle(rawImg, cv::Point((int)colorPt.X, (int)colorPt.Y), 5, cv::Scalar(0, 255, 0), -1);
				CameraSpacePoint p = joints[JointType_Head].Position;
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

			// map camera points to color space for validation
			if(mapper){
				for(int i = 0; i < numObjs; i++){
					for(int j = 0; j < (int)trackers[i]->objects.size(); j++){
							ColorSpacePoint cp;
							Object obj = trackers[i]->objects[j];
							CameraSpacePoint camPt; camPt.X = obj.position.x; camPt.Y = obj.position.y; camPt.Z = obj.position.z;
							mapper->MapCameraPointToColorSpace(camPt, &cp);
							cv::circle(rawImg, cv::Point((int)cp.X, (int)cp.Y), 5, cv::Scalar(0, 255, 0), -1);
					}
				}
			}
			
			// draw regions
			for(int i = 0; i < mat.regions.size(); i++){
				cv::Rect_<float> rect = mat.regions[i];
				cv::Point2f p1 = mat.mattToColor(cv::Point2f(rect.x, rect.y));
				cv::Point2f p2 = mat.mattToColor(cv::Point2f(rect.x + rect.width, rect.y));
				cv::Point2f p3 = mat.mattToColor(cv::Point2f(rect.x + rect.width, rect.y + rect.height));
				cv::Point2f p4 = mat.mattToColor(cv::Point2f(rect.x, rect.y + rect.height));
				cv::Scalar color = cv::Scalar(0, 255, 0);
				for(int j = 0; j < numObjs; j++){
					for(int k = 0; k < (int)trackers[j]->objects.size(); k++){
						Object obj = trackers[j]->objects[k];
						if(mat.hitTestRegion(obj.centroid, i)){
							color = cv::Scalar(0, 0, 255);
						}
						if( i == 0)
						std::cout << mat.distanceFromRegion(obj.centroid, i) << "," << mat.distanceFromRegion(obj.position, i) << std::endl;
					}
				}
				cv::line(rawImg, p1, p2, color);
				cv::line(rawImg, p2, p3, color);
				cv::line(rawImg, p3, p4, color);
				cv::line(rawImg, p4, p1, color);
			}

			// draw box rect
			cv::rectangle(rawImg, mat.getBoxBoundingRectColor(), cv::Scalar(0, 255, 255));

			velDrawer.show("velocity", 0.2f);
			speedDrawer.show("speed", 0.1f);

			cv::resize(rawImg, rawImg, cv::Size(kinect.imageWidth/2, kinect.imageHeight/2));
			cv::imshow("raw", rawImg);
		}

		int key = cv::waitKey(30) & 0xFF;
		switch(key){
		case 27:
			running = false;
			break;
		case'f':
			cv::setWindowProperty("raw", CV_WND_PROP_FULLSCREEN, 1 - cv::getWindowProperty("raw", CV_WND_PROP_FULLSCREEN));
			break;
		}
	}

	for(int i = 0; i < numObjs; i++){
		delete objDetectors[i];
		delete trackers[i];
	}

	return 0;
}

