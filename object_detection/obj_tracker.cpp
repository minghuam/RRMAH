#include "obj_tracker.h"
#include <queue>

class ObjCompare{
public:
	bool operator()(const Object &obj1, const Object &obj2){
		return obj1.area > obj2.area;
	}
};

ObjTracker::ObjTracker(){
	minProb = 0.1;
	minArea = 200;
	numObjs = 1;
}

ObjTracker::ObjTracker(float minProb, float minArea, int numObjs):
	minProb(minProb), minArea(minArea), numObjs(numObjs)
{}

void ObjTracker::track(cv::Mat &Imsk){
	objects.clear();

	// probability threshold
	cv::Mat I;
	Imsk.copyTo(I);
	I = I > minProb;
	I.copyTo(Imsk);

	// size threshold
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(I, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	std::priority_queue<Object, std::vector<Object>, ObjCompare> objs;

	for(int i = 0; i < contours.size(); i++){
		float area = cv::contourArea(cv::Mat(contours[i]));
		if(area < minArea){
			continue;
		}

		cv::Moments mom = cv::moments(contours[i], false);
		cv::Point centroid((int)(mom.m10/mom.m00), (int)(mom.m01/mom.m00));
		cv::Rect bbox = cv::boundingRect(contours[i]);

		if(objs.size() == numObjs){
			Object obj = objs.top();
			if(area > obj.area){
				objs.pop();
				objs.push(Object(area, centroid, bbox));
			}
		}else{
			objs.push(Object(area, centroid, bbox));
		}
	}

	while(objs.size()){
		objects.push_back(objs.top());
		objs.pop();
	}
}