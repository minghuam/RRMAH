#include "stdafx.h"
#include "obj_tracker.h"
#include <queue>
#include <unordered_map>

class ObjCompare{
public:
	bool operator()(const Object &obj1, const Object &obj2){
		return obj1.area > obj2.area;
	}
};

class DistancePair{
public:
	Object *newObj;
	Object *oldObj;

	float distance;

	DistancePair(Object *newObj, Object *oldObj){
		this->newObj = newObj;
		this->oldObj = oldObj;
		this->distance = Object::distance(*newObj, *oldObj);
	}
};

class DistancePairCompare{
public:
	bool operator()(const DistancePair &pair1, const DistancePair &pair2){
		return pair1.distance > pair2.distance;
	}
};

ObjTracker::ObjTracker(){
	minProb = 0.1f;
	minArea = 200;
	numObjs = 1;
	nextObjId = 0;
}

ObjTracker::ObjTracker(float minProb, float minArea, int numObjs):
	minProb(minProb), minArea(minArea), numObjs(numObjs), nextObjId(0)
{}

void ObjTracker::track(cv::Mat &Imsk){
	std::vector<Object> newObjs;

	// probability threshold
	cv::Mat I;
	Imsk.copyTo(I);
	I = I > minProb;
	I.copyTo(Imsk);

	// size threshold
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(I, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	// get the largest contours
	std::priority_queue<Object, std::vector<Object>, ObjCompare> objs;
	for(int i = 0; i < (int)contours.size(); i++){
		float area = (float)cv::contourArea(cv::Mat(contours[i]));
		if(area < minArea){
			continue;
		}

		// calculate centroid
		cv::Moments mom = cv::moments(contours[i], false);
		cv::Point centroid((int)(mom.m10/mom.m00), (int)(mom.m01/mom.m00));
		cv::Rect bbox = cv::boundingRect(contours[i]);

		if(objs.size() == numObjs){
			Object obj = objs.top();
			if(area > obj.area){
				objs.pop();
				objs.push(Object(area, centroid, bbox, -1));
			}
		}else{
			objs.push(Object(area, centroid, bbox, -1));
		}
	}

	while(objs.size()){
		newObjs.push_back(objs.top());
		objs.pop();
	}

	if(objects.size() > 0){
		// pair-wise distances
		std::priority_queue<DistancePair, std::vector<DistancePair>, DistancePairCompare> pairs;
		for(int i = 0; i < (int)newObjs.size(); i++){
			for(int j = 0; j < (int)objects.size(); j++){
				DistancePair pair(&newObjs[i], &objects[j]);
				pairs.push(pair);
			}
		}

		// assign id
		while(pairs.size()){
			DistancePair p = pairs.top();
			pairs.pop();
			if(p.newObj->id == -1 && p.oldObj->id != -1){
				p.newObj->id = p.oldObj->id;
				p.oldObj->id = -1;
			}
		}
	}
	
	for(int i = 0; i < (int)newObjs.size(); i++){
		if(newObjs[i].id == -1){
			newObjs[i].id = nextObjId++;
		}
	}

	objects = newObjs;
}