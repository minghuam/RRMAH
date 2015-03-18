#include "stdafx.h"
#include "obj_tracker.h"
#include <queue>
#include <unordered_map>
#include <time.h>

// compare object areas
class ObjSizeCompare{
public:
	bool operator()(const Object &obj1, const Object &obj2){
		return obj1.area > obj2.area;
	}
};

// compare object positions
class ObjPosCompare{
public:
	bool operator()(const Object &obj1, const Object &obj2){
		return obj1.position.x < obj2.position.x;
	}
};

// pairwise distance
class DistancePair{
public:
	Object *newObj;
	Object *oldObj;

	float distance;

	DistancePair(Object *newObj, Object *oldObj){
		this->newObj = newObj;
		this->oldObj = oldObj;
		this->distance = Object::distance3d(*newObj, *oldObj);
	}
};

// compare pairwise distance
class DistancePairCompare{
public:
	bool operator()(const DistancePair &pair1, const DistancePair &pair2){
		return pair1.distance > pair2.distance;
	}
};

std::ostream& operator <<(std::ostream& os, const Object& obj){
	os << "id: " << obj.id << ", cx: " << obj.centroid.x << ", cy: " << obj.centroid.y << \
		", x: " << obj.position.x << ", y: " << obj.position.y << ", z: " << obj.position.z << \
		", vx: " << obj.velocity.x << ", vy: " << obj.velocity.y << ", vz: " << obj.velocity.z << \
		", dx: " << obj.displacement.x << ", dy: " << obj.displacement.y << ", dz: " << obj.displacement.z << \
		", t: " << obj.timestamp;
	return os;
}

ObjTracker::ObjTracker(){
	minProb = 0.1f;
	minArea = 200;
	numObjs = 1;
	nextObjId = 0;
	alpha = 0.3f;
	maxDisplacement = 200.0f;	
}

ObjTracker::ObjTracker(float minProb, float minArea, int numObjs):
	minProb(minProb), minArea(minArea), numObjs(numObjs), nextObjId(0)
{}

void ObjTracker::track(cv::Mat &Imsk, cv::Rect roi, Kinect2 &kinect, cv::Size maxSize){
	std::vector<Object> newObjs;

	// probability threshold
	cv::Mat I;
	Imsk.copyTo(I);
	I = I > minProb;
	I.copyTo(Imsk);

	// limit size
	if(maxSize.width > 0 && maxSize.height > 0){
		// get contours
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat temp;
		Imsk.copyTo(temp);
		cv::findContours(temp, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		for(int i = 0; i < (int)contours.size(); i++){
			// get restricted roi
			cv::Rect oroi = cv::boundingRect(contours[i]);
			cv::Rect rroi = oroi;
			if(rroi.width > maxSize.width || rroi.height > maxSize.height){
				// get the bottom/left/right pixel
				cv::Point bottom(0, INT_MIN);
				cv::Point left(INT_MAX, 0);
				cv::Point right(INT_MIN, 0);
				for(int j = 0; j < (int)contours[i].size(); j++){
					if(bottom.y < contours[i][j].y){
						bottom = contours[i][j];
					}
					if(left.x > contours[i][j].x){
						left = contours[i][j];
					}
					if(right.x < contours[i][j].x){
						right = contours[i][j];
					}
				}
				// restrict height
				if(rroi.height > maxSize.height){
					// get bottom part only
					rroi = cv::Rect(rroi.x, bottom.y - maxSize.height + 1, rroi.width, maxSize.height);
				}
				// restrict width
				if(rroi.width > maxSize.height){
					if(abs(bottom.x - left.x) < abs(bottom.x - right.x)){
						// get left part
						rroi = cv::Rect(left.x, rroi.y, maxSize.width, rroi.height);
					}else{
						// get right part
						rroi = cv::Rect(right.x - maxSize.width + 1, rroi.y, maxSize.width, rroi.height);
					}
				}
				// modify mask
				cv::Mat smallMsk;
				Imsk(rroi).copyTo(smallMsk);
				cv::Mat(oroi.height, oroi.width, CV_8U, cv::Scalar(0)).copyTo(Imsk(oroi));
				smallMsk.copyTo(Imsk(rroi));
			}
		}
	}


	// size threshold
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat temp;
	Imsk.copyTo(temp);
	cv::findContours(temp, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	// get the largest contours
	std::priority_queue<Object, std::vector<Object>, ObjSizeCompare> objs;
	for(int i = 0; i < (int)contours.size(); i++){
		// calculate contour area
		float area = (float)cv::contourArea(cv::Mat(contours[i]));
		if(area < minArea){
			continue;
		}

		// transform
		float scaleX = roi.width/(float)Imsk.cols;
		float scaleY = roi.height/(float)Imsk.rows;
		float deltaX = roi.x;
		float deltaY = roi.y;

		// calculate centroid and bbox
		cv::Moments mom = cv::moments(contours[i], false);
		int cx = (int)(mom.m10/mom.m00);
		int cy = (int)(mom.m01/mom.m00);
		float colorX = cx * scaleX + deltaX;
		float colorY = cy * scaleY + deltaY;
		cv::Point centroid(colorX, colorY);
		
		cv::Rect bbox = cv::boundingRect(contours[i]);
		bbox.x = bbox.x * scaleX + deltaX;
		bbox.y = bbox.y * scaleY + deltaY;
		bbox.width *= scaleX;
		bbox.height *= scaleY;

		// get 3d position
		CameraSpacePoint p = kinect.colorPointToCameraPoint(colorX, colorY);
		cv::Point3f position(p.X, p.Y, p.Z);

		// push to heap
		if(objs.size() == numObjs){
			Object obj = objs.top();
			if(area > obj.area){
				objs.pop();
				objs.push(Object(area, centroid, position, bbox, -1));
			}
		}else{
			objs.push(Object(area, centroid, position, bbox, -1));
		}
	}

	// get largest objects
	while(objs.size()){
		newObjs.push_back(objs.top());
		objs.pop();
	}

	if(objects.size() > 0){
		// calculate pair-wise distances
		std::priority_queue<DistancePair, std::vector<DistancePair>, DistancePairCompare> pairs;
		for(int i = 0; i < (int)newObjs.size(); i++){
			for(int j = 0; j < (int)objects.size(); j++){
				DistancePair pair(&newObjs[i], &objects[j]);
				pairs.push(pair);
			}
		}

		// assign id, calculate displacement and velocity
		while(pairs.size()){
			DistancePair p = pairs.top();
			pairs.pop();
			if(p.newObj->id == -1 && p.oldObj->id != -1){

				// if displacement is to large, assign new id
				cv::Point3f deltaPos = 1000.0f * (p.newObj->position - p.oldObj->position);
				float absDisplacement = sqrt(deltaPos.x * deltaPos.x + deltaPos.y * deltaPos.y + deltaPos.z * deltaPos.z);
				if(absDisplacement > maxDisplacement){
					continue;
				}

				// simple low-pass filter
				p.newObj->position.x = p.newObj->position.x * alpha + (1.0f - alpha) * p.oldObj->position.x;
				p.newObj->position.y = p.newObj->position.y * alpha + (1.0f - alpha) * p.oldObj->position.y;
				p.newObj->position.z = p.newObj->position.z * alpha + (1.0f - alpha) * p.oldObj->position.z;

				p.newObj->id = p.oldObj->id;
				p.oldObj->id = -1;
				p.newObj->displacement = deltaPos;
				float time = p.newObj->timestamp - p.oldObj->timestamp;
				if(time > 0.0f){
					p.newObj->velocity.x = p.newObj->displacement.x/time;
					p.newObj->velocity.y = p.newObj->displacement.y/time;
					p.newObj->velocity.z = p.newObj->displacement.z/time;
				}
			}
		}
	}
	
	// assign new object id
	for(int i = 0; i < (int)newObjs.size(); i++){
		if(newObjs[i].id == -1){
			newObjs[i].id = nextObjId++;
		}
	}

	// sort
	sort(newObjs.begin(), newObjs.end(), ObjPosCompare());

	// return
	objects = newObjs;
}