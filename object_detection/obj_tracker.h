#ifndef OBJ_TRACKER_H
#define OBJ_TRACKER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "kinect2.h"
#include <time.h>
#include <iostream>

class Object{
public:
	float area;
	// 2D position in color space(in pixles)
	cv::Point centroid;
	// 3D position in camera space(in meters)
	cv::Point3f position;
	// 3D velocity in camera space(in mm/sec)
	cv::Point3f velocity;
	// 3D displacement since last frame(in mm)
	cv::Point3f displacement;
	// absolut timestamp(in sec)
	float timestamp;
	// object bounding rectangle
	cv::Rect bbox;
	// object id
	int id;

	Object(){
		id = -1;
	}

	Object(float area, cv::Point centroid, cv::Point3f position, cv::Rect bbox, int id){
		this->area = area;
		this->centroid = centroid;
		this->position = position;
		this->bbox = bbox;
		this->id = id;
		this->velocity = cv::Point3f(.0f, .0f, .0f);
		this->timestamp = ((float)clock())/CLOCKS_PER_SEC;
	}

	static float distance2d(const Object &obj1, const Object &obj2){
		float dx = float(obj1.centroid.x - obj2.centroid.x);
		float dy = float(obj1.centroid.y - obj2.centroid.y);
		return sqrt(dx*dx + dy*dy);
	}

	static float distance3d(const Object &obj1, const Object &obj2){
		float dx = float(obj1.position.x - obj2.position.x);
		float dy = float(obj1.position.y - obj2.position.y);
		float dz = float(obj1.position.z - obj2.position.z);
		return sqrt(dx*dx + dy*dy + dz*dz);
	}

	friend std::ostream& operator<< (std::ostream& os, const Object& obj);
};

class ObjTracker{
private:
	unsigned int nextObjId;

public:
	// minimum probability threshold for blobs(0 - 1.0f)
	float minProb;
	// minimum blob area size(in pixels)
	float minArea;
	// maximum displacement in camera space(mm)
	float maxDisplacement;

	int numObjs;

	// simple low-pass filter
	float alpha;

	std::vector<Object> objects;

	ObjTracker();
	ObjTracker(float minProb, float minArea, int numObjs);
	void track(cv::Mat &Imsk, cv::Rect roi, Kinect2 &kinect, cv::Size maxSize = cv::Size(0,0));

};

#endif /* OBJ_TRACKER_H */