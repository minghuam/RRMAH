#ifndef OBJ_TRACKER_H
#define OBJ_TRACKER_H

#include <opencv2/opencv.hpp>
#include <vector>

class Object{
public:
	float area;
	cv::Point centroid;
	cv::Rect bbox;
	int id;

	Object(){
		id = -1;
	}

	Object(float area, cv::Point centroid, cv::Rect bbox, int id){
		this->area = area;
		this->centroid = centroid;
		this->bbox = bbox;
		this->id = id;
	}

	static float distance(const Object &obj1, const Object &obj2){
		float dx = float(obj1.centroid.x - obj2.centroid.x);
		float dy = float(obj1.centroid.y - obj2.centroid.y);
		return sqrt(dx*dx + dy*dy);
	}
};

class ObjTracker{
private:
	unsigned int nextObjId;

public:
	float minProb;
	float minArea;
	int numObjs;

	std::vector<Object> objects;

	ObjTracker();
	ObjTracker(float minProb, float minArea, int numObjs);
	void track(cv::Mat &Imsk);

};

#endif /* OBJ_TRACKER_H */