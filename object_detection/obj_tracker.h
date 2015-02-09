#ifndef OBJ_TRACKER_H
#define OBJ_TRACKER_H

#include <opencv2/opencv.hpp>
#include <vector>

class Object{
public:
	float area;
	cv::Point centroid;
	cv::Rect bbox;
	Object(float area, cv::Point centroid, cv::Rect bbox){
		this->area = area;
		this->centroid = centroid;
		this->bbox = bbox;
	}
};

class ObjTracker{
private:

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