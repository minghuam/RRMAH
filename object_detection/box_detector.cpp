#include "stdafx.h"
#include "box_detector.h"
#include <queue>

// pairwise distance
class DistancePair{
public:
	Object *newObj;
	Object *oldObj;

	float distance;

	DistancePair(Object *newObj, Object *oldObj){
		this->newObj = newObj;
		this->oldObj = oldObj;
		this->distance = Object::distance2d(*newObj, *oldObj);
	}
};

// compare pairwise distance
class DistancePairCompare{
public:
	bool operator()(const DistancePair &pair1, const DistancePair &pair2){
		return pair1.distance > pair2.distance;
	}
};

BoxDetector::BoxDetector(){
	boxTracker = new ObjTracker(0.2f, 100, 6);
	boxDetector = new ObjDetector();
	threshold = 10;
}

BoxDetector::~BoxDetector(){
	delete boxTracker;
	boxTracker = NULL;
	delete boxDetector;
	boxDetector = NULL;
}

int BoxDetector::load(std::string modelDir){
	return boxDetector->load(modelDir);
}

unsigned char BoxDetector::detect(cv::Mat &roiImg, cv::Mat &boxMsk, Kinect2 &kinect, Matt &mat){
	unsigned char ret = 0;

	cv::Mat roiMsk;
	boxDetector->predict(roiImg, roiMsk, 0.1f, 100);

	cv::resize(roiMsk, roiMsk, cv::Size(mat.imageROI.width, mat.imageROI.height));
	cv::Rect boxRoi = mat.getBoxBoundingRectColor();
	cv::Rect boxRoiInRoi = cv::Rect(boxRoi.x - mat.imageROI.x, boxRoi.y - mat.imageROI.y, \
		boxRoi.width, boxRoi.height);
	roiMsk(boxRoiInRoi).copyTo(boxMsk);
	boxTracker->track(boxMsk, boxRoi, kinect);

	if(boxTracker->objects.size() > 0){
		// dummy objects
		std::vector<Object> baseObjs;
		for(int i = 0; i < (int)mat.boxBases.size(); i++){
			cv::Point pos = mat.boxBases[i];
			baseObjs.push_back(Object(0, pos, cv::Point3f(), cv::Rect(), i));
		}
		// calculate pair-wise distances
		std::priority_queue<DistancePair, std::vector<DistancePair>, DistancePairCompare> pairs;
		for(int i = 0; i < (int)baseObjs.size(); i++){
			for(int j = 0; j < (int)boxTracker->objects.size(); j++){
				DistancePair pair(&boxTracker->objects[j], &baseObjs[i]);
				pairs.push(pair);
			}
		}
		while(pairs.size()){
			DistancePair p = pairs.top();
			pairs.pop();
			if(p.distance > threshold){
				break;
			}
			if(((1 << p.oldObj->id) & ret) == 0){
				ret = ret | (1 << p.oldObj->id);
			}
		}
	}

	return ret;
}