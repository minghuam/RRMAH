#ifndef OBJ_DETECTOR_H
#define OBJ_DETECTOR_H

#include "obj_color_feature.h"
#include "obj_global_feature.h"
#include <opencv2/opencv.hpp>

class ObjDetector{
private:
	CvRTParams rt_params;
	
	ObjColorFeature color_feature;
	ObjGlobalFeature global_feature;

	cv::Mat gdescriptor;
	std::vector<CvRTrees*> random_trees;

	cv::flann::Index *flannIndex;

public:
	ObjDetector();
	~ObjDetector();

	int load(const char *dir);
	int save(const char *dir);
	void train(cv::Mat &Iraw, cv::Mat &Imsk);
	void trainBatch(std::vector<std::string> &imgDir, std::vector<std::string> &mskDir);
	void predict(cv::Mat &Iraw, cv::Mat &Imsk);
};

#endif /* OBJ_DETECTOR_H */