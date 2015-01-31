#ifndef OBJ_COLOR_FEATURE_H
#define OBJ_COLOR_FEATURE_H

#include <opencv2/opencv.hpp>

class ObjColorFeature{
private:
	int patchSize;
public:
	ObjColorFeature();
	int getPatchSize();
	void computeFeature(const cv::Mat &bgr, cv::Mat &feat);
	void computeLabel(const cv::Mat &bgr, cv::Mat &label);
};

#endif