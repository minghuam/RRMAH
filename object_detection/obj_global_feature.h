#ifndef OBJ_GLOBAL_FEATURE
#define OBJ_GLOBAL_FEATURE

#include <opencv2/opencv.hpp>

class ObjGlobalFeature
{
public:
	ObjGlobalFeature();
	void computeFeature(const cv::Mat &bgr, cv::Mat &feat);

};

#endif