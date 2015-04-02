#ifndef MATT_H
#define MATT_H

#include <opencv2/opencv.hpp>
#include <iostream>

#define NUM_CTRL_PTS       8
#define NUM_REGIONS        5
#define REGION_UNKNOWN     -1
#define REGION_START       0
#define REGION_LEFT        1
#define REGION_RIGHT       2
#define REGION_BODY_MIDDLE 3
#define REGION_IPSILATERAL 4

class Matt{
private:
	// regions in color space
	std::vector<std::vector<cv::Point2f> > colorRegions;

public:
	Matt();
	// regions in mat space
	std::vector<cv::Rect_<float> > regions;
	// box region in mat space
	cv::Rect_<float> boxRect;
	// corners in color space
	std::vector<cv::Point> corners;
	// object bases
	std::vector<cv::Point> boxBases;

	// image roi
	cv::Rect imageROI;
	cv::Mat cam2Matt;
	cv::Mat matt2Color;
	cv::Mat color2Matt;

	int load(std::string config);
	void save(std::string config);

	cv::Point3f cameraToMatt(cv::Point3f camPt);
	cv::Point2f mattToColor(cv::Point2f matPt);
	cv::Point2f colorToMatt(cv::Point colorPt);
	int hitTestRegion(cv::Point3f camPt, int region);
	int hitTestRegion(cv::Point colorPt, int region);

	// return distance in mat space
	float distanceFromRegion(cv::Point3f camPt, int region);
	float distanceFromRegion(cv::Point colorPt, int region);

	// get box rect bounding box in color space
	cv::Rect getBoxBoundingRectColor();


};

#endif