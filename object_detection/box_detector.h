#ifndef BOX_DETECTOR_H
#define BOX_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "matt.h"
#include "obj_detector.h"
#include "obj_tracker.h"

#define BOX_UNKNOWN_BIT          0
#define BOX_HOURGLASS_BIT        1
#define BOX_CYLINDER_BIT         2
#define BOX_KEYTOP_BIT           3
#define BOX_ROUNDTOP_BIT         4
#define BOX_SCREW_BIT            5

class BoxDetector{
private:
	ObjDetector *boxDetector;
	ObjTracker *boxTracker;
	int threshold;

public:
	BoxDetector();
	~BoxDetector();

	int load(std::string modelDir);
	unsigned char detect(cv::Mat &roiImg, cv::Mat &boxMsk, Kinect2 &kinect, Matt &mat);
};

#endif /* BOX_DETECTOR_H */