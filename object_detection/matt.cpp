#include "stdafx.h"
#include "matt.h"

Matt::Matt(){
	corners = std::vector<cv::Point>(NUM_CTRL_PTS, cv::Point());
	regions = std::vector<cv::Rect_<float> >(NUM_REGIONS, cv::Rect_<float>());
}

int Matt::load(std::string config){
	cv::FileStorage fs;
	if(fs.open(config, cv::FileStorage::READ)){
		fs["startRect"] >> regions[0];
		fs["leftRect"] >> regions[1];
		fs["rightRect"] >> regions[2];
		fs["bodyMiddleRect"] >> regions[3];
		fs["ipsilateralRect"] >> regions[4];
		fs["corner1"] >> corners[0];
		fs["corner2"] >> corners[1];
		fs["corner3"] >> corners[2];
		fs["corner4"] >> corners[3];
		fs["corner5"] >> corners[4];
		fs["corner6"] >> corners[5];
		fs["corner7"] >> corners[6];
		fs["corner8"] >> corners[7];
		fs["cam2Matt"] >> cam2Matt;
		fs["matt2Color"] >> matt2Color;
		fs["color2Matt"] >> color2Matt;
		fs.release();
	}

	std::cout << "start region: " << regions[0] << std::endl;
	std::cout << "left region: " << regions[1] << std::endl;
	std::cout << "right region: " << regions[2] << std::endl;
	std::cout << "bodyMiddleRect region: " << regions[3] << std::endl;
	std::cout << "ipsilateralRect region: " << regions[4] << std::endl;

	// convert mat space regions to color space polygons
	for(int i = 0; i < regions.size(); i++){
		std::vector<cv::Point2f> points;
		cv::Rect_<float> rect = regions[i];
		points.push_back(mattToColor(cv::Point2f(rect.x, rect.y)));
		points.push_back(mattToColor(cv::Point2f(rect.x + rect.width, rect.y)));
		points.push_back(mattToColor(cv::Point2f(rect.x + rect.width, rect.y + rect.height)));
		points.push_back(mattToColor(cv::Point2f(rect.x, rect.y + rect.height)));
		colorRegions.push_back(points);
	}

	// calculate region centers in camera space


	return 0;
}

void Matt::save(std::string config){
	cv::FileStorage fs;
	if(fs.open(config, cv::FileStorage::WRITE)){
		fs << "startRect" << regions[0];
		fs << "leftRect" << regions[1];
		fs << "rightRect" << regions[2];
		fs << "bodyMiddleRect" << regions[3];
		fs << "ipsilateralRect" << regions[4];
		fs << "corner1" << corners[0];
		fs << "corner2" << corners[1];
		fs << "corner3" << corners[2];
		fs << "corner4" << corners[3];
		fs << "corner5" << corners[4];
		fs << "corner6" << corners[5];
		fs << "corner7" << corners[6];
		fs << "corner8" << corners[7];
		fs << "cam2Matt" << cam2Matt;
		fs << "matt2Color" << matt2Color;
		fs << "color2Matt" << color2Matt;
		fs.release();
	}
}

cv::Point3f Matt::cameraToMatt(cv::Point3f camPt){
	cv::Mat from(1, 1, CV_32FC3);
	cv::Mat to;
	from.ptr<cv::Point3f>()[0] = camPt;
	cv::transform(from, to, cam2Matt);
	return cv::Point3f(to.ptr<cv::Point3f>()[0]);
}

cv::Point2f Matt::mattToColor(cv::Point2f matPt){
	cv::Mat from(1, 1, CV_32FC2);
	cv::Mat to;
	from.ptr<cv::Point2f>()[0] = matPt;
	cv::perspectiveTransform(from, to, matt2Color);
	return cv::Point2f(to.ptr<cv::Point2f>()[0]);
}

cv::Point2f Matt::colorToMatt(cv::Point colorPt){
	cv::Mat from(1, 1, CV_32FC2);
	cv::Mat to;
	from.ptr<cv::Point2f>()[0] = colorPt;
	cv::perspectiveTransform(from, to, color2Matt);
	return cv::Point2f(to.ptr<cv::Point2f>()[0]);
}

int Matt::hitTestRegion(cv::Point3f camPt, int region){
	if(region < 0){
		return 0;
	}
	cv::Point3f matPt = cameraToMatt(camPt);
	cv::Rect_<float> rect = regions[region];
	if(matPt.x >= rect.x && matPt.x <= rect.x + rect.width && matPt.y >= rect.y && matPt.y <= rect.y + rect.height){
		return 1;
	}else{
		return 0;
	}
}

int Matt::hitTestRegion(cv::Point colorPt, int region){
	if(region < 0){
		return 0;
	}
	if(cv::pointPolygonTest(colorRegions[region], colorPt, false) > 0){
		return 1;
	}else{
		return 0;
	}
}

float Matt::distanceFromRegion(cv::Point3f camPt, int region){
	if(region < 0){
		return 0;
	}
	// fixme: z term is always zero
	cv::Point3f matPt = cameraToMatt(camPt);
	cv::Rect_<float> rect = regions[region];
	cv::Point3f center = cv::Point3f(rect.x + rect.width/2.0f, rect.y + rect.height/2.0f, 0.0f);
	return sqrt((center.x - matPt.x)*(center.x - matPt.x) + (center.y - matPt.y)*(center.y - matPt.y));
}

float Matt::distanceFromRegion(cv::Point colorPt, int region){
	if(region < 0){
		return 0;
	}
	cv::Point2f matPt = colorToMatt(colorPt);
	cv::Rect_<float> rect = regions[region];
	cv::Point2f center = cv::Point2f(rect.x + rect.width/2.0f, rect.y + rect.height/2.0f);
	return sqrt((center.x - matPt.x)*(center.x - matPt.x) + (center.y - matPt.y)*(center.y - matPt.y));
}