#include "stdafx.h"
#include "mask_generator.h"
#include <opencv2/opencv.hpp>
#include "kinect2.h"
#include "dir_helper.h"

const int imgWidth = 1920;
const int imgHeight = 1080;
cv::Point mouseDownPt;
cv::Rect imageROI;

int clamp(int x, int low, int high){
	if(x < low) return low;
	if(x > high) return high;
	return x;
}

void onMouse(int mouseEvent, int x, int y, int flags, void *param){
	int left, right, top, bottom;
	switch (mouseEvent)
	{
	case CV_EVENT_LBUTTONDOWN:
		mouseDownPt = cv::Point(x, y);
		break;
	case CV_EVENT_LBUTTONUP:
		left = clamp(min(x, mouseDownPt.x), 0, imgWidth);
		top = clamp(min(y, mouseDownPt.y), 0, imgHeight);
		right = clamp(max(x, mouseDownPt.x), 0, imgWidth);
		bottom = clamp(max(y, mouseDownPt.y), 0, imgHeight);
		if(right > left && bottom > top){
			imageROI = cv::Rect(left, top, right - left, bottom - top);
		}
		break;
	default:
		break;
	}
}

int main(int argc, char **argv){

	if(argc < 4){
		std::cout << "usage : " << argv[0] << " img_dir mask_dir depth_dir" << std::endl;
		return 0;
	}

	std::string imgDir = argv[1];
	std::string mskDir = argv[2];
	std::string depthDir = argv[3];
	
	MaskGenerator mg;

	cv::Mat rawImg;
	cv::Mat roiImg;
	cv::Mat mskImg;
	cv::Mat depthImg;

	Kinect2 kinect;
	if(kinect.open()){
		std::cout << "Failed to open kinect." << std::endl;
	}

	std::cout << "press 'f' to toggle fullscreen" << std::endl;
	std::cout << "press 's' to save roi.xml" << std::endl;
	std::cout << "press 'x' to save image" << std::endl;
	std::cout << "press 'b' to reset background model" << std::endl;
	
	char buf[64];
	std::string savePath;
	int saveIndex = 0;

	cv::namedWindow("raw", 0);
	cv::setMouseCallback("raw", onMouse);

	cv::FileStorage fs;
	fs.open("roi.xml", cv::FileStorage::READ);
	fs["roi"] >> imageROI;
	fs.release();
	if(imageROI.width == 0){
		imageROI = cv::Rect(0, 0, imgWidth, imgHeight);
	}

	while(1){ 
		kinect.update();
		if(kinect.getIsColorFrameNew()){
			rawImg = cv::Mat(kinect.imageHeight, kinect.imageWidth, CV_8UC4, kinect.getImageData());
			cv::resize(rawImg, rawImg, cv::Size(imgWidth, imgHeight));			
			
			cv::rectangle(rawImg, imageROI, cv::Scalar(0, 0, 255));
			roiImg = rawImg(imageROI);

			cv::imshow("roi", roiImg);
			cv::imshow("raw", rawImg);
		
			if(!mg.is_ready()){
				mg.learn(roiImg, mskImg);
			}else{
				mg.update(roiImg, mskImg);
				cv::imshow("mask", mskImg);
			}
		}
		if(kinect.getIsDepthFrameNew()){
			
		}

		int key = cv::waitKey(30) & 0xFF;
		if(key == 27){
			break;
		}
		
		switch(key){
		case'f':
			cv::setWindowProperty("raw", CV_WND_PROP_FULLSCREEN, 1 - cv::getWindowProperty("raw", CV_WND_PROP_FULLSCREEN));
			break;
		case 's':
			fs.open("roi.xml", cv::FileStorage::WRITE);
			fs << "roi" << imageROI;
			fs.release();
			break;
		case 'x':
			sprintf(buf, "img_%04d.jpg", saveIndex);
			savePath = DirHelper::combinePath(imgDir, std::string(buf));
			std::cout << savePath << std::endl;
			cv::imwrite(savePath, roiImg);
			sprintf(buf, "msk_%04d.jpg", saveIndex);
			savePath = DirHelper::combinePath(mskDir, std::string(buf));
			std::cout << savePath << std::endl;
			cv::imwrite(savePath, mskImg);
			saveIndex++;
			break;
		case 'b':
			mg.restart();
			break;
		default:
			break;
		}
	}

	return 0;
}