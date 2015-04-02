#include "stdafx.h"
#include "obj_color_feature.h"
#include <opencv2/opencv.hpp>
#include "obj_detector.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "dir_helper.h"


int _tmain(int argc, _TCHAR* argv[])
{
	if(argc < 2){
		std::cout << "invalid arguments: dataDir" << std::endl;
		return 0;
	}
	
	// parse arguments
	std::string rootDir = argv[1];
	std::string imageFolder = DirHelper::combinePath(rootDir, "image");
	std::string maskFolder = DirHelper::combinePath(rootDir, "mask");
	std::string modelFolder = DirHelper::combinePath(rootDir, "model");

	if(!DirHelper::exists(imageFolder)){
		std::cout << imageFolder << " does not exists!" << std::endl;
		return -1;
	}

	if(!DirHelper::exists(maskFolder)){
		std::cout << maskFolder << " does not exists!" << std::endl;
		return -1;
	}

	if(!DirHelper::exists(modelFolder)){
		std::cout << modelFolder << " does not exists!" << std::endl;
		return -1;
	}

	cv::setUseOptimized( true );
	ObjColorFeature objFeature;
	ObjDetector objDetector;

	std::vector<std::string> images = DirHelper::listFiles(imageFolder, "jpg");
	std::vector<std::string> masks = DirHelper::listFiles(maskFolder, "jpg");
	assert(images.size() == masks.size());

	// train
	std::cout << "training...";

	if(objDetector.trainBatchAsync(images, masks, 4)){
		std::cerr << "failed to train in batch!" << std::endl;
		return -1;
	}
	while(!objDetector.isModelReady()){
		std::cout << ".";
		Sleep(1000);
	}

	objDetector.save(modelFolder);

	return 0;
}

