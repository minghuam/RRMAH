#include "obj_color_feature.h"
#include <opencv2/opencv.hpp>
#include "tinylog.hpp"
#include "tinydir.hpp"
#include "obj_detector.h"

int main(int argc, char **argv){

	ObjColorFeature objFeature;
	ObjDetector objDetector1;
	ObjDetector objDetector2;
	ObjDetector objDetector3;

	const char *trainImgDir1 = "./obj1/train/image";
	const char *trainMskDir1 = "./obj1/train/mask";
	const char *trainImgDir2 = "./obj2/train/image";
	const char *trainMskDir2 = "./obj2/train/mask";
	const char *trainImgDir3 = "./hand/train/image";
	const char *trainMskDir3 = "./hand/train/mask";

	std::vector<std::string> trainImgs1 = list_dir(trainImgDir1, ".jpg");
	std::vector<std::string> trainMsks1 = list_dir(trainMskDir1, ".jpg");
	assert(trainImgs1.size() == trainMsks1.size());

	std::vector<std::string> trainImgs2 = list_dir(trainImgDir2, ".jpg");
	std::vector<std::string> trainMsks2 = list_dir(trainMskDir2, ".jpg");
	assert(trainImgs2.size() == trainMsks2.size());

	std::vector<std::string> trainImgs3 = list_dir(trainImgDir3, ".jpg");
	std::vector<std::string> trainMsks3 = list_dir(trainMskDir3, ".jpg");
	assert(trainImgs3.size() == trainMsks3.size());

	for(int i = 0; i < trainImgs1.size(); i++){
		cv::Mat rawImg = cv::imread(trainImgs1[i]);
		cv::Mat mskImg = cv::imread(trainMsks1[i]);

		double start = cv::getTickCount();
		objDetector1.train(rawImg, mskImg);
		double seconds = (cv::getTickCount() - start)/cv::getTickFrequency();
		LOGF("training time: %f sec.", seconds);
	}

	for(int i = 0; i < trainImgs2.size(); i++){
		cv::Mat rawImg = cv::imread(trainImgs2[i]);
		cv::Mat mskImg = cv::imread(trainMsks2[i]);

		double start = cv::getTickCount();
		objDetector2.train(rawImg, mskImg);
		double seconds = (cv::getTickCount() - start)/cv::getTickFrequency();
		LOGF("training time: %f sec.", seconds);
	}

	for(int i = 0; i < trainImgs3.size(); i++){
		cv::Mat rawImg = cv::imread(trainImgs3[i]);
		cv::Mat mskImg = cv::imread(trainMsks3[i]);

		double start = cv::getTickCount();
		objDetector3.train(rawImg, mskImg);
		double seconds = (cv::getTickCount() - start)/cv::getTickFrequency();
		LOGF("training time: %f sec.", seconds);
	}

	cv::VideoCapture cap(0);
	if(!cap.isOpened()){
		std::cout << "Failed to open camera!" << std::endl;
		return -1;
	}
	cv::Mat rawImg;
	cv::Mat testMsk1;
	cv::Mat testMsk2;
	cv::Mat testMsk3;
	while(1){
		if(!cap.read(rawImg)){
			std::cout << "Camera error!" << std::endl;
			break;
		}
		
		cv::imshow("raw", rawImg);
		cv::resize(rawImg, rawImg, cv::Size(320, 240));
		
		double start = cv::getTickCount();
		objDetector1.predict(rawImg, testMsk1);
		objDetector2.predict(rawImg, testMsk2);
		objDetector3.predict(rawImg, testMsk3);
		double seconds = (cv::getTickCount() - start)/cv::getTickFrequency();
		LOGF("test time: %f sec.", seconds);

		cv::imshow("msk1", testMsk1);
		cv::imshow("msk2", testMsk2);
		cv::imshow("msk3", testMsk3);
		
		int key = cv::waitKey(30) & 0xFF;
		if(key == 27){
			break;
		}
	}

	return 0;
}