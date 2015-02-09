#include "obj_color_feature.h"
#include <opencv2/opencv.hpp>
#include "tinylog.hpp"
#include "tinydir.hpp"
#include "obj_detector.h"
#include "obj_tracker.h"
#include <thread>
#include <mutex>
#include <condition_variable>

std::mutex g_mutex;
std::condition_variable g_cv;
int g_count = 3;

void predictAsync(ObjDetector *obj, cv::Mat *Iraw, cv::Mat *Imsk){
	obj->predict(*Iraw, *Imsk, 0.2, 100);
	std::unique_lock<std::mutex> lck(g_mutex);
	g_count--;
	g_cv.notify_all();
}

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

	#if 0
	objDetector1.trainBatchAsync(trainImgs1, trainMsks1, 8);
	while(!objDetector1.isModelReady()){
		std::cout << "waiting...\n";
		sleep(1);
	}
	//objDetector1.save("./obj1/model");

	objDetector2.trainBatchAsync(trainImgs2, trainMsks2, 8);
	while(!objDetector2.isModelReady()){
		std::cout << "waiting...\n";
		sleep(1);
	}
	//objDetector2.save("./obj2/model");

	objDetector3.trainBatchAsync(trainImgs3, trainMsks3, 8);
	while(!objDetector3.isModelReady()){
		std::cout << "waiting...\n";
		sleep(1);
	}
	//objDetector3.save("./hand/model");
	#endif
	
	#if 1
	objDetector1.load("./obj1/model");
	objDetector2.load("./obj2/model");
	objDetector3.load("./hand/model");
	#endif

	cv::VideoCapture cap(0);
	if(!cap.isOpened()){
		std::cout << "Failed to open camera!" << std::endl;
		return -1;
	}
	cv::Mat rawImg;
	cv::Mat testMsk1;
	cv::Mat testMsk2;
	cv::Mat testMsk3;
	
	std::thread thr1;
	std::thread thr2;
	std::thread thr3;

	ObjTracker tracker1;
	ObjTracker tracker2;
	tracker2.numObjs = 2;
	ObjTracker tracker3;
	tracker3.numObjs = 2;
	tracker3.minArea = 500;

	while(1){
		if(!cap.read(rawImg)){
			std::cout << "Camera error!" << std::endl;
			break;
		}
		
		cv::resize(rawImg, rawImg, cv::Size(320, 240));
		
		double start = cv::getTickCount();

		thr1 = std::thread(predictAsync, &objDetector1, &rawImg, &testMsk1);
		thr2 = std::thread(predictAsync, &objDetector2, &rawImg, &testMsk2);
		thr3 = std::thread(predictAsync, &objDetector3, &rawImg, &testMsk3);

		std::unique_lock<std::mutex> lck(g_mutex);
		while(g_count){
			g_cv.wait(lck);
		}
		thr1.join();
		thr2.join();
		thr3.join();
		g_count = 3;

		tracker1.track(testMsk1);
		tracker2.track(testMsk2);
		tracker3.track(testMsk3);

		for(int i = 0; i < tracker1.objects.size(); i++){
			Object obj = tracker1.objects[i];
			cv::rectangle(rawImg, obj.bbox, cv::Scalar(0, 0, 255));
		}

		for(int i = 0; i < tracker2.objects.size(); i++){
			Object obj = tracker2.objects[i];
			cv::rectangle(rawImg, obj.bbox, cv::Scalar(0, 255, 0));
		}

		for(int i = 0; i < tracker3.objects.size(); i++){
			Object obj = tracker3.objects[i];
			cv::rectangle(rawImg, obj.bbox, cv::Scalar(0, 255, 255));
		}
		
		double seconds = (cv::getTickCount() - start)/cv::getTickFrequency();
		LOGF("test time: %f sec.", seconds);
#if 1
		cv::imshow("msk1", testMsk1);
		cv::imshow("msk2", testMsk2);
		cv::imshow("msk3", testMsk3);
#endif		
		cv::imshow("raw", rawImg);

		int key = cv::waitKey(30) & 0xFF;
		if(key == 27){
			break;
		}
	}

	return 0;
}