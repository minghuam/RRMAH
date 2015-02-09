#ifndef OBJ_DETECTOR_H
#define OBJ_DETECTOR_H

#include "obj_color_feature.h"
#include "obj_global_feature.h"
#include <opencv2/opencv.hpp>
#include <atomic>
#include <thread>
#include <mutex>

class ObjDetector{
private:
	CvRTParams rt_params;
	
	std::mutex g_mutex;
	cv::Mat gdescriptor;
	std::vector<CvRTrees*> random_trees;
	cv::flann::Index *flannIndex;

	std::vector<std::thread> threads;
	std::atomic<int> th_counter;
	bool is_model_ready;

	void train(cv::Mat &Iraw, cv::Mat &Imsk);
	void trainAsync(std::vector<std::string> &images, \
		std::vector<std::string> &masks, int start, int end);

	void reset();

public:
	ObjDetector();
	~ObjDetector();

	int load(std::string dir);
	int save(std::string dir);
	int trainBatchAsync(std::vector<std::string> &images, \
		std::vector<std::string> &masks, int nthreads);
	bool isModelReady();
	int predict(cv::Mat &Iraw, cv::Mat &Imsk, float prob_threshold, float minArea);

};

#endif /* OBJ_DETECTOR_H */