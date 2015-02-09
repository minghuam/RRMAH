#include "obj_detector.h"
#include "tinydir.hpp"
#include <sstream>

ObjDetector::ObjDetector(){
	rt_params.max_depth = 10;
	rt_params.regression_accuracy = 0.00f;
	rt_params.min_sample_count = 10;
	flannIndex = NULL;
	th_counter = 0;
	is_model_ready = false;
}

ObjDetector::~ObjDetector(){
	reset();
}

void ObjDetector::reset(){
	for(auto &rt : random_trees){
		delete rt;
	}
	random_trees.clear();

	if(flannIndex){
		delete flannIndex;
		flannIndex = NULL;
	}

	for(auto &th : threads){
		th.join();
	}
	threads.clear();

	gdescriptor = cv::Mat();
}

int ObjDetector::load(std::string dir){
	reset();

	std::vector<std::string> files = list_dir(dir.c_str(), ".gdescriptor");
	if(files.size() != 1){
		return -1;
	}

	std::cout << "loading: " << files[0] << std::endl;
	cv::FileStorage fs;
	fs.open(files[0], cv::FileStorage::READ);
	fs["gdescriptor"] >> gdescriptor;
	fs.release();
	flannIndex = new cv::flann::Index(gdescriptor, cv::flann::KMeansIndexParams());

	files = list_dir(dir.c_str(), ".classifier");
	for(int i = 0; i < files.size(); i++){
		std::cout << "loading: " << files[i] << std::endl;
		CvRTrees *rt = new CvRTrees();
		rt->load(files[i].c_str());
		random_trees.push_back(rt);
	}

	is_model_ready = true;

	return 0;
}

int ObjDetector::save(std::string dir){
	cv::FileStorage fs;
	std::string xml = dir + "/feat.gdescriptor";
	std::cout << "saving: " + xml << std::endl;
	fs.open(xml, cv::FileStorage::WRITE);
	fs << "gdescriptor" << gdescriptor;
	fs.release();

	char buf[64];
	for(int i = 0; i < random_trees.size(); i++){
		sprintf(buf, "%s/%04d.classifier", dir.c_str(), i);
		std::cout << buf << std::endl;
		random_trees[i]->save(buf);
	}

	return 0;
}

void ObjDetector::train(cv::Mat &Iraw, cv::Mat &Imsk){
	cv::Mat feat, label, gfeat;

	ObjColorFeature color_feature;
	ObjGlobalFeature global_feature;

	color_feature.computeFeature(Iraw, feat);
	color_feature.computeLabel(Imsk, label);

	global_feature.computeFeature(Iraw, gfeat);
	gdescriptor.push_back(gfeat);

	// build flann search model
	if(flannIndex){
		delete flannIndex;
	}
	flannIndex = new cv::flann::Index(gdescriptor, cv::flann::KMeansIndexParams());

	cv::Mat varType = cv::Mat::ones(feat.cols + 1, 1, CV_8UC1) * CV_VAR_NUMERICAL;
	CvRTrees *rt = new CvRTrees();
	rt->train(feat, CV_ROW_SAMPLE, label, \
		cv::Mat(), cv::Mat(), varType, cv::Mat(), rt_params);
	random_trees.push_back(rt);
}

void ObjDetector::trainAsync(std::vector<std::string> &images, \
	std::vector<std::string> &masks, int start, int end){

	ObjColorFeature color_feature;
	ObjGlobalFeature global_feature;
	cv::Mat feat, label, gfeat;

	std::stringstream ss;
	ss << "start: " << start << ", end: " << end << "\n";
	std::cout << ss.str();

	char tmp[64];
	for(int i = start; i < end; i++){
		cv::Mat Iraw = cv::imread(images[i]);
		cv::Mat Imsk = cv::imread(masks[i]);
		color_feature.computeFeature(Iraw, feat);
		color_feature.computeLabel(Imsk, label);

		global_feature.computeFeature(Iraw, gfeat);

		cv::Mat varType = cv::Mat::ones(feat.cols + 1, 1, CV_8UC1) * CV_VAR_NUMERICAL;
		
		CvRTrees* rt0 = new CvRTrees();
		rt0->train(feat, CV_ROW_SAMPLE, label, \
			cv::Mat(), cv::Mat(), varType, cv::Mat(), rt_params);

		sprintf(tmp, "/tmp/random_forest.%d", i);
		rt0->save(tmp);
		delete rt0;

		CvRTrees* rt = new CvRTrees();
		rt->load(tmp);

		g_mutex.lock();
		gdescriptor.push_back(gfeat);
		random_trees.push_back(rt);
		g_mutex.unlock();
	}

	th_counter -= 1;

	if(th_counter == 0){
		std::cout << "building search model...\n";
		// build flann search model
		if(flannIndex){
			delete flannIndex;
		}
		flannIndex = new cv::flann::Index(gdescriptor, cv::flann::KMeansIndexParams());
		is_model_ready = true;
	}
}

bool ObjDetector::isModelReady(){
	return is_model_ready;
}

int ObjDetector::trainBatchAsync(std::vector<std::string> &images, \
	std::vector<std::string> &masks, int nthreads){
	if(images.size() != masks.size() || nthreads <= 0 || th_counter > 0){
		return -1;
	}

	int batchSize = images.size()/nthreads;
	if(batchSize == 0){
		batchSize = images.size();
		nthreads = 1;
	}

	reset();
	th_counter = nthreads;

	for(int i = 0; i < nthreads; i++){
		int start = i * batchSize;
		int end = (i + 1) * batchSize;
		if( i == nthreads - 1){
			end = images.size();
		}
		threads.push_back(std::thread(\
			&ObjDetector::trainAsync, this, images, masks, start, end));
	}
}

int ObjDetector::predict(cv::Mat &Iraw, cv::Mat &Imsk, float prob_threshold, float minArea){

	if(!is_model_ready){
		return -1;
	}

	// calculate local and global features
	cv::Mat feat, gfeat;
	ObjColorFeature color_feature;
	ObjGlobalFeature global_feature;

	color_feature.computeFeature(Iraw, feat);
	global_feature.computeFeature(Iraw, gfeat);

	// search for closest models
	int k = 1;
	std::vector<int> indices;
	std::vector<float> dists;
	flannIndex->knnSearch(gfeat, indices, dists, k);

	// predict with these models and average results
	Imsk = cv::Mat::zeros(Iraw.rows, Iraw.cols, CV_32F);
	int patchSize = color_feature.getPatchSize();
	int maxOffset = patchSize/2;
	float weight = 1.0f;
	float totalWeights = 0.0f;
	int sampleIndex = 0;
	for(int row = maxOffset; row < Imsk.rows - maxOffset; row++){
		float *dst = Imsk.ptr<float>(row);
		for(int col = maxOffset; col < Imsk.cols - maxOffset; col++){
			float val = 0.0f;
			float weight = 1.0f;
			float totalWeights = 0.0f;
			for(int i = 0; i < indices.size(); i++){
				val += weight * random_trees[indices[i]]->predict(feat.row(sampleIndex));
				totalWeights += weight;
				weight *= 0.9f;
			}
			*(dst + col) = val/totalWeights;
			sampleIndex++;
		}
	}

	cv::GaussianBlur(Imsk, Imsk, cv::Size(13, 13), 0, 0, cv::BORDER_REFLECT);
	
	#if 0
	// probability threshold
	Imsk = Imsk > prob_threshold;

	// size threshold
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(Imsk, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	Imsk *= 0;

	for(int i = 0; i < contours.size(); i++){
		float area = cv::contourArea(cv::Mat(contours[i]));
		if(area < minArea){
			continue;
		}
		cv::drawContours(Imsk, contours, i, CV_RGB(255, 255, 255), CV_FILLED, CV_AA);
	}
	#endif
	
	return 0;
}