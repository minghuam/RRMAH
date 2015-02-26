#include "stdafx.h"
#include "obj_detector.h"
#include "tinydir.hpp"
#include <sstream>
#include "dir_helper.h"

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


bool ObjDetector::isModelReady(){
	return is_model_ready;
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

	std::vector<std::string> files = DirHelper::listFiles(dir, "gdescriptor");
	if(files.size() != 1){
		return -1;
	}

	std::cout << "loading: " << files[0] << std::endl;
	cv::FileStorage fs;
	fs.open(files[0], cv::FileStorage::READ);
	fs["gdescriptor"] >> gdescriptor;
	fs.release();
	flannIndex = new cv::flann::Index(gdescriptor, cv::flann::KMeansIndexParams());
	
	files = DirHelper::listFiles(dir, "classifier");

	for(int i = 0; i < (int)files.size(); i++){
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
	
	std::string xml = DirHelper::combinePath(dir, "feat.gdescriptor");

	std::cout << "saving: " + xml << std::endl;
	fs.open(xml, cv::FileStorage::WRITE);
	fs << "gdescriptor" << gdescriptor;
	fs.release();

	char buf[64];
	for(int i = 0; i < (int)random_trees.size(); i++){
		sprintf(buf, "%04d.classifier", i);
		std::string path = DirHelper::combinePath(dir, std::string(buf));
		std::cout << path << std::endl;
		random_trees[i]->save(path.c_str());
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

int ObjDetector::trainAsync(std::vector<std::string> &images, \
	std::vector<std::string> &masks, int start, int end){

	ObjColorFeature color_feature;
	ObjGlobalFeature global_feature;
	cv::Mat feat, label, gfeat;
	
	std::string tempDir = "./temp";

	if(DirHelper::safeMakeDirectory(tempDir)){
		std::cerr << "failed to create temp directory" << std::endl;
		return -1;
	}

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

		std::string p = DirHelper::combinePath(tempDir, "random_forest." + std::to_string(i));
		rt0->save(p.c_str());
		delete rt0;

		CvRTrees* rt = new CvRTrees();
		rt->load(p.c_str());

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

	return 0;
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

	return 0;
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
	int k = 2;
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
			for(int i = 0; i < (int)indices.size(); i++){
				val += weight * random_trees[indices[i]]->predict(feat.row(sampleIndex));
				totalWeights += weight;
				weight *= 0.9f;
			}
			*(dst + col) = val/totalWeights;
			sampleIndex++;
		}
	}
	std::cout << "model: ";
	for(int i = 0; i < (int)indices.size(); i++){
		std::cout << indices[i] + 1 << " ";
	}
	std::cout << std::endl;

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