#include "obj_detector.h"

ObjDetector::ObjDetector(){
	rt_params.max_depth = 10;
	rt_params.regression_accuracy = 0.00f;
	rt_params.min_sample_count = 10;

	flannIndex = NULL;
}

ObjDetector::~ObjDetector(){
	for(int i = 0; i < random_trees.size(); i++){
		CvRTrees *rt = random_trees[i];
		delete rt;
	}

	if(flannIndex){
		delete flannIndex;
	}
}

int ObjDetector::load(const char *dir){
	return 0;
}

int ObjDetector::save(const char *dir){
	return 0;
}

void ObjDetector::train(cv::Mat &Iraw, cv::Mat &Imsk){
	cv::Mat feat, label, gfeat;
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
	rt->train(feat, CV_ROW_SAMPLE, label, cv::Mat(), cv::Mat(), varType, cv::Mat(), rt_params);
	random_trees.push_back(rt);

}

void ObjDetector::trainBatch(std::vector<std::string> &imgDir, std::vector<std::string> &mskDir){

}

void ObjDetector::predict(cv::Mat &Iraw, cv::Mat &Imsk){
	// calculate local and global features
	cv::Mat feat, gfeat;
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

	cv::GaussianBlur(Imsk, Imsk, cv::Size(15, 15), 0, 0, cv::BORDER_REFLECT);
}
