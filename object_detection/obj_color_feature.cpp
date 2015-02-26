#include "stdafx.h"
#include "obj_color_feature.h"
#include <cassert>
#include <iostream>

ObjColorFeature::ObjColorFeature(){
	patchSize = 3;
	assert(patchSize % 2);
}

int ObjColorFeature::getPatchSize(){
	return patchSize;
}

void ObjColorFeature::computeLabel(const cv::Mat &bgr, cv::Mat &label){
	int nSamples = (bgr.rows - patchSize + 1) * (bgr.cols - patchSize + 1);
	label = cv::Mat::zeros(nSamples, 1, CV_32F);
	int maxOffset = patchSize/2;
	int sampleRow = 0;
	for(int row = maxOffset; row < bgr.rows - maxOffset; row++){
		for(int col = maxOffset; col < bgr.cols - maxOffset; col++){
			label.at<float>(sampleRow++, 0) = bgr.at<cv::Vec3b>(row, col)(0)/255.0f;
		}
	}
}

void ObjColorFeature::computeFeature(const cv::Mat &bgr, cv::Mat &feat){
	/** http://docs.opencv.org/doc/tutorials/core/how_to_scan_images/how_to_scan
	  * _images.html#performance-difference
	 */
	cv::Mat I;
	cv::cvtColor(bgr, I, CV_BGR2HSV_FULL);

	int nSamples = (I.rows - patchSize + 1) * (I.cols - patchSize + 1);
	feat = cv::Mat::zeros(nSamples, patchSize * patchSize * 3, CV_32F);

	int maxOffset = patchSize/2;
	int sampleRow = 0;
	for(int row = maxOffset; row < I.rows - maxOffset; row++){
		for(int col = maxOffset; col < I.cols - maxOffset; col++){
			float *dst = feat.ptr<float>(sampleRow);
			for(int prow = row - maxOffset; prow <= row + maxOffset; prow++){
				uchar *src = I.ptr<uchar>(prow) + ((col - maxOffset) * 3);
				int size = patchSize * 3;
				while(size){
					*dst = (*src)/255.0f;
					dst++;
					src++;
					size--;
				}
			}
			sampleRow++;
		}
	}
}