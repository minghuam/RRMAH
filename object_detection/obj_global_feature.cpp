#include "stdafx.h"
#include "obj_global_feature.h"
#include <cassert>

ObjGlobalFeature::ObjGlobalFeature(){

}

void ObjGlobalFeature::computeFeature(const cv::Mat &bgr, cv::Mat &feat){

	int bins[] = {4,4,4};
	
	cv::Mat hsv;
    cv::cvtColor(bgr,hsv,CV_BGR2HSV_FULL);
    
	int histSize[] = {bins[0], bins[1], bins[2]};
    cv::Mat his;
    his.create(3, histSize, CV_32F);
    his = cv::Scalar(0);   
    assert(hsv.type() == CV_8UC3);
    cv::MatConstIterator_<cv::Vec3b> it = hsv.begin<cv::Vec3b>();
    cv::MatConstIterator_<cv::Vec3b> it_end = hsv.end<cv::Vec3b>();
    for( ; it != it_end; ++it )
    {
        const cv::Vec3b& pix = *it;
        his.at<float>(pix[0]*bins[0]/256, pix[1]*bins[1]/256,pix[2]*bins[2]/256) += 1.f;
    }
	
    // ==== Remove small values ==== //
    float minProb = 0.01f;
    minProb *= hsv.rows*hsv.cols;
    cv::Mat plane;
    const cv::Mat *_his = &his;
	
    cv::NAryMatIterator itt = cv::NAryMatIterator(&_his, &plane, 1);   
    cv::threshold(itt.planes[0], itt.planes[0], minProb, 0, cv::THRESH_TOZERO);
    double s = sum(itt.planes[0])[0];
	
    // ==== Normalize (L1) ==== //
    s = 1./s * 255.;
    itt.planes[0] *= s;
    itt.planes[0].copyTo(feat);
}