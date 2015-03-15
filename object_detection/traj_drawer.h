#ifndef TRAJ_DRAWER_H
#define TRAJ_DRAWER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

template <typename T>
class TrajDrawer{
private:
	std::vector<T> points;
	int length;
	int dim;
	cv::Mat trajImg;
	
public:
	TrajDrawer(){
		length = 100;
		dim = 3;
		trajImg = cv::Mat(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));
	}

	TrajDrawer(int length, int dim){
		this->length = length;
		this->dim = dim;
		trajImg = cv::Mat(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));
	}

	void add(T pt){
		points.insert(points.begin(), pt);
		if(points.size() > length){
			points.pop_back();
		}
	}

	void clear(){
		points.clear();
	}

	void show(std::string title, float scale){
		int cx = 0;
		float stepX = (float)trajImg.cols/(float)length;
		int plotHeight = trajImg.rows/dim;
		int cy = plotHeight/2;

		// clear
		trajImg = cv::Mat(trajImg.rows, trajImg.cols, CV_8UC3, cv::Scalar(0, 0, 0));
		char text[32];
		
		// x
		if(dim < 1) {
			cv::imshow(title, trajImg);
			return;
		}
		cv::line(trajImg, cv::Point(0, cy), cv::Point(trajImg.cols, cy), cv::Scalar(255, 255, 255));
		std::cout << points.size() << std::endl;
		for(int i = 0; i < (int)points.size() - 1; i++){
			cv::Point pt1((int)(stepX * i), (int)(cy - points[i].x * scale));
			cv::Point pt2((int)(stepX * (i + 1)), (int)(cy - points[i+1].x * scale));
			cv::line(trajImg, pt1, pt2, cv::Scalar(0, 0, 255));
		}
		if(points.size() > 0){
			sprintf(text, "%f", points[0].x);
			cv::putText(trajImg, text, cv::Point(10, cy - plotHeight/3), CV_FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 255), 1);
		}

		// y
		if(dim < 2) {
			cv::imshow(title, trajImg);
			return;
		}
		cy += plotHeight;
		cv::line(trajImg, cv::Point(0, cy), cv::Point(trajImg.cols, cy), cv::Scalar(255, 255, 255));
		for(int i = 0; i < (int)points.size() - 1; i++){
			cv::Point pt1((int)(stepX * i), (int)(cy - points[i].y * scale));
			cv::Point pt2((int)(stepX * (i + 1)), (int)(cy - points[i+1].y * scale));
			cv::line(trajImg, pt1, pt2, cv::Scalar(0, 255, 0));
		}
		if(points.size() > 0){
			sprintf(text, "%f", points[0].y);
			cv::putText(trajImg, text, cv::Point(10, cy - plotHeight/3), CV_FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 0), 1);
		}
		// z
		if(dim < 3) {
			cv::imshow(title, trajImg);
			return;
		}
		cy += plotHeight;
		cv::line(trajImg, cv::Point(0, cy), cv::Point(trajImg.cols, cy), cv::Scalar(255, 255, 255));
		for(int i = 0; i < (int)points.size() - 1; i++){
			cv::Point pt1((int)(stepX * i), (int)(cy - points[i].z * scale));
			cv::Point pt2((int)(stepX * (i + 1)), (int)(cy - points[i+1].z * scale));
			cv::line(trajImg, pt1, pt2, cv::Scalar(255, 0, 0));
		}
		if(points.size() > 0){
			sprintf(text, "%f", points[0].z);
			cv::putText(trajImg, text, cv::Point(10, cy - plotHeight/3), CV_FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255, 0, 0), 1);
		}
		cv::imshow(title, trajImg);
	}
};

#endif /* TRAJ_DRAWER_H */