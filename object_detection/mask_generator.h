#ifndef MASK_GENERATOR_H
#define MASK_GENERATOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

using namespace cv;

class MaskGenerator{
private:
	BackgroundSubtractorMOG2 MOG2;
	int erosion_size;
    int dilation_size;
    int learning_frame;
    int max_learning_frames;
    int skip_frame;
    int max_skipping_frames;

public:
    MaskGenerator();
    void restart();
    bool is_ready();
    bool learn(cv::Mat &I, cv::Mat &mask);
    void update(cv::Mat &src, cv::Mat &dst, bool learn_bg = false);
};


#endif
