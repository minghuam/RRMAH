#include "stdafx.h"
#include "mask_generator.h"
#include <iostream>

MaskGenerator::MaskGenerator(){
    erosion_size = 5;
    dilation_size = 3;
    max_learning_frames = 300;
    learning_frame = 0;
    skip_frame = 0;
    max_skipping_frames = 1;
}

void MaskGenerator::restart(){
    learning_frame = 0;
    skip_frame = 0;
    MOG2 = BackgroundSubtractorMOG2();
}

/** @brief learn background model and return true when done
 */
bool MaskGenerator::learn(cv::Mat &I, cv::Mat &mask){
    
    if(learning_frame >= max_learning_frames){
        return true;
    }
    
    // sampling every max_skipping_frames
    skip_frame++;
    if(skip_frame < max_skipping_frames){
        return false;
    }else{
        skip_frame = 0;
    }
    
    // learn background model
    MOG2(I, mask, -1);
    learning_frame++;
    
    return learning_frame >= max_learning_frames;
}

bool MaskGenerator::is_ready(){
    return learning_frame >= max_learning_frames;
}

/** @brief generate hand mask
 */
void MaskGenerator::update(cv::Mat &src, cv::Mat &dst, bool learn_bg){

    // background subtraction
    Mat fg_mask;
    Mat bgr;
    GaussianBlur(src, bgr, cv::Size(7,7), 1,1);
    MOG2(bgr, fg_mask, learn_bg ? -1 : 0);
    
    // threshold
    fg_mask = fg_mask > 240;
    
    // smooth
    medianBlur(fg_mask, fg_mask, 5);
    
    // erosion
	Mat element = getStructuringElement( MORPH_ELLIPSE,
                                        cv::Size( 2*erosion_size+1, 2*erosion_size+1 ),
                                        cv::Point( erosion_size, erosion_size ) );
	erode( fg_mask, fg_mask, element );
    
    element = getStructuringElement( MORPH_ELLIPSE,
                                    cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                    cv::Point( dilation_size, dilation_size ) );
    
    dilate(fg_mask, fg_mask, element);
    
    
    //more smoothing
	GaussianBlur(fg_mask, fg_mask, cv::Size(3,3), 5);
	fg_mask = fg_mask > 240;

    // copy
    fg_mask.copyTo(dst);
}