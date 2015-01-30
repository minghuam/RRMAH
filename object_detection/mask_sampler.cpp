#include "mask_generator.h"
#include <opencv2/opencv.hpp>

#define GOOD_PATH(x) if(x[x.size() - 1] == '/') \
						x = x.substr(0, x.size() - 1)

int main(int argc, char **argv){

	if(argc < 3){
		std::cout << "usage : " << argv[0] << " img_dir mask_dir" << std::endl;
		return 0;
	}

	std::string imgDir = argv[1];
	std::string mskDir = argv[2];

	GOOD_PATH(imgDir);
	GOOD_PATH(mskDir);

	cv::VideoCapture cap(0);
	if(!cap.isOpened()){
		std::cout << "Failed to open camera!" << std::endl;
		return -1;
	}

	MaskGenerator mg;

	cv::Mat rawImg;
	cv::Mat mskImg;
	
	char buf[64];
	int saveIndex = 0;

	while(1){
		if(!cap.read(rawImg)){
			std::cout << "Camera error!" << std::endl;
			break;
		}
		
		cv::imshow("raw", rawImg);
		
		if(!mg.is_ready()){
			mg.learn(rawImg, mskImg);
		}else{
			mg.update(rawImg, mskImg);
			cv::imshow("mask", mskImg);
		}

		int key = cv::waitKey(30) & 0xFF;
		if(key == 27){
			break;
		}
		switch(key){
			case 's':
				sprintf(buf, "%s/img_%04d.jpg", imgDir.c_str(), saveIndex);
				std::cout << buf << std::endl;
				cv::imwrite(buf, rawImg);
				sprintf(buf, "%s/msk_%04d.jpg", mskDir.c_str(), saveIndex);
				std::cout << buf << std::endl;
				cv::imwrite(buf, mskImg);
				saveIndex++;
				break;
			case 'b':
				mg.restart();
				break;
			default:
				break;
		}
	}

	if(cap.isOpened()){
		cap.release();
	}

	return 0;
}