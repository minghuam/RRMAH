// GrabCut.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "grabcut.h"
#include <opencv2/opencv.hpp>
#include "Poco/File.h"
#include "Poco/Path.h"
#include <vector>
#include <iostream>

int _tmain(int argc, _TCHAR* argv[])
{
	if(argc < 3){
		std::string exe(argv[0]);
		std::cout << "usage: " << exe << ", image_dir, mask_dir" << std::endl;
		return 0;
	}

	std::cout << "press 'f' to switch to foreground mode" << std::endl;
	std::cout << "press 'b' to swtich to background mode" << std::endl;
	std::cout << "press 'd' to save and go to next image" << std::endl;
	std::cout << "press 'esc' to force quit" << std::endl;
	std::cout << "press 'r' to reset" << std::endl;
	std::cout << "press 's' to skip current image" << std::endl;


	Poco::File imgDir(argv[1]);
	Poco::Path imgRoot(argv[1]);
	Poco::Path mskRoot(argv[2]);
	if(!imgDir.isDirectory()){
		std::cout << "invalid input directory. " << std::endl;
		return 0;
	}

	// list images
	std::vector<std::string> images;
	imgDir.list(images);
	std::vector<std::string>::iterator it = images.begin();
	while(it != images.end()){
		Poco::Path p(imgRoot, *it);
		if(p.isFile() && p.getExtension() == "jpg"){
			*it = p.toString();
			++it;
			continue;
		}
		it = images.erase(it);
	}

	// run grab cut
	GrabCut gb;
	for(int i = 0; i < (int)images.size(); i++){
		std::cout << "loading " + images[i] << std::endl;
		cv::Mat img = cv::imread(images[i]);
		if(img.rows * img.cols == 0){
			std::cout << "failed to read image." << std::endl;
			return 0;
		}

		std::string savePath = Poco::Path(mskRoot, Poco::Path(images[i]).getFileName()).toString();
		cv::Mat msk;
		int ret = gb.run(img, msk);
		if(ret == 0){
			std::cout << "saving " + savePath << std::endl;
			cv::imwrite(savePath, msk);
		}else if(ret == 1){
			continue;
		}else{
			break;
		}
	}

	return 0;
}