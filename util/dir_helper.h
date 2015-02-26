#ifndef DIR_HELPER_H
#define DIR_HELPER_H

#include "Poco/File.h"
#include "Poco/Path.h"
#include "Poco/Exception.h"
#include <vector>

class DirHelper{
public:
	static std::vector<std::string> listFiles(std::string dir, std::string ext){
		std::vector<std::string> files;
		// check is directory
		Poco::File root(dir);
		if(!root.isDirectory()){
			return files;
		}

		// list files and filter with extension
		root.list(files);
		Poco::Path rootPath(dir);
		std::vector<std::string>::iterator it = files.begin();
		while(it != files.end()){
			Poco::Path p(rootPath, *it);
			if(p.isFile() && p.getExtension() == ext){
				*it = p.toString();
				++it;
				continue;
			}
			it = files.erase(it);
		}

		return files;
	}

	static int safeMakeDirectory(std::string dir){
		Poco::Path path(dir);
		Poco::File f(dir);
		if(f.exists()){
			return 0;
		}
		try{
			f.createDirectory();
		}catch(Poco::FileException &e){
			std::cout << e.displayText() << std::endl;
			return -1;
		}
		return 0;
	}

	static std::string combinePath(std::string parent, std::string child){
		return Poco::Path(parent, child).toString();
	}

	static bool exists(std::string path){
		return Poco::File(path).exists();
	}

};

#endif