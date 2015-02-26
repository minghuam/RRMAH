/** @file tinydir.hpp
*	@brief Useful functions to handle directories.
*	@author minghuam
*	@todo	Robust extension parsing and multiple filters
*/

#ifndef _TINYDIR_HPP
#define _TINYDIR_HPP

/* -- Includes -- */
#include <string>
#include <vector>
#include <algorithm>

/*  for unix: dirent */
#ifdef _WIN32
#include <Windows.h>
#else
#include <dirent.h>
#endif

/** @brief Get the extension of a file.
*	@param file Input file path.
*	@return File extension string.
*/
static const char* get_extension(const char *file){
	int len = strlen(file);
	if(len == 0){
		return NULL;
	}

	int dot_pos = len - 1;
	while(dot_pos > -1){
		if(file[dot_pos] == '.'){
			break;
		}
		dot_pos--;
	}

	if(dot_pos == -1){
		return NULL;
	}

	return &file[dot_pos];
}

/** @brief List a given directory.
* 	@details List a given directory and return all files 
		including sub-directories. If extenstion is specified, 
		return files with that extension only.
*	@param dir Directory path to list.
*	@param ext File extension filter.
*	@return A list of files or sub-directories.
*/
static std::vector<std::string> list_dir(const char *dir, const char *ext = NULL){
	std::vector<std::string> files;
	if(!dir){
		return files;
	}

#ifdef _WIN32
	HANDLE hFind;
	WIN32_FIND_DATA data;
	std::string file(dir);
	if(ext){
		file = file + std::string(ext);
	}
	hFind = FindFirstFile(file.c_str(), &data);
	if(hFind != INVALID_HANDLE_VALUE){
		do{
			std::string fullPath = std::string(dir) + std::string(data.cFileName);
			files.push_back(fullPath);
		}while(FindNextFile(hFind, &data));
		FindClose(hFind);
	}

#else
	std::string s_dir = dir;
	if(s_dir[s_dir.size() - 1] != '/'){
		s_dir = s_dir + "/";
	}

	DIR *d = opendir(dir);
	struct dirent *entry;
	while((entry = readdir(d)) != NULL){
		if(ext && strcmp(ext, get_extension(entry->d_name))){
			continue;
		}
		files.push_back(s_dir + std::string(entry->d_name));
	}
#endif

	
	sort(files.begin(), files.end());

	return files;
}

#endif /* _TINYDIR_HPP */