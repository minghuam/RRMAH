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
#include <dirent.h>

/** @brief Get the extension of a file.
*	@param file Input file path.
*	@return File extension string.
*/
const char* get_extension(const char *file){
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
std::vector<std::string> list_dir(const char* dir, const char* ext = NULL){
	std::vector<std::string> files;
	if(dir == NULL){
		return files;
	}
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

	sort(files.begin(), files.end());

	return files;
}

#endif /* _TINYDIR_HPP */