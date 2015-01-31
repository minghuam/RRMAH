/** @file tinylog.hpp
*	@brief Simple log functions
*	@author minghuam
*/

#ifndef __TINYLOG_HPP
#define __TINYLOG_HPP

/* -- Includes -- */
#include <iostream>
#include <fstream>
#include <string>
#include <cstdarg>

/**	@brief Log a message string.
*	@param msg Message string to be logged.
*	@param stream Stream to write into.
*/
static void LOG(const std::string &msg, std::ostream &stream = std::cout){
	stream << msg << std::endl;
}

/** @brief Log into stdout with variable length messages.
*	@param format Message in string format.
*/
static void LOGF(const char* format, ...){
	char msg[256];
	va_list args;
	va_start(args, format);
	vsprintf(msg,format, args);
	std::cout << msg << std::endl;
	va_end (args);
}

/** @brief Log into stream with variable length messages.
*	@param stream Stream to write into.
*	@param format Message in string format.
*/
static void LOGSF(std::ostream &stream, const char* format, ...){
	char msg[256];
	va_list args;
	va_start(args, format);
	vsprintf(msg,format, args);
	stream << msg << std::endl;
	va_end (args);
}

#endif /* __TINYLOG_HPP */