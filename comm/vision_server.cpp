#include "stdafx.h"
#include "vision_server.h"

#include "Poco/JSON/JSON.h"
#include "Poco/JSON/Parser.h"
#include "Poco/Path.h"
#include "Poco/File.h"
#include "Poco/FileStream.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/Net/ServerSocket.h"
#include "Poco/Net/StreamSocket.h"
#include "Poco/Net/SocketStream.h"
#include "Poco/Net/SocketAddress.h"

#include <iostream>
#include <sstream>
#include <vector>
#include <random>

#define MAX_NUM_MSGS 32
#define MAX_MSG_LEN 1024

void parseMessages(VisionServer *visionServer){
	char buf[MAX_MSG_LEN + 1] = {0};
	Poco::JSON::Parser parser;

	while(1){
		int nbytes = -1;
		try{
			nbytes = visionServer->sockClient->receiveBytes(buf, MAX_MSG_LEN);
		}catch(Poco::Exception &e){
			std::cout << e.displayText() << std::endl;
			break;
		}
		if(nbytes == MAX_MSG_LEN){
			std::cout << "msg too long!" << std::endl;
			continue;
		}
		
		if(nbytes == 0){
			std::cout << "client closed." << std::endl;
			break;
		}else if(nbytes > 0){
			buf[nbytes] = 0;
			std::string s(buf);

			parser.reset();
			try{
				parser.parse(s);
				Poco::Dynamic::Var ret = parser.result();
				Poco::JSON::Object::Ptr obj = ret.extract<Poco::JSON::Object::Ptr>();
				std::string cmd = obj->getValue<std::string>("type");

				visionServer->putMsg(cmd);

				Poco::JSON::Object::Ptr jsonObj = new Poco::JSON::Object();
				jsonObj->set("ack", "ok");
				std::stringstream os;
				Poco::JSON::Stringifier::stringify(jsonObj, os);
				std::string s = os.str();
				visionServer->sockClient->sendBytes(s.c_str(), s.length());

			}catch(Poco::Exception &e){
				std::cout << e.displayText() << std::endl;
				break;
			}
		}
	}
}

void acceptConnections(VisionServer *visionServer){
	std::thread thr;
	while(1){
		if(visionServer->sockServer == NULL){
			break;
		}
		try{
			Poco::Net::StreamSocket socket = visionServer->sockServer->acceptConnection();
			std::cout << "new client connected" << std::endl;
			visionServer->sockClient = &socket;
			thr = std::thread(parseMessages, visionServer);
			thr.join();
		}catch(Poco::Exception &e){
			std::cout << e.displayText() << std::endl;
			break;
		}
	}

	if(thr.joinable())
		thr.join();
}

VisionServer::VisionServer(){
	sockServer = NULL;
	sockClient = NULL;
	isRunning = false;
}

VisionServer::~VisionServer(){
	stop();	
}

int VisionServer::start(int port){
	if(isRunning){
		return -1;
	}
	isRunning = true;
	sockServer = new Poco::Net::ServerSocket(port);
	mainThread = std::thread(acceptConnections, this);
	return 0;
}

void VisionServer::stop(){
	if(isRunning){
		isRunning = false;
		if(sockClient != NULL){
			sockClient->close();
			sockClient = NULL;
		}
		if(sockServer != NULL){
			sockServer->close();
			sockServer = NULL;
		}
		mainThread.join();
	}
}

std::string VisionServer::getMsg(){
	std::string ret = "";
	msgQLock.lock();
	if(msgQueue.size() > 0){
		ret = msgQueue.front();
		msgQueue.pop();
	}
	msgQLock.unlock();
	return ret;
}

void VisionServer::putMsg(std::string msg){
	msgQLock.lock();
	if(msg.size() == MAX_NUM_MSGS){
		msgQueue.pop();
	}
	msgQueue.push(msg);
	msgQLock.unlock();
}