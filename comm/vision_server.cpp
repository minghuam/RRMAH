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
			std::cout << "parseMessages: " << e.displayText() << std::endl;
			break;
		}
		if(nbytes == MAX_MSG_LEN){
			std::cout << "parseMessages: " << "msg too long!" << std::endl;
			continue;
		}
		
		if(nbytes == 0){
			std::cout << "parseMessages: " << "client closed." << std::endl;
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

				visionServer->putInMsg(cmd);

				Poco::JSON::Object::Ptr jsonObj = new Poco::JSON::Object();
				jsonObj->set("ack", "ok");
				std::stringstream os;
				Poco::JSON::Stringifier::stringify(jsonObj, os);
				std::string s = os.str();
				visionServer->sockClient->sendBytes(s.c_str(), s.length());

			}catch(Poco::Exception &e){
				std::cout << "parseMessages: " << e.displayText() << std::endl;
				break;
			}
		}
	}
}


void sendMessages(VisionServer *visionServer){
	char buf[MAX_MSG_LEN + 1] = {0};
	Poco::JSON::Parser parser;

	std::string msg = "";
	while(1){
		msg = visionServer->getOutMsg();
		if(msg == ""){
			if(visionServer->clientClosed){
				break;
			}
			Sleep(30);
		}else{
			try{
				Poco::JSON::Object::Ptr jsonObj = new Poco::JSON::Object();
				// fix me!
				int score1 = 0;
				int score2 = 0;
				sscanf(msg.c_str(), "%d,%d", &score1, &score2);
				jsonObj->set("score1", score1);
				jsonObj->set("score2", score2);
				std::stringstream os;
				Poco::JSON::Stringifier::stringify(jsonObj, os);
				std::string s = os.str();
				visionServer->sockClient->sendBytes(s.c_str(), s.length());
			}catch(Poco::Exception &e){
				std::cout << "sendMessages: " << e.displayText() << std::endl;
				break;
			}
		}
	}
}

void acceptConnections(VisionServer *visionServer){
	std::thread thrRead;
	std::thread thrWrite;
	while(1){
		if(visionServer->sockServer == NULL){
			break;
		}
		try{
			Poco::Net::StreamSocket socket = visionServer->sockServer->acceptConnection();
			std::cout << "new client connected" << std::endl;
			visionServer->sockClient = &socket;
			visionServer->clientClosed = false;
			thrRead = std::thread(parseMessages, visionServer);
			thrWrite = std::thread(sendMessages, visionServer);
			thrRead.join();
			std::cout << "read thread joined" << std::endl;
			visionServer->clientClosed = true;
			thrWrite.join();
			std::cout << "write thread joined" << std::endl;
		}catch(Poco::Exception &e){
			std::cout << e.displayText() << std::endl;
			break;
		}
	}

	if(thrRead.joinable())
		thrRead.join();
	if(thrWrite.joinable())
		thrWrite.join();
}

VisionServer::VisionServer(){
	sockServer = NULL;
	sockClient = NULL;
	isRunning = false;
	clientClosed = true;
}

VisionServer::~VisionServer(){
	stop();	
}

int VisionServer::start(int port){
	if(isRunning){
		return -1;
	}
	isRunning = true;
	clientClosed = false;
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
	clientClosed = true;
}

std::string VisionServer::getInMsg(){
	std::string ret = "";
	msgQInLock.lock();
	if(msgQueueIn.size() > 0){
		ret = msgQueueIn.front();
		msgQueueIn.pop();
	}
	msgQInLock.unlock();
	return ret;
}

void VisionServer::putInMsg(std::string msg){
	msgQInLock.lock();
	if(msg.size() == MAX_NUM_MSGS){
		msgQueueIn.pop();
	}
	msgQueueIn.push(msg);
	msgQInLock.unlock();
}

std::string VisionServer::getOutMsg(){
	std::string ret = "";
	msgQOutLock.lock();
	if(msgQueueOut.size() > 0){
		ret = msgQueueOut.front();
		msgQueueOut.pop();
	}
	msgQOutLock.unlock();
	return ret;
}

void VisionServer::putOutMsg(std::string msg){
	msgQOutLock.lock();
	if(msg.size() == MAX_NUM_MSGS){
		msgQueueOut.pop();
	}
	msgQueueOut.push(msg);
	msgQOutLock.unlock();
}

void VisionServer::sendScore(int score1, int score2){
	char buf[32];
	sprintf(buf, "%d,%d", score1, score2);
	std::string msg(buf);
	putOutMsg(msg);
}