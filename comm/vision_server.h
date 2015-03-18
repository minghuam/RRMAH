#pragma once
#include <queue>
#include <mutex>
#include <thread>
#include "Poco/Net/ServerSocket.h"

class VisionServer
{
private:
	bool isRunning;
	std::thread mainThread;
	std::mutex msgQLock;
	std::queue<std::string> msgQueue;

public:
	
	Poco::Net::ServerSocket *sockServer;
	Poco::Net::StreamSocket *sockClient;

	VisionServer();
	~VisionServer();

	int start(int port);
	void stop();
	std::string getMsg();
	void putMsg(std::string msg);
};

