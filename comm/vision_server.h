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
	std::mutex msgQInLock;
	std::mutex msgQOutLock;
	std::queue<std::string> msgQueueIn;
	std::queue<std::string> msgQueueOut;

public:
	
	Poco::Net::ServerSocket *sockServer;
	Poco::Net::StreamSocket *sockClient;

	VisionServer();
	~VisionServer();
	
	bool clientClosed;
	int start(int port);
	void stop();
	std::string getInMsg();
	void putInMsg(std::string msg);

	std::string getOutMsg();
	void putOutMsg(std::string msg);

	void sendScore(int score1, int score2);
};

