#include "stdafx.h"

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
#include <thread>

//using namespace Poco::JSON;

Poco::JSON::Object::Ptr createRandomObject(int id, std::string type){
	Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();
	obj->set("id", id);
	obj->set("type", type);
	obj->set("x", rand() % 100 / 100.0);
	obj->set("y", rand() % 100 / 100.0);
	obj->set("z", rand() % 100 / 100.0);
	return obj;
}

std::string constructEvent(){
	Poco::JSON::Object::Ptr evt = new Poco::JSON::Object();
	Poco::JSON::Array::Ptr arr = new Poco::JSON::Array();

	static int id = 1;
	for(int i = 0; i < 2; i++){
		arr->set(i, createRandomObject(id++, "hand"));
	}
	evt->set("objects", arr);
	evt->set("type", "event");
	evt->set("testID", 12345);

	std::stringstream os;
	Poco::JSON::Stringifier::stringify(evt, os);
	return os.str();
}

bool isRunning = true;
bool isStreaming = false;
void run(Poco::Net::StreamSocket *socket){
	std::cout << "starting new thread..." << std::endl;
	while(isRunning){
		if(isStreaming){
			std::string s = constructEvent();
			try{
				std::cout << "sending..." << std::endl;
				socket->sendBytes(s.c_str(), s.length());
			}catch(Poco::Exception &e){
				std::cerr << e.displayText() << std::endl;
				isRunning = false;
			}
		}
		Sleep(50);
	}
}

int _tmain(int argc, _TCHAR* argv[])
{

#if 0
	Poco::Path filePath("./test.json");
	if(filePath.isFile()){
		Poco::File f(filePath);
		if(!f.exists()){
			std::cout << "File does not exists!" << std::endl;
			return 0;
		}
	}
	
	Parser parser;
	Poco::FileInputStream in(filePath.toString());
	parser.parse(in);
	Poco::Dynamic::Var ret = parser.result();
	Object::Ptr obj = ret.extract<Object::Ptr>();
	std::vector<std::string> names;
	obj->getNames(names);
	Object::Ptr widget = obj->getObject("widget");
	Object::Ptr window = widget->getObject("window");
	std::cout << window->getValue<std::string>("title") << std::endl;
#endif

	if(argc < 2){
		std::cout << "invalid arguments: port" << std::endl;
		return -1;
	}

	int port = -1;
	try{
		port = std::stoi(argv[1]);
	}catch(int){
		std::cout << "invalid arguments: port" << std::endl;
		return -1;
	}

	// bind and listen
	Poco::Net::ServerSocket server(port);
	Poco::JSON::Parser parser;

	while(1){
		Poco::Net::StreamSocket socket = server.acceptConnection();
		
		isStreaming = false;
		isRunning = true;
		std::thread thr(run, &socket);
	
		bool isOpen = true;
		const int NUM_BYTES = 1024;
		char buf[NUM_BYTES + 1] = {0};

		while(isOpen){
			int nbytes = -1;
			try{
				nbytes = socket.receiveBytes(buf, NUM_BYTES);
			}catch(Poco::Exception &e){
				std::cout << e.displayText() << std::endl;
				break;
			}

			if(nbytes == NUM_BYTES){
				std::cout << "msg too long!" << std::endl;
				continue;
			}

			if(nbytes == 0){
				std::cout << "client closed." << std::endl;
				isOpen = false;
				isRunning = false;
				isStreaming = false;
				thr.join();
			}else if(nbytes > 0){
				buf[nbytes] = 0;
				std::string s(buf);
				std::cout << "recv: " << nbytes << " bytes." << std::endl;
				std::cout << s << std::endl;

				parser.reset();
				try{
					parser.parse(s);
					Poco::Dynamic::Var ret = parser.result();
					Poco::JSON::Object::Ptr obj = ret.extract<Poco::JSON::Object::Ptr>();
					std::string cmd = obj->getValue<std::string>("type");

					if(cmd == "startTask"){ }

					Poco::JSON::Object::Ptr jsonObj = new Poco::JSON::Object();
					jsonObj->set("ack", "ok");
					std::stringstream os;
					Poco::JSON::Stringifier::stringify(jsonObj, os);
					std::string s = os.str();
					socket.sendBytes(s.c_str(), s.length());

					/*
					if(msg_type == "event"){
						std::cout << "testID: " << obj->getValue<int>("testID") << std::endl;
						Poco::JSON::Array::Ptr arr = obj->getArray("objects");
						for(int i = 0; i < arr->size(); i++){
							obj = arr->getObject(i);
							std::cout << "####################" << std::endl;
							std::cout << "id: " << obj->getValue<int>("id") << std::endl;
							std::cout << "type: " << obj->getValue<std::string>("type") << std::endl;
							std::cout << "x: " << obj->getValue<double>("x") << std::endl;
							std::cout << "y: " << obj->getValue<double>("y") << std::endl;
							std::cout << "z: " << obj->getValue<double>("z") << std::endl;
						}
					}else if(msg_type == "request"){
						bool enable = obj->getValue<bool>("enableEvent");
						std::cout << "enableEvent: " << enable << std::endl;
						isStreaming = enable;
					}
					*/
				}catch(Poco::Exception &e){
					std::cout << e.displayText() << std::endl;
				}
			}
		}

		socket.close();
	}

	return 0;
}

