#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <vector>

#define RECV_SIZE 4096

class TCPClient {
   private:
	int sock;
	std::string address;
	int port;
	struct sockaddr_in server;

   public:
	TCPClient();
	bool setup(std::string address, int port);
	bool _connect();
	void _disconnect();
	bool send_data(std::shared_ptr<std::vector<char>> data);
	std::shared_ptr<std::vector<char>> recieve_data();
	~TCPClient() { this->_disconnect(); };
};
