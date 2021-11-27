#include "tcp_client.hpp"

TCPClient::TCPClient() {
	this->sock = -1;
	this->port = 0;
	this->address = "";
}

bool TCPClient::setup(std::string address, int port) {
	this->address = address;
	this->port = port;
	if (this->sock == -1) {
		this->sock = socket(AF_INET, SOCK_STREAM, 0);
		if (this->sock == -1) {
			std::cout << "Failed to create socket." << std::endl;
		}
	}
	if ((signed)inet_addr(address.c_str()) == -1) {
		struct hostent *he;
		struct in_addr **addr_list;
		if ((he = gethostbyname(address.c_str())) == NULL) {
			std::cout << "Failed to resolve hostname" << std::endl;
			return false;
		}
		addr_list = (struct in_addr **)he->h_addr_list;
		for (int i = 0; addr_list[i] != NULL; i++) {
			this->server.sin_addr = *addr_list[i];
			break;
		}
	} else {
		this->server.sin_addr.s_addr = inet_addr(address.c_str());
	}
	this->server.sin_family = AF_INET;
	this->server.sin_port = htons(port);

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 1000;
	setsockopt(this->sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

	this->_connect();
	return true;
}

bool TCPClient::_connect() {
	if (connect(this->sock, (struct sockaddr *)&this->server,
				sizeof(this->server)) < 0) {
		std::cout << "Connetion failed to: " << this->sock << std::endl;
		return false;
	}
	return true;
}

void TCPClient::_disconnect() { 
	close(this->sock);
	}

bool TCPClient::send_data(std::shared_ptr<std::vector<char>> data) {
	if (this->sock != -1) {
		// this->_connect();
		{
			if (send(this->sock, data->data(), data->size(), 0) < 0) {
				std::cout << "Failed to send on socket: " << this->sock
						  << std::endl;
				return false;
			}
		}
		// this->_disconnect();
	} else {
		std::cout << "Please setup before trying to send!" << std::endl;
		return false;
	}
	return true;
}

std::shared_ptr<std::vector<char>> TCPClient::recieve_data() {
	char buf[RECV_SIZE];
	int b;
	// this->_connect();
	{
		if ((b = recv(this->sock, buf, RECV_SIZE, 0)) < 0) {
			// std::cout << "Failed to recv on socket: " << this->sock << std::endl;
			return nullptr;
		}
	}
	// this->_disconnect();
	auto vec = std::make_shared<std::vector<char>>(buf, buf + b);
	return vec;
}