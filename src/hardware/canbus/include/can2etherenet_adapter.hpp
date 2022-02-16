#pragma once
#include <iostream>
#include <string>

#include "tcp_client.hpp"

class Can2Ethernet : TCPClient {
   private:
	std::string ip;
	int port;
	std::string address;

   public:
	Can2Ethernet(std::string ip, int port);
	void tx(uint32_t id, bool std_id, uint8_t* data);
	std::shared_ptr<std::vector<char>> rx();
	~Can2Ethernet() { this->_disconnect(); };
};

/*
-> Target Steering Angle
-> Torque
-> Regen
-> TV
-> EBS Activate
-> GPS RTK
-> State (Flow diagram in rules xoxo)

<- Steering Angle
<- Current Torque
<- Current Regen
<- Current TV State
<- EBS State
<- Car State (RTD)
<- IMU + GPS
<- AS Mission
<- Go Button
*/