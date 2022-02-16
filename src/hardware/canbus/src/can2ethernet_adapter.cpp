#include "can2etherenet_adapter.hpp"

Can2Ethernet::Can2Ethernet(std::string ip, int port) {
	this->ip = ip;
	this->port = port;
	this->address = ip + std::to_string(port);
	this->setup(this->ip, this->port);
}

void Can2Ethernet::tx(uint32_t id, bool std_id, uint8_t* data) {
	auto d = std::make_shared<std::vector<char>>();
	uint8_t enetByte = 8;
	enetByte = enetByte | ((std_id << 7) & 1);
	d->emplace_back(enetByte);
	for (int i = 0; i < 4; i++) {
		uint8_t ourByte = (id >> (24 - i * 8)) & 0xFF;
		d->emplace_back(ourByte);
	}
	for (int i = 0; i < 8; i++) {
		d->emplace_back(data[i]);
	}
	auto didSend = this->send_data(d);
	if(!didSend) {
		std::cout << "Failed to send_data!" << std::endl;
	}
}
std::shared_ptr<std::vector<char>> Can2Ethernet::rx() {
	return this->recieve_data();
}
