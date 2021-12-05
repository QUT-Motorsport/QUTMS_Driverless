#include "steering_control.hpp"

SteeringControl::SteeringControl(std::shared_ptr<Can2Ethernet> can, c5e_config_t config) {
	this->can = can;
	this->velocity = config.default_velocity;
	this->accelerations = std::make_pair<int32_t, int32_t>(config.default_accelerations, config.default_accelerations);
	this->limits = std::make_pair<int32_t, int32_t>(config.default_limits, -config.default_limits);

	this->current = config.default_current;
	this->target = 0;

	/* To initialise the controller to a usable state, we must set the:
		Target Position = 0
		Min Position Limit = -DEFAULT_LIMITS
		Max Position Limit = DEFAULT_LIMITS
		Home Offset = 0
		Motion Profile Type = trapezoidal ramp
		Profile Velocity = DEFAULT_VELOCITY
		End Velocity = 0
		Profile Acceleration = DEFAULT_ACCELERATIONS
		Profile Deceleration = DEFAULT_ACCELERATIONS
		Quick Stop Deceleration = DEFAUL_ACCELERATIONS
		Max Acceleration = DEFAULT_ACCELERATIONS
		MAX Deceleration = DEFAULT_ACCELERATIONS
		Mode of Operation = 1 (Profile Position)
	*/

	uint32_t id;	 // Packet id out
	uint8_t out[8];	 // Data out

	std::cout << "Performing C5-E Setup... ";

	sdo_write(C5_E_ID, 0x607A, 0x00, (uint8_t*)&this->target, 4, &id, out);	 // Target
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x607D, 0x01, (uint8_t*)&this->limits.second, 4, &id, out);	// Min Limit
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x607D, 0x02, (uint8_t*)&this->limits.first, 4, &id, out);  // Max Limit
	this->can->tx(id, 0, out);

	int32_t ho = 0;
	sdo_write(C5_E_ID, 0x607C, 0x00, (uint8_t*)&ho, 4, &id, out);  // Home Offset
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x6086, 0x00, (uint8_t*)&ho, 2, &id, out);  // Motion Profile Type (trap)
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x6081, 0x00, (uint8_t*)&this->velocity, 4, &id, out);  // Profile Velocity
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x6082, 0x00, (uint8_t*)&ho, 4, &id, out);  // End Velocity
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x6083, 0x00, (uint8_t*)&this->accelerations.first, 4, &id, out);  // Profile Accelerataion
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x6084, 0x00, (uint8_t*)&this->accelerations.second, 4, &id, out);  // Profile Deceleration
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x6085, 0x00, (uint8_t*)&this->accelerations.first, 4, &id, out);  // Quick Stop Deceleration
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x60C5, 0x00, (uint8_t*)&this->accelerations.first, 4, &id, out);  // Max Acceleration
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x60C6, 0x00, (uint8_t*)&this->accelerations.second, 4, &id, out);  // Max Deceleration
	this->can->tx(id, 0, out);

	int8_t ppm = 1;
	sdo_write(C5_E_ID, 0x6060, 0x00, (uint8_t*)&ppm, 1, &id, out);	// Modes of Operation
	this->can->tx(id, 0, out);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	uint16_t control_word = 6;
	sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t*)&control_word, 2, &id, out);	 // Shutdown
	this->can->tx(id, 0, out);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	control_word = 7;
	sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t*)&control_word, 2, &id, out);	 // Switched On
	this->can->tx(id, 0, out);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	control_word = 15;
	sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t*)&control_word, 2, &id, out);	 // Op Enabled
	this->can->tx(id, 0, out);

	std::cout << "Done (Operation Enabled)" << std::endl;
}

void SteeringControl::target_position(int32_t target) {
	// Set Target
	this->target = target;

	uint32_t id;	 // Packet id out
	uint8_t out[8];	 // Data out

	uint16_t control_word = 47;
	sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t*)&control_word, 2, &id, out);	 // Control Word
	this->can->tx(id, 0, out);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	sdo_write(C5_E_ID, 0x607A, 0x00, (uint8_t*)&this->target, 4, &id, out);	 // Target
	this->can->tx(id, 0, out);

	// Set Control Word
	control_word = 63;
	sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t*)&control_word, 2, &id, out);	 // Control Word
	this->can->tx(id, 0, out);
}

void SteeringControl::target_position(int32_t target, int32_t velocity) {
	this->target = target;

	uint32_t id;	 // Packet id out
	uint8_t out[8];	 // Data out

	sdo_write(C5_E_ID, 0x607A, 0x00, (uint8_t*)&this->target, 4, &id, out);	 // Target
	this->can->tx(id, 0, out);

	this->velocity = velocity;

	sdo_write(C5_E_ID, 0x6081, 0x00, (uint8_t*)&this->velocity, 4, &id, out);  // Profile Velocity
	this->can->tx(id, 0, out);

	uint16_t control_word = 47;
	sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t*)&control_word, 2, &id, out);	 // Control Word
	this->can->tx(id, 0, out);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	// Set Control Word
	control_word = 63;
	sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t*)&control_word, 2, &id, out);	 // Control Word
	this->can->tx(id, 0, out);
}

void SteeringControl::set_velocity(int32_t velocity) {
	this->velocity = velocity;

	uint32_t id;	 // Packet id out
	uint8_t out[8];	 // Data out

	sdo_write(C5_E_ID, 0x6081, 0x00, (uint8_t*)&this->velocity, 4, &id, out);  // Profile Velocity
	this->can->tx(id, 0, out);
}

void SteeringControl::set_acceleration(std::pair<uint32_t, uint32_t> accelerations) {
	this->accelerations = accelerations;

	uint32_t id;	 // Packet id out
	uint8_t out[8];	 // Data out

	sdo_write(C5_E_ID, 0x6083, 0x00, (uint8_t*)&this->accelerations.first, 4, &id, out);  // Profile Accelerataion
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x6084, 0x00, (uint8_t*)&this->accelerations.second, 4, &id, out);  // Profile Deceleration
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x6085, 0x00, (uint8_t*)&this->accelerations.first, 4, &id, out);  // Quick Stop Deceleration
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x60C5, 0x00, (uint8_t*)&this->accelerations.first, 4, &id, out);  // Max Acceleration
	this->can->tx(id, 0, out);

	sdo_write(C5_E_ID, 0x60C6, 0x00, (uint8_t*)&this->accelerations.second, 4, &id, out);  // Max Deceleration
	this->can->tx(id, 0, out);
}

bool SteeringControl::reached_target() {
	uint32_t id;
	uint8_t out[8];

	sdo_read(C5_E_ID, 0x6041, 0x00, &id, out);
	this->can->tx(id, 0, out);

	auto res = this->can->rx();
	if(res != nullptr) {
		if((res->at(res->size() - 3) & 0x04)) {
			return true;
		}
	}
	return false;
}

void SteeringControl::shutdown() {
	uint32_t id;
	uint8_t out[8];
	
	uint16_t control_word = 6;
	sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t*)&control_word, 2, &id, out);	 // Shutdown
	this->can->tx(id, 0, out);
}

void SteeringControl::set_c5e_config(c5e_config_t config) {
	this->defaults.default_accelerations = config.default_accelerations;
	this->defaults.default_current = config.default_current;
	this->defaults.default_limits = config.default_limits;
	this->defaults.default_velocity = config.default_velocity;
}

c5e_config_t SteeringControl::get_c5e_config() {
	return this->defaults;
}

SteeringControl::~SteeringControl() {}
