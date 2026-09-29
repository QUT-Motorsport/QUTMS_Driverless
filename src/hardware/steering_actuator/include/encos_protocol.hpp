#ifndef STEERING_ACTUATOR__ENCOS_PROTOCOL_HPP_
#define STEERING_ACTUATOR__ENCOS_PROTOCOL_HPP_

#include <stdint.h>
#include <string.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace encos {

const uint8_t ERROR_NONE = 0;

const uint8_t STOP_FRAME[3] = {0x62, 0x00, 0x00};

inline uint32_t float_to_bits(float f) {
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

inline float bits_to_float(uint32_t u) {
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

inline void pack_servo_position(float pos_deg, float speed_rpm, float current_a, uint8_t ack, uint8_t out[8]) {
    uint64_t spd = (uint64_t)std::clamp<long>(std::lround(speed_rpm * 10.0f), 0, 0x7FFF);
    uint64_t cur = (uint64_t)std::clamp<long>(std::lround(current_a * 10.0f), 0, 0xFFF);
    uint64_t u = (1ULL << 61) | ((uint64_t)float_to_bits(pos_deg) << 29) | (spd << 14) | (cur << 2) | (ack & 0x3);
    for (int i = 0; i < 8; i++) {
        out[i] = (uint8_t)(u >> (56 - 8 * i));
    }
}

struct Feedback {
    uint8_t type = 0;
    uint8_t error = 0;
    bool has_position = false;
    float position_deg = 0;
    float current_a = 0;
    float temperature_c = 0;
};

inline bool decode_reply(const uint8_t *data, size_t len, Feedback &fb) {
    if (len < 1) return false;
    fb.type = data[0] >> 5;
    fb.error = data[0] & 0x1F;
    if (fb.type == 2 && len >= 8) {
        uint32_t pos = (uint32_t)data[1] << 24 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 8 | data[4];
        int16_t cur = (int16_t)((uint16_t)data[5] << 8 | data[6]);
        fb.has_position = true;
        fb.position_deg = bits_to_float(pos);
        fb.current_a = cur / 100.0f;
        fb.temperature_c = (data[7] - 50) / 2.0f;
    }
    return true;
}

inline std::string error_text(uint8_t code) {
    switch (code) {
        case 0:
            return "none";
        case 1:
            return "overheat";
        case 2:
            return "overcurrent";
        case 3:
            return "voltage too low";
        case 4:
            return "encoder error";
        case 6:
            return "brake voltage too high";
        case 7:
            return "DRV driver error";
        default:
            return "unknown (" + std::to_string(code) + ")";
    }
}

}  // namespace encos

#endif  // STEERING_ACTUATOR__ENCOS_PROTOCOL_HPP_
