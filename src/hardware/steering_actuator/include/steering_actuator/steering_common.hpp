#include <stddef.h>
#include <stdint.h>
#include <string>
#include <map>

typedef enum c5e_object_id {
    HOME_OFFSET = 0x607C,
    MOTION_PROFILE_TYPE = 0x6086,
    PROFILE_VELOCITY = 0x6081,
    END_VELOCITY = 0x6082,
    PROFILE_ACCELERATION = 0x6083,
    PROFILE_DECELERATION = 0x6084,
    QUICK_STOP_DECELERATION = 0x6085,
    MAX_ACCELERATION = 0x60C5,
    MAX_DECELERATION = 0x60C6,
    MODE_OF_OPERATION = 0x6060,
    TARGET_POSITION = 0x607A,
    CONTROL_WORD = 0x6040,
    STATUS_WORD = 0x6041,
    POSITION_ACTUAL_VAL = 0x6064,
    ERROR_REGISTER = 0x1001,
} c5e_object_id_t;

// name enum for IDs
std::map<uint16_t, std::string> c5e_object_names = {
    {HOME_OFFSET, "Home offset"},
    {MOTION_PROFILE_TYPE, "Motion profile type"},
    {PROFILE_VELOCITY, "Profile velocity"},
    {END_VELOCITY, "End velocity"},
    {PROFILE_ACCELERATION, "Profile acceleration"},
    {PROFILE_DECELERATION, "Profile deceleration"},
    {QUICK_STOP_DECELERATION, "Quick stop deceleration"},
    {MAX_ACCELERATION, "Max acceleration"},
    {MAX_DECELERATION, "Max deceleration"},
    {MODE_OF_OPERATION, "Mode of operation"},
    {TARGET_POSITION, "Target position"},
    {CONTROL_WORD, "Control word"},
    {STATUS_WORD, "Status word"},
    {POSITION_ACTUAL_VAL, "Position actual value"},
    {ERROR_REGISTER, "Error register"},
};

struct c5e_state {
    std::string name;
    uint16_t mask;
    uint16_t state_id;
    uint16_t control_word;

    bool operator==(const c5e_state &rhs) {
        return (this->name == rhs.name && this->mask == rhs.mask && this->state_id == rhs.state_id);
    }

    bool operator!=(const c5e_state &rhs) { return !(*this == rhs); }
};

/* State ID Definitions
 * NRTSO   = Not ready to switch on
 * SOD     = Switch on disabled
 * RTSO    = Ready to switch on
 * SO      = Switched on
 * OE      = Operation enabled
 * QSA     = Quick stop active
 * FRA     = Fault reaction active
 * F       = Fault
 */

typedef enum c5e_state_id {
    NRTSO = 0b0000000000000000,
    SOD =   0b0000000001000000,
    RTSO =  0b0000000000100001,
    SO =    0b0000000000100011,
    OE =    0b0000000000100111,
    QSA =   0b0000000000000111,
    FRA =   0b0000000000001111,
    F =     0b0000000000001000
} c5e_state_id_t;

// State Map Definitions
std::map<uint16_t, c5e_state> states = {
    {NRTSO, {"Not ready to switch on", 0b0000000001001111, NRTSO, 0b0000}},
    {SOD, {"Switch on disabled", 0b0000000001001111, SOD, 0b0000}},
    {RTSO, {"Ready to switch on", 0b0000000001101111, RTSO, 0b0110}},
    {SO, {"Switched on", 0b0000000001101111, SO, 0b0111}},
    {OE, {"Operation enabled", 0b0000000001101111, OE, 0b1111}},
    {QSA, {"Quick stop active", 0b0000000001101111, QSA, 0b0000}},
    {FRA, {"Fault reaction active", 0b0000000001001111, FRA, 0b0000}},
    {F, {"Fault", 0b0000000001001111, F, 0b0000}},
};

// control word definitions
// last 4 bits are the state control (Operation enabled)
// 1 in bit 7 means relative positioning
const uint16_t MODE_ABSOLUTE =  0b00101111;
const uint16_t MODE_RELATIVE =  0b01101111;
// transition to 1 in bit 4 starts the travel command
const uint16_t TRIGGER_MOTION = 0b00010000;
// transition to 1 in bit 7 resets the fault
const uint16_t FAULT_RESET =    0b10000000;

// Print state function
c5e_state parse_state(uint16_t status_word) {
    for (const auto &[key, actuator_state] : states) {
        if ((status_word & actuator_state.mask) == actuator_state.state_id) {
            return actuator_state;
        }
    }
    return states[F];
}
