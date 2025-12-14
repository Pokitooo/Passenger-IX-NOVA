#ifndef NOVA_STATE_DEF_H
#define NOVA_STATE_DEF_H

#include <Arduino.h>

namespace nova {
    enum class state_t : uint8_t {
        STARTUP = 0,
        IDLE_SAFE,
        ARMED,
        PAD_PREOP,
        ASCENT,
        APOGEE,
        DEPLOY,
        DESCENT,
        LANDED
    };

    inline const char *state_string(const state_t state) {
        switch (state) {
            case state_t::STARTUP:
                return "STARTUP";
            case state_t::IDLE_SAFE:
                return "IDLE_SAFE";
            case state_t::ARMED:
                return "ARMED";
            case state_t::PAD_PREOP:
                return "PAD_PREOP";
            case state_t::ASCENT:
                return "ASCENT";
            case state_t::APOGEE:
                return "APOGEE";
            case state_t::DEPLOY:
                return "DEPLOY";
                return "MAIN_DESC";
            case state_t::LANDED:
                return "LANDED";
            default:
                __builtin_unreachable();
        }
    }

    enum class pyro_state_t : uint8_t {
        DISARMED = 0,
        ARMED,
        FIRING,
        FIRED
    };

    inline const char *pyro_state_string(const pyro_state_t state) {
        switch (state) {
            case pyro_state_t::DISARMED:
                return "D";
            case pyro_state_t::ARMED:
                return "A";
            case pyro_state_t::FIRING:
                return "F";
            case pyro_state_t::FIRED:
                return "X";
            default:
                __builtin_unreachable();
        }
    }
}  // namespace luna

#endif  //LUNA_STATE_DEF_H
