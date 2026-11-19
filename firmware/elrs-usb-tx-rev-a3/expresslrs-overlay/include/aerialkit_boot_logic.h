#pragma once

#include <stdint.h>

struct aerialkit_heartbeat_spec_t
{
    uint8_t green;
    uint8_t red;
    uint8_t blue;
    uint8_t pulses;
    uint16_t on_ms;
    uint16_t off_ms;
};

constexpr uint8_t AERIALKIT_WS2812_LEVEL = 26;

constexpr aerialkit_heartbeat_spec_t aerialkit_heartbeat_spec(bool rx_role)
{
    return rx_role
               ? aerialkit_heartbeat_spec_t{
                     AERIALKIT_WS2812_LEVEL, 0, 0, 2, 350, 250}
               : aerialkit_heartbeat_spec_t{
                     0, 0, AERIALKIT_WS2812_LEVEL, 1, 350, 250};
}

constexpr bool aerialkit_role_matches(bool role_pin_high, bool rx_role)
{
    return role_pin_high == rx_role;
}
