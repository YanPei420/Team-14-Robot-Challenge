#pragma once

#include <Arduino.h>

namespace RobotFSMConfig
{
constexpr int16_t LINE_FOLLOW_SPEED = 120;
constexpr int16_t TUNNEL_SPEED = 90;
constexpr int16_t GRID_EXPLORE_SPEED = 100;
constexpr int16_t ALIGN_SPEED = 70;
constexpr int16_t RETURN_SPEED = 130;

constexpr uint32_t SOIL_QUERY_TIMEOUT_MS = 1500;
constexpr uint32_t SEARCH_RFID_MS = 500;
constexpr uint32_t FINE_ADJUST_MS = 900;
constexpr uint32_t OPEN_HOPPER_MS = 400;
constexpr uint32_t DROP_SEED_MS = 700;
constexpr uint32_t VERIFY_DROP_MS = 400;
constexpr uint32_t REVIVE_PAUSE_MS = 1000;
constexpr uint32_t ARENA_TIME_LIMIT_MS = 4UL * 60UL * 1000UL;
} // namespace RobotFSMConfig
