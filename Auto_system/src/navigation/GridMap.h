#pragma once

#include <Arduino.h>

namespace RobotNavigation
{
constexpr uint8_t GRID_COLUMNS = 9;
constexpr uint8_t GRID_ROWS = 9;
constexpr uint8_t RFID_UID_MAX_LENGTH = 24;

enum class SoilStatus : uint8_t
{
    Unknown,
    Infertile,
    Fertile
};

struct GridCoordinate
{
    uint8_t column = 0;
    uint8_t row = 0;

    bool isValid() const;
};

struct GridCell
{
    GridCoordinate coordinate;
    SoilStatus soil = SoilStatus::Unknown;
    bool visited = false;
    bool planted = false;
};

struct RfidMapEntry
{
    char uid[RFID_UID_MAX_LENGTH] = "";
    GridCoordinate coordinate;
    SoilStatus soil = SoilStatus::Unknown;
};

class GridMap
{
public:
    GridMap();

    void reset();

    bool parseCoordinate(const char* text, GridCoordinate& coordinate) const;
    bool coordinateToString(
        GridCoordinate coordinate,
        char* destination,
        size_t destinationSize
    ) const;

    GridCell* cell(GridCoordinate coordinate);
    const GridCell* cell(GridCoordinate coordinate) const;

    bool markVisited(GridCoordinate coordinate);
    bool markSoil(GridCoordinate coordinate, SoilStatus soil);
    bool markPlanted(GridCoordinate coordinate);

    bool setRfidTag(
        uint8_t index,
        const char* uid,
        GridCoordinate coordinate,
        SoilStatus soil
    );
    bool lookupRfidTag(
        const String& uid,
        GridCoordinate& coordinate,
        SoilStatus& soil
    ) const;

    bool chooseNextTarget(GridCoordinate& target) const;
    uint8_t visitedCount() const;
    uint8_t plantedCount() const;

private:
    GridCell cells_[GRID_COLUMNS * GRID_ROWS];
    RfidMapEntry rfidTags_[GRID_COLUMNS * GRID_ROWS];

    bool indexFor(GridCoordinate coordinate, uint8_t& index) const;
    static bool uidEquals(const char* knownUid, const String& observedUid);
    static void normalizeUid(const char* source, char* destination, size_t size);
};
} // namespace RobotNavigation
