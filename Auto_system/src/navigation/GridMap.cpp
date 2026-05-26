#include "GridMap.h"

#include <ctype.h>
#include <string.h>

namespace RobotNavigation
{
bool GridCoordinate::isValid() const
{
    return column < GRID_COLUMNS && row < GRID_ROWS;
}

GridMap::GridMap()
{
    reset();
}

void GridMap::reset()
{
    for (uint8_t row = 0; row < GRID_ROWS; ++row)
    {
        for (uint8_t column = 0; column < GRID_COLUMNS; ++column)
        {
            const uint8_t index = row * GRID_COLUMNS + column;
            cells_[index].coordinate = {column, row};
            cells_[index].soil = SoilStatus::Unknown;
            cells_[index].visited = false;
            cells_[index].planted = false;
            rfidTags_[index] = RfidMapEntry{};
        }
    }
}

bool GridMap::parseCoordinate(
    const char* text,
    GridCoordinate& coordinate
) const
{
    if (text == nullptr || text[0] == '\0' || text[1] == '\0')
    {
        return false;
    }

    const char columnChar = static_cast<char>(toupper(text[0]));
    if (columnChar < 'A' || columnChar >= static_cast<char>('A' + GRID_COLUMNS))
    {
        return false;
    }

    uint16_t rowNumber = 0;
    for (uint8_t i = 1; text[i] != '\0'; ++i)
    {
        if (!isdigit(text[i]))
        {
            return false;
        }

        rowNumber = rowNumber * 10 + static_cast<uint16_t>(text[i] - '0');
    }

    if (rowNumber == 0 || rowNumber > GRID_ROWS)
    {
        return false;
    }

    coordinate.column = static_cast<uint8_t>(columnChar - 'A');
    coordinate.row = static_cast<uint8_t>(rowNumber - 1);
    return true;
}

bool GridMap::coordinateToString(
    GridCoordinate coordinate,
    char* destination,
    size_t destinationSize
) const
{
    if (!coordinate.isValid() || destination == nullptr || destinationSize < 3)
    {
        return false;
    }

    snprintf(
        destination,
        destinationSize,
        "%c%u",
        static_cast<char>('A' + coordinate.column),
        static_cast<unsigned>(coordinate.row + 1)
    );
    return true;
}

GridCell* GridMap::cell(GridCoordinate coordinate)
{
    uint8_t index = 0;
    if (!indexFor(coordinate, index))
    {
        return nullptr;
    }

    return &cells_[index];
}

const GridCell* GridMap::cell(GridCoordinate coordinate) const
{
    uint8_t index = 0;
    if (!indexFor(coordinate, index))
    {
        return nullptr;
    }

    return &cells_[index];
}

bool GridMap::markVisited(GridCoordinate coordinate)
{
    GridCell* targetCell = cell(coordinate);
    if (targetCell == nullptr)
    {
        return false;
    }

    targetCell->visited = true;
    return true;
}

bool GridMap::markSoil(GridCoordinate coordinate, SoilStatus soil)
{
    GridCell* targetCell = cell(coordinate);
    if (targetCell == nullptr)
    {
        return false;
    }

    targetCell->soil = soil;
    return true;
}

bool GridMap::markPlanted(GridCoordinate coordinate)
{
    GridCell* targetCell = cell(coordinate);
    if (targetCell == nullptr)
    {
        return false;
    }

    targetCell->planted = true;
    targetCell->visited = true;
    targetCell->soil = SoilStatus::Fertile;
    return true;
}

bool GridMap::setRfidTag(
    uint8_t index,
    const char* uid,
    GridCoordinate coordinate,
    SoilStatus soil
)
{
    if (
        index >= GRID_COLUMNS * GRID_ROWS ||
        uid == nullptr ||
        !coordinate.isValid()
    )
    {
        return false;
    }

    strncpy(rfidTags_[index].uid, uid, sizeof(rfidTags_[index].uid) - 1);
    rfidTags_[index].uid[sizeof(rfidTags_[index].uid) - 1] = '\0';
    rfidTags_[index].coordinate = coordinate;
    rfidTags_[index].soil = soil;
    markSoil(coordinate, soil);

    return true;
}

bool GridMap::lookupRfidTag(
    const String& uid,
    GridCoordinate& coordinate,
    SoilStatus& soil
) const
{
    for (const RfidMapEntry& entry : rfidTags_)
    {
        if (entry.uid[0] == '\0')
        {
            continue;
        }

        if (uidEquals(entry.uid, uid))
        {
            coordinate = entry.coordinate;
            soil = entry.soil;
            return true;
        }
    }

    return false;
}

bool GridMap::chooseNextTarget(GridCoordinate& target) const
{
    for (const GridCell& candidate : cells_)
    {
        if (
            candidate.soil == SoilStatus::Fertile &&
            !candidate.planted
        )
        {
            target = candidate.coordinate;
            return true;
        }
    }

    for (const GridCell& candidate : cells_)
    {
        if (!candidate.visited)
        {
            target = candidate.coordinate;
            return true;
        }
    }

    return false;
}

uint8_t GridMap::visitedCount() const
{
    uint8_t count = 0;
    for (const GridCell& candidate : cells_)
    {
        if (candidate.visited)
        {
            ++count;
        }
    }

    return count;
}

uint8_t GridMap::plantedCount() const
{
    uint8_t count = 0;
    for (const GridCell& candidate : cells_)
    {
        if (candidate.planted)
        {
            ++count;
        }
    }

    return count;
}

bool GridMap::indexFor(GridCoordinate coordinate, uint8_t& index) const
{
    if (!coordinate.isValid())
    {
        return false;
    }

    index = coordinate.row * GRID_COLUMNS + coordinate.column;
    return true;
}

bool GridMap::uidEquals(const char* knownUid, const String& observedUid)
{
    char normalizedKnown[RFID_UID_MAX_LENGTH];
    char normalizedObserved[RFID_UID_MAX_LENGTH];

    normalizeUid(knownUid, normalizedKnown, sizeof(normalizedKnown));
    normalizeUid(observedUid.c_str(), normalizedObserved, sizeof(normalizedObserved));

    return strcmp(normalizedKnown, normalizedObserved) == 0;
}

void GridMap::normalizeUid(
    const char* source,
    char* destination,
    size_t size
)
{
    if (destination == nullptr || size == 0)
    {
        return;
    }

    destination[0] = '\0';

    if (source == nullptr)
    {
        return;
    }

    size_t writeIndex = 0;
    for (size_t readIndex = 0; source[readIndex] != '\0'; ++readIndex)
    {
        const char c = source[readIndex];
        if (c == ' ' || c == ':' || c == '-')
        {
            continue;
        }

        if (writeIndex + 1 >= size)
        {
            break;
        }

        destination[writeIndex] = static_cast<char>(toupper(c));
        ++writeIndex;
    }

    destination[writeIndex] = '\0';
}
} // namespace RobotNavigation
