#include "autobot/world/SpatialHash.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>

namespace autobot::world {
namespace {

struct RectExtents {
    float minX;
    float minY;
    float maxX;
    float maxY;
};

RectExtents extents(WorldRect const& rect) {
    const float x2 = rect.x + rect.width;
    const float y2 = rect.y + rect.height;
    return {
        std::min(rect.x, x2),
        std::min(rect.y, y2),
        std::max(rect.x, x2),
        std::max(rect.y, y2),
    };
}

bool intersects(WorldRect const& a, WorldRect const& b) {
    const auto ea = extents(a);
    const auto eb = extents(b);
    return ea.maxX >= eb.minX
        && ea.minX <= eb.maxX
        && ea.maxY >= eb.minY
        && ea.minY <= eb.maxY;
}

bool finiteRect(WorldRect const& rect) {
    return std::isfinite(rect.x)
        && std::isfinite(rect.y)
        && std::isfinite(rect.width)
        && std::isfinite(rect.height);
}

std::vector<SpatialCell> const kEmptyCells{};

} // namespace

SpatialHash::SpatialHash(float cellSize)
  : m_cellSize(cellSize > 0.0f ? cellSize : 120.0f) {}

void SpatialHash::clear() {
    m_cells.clear();
    m_objectCells.clear();
}

std::size_t SpatialHash::CellKeyHash::operator()(SpatialCell const& key) const noexcept {
    const auto x = static_cast<std::uint32_t>(key.x);
    const auto y = static_cast<std::uint32_t>(key.y);
    const std::uint64_t combined = (static_cast<std::uint64_t>(x) << 32u) | y;
    return std::hash<std::uint64_t>{}(combined);
}

int SpatialHash::cellCoordinate(float value) const {
    return static_cast<int>(std::floor(value / m_cellSize));
}

void SpatialHash::build(std::vector<CollisionPrimitive> const& primitives) {
    clear();
    ++m_generation;
    m_objectCells.resize(primitives.size());

    for (std::size_t index = 0; index < primitives.size(); ++index) {
        auto const& primitive = primitives[index];
        auto const& bounds = primitive.broadphaseBounds;
        if (!primitive.enabled || !primitive.indexable || !finiteRect(bounds)) continue;

        const auto e = extents(bounds);
        const int minCellX = cellCoordinate(e.minX);
        const int maxCellX = cellCoordinate(e.maxX);
        const int minCellY = cellCoordinate(e.minY);
        const int maxCellY = cellCoordinate(e.maxY);

        auto& occupied = m_objectCells[index];
        const auto width = static_cast<std::size_t>(maxCellX - minCellX + 1);
        const auto height = static_cast<std::size_t>(maxCellY - minCellY + 1);
        occupied.reserve(width * height);

        for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
            for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
                SpatialCell cell{cellX, cellY};
                m_cells[cell].push_back(index);
                occupied.push_back(cell);
            }
        }
    }
}

void SpatialHash::refreshPrimitive(
    std::size_t primitiveIndex,
    CollisionPrimitive const& primitive
) {
    if (primitiveIndex >= m_objectCells.size()) {
        m_objectCells.resize(primitiveIndex + 1);
    }

    auto& occupied = m_objectCells[primitiveIndex];
    for (auto const& cell : occupied) {
        auto it = m_cells.find(cell);
        if (it == m_cells.end()) continue;
        auto& bucket = it->second;
        bucket.erase(
            std::remove(bucket.begin(), bucket.end(), primitiveIndex),
            bucket.end()
        );
        if (bucket.empty()) m_cells.erase(it);
    }
    occupied.clear();

    auto const& bounds = primitive.broadphaseBounds;
    if (!primitive.enabled || !primitive.indexable || !finiteRect(bounds)) {
        ++m_generation;
        return;
    }

    const auto e = extents(bounds);
    const int minCellX = cellCoordinate(e.minX);
    const int maxCellX = cellCoordinate(e.maxX);
    const int minCellY = cellCoordinate(e.minY);
    const int maxCellY = cellCoordinate(e.maxY);
    const auto width = static_cast<std::size_t>(maxCellX - minCellX + 1);
    const auto height = static_cast<std::size_t>(maxCellY - minCellY + 1);
    occupied.reserve(width * height);

    for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
        for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
            SpatialCell cell{cellX, cellY};
            auto& bucket = m_cells[cell];
            if (std::find(bucket.begin(), bucket.end(), primitiveIndex) == bucket.end()) {
                bucket.push_back(primitiveIndex);
            }
            occupied.push_back(cell);
        }
    }
    ++m_generation;
}

std::vector<std::size_t> SpatialHash::queryRegion(
    WorldRect const& region,
    std::vector<CollisionPrimitive> const& primitives
) const {
    SpatialQueryDebug ignored{};
    return queryRegionDetailed(region, primitives, ignored);
}

std::vector<std::size_t> SpatialHash::queryRegionDetailed(
    WorldRect const& region,
    std::vector<CollisionPrimitive> const& primitives,
    SpatialQueryDebug& debug
) const {
    debug = {};
    std::vector<std::size_t> result;
    if (!finiteRect(region) || m_cells.empty()) return result;

    const auto e = extents(region);
    const int minCellX = cellCoordinate(e.minX);
    const int maxCellX = cellCoordinate(e.maxX);
    const int minCellY = cellCoordinate(e.minY);
    const int maxCellY = cellCoordinate(e.maxY);

    std::unordered_set<std::size_t> seen;
    seen.reserve(128);

    for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
        for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
            ++debug.cellsVisited;
            auto it = m_cells.find({cellX, cellY});
            if (it == m_cells.end()) continue;

            debug.objectsInCells += it->second.size();
            for (auto index : it->second) {
                if (index >= primitives.size()) {
                    ++debug.invalidIndices;
                    continue;
                }
                if (!seen.insert(index).second) {
                    ++debug.duplicatesSuppressed;
                    continue;
                }
                ++debug.objectsAfterDedup;

                auto const& primitive = primitives[index];
                if (primitive.enabled && primitive.indexable
                    && intersects(primitive.broadphaseBounds, region)) {
                    result.push_back(index);
                    ++debug.objectsAfterIntersection;
                }
            }
        }
    }

    return result;
}

bool SpatialHash::contains(std::size_t primitiveIndex) const {
    return primitiveIndex < m_objectCells.size() && !m_objectCells[primitiveIndex].empty();
}

std::vector<SpatialCell> const& SpatialHash::cellsFor(std::size_t primitiveIndex) const {
    if (primitiveIndex >= m_objectCells.size()) return kEmptyCells;
    return m_objectCells[primitiveIndex];
}

} // namespace autobot::world
