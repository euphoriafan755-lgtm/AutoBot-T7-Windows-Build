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

} // namespace

SpatialHash::SpatialHash(float cellSize)
  : m_cellSize(cellSize > 0.0f ? cellSize : 120.0f) {}

void SpatialHash::clear() {
    m_cells.clear();
}

std::size_t SpatialHash::CellKeyHash::operator()(CellKey const& key) const noexcept {
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

    for (std::size_t index = 0; index < primitives.size(); ++index) {
        auto const& primitive = primitives[index];
        auto const& bounds = primitive.broadphaseBounds;
        if (!primitive.enabled || !finiteRect(bounds)) continue;

        const auto e = extents(bounds);
        const int minCellX = cellCoordinate(e.minX);
        const int maxCellX = cellCoordinate(e.maxX);
        const int minCellY = cellCoordinate(e.minY);
        const int maxCellY = cellCoordinate(e.maxY);

        for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
            for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
                m_cells[{cellX, cellY}].push_back(index);
            }
        }
    }
}

std::vector<std::size_t> SpatialHash::queryRegion(
    WorldRect const& region,
    std::vector<CollisionPrimitive> const& primitives
) const {
    std::vector<std::size_t> result;
    if (!finiteRect(region) || m_cells.empty()) return result;

    const auto e = extents(region);
    const int minCellX = cellCoordinate(e.minX);
    const int maxCellX = cellCoordinate(e.maxX);
    const int minCellY = cellCoordinate(e.minY);
    const int maxCellY = cellCoordinate(e.maxY);

    std::unordered_set<std::size_t> seen;
    seen.reserve(64);

    for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
        for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
            auto it = m_cells.find({cellX, cellY});
            if (it == m_cells.end()) continue;

            for (auto index : it->second) {
                if (index >= primitives.size()) continue;
                if (!seen.insert(index).second) continue;

                auto const& primitive = primitives[index];
                if (primitive.enabled && intersects(primitive.broadphaseBounds, region)) {
                    result.push_back(index);
                }
            }
        }
    }

    return result;
}

} // namespace autobot::world
