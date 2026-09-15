#pragma once

#include "autobot/world/CollisionTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace autobot::world {

class SpatialHash final {
public:
    explicit SpatialHash(float cellSize = 120.0f);

    void clear();
    void build(std::vector<CollisionPrimitive> const& primitives);

    [[nodiscard]] std::vector<std::size_t> queryRegion(
        WorldRect const& region,
        std::vector<CollisionPrimitive> const& primitives
    ) const;

    [[nodiscard]] std::vector<std::size_t> queryRegionDetailed(
        WorldRect const& region,
        std::vector<CollisionPrimitive> const& primitives,
        SpatialQueryDebug& debug
    ) const;

    [[nodiscard]] bool contains(std::size_t primitiveIndex) const;
    [[nodiscard]] std::vector<SpatialCell> const& cellsFor(std::size_t primitiveIndex) const;

    [[nodiscard]] float cellSize() const { return m_cellSize; }
    [[nodiscard]] std::size_t cellCount() const { return m_cells.size(); }

private:
    struct CellKeyHash {
        std::size_t operator()(SpatialCell const& key) const noexcept;
    };

    [[nodiscard]] int cellCoordinate(float value) const;

    float m_cellSize = 120.0f;
    std::unordered_map<SpatialCell, std::vector<std::size_t>, CellKeyHash> m_cells;
    std::vector<std::vector<SpatialCell>> m_objectCells;
};

} // namespace autobot::world
