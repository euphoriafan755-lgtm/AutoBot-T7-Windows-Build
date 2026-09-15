#pragma once

#include "autobot/world/CollisionTypes.hpp"

#include <cstddef>
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

    [[nodiscard]] float cellSize() const { return m_cellSize; }
    [[nodiscard]] std::size_t cellCount() const { return m_cells.size(); }

private:
    struct CellKey {
        int x = 0;
        int y = 0;

        bool operator==(CellKey const& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct CellKeyHash {
        std::size_t operator()(CellKey const& key) const noexcept;
    };

    [[nodiscard]] int cellCoordinate(float value) const;

    float m_cellSize = 120.0f;
    std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> m_cells;
};

} // namespace autobot::world
