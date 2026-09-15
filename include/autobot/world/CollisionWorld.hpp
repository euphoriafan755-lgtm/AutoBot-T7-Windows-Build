#pragma once

#include "autobot/world/CollisionTypes.hpp"
#include "autobot/world/SpatialHash.hpp"

#include <cstddef>
#include <vector>

namespace autobot::world {

class CollisionWorld final {
public:
    bool build(StaticWorld const& source, double parseTimeMs);

    [[nodiscard]] bool ready() const { return m_ready; }
    [[nodiscard]] std::vector<CollisionPrimitive> const& primitives() const { return m_primitives; }
    [[nodiscard]] std::vector<ClassificationAuditEntry> const& classificationAudit() const { return m_audit; }
    [[nodiscard]] std::vector<RawClassificationAuditEntry> const& rawClassificationAudit() const { return m_rawAudit; }
    [[nodiscard]] CollisionWorldMetrics const& metrics() const { return m_metrics; }

    [[nodiscard]] CollisionQueryResult queryRegion(WorldRect const& region) const;
    [[nodiscard]] CollisionQueryResult queryAhead(
        float playerX,
        float playerY,
        float width = 720.0f,
        float height = 360.0f
    ) const;

    [[nodiscard]] std::size_t primitiveForSource(std::size_t sourceIndex) const;
    [[nodiscard]] bool primitiveIndexed(std::size_t primitiveIndex) const;
    [[nodiscard]] std::vector<SpatialCell> const& cellsForPrimitive(std::size_t primitiveIndex) const;
    [[nodiscard]] std::vector<std::size_t> const& dynamicWatchPrimitiveIndices() const { return m_dynamicWatchPrimitiveIndices; }
    [[nodiscard]] std::uint64_t spatialHashGeneration() const { return m_spatialHash.generation(); }

    bool updatePrimitiveFromWorldObject(
        std::size_t sourceIndex,
        WorldObject const& liveObject,
        bool& reindexed
    );

private:
    static CollisionPrimitive makePrimitive(WorldObject const& object, std::size_t sourceIndex);
    static QueryBenchmark benchmarkQueries(
        SpatialHash const& index,
        std::vector<CollisionPrimitive> const& primitives
    );
    static std::vector<ClassificationAuditEntry> buildClassificationAudit(StaticWorld const& source);
    static std::vector<RawClassificationAuditEntry> buildRawClassificationAudit(StaticWorld const& source);

    bool m_ready = false;
    std::vector<CollisionPrimitive> m_primitives;
    SpatialHash m_spatialHash{120.0f};
    std::vector<std::size_t> m_sourceToPrimitive;
    std::vector<std::size_t> m_dynamicWatchPrimitiveIndices;
    std::vector<ClassificationAuditEntry> m_audit;
    std::vector<RawClassificationAuditEntry> m_rawAudit;
    CollisionWorldMetrics m_metrics{};
};

} // namespace autobot::world
