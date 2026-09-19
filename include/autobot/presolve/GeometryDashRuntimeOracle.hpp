#pragma once

#include "autobot/presolve/UniversalSearchCore.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

class PlayLayer;

namespace autobot::presolve {

struct RuntimeOracleValidation {
    std::size_t roundTripChecks = 0;
    std::size_t effectStateChecks = 0;
    std::size_t rngChecks = 0;
    std::size_t dynamicWorldChecks = 0;
    std::size_t effectTransitionsObserved = 0;
    std::size_t rngTransitionsObserved = 0;
    std::size_t dynamicTransitionsObserved = 0;
    bool roundTripPass = true;
    bool effectStatePass = true;
    bool rngPass = true;
    bool dynamicWorldPass = true;
};

class GeometryDashRuntimeOracle final : public IUniversalStateOracle {
public:
    using StepCallback = std::function<void(float)>;

    explicit GeometryDashRuntimeOracle(PlayLayer* layer, double stepDt = 1.0 / 240.0);
    ~GeometryDashRuntimeOracle() override;

    GeometryDashRuntimeOracle(GeometryDashRuntimeOracle const&) = delete;
    GeometryDashRuntimeOracle& operator=(GeometryDashRuntimeOracle const&) = delete;

    void setStepCallback(StepCallback callback);
    void setStepDt(double value);
    [[nodiscard]] double stepDt() const;
    [[nodiscard]] std::string const& lastError() const;
    [[nodiscard]] RuntimeOracleValidation validation() const;

    [[nodiscard]] UniversalObservation observe() const override;
    [[nodiscard]] std::optional<UniversalToken> capture() override;
    bool restore(UniversalToken token) override;
    [[nodiscard]] UniversalObservation step(UniversalAction action) override;
    void discard(UniversalToken token) override;
    [[nodiscard]] std::optional<UniversalCanonicalState> canonicalState(UniversalToken token) const override;
    [[nodiscard]] std::uint64_t decisionEpoch() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace autobot::presolve
