#include "autobot/control/InputController.hpp"

#include <cassert>
#include <iostream>

using namespace autobot::control;

int main() {
    auto press = InputController::transition(false, InputAction::Press);
    assert(press.emitPress);
    assert(!press.emitRelease);
    assert(press.nextHolding);
    assert(press.effectiveAction == InputAction::Press);

    auto hold = InputController::transition(true, InputAction::Hold);
    assert(!hold.emitPress);
    assert(!hold.emitRelease);
    assert(hold.nextHolding);

    auto repeatedPress = InputController::transition(true, InputAction::Press);
    assert(!repeatedPress.emitPress);
    assert(!repeatedPress.emitRelease);
    assert(repeatedPress.nextHolding);
    assert(repeatedPress.effectiveAction == InputAction::Hold);

    auto release = InputController::transition(true, InputAction::Release);
    assert(!release.emitPress);
    assert(release.emitRelease);
    assert(!release.nextHolding);

    auto safeStop = InputController::transition(true, InputAction::SafeStop);
    assert(!safeStop.emitPress);
    assert(safeStop.emitRelease);
    assert(!safeStop.nextHolding);

    auto safeStopIdle = InputController::transition(false, InputAction::SafeStop);
    assert(!safeStopIdle.emitPress);
    assert(!safeStopIdle.emitRelease);
    assert(!safeStopIdle.nextHolding);

    std::cout << "INPUT_CONTROLLER_STATE_MACHINE=PASS\n";
    return 0;
}
