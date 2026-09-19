#include <Geode/Geode.hpp>
#include <Geode/modify/MenuLayer.hpp>

#include "autobot/update/UpdateManager.hpp"

using namespace geode::prelude;

class $modify(AutoBotT7UpdateMenuHook, MenuLayer) {
    bool init() {
        if (!MenuLayer::init()) return false;
        autobot::update::UpdateManager::get().checkOnce();
        return true;
    }
};
