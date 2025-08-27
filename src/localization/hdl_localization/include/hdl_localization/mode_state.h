#pragma once

namespace hdl_localization {

    enum class ModeState
    {
        INIT,
        PASSIVE,
        READY,
        ACTIVE,
        SUCCESS,
    };

    enum class LocalizationStatus : uint8_t {
        INITIALIZING = 0,
        RELOCALIZING = 1,
        RELOCALIZATION_SUCCESS = 2,
        NORMAL = 3,
        LOST = 4
    };
}  // namespace hdl_localization