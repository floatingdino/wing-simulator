// Driver configuration (non-realtime, read once when the plug-in loads).
//
// Source: the property list WINGSIM_CONFIG_PATH (or $WINGSIM_CONFIG, used by the
// tests). /Library/Preferences/Audio is the only preferences directory the
// coreaudiod sandbox lets a plug-in read, and the sandbox does not allow
// cfprefsd lookups for foreign domains, so the file is read directly. Set it with:
//
//   sudo defaults write /Library/Preferences/Audio/com.wingsimulator.driver \
//        InputDelayFrames -int 128
//   sudo launchctl kickstart -k system/com.apple.audio.coreaudiod
//
// Keys:
//   InputDelayFrames  frames the input is delayed behind the output so the
//                     daemon has time to process (reported as input latency).
#pragma once

#include <cstdint>
#include <string>

namespace wingsim {

#define WINGSIM_CONFIG_PATH "/Library/Preferences/Audio/com.wingsimulator.driver.plist"

struct DriverConfig
{
    uint32_t inputDelayFrames = 0;
    std::string source;  // file the values came from, or "default"
    std::string warning; // non-empty if the file was unreadable / a value was rejected
};

// defaultInputDelay is used when the key is absent; values above maxInputDelay
// are clamped (with a warning).
DriverConfig LoadDriverConfig(uint32_t defaultInputDelay, uint32_t maxInputDelay);

} // namespace wingsim
