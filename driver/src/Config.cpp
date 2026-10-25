#include "Config.hpp"

#include <CoreFoundation/CoreFoundation.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace wingsim {

namespace {

bool ReadFile(const char* path, std::vector<uint8_t>& out)
{
    FILE* f = std::fopen(path, "rb");
    if (!f) {
        return false;
    }
    uint8_t buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0 && out.size() < (1u << 20)) {
        out.insert(out.end(), buf, buf + n);
    }
    std::fclose(f);
    return true;
}

// Integer value (CFNumber, or a CFString holding a decimal number).
bool ToInt(CFTypeRef v, long long& out)
{
    if (CFGetTypeID(v) == CFNumberGetTypeID()) {
        return CFNumberGetValue(static_cast<CFNumberRef>(v), kCFNumberLongLongType, &out);
    }
    if (CFGetTypeID(v) == CFStringGetTypeID()) {
        char s[32] = {};
        if (!CFStringGetCString(static_cast<CFStringRef>(v), s, sizeof s, kCFStringEncodingUTF8)) {
            return false;
        }
        char* end = nullptr;
        out = std::strtoll(s, &end, 10);
        return end != s && *end == '\0';
    }
    return false;
}

} // namespace

DriverConfig LoadDriverConfig(uint32_t defaultInputDelay, uint32_t maxInputDelay)
{
    DriverConfig cfg;
    cfg.inputDelayFrames = defaultInputDelay;
    cfg.source = "default";

    const char* env = std::getenv("WINGSIM_CONFIG");
    const char* path = (env && *env) ? env : WINGSIM_CONFIG_PATH;

    std::vector<uint8_t> bytes;
    if (!ReadFile(path, bytes)) {
        return cfg; // no config file: defaults (the normal case)
    }
    CFDataRef data = CFDataCreate(nullptr, bytes.data(), static_cast<CFIndex>(bytes.size()));
    CFPropertyListRef plist =
        CFPropertyListCreateWithData(nullptr, data, kCFPropertyListImmutable, nullptr, nullptr);
    CFRelease(data);
    if (!plist || CFGetTypeID(plist) != CFDictionaryGetTypeID()) {
        cfg.warning = std::string(path) + ": not a property list dictionary, using defaults";
        if (plist) {
            CFRelease(plist);
        }
        return cfg;
    }
    cfg.source = path;
    const auto dict = static_cast<CFDictionaryRef>(plist);
    if (CFTypeRef v = CFDictionaryGetValue(dict, CFSTR("InputDelayFrames"))) {
        long long n = 0;
        if (!ToInt(v, n) || n < 0) {
            cfg.warning = "InputDelayFrames: not a non-negative integer, using default";
        } else if (n > static_cast<long long>(maxInputDelay)) {
            cfg.inputDelayFrames = maxInputDelay;
            cfg.warning = "InputDelayFrames " + std::to_string(n) + " clamped to " +
                          std::to_string(maxInputDelay);
        } else {
            cfg.inputDelayFrames = static_cast<uint32_t>(n);
        }
    }
    CFRelease(plist);
    return cfg;
}

} // namespace wingsim
