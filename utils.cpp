#include <iomanip>
#include <chrono>

#include "utils.h"

std::string getTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto nowAsTimeT = std::chrono::system_clock::to_time_t(now);
    const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    std::stringstream nowSs;
    nowSs << std::put_time(std::localtime(&nowAsTimeT), "%F.%T")
            << '.' << std::setfill('0') << std::setw(3) << nowMs.count();
    return nowSs.str();
}