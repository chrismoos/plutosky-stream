#pragma once

#include <cstdint>

namespace PlutoSky {

// Status events arrive once per packet's worth of time, so a starved or
// overflowing stream would log hundreds of lines a second without a limit.
class StatusLogLimiter {
public:
    explicit StatusLogLimiter(int64_t intervalMs) : intervalMs_(intervalMs) {}

    // True when a line is due. folded is how many reports were skipped since
    // the last line.
    bool due(int64_t nowMs, uint64_t &folded)
    {
        if (logged_ && nowMs - lastMs_ < intervalMs_)
        {
            ++skipped_;
            return false;
        }
        folded = skipped_;
        skipped_ = 0;
        logged_ = true;
        lastMs_ = nowMs;
        return true;
    }

private:
    int64_t intervalMs_;
    int64_t lastMs_ = 0;
    uint64_t skipped_ = 0;
    bool logged_ = false;
};

// A running transmit stream whose client has stopped writing keeps reporting
// underruns until it is stopped, which is what happens while a client such as
// osmo-trx shuts down.
inline bool txStarved(int64_t nowMs, int64_t lastWriteMs, bool underrunGrew, int64_t delayMs)
{
    return underrunGrew && lastWriteMs != 0 && nowMs - lastWriteMs >= delayMs;
}

}  // namespace PlutoSky
