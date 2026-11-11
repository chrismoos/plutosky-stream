#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace PlutoSky {

// Limits UDP packet bursts independently of the board's outstanding-packet window.
class TxPacer {
public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    static constexpr size_t kBurstPackets = 1;
    static constexpr double kDefaultTxOnlyRateLimitSps = 36000000.0;
    // Writers that hand over a batch at a time need headroom above the sample
    // rate to catch up after a slow batch, so the default is not tied to it.
    static constexpr double kDefaultDuplexRateLimitSps = 10000000.0;
    static constexpr double kRateLimitUnset = 0.0;

    // configured is kRateLimitUnset or a value from parseRateLimit.
    static double duplexRateLimit(double configured, double sampleRate)
    {
        if (configured == kRateLimitUnset) return std::max(kDefaultDuplexRateLimitSps, sampleRate);
        return std::max(configured, sampleRate);
    }

    static double parseRateLimit(const std::string &value, const char *name)
    {
        double limit = 0.0;
        size_t parsed = 0;
        try { limit = std::stod(value, &parsed); }
        catch (const std::exception &)
        {
            throw std::runtime_error(std::string("plutosky: invalid ") + name);
        }
        if (parsed != value.size() || !std::isfinite(limit) || limit < 1.0)
        {
            throw std::runtime_error(std::string("plutosky: ") + name + " must be at least 1");
        }
        return limit;
    }

    Time readyAt(Time now) const
    {
        return remaining_ == 0 ? std::max(now, next_) : now;
    }

    void sent(Time now, size_t samples, double rateLimitSps)
    {
        const auto interval = std::chrono::nanoseconds(static_cast<long long>(
            std::ceil(samples * kNanosecondsPerSecond / rateLimitSps)));
        if (remaining_ == 0)
        {
            remaining_ = kBurstPackets;
            // Preserve ordinary packet cadence without catching up after a long pause.
            if (next_ == Time{} || now > next_ + interval / 2) next_ = now;
        }
        --remaining_;
        next_ += std::chrono::duration_cast<Clock::duration>(interval);
    }

    void reset()
    {
        remaining_ = 0;
        next_ = Time{};
    }

private:
    static constexpr double kNanosecondsPerSecond = 1e9;
    size_t remaining_ = 0;
    Time next_{};
};

}
