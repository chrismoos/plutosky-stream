#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace PlutoSky {

inline long long integerHz(double value, const char *name)
{
    if (!std::isfinite(value) || value < 0.0 ||
        value >= static_cast<double>(std::numeric_limits<long long>::max()))
    {
        throw std::invalid_argument(std::string("plutosky: ") + name +
                                    " must be a finite, non-negative number of Hz");
    }
    return std::llround(value);
}

}
