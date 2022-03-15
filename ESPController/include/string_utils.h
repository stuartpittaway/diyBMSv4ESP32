#ifndef stringutils_H_
#define stringutils_H_

#pragma once

#include <string>

/// Utility function to convert a floating point value to a string with four decimal point precision.
///
/// @param value Floating point value to be converted to a string.
static inline std::string float_to_string(float value)
{
    char temp_buf[16] = {};
    snprintf(temp_buf, sizeof(temp_buf), "%.4f", value);
    return std::string(temp_buf);
}

#endif // stringutils_H_