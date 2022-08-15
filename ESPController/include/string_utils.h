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

/// Left pads with zeros an unsigned integer
///
/// @param zeros Required length of output string
/// @param number Number to be converted
static inline std::string pad_zero(uint16_t zeros, uint16_t number)
{
  std::string n;
  n.reserve(5);
  n.append(std::to_string(number));
  return std::string(zeros - min(zeros, (uint16_t)(n.length())), '0').append(n);
}

/// Returns 8 character binary representation of a uint8
///
/// @param number Number to be converted
static std::string uint8_to_binary_string(uint8_t number)
{
  std::string n;
  n.reserve(10);
  uint8_t bit = B10000000;
  for (size_t i = 0; i < 8; i++)
  {
    n.append((number && bit) ? "1" : "0");
    bit = bit >> 1;
  }
  return n;
}

#endif // stringutils_H_