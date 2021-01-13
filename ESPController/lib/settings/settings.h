#ifndef Settings_H // include guard
#define Settings_H

/*
Settings storage with checksum (works with ESP32 devices)
(c)2021 Stuart Pittaway

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your
  contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures
  that legally restrict others from doing anything the license permits.

*/

#include <Preferences.h>
#include "crc16.h"

class Settings
{
public:
  static void WriteConfig(const char *tag, char *settings, int size);
  static bool ReadConfig(const char *tag, char *settings, int size);
  static void FactoryDefault(const char *tag);
};
#endif
