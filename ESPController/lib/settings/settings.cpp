#include "settings.h"

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-set";

void Settings::WriteConfig(const char *tag, char *settings, int size)
{
  ESP_LOGD(TAG, "WriteConfig %s", tag);

  Preferences prefs;
  prefs.begin(tag);

  prefs.putBytes("bytes", settings, size);

  //Generate and save the checksum for the setting data block
  prefs.putUShort("checksum", CRC16::CalculateArray((uint8_t *)settings, size));

  prefs.end();
}

bool Settings::ReadConfig(const char *tag, char *settings, int size)
{
  ESP_LOGD(TAG, "ReadConfig %s", tag);

  Preferences prefs;

  prefs.begin(tag);


  size_t schLen = prefs.getBytesLength("bytes");

  if (schLen == size)
  {

    prefs.getBytes("bytes", settings, schLen);

    // Calculate the checksum
    uint16_t checksum = CRC16::CalculateArray((uint8_t *)settings, size);
    uint16_t existingChecksum = prefs.getUShort("checksum");
    prefs.end();

    if (checksum == existingChecksum)
    {
      //Return TRUE
      ESP_LOGD(TAG, "checksum verified");
      return true;
    }
  }

  prefs.end();
  //Wrong size/checksum
  return false;
}

void Settings::FactoryDefault(const char *tag)
{
  ESP_LOGI(TAG, "FactoryDefault %s", tag);
  Preferences prefs;
  prefs.begin(tag);
  prefs.clear();
  //prefs.remove("bytes");
  //prefs.remove("checksum");
  prefs.end();
}
