#include "defines.h"
#include "SoftAP.h"

wifi_eeprom_settings DIYBMSSoftAP::_config;

String DIYBMSSoftAP::networks;

AsyncWebServer * DIYBMSSoftAP::_myserver;

char* DIYBMSSoftAP::WifiSSID() {
  return _config.wifi_ssid;
}

char* DIYBMSSoftAP::WifiPassword() {
  return _config.wifi_passphrase;
}

void DIYBMSSoftAP::handleRoot(AsyncWebServerRequest *request)
{
  String s;
  s += DIYBMSSoftAP::networks;
  request->send(200, "text/html", s);
}

void DIYBMSSoftAP::handleSave(AsyncWebServerRequest *request) {
  String s;
  String ssid = request->arg("ssid");
  String password = request->arg("pass");

  if ((ssid.length() <= sizeof(_config.wifi_ssid)) && (password.length() <= sizeof(_config.wifi_passphrase))) {

    memset(&_config, 0, sizeof(_config));

    ssid.toCharArray(_config.wifi_ssid, sizeof(_config.wifi_ssid));
    password.toCharArray(_config.wifi_passphrase, sizeof(_config.wifi_passphrase));

    Settings::WriteConfigToEEPROM((char*)&_config, sizeof(_config), EEPROM_WIFI_START_ADDRESS);

    s = F("<p>WIFI settings saved, will reboot in 5 seconds.</p>");

    request->send(200, "text/html", s);

    //Delay 6 seconds
    for (size_t i = 0; i < 60; i++) { delay(100); }

    ESP.restart();

  } else {
    s =  F("<p>WIFI settings too long.</p>");
    request->send(200, "text/html", s);
  }
}

bool DIYBMSSoftAP::LoadConfigFromEEPROM() {
   return (Settings::ReadConfigFromEEPROM((char*)&_config, sizeof(_config), EEPROM_WIFI_START_ADDRESS));
}

void DIYBMSSoftAP::SetupAccessPoint(AsyncWebServer  *webserver) {
  _myserver=webserver;
  const char* ssid = "DIY_BMS_CONTROLLER";

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(100);
  int n = WiFi.scanNetworks();

  if (n == 0)
    DIYBMSSoftAP::networks = "no networks found";
  else
  {
    DIYBMSSoftAP::networks="";
    for (int i = 0; i < n; ++i)
    {
        DIYBMSSoftAP::networks += "<option>"+WiFi.SSID(i)+"</option>";
        SERIAL_DEBUG.println(WiFi.SSID(i));
        delay(5);
    }
  }

  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(ssid);

  _myserver->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {    
    request->redirect("/softap.htm");
  });

  _myserver->serveStatic("/softap.htm", SPIFFS, "/softap/softap.htm").setTemplateProcessor(DIYBMSSoftAP::TemplateProcessor);
  _myserver->on("/save", HTTP_POST, handleSave);
  //_myserver->onNotFound(handleNotFound);
  _myserver->begin();

  IPAddress IP = WiFi.softAPIP();
  SERIAL_DEBUG.print("Access point IP address: ");
  SERIAL_DEBUG.println(IP);
}

String DIYBMSSoftAP::TemplateProcessor(const String& var)
{
  //SERIAL_DEBUG.print("TemplateProcessor: ");  SERIAL_DEBUG.println(var);

  if(var == "SSID")
    return DIYBMSSoftAP::networks;

  return String();
}
