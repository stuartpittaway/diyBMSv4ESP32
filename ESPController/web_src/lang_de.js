// This file is for German language translation
// https://www.w3schools.com/tags/ref_language_codes.asp

$("#home").text("Home");
$("#modules").text("Module");
$("#settings").text("Einstellungen");
$("#rules").text("Regeln");
$("#more").text("Mehr");

$("#integration").text("Einbindung");
$("#currentmonitor").text("DIYBMS Amperemeter");
$("#storage").text("Speicher");
$("#avrprogrammer").text("AVR-Programmierer");
$("#about").text("Informationen");

$("#modulesPage > h1").text("Modul Seiten");

$("#globalConfig > h2").text("Globale Einstellungen");
$("#globalConfig > p").text("Konfigurieren Sie alle Module so, dass sie die folgenden Parameter verwenden:");

$("#mpBank").text("Gruppe");
$("#mpModule").text("Modul");
$("#mpVoltage").text("Volt");
$("#mpvmin").text("V. Minimum");
$("#mpvmax").text("V. Maximum");
$("#mptint").text("Temp intern °C");
$("#mptext").text("Temp extern °C");
$("#mpbypass").text("Bypass %");
$("#mpbpc").text("Falsche Paketanzahl");
$("#mppktr").text("Erhaltene Pakete");
$("#mpbal").text("Verbrauchte Ausgleichsenergie (mAh)");

$("#error1").text("Der Controller hat Probleme mit der Kommunikation der Überwachungsmodule.");
$("#error3").text("Der Controller ist so konfiguriert, dass mehr Module verwendet werden, als er unterstützen kann.");
$("#error4").text("Warten auf die Antwort der Module");
$("#error5").text("Das Modul hat NULL-Volt gelesen. Überprüfen Sie die Konfiguration");
$("#error6").text("Der Controller hat nicht genügend Speicher.");
$("#error7").text("Not-Stop");
$("#iperror").text("Kommunikation mit dem Controller für Statusaktualisierungen nicht möglich.");
$("#jslibrary").text("Die JavaScript-Bibliothek wurde nicht korrekt geladen. Bitte aktualisieren Sie die Seite.");
$("#saveerror").text("Die Einstellungen konnte nicht gespeichert werden.");
$("#savesuccess").text("Einstellungen gespeichert");

$("#warning1").text("Warnung: Die Bypass-Spannung der Module unterscheidet sich von der globalen Einstellung");
$("#warning2").text("Warnung: Die Bypass-Temperatur der Module unterscheidet sich von der globalen Einstellung");
$("#warning3").text("Warnung: Module haben verschiedene Codeversionen, das könnte zu Instabilität führen");
$("#warning4").text("Warnung: Module haben verschiedene Hardware oder Board-Versionen");
$("#warning5").text("Warnung: Prtokollierung aktiviert, SD-Karte jedoch nicht installiert oder gefunden");
$("#warning6").text("Einige Funktionen sind deaktiviert, während der AVR-Programmiermodus aktiviert ist");

$("label[for='NTPServer']").text("NTP Server");
$("label[for='NTPZoneHour']").text("Zeitzone (Stunden)");




//DIYBMS supports relay modules to safely disconnect chargers, contactors or consumers. The rules allow you to configure the relays for your situation.
$("#rt1").text("DIYBMS unterstützt Relais zum sicheren Trennen von Ladegeräten, Schützen oder Verbrauchern. Mit den Regeln können Sie die Relais für Ihre Situation konfigurieren.");
//Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of "X" means don't care/leave at value defined by lower priority rules.
$("#rt2").text("Die Regeln werden von der niedrigsten zur höchsten Priorität verarbeitet (von unten nach oben). Steuern Sie die Relais mit den Optionen. 'X' bedeutet, dass es nicht aktiv ist 'On' für einschalten und 'Off' für ausschalten.");
//Rules are triggered when the relevant value meets or exceeds the 'trigger' value. The rule will only disable when the value then passes the 'reset' value. This can help prevent relay clatter and rules firing on/off rapidly.
$("#rt3").text("Die Regeln werden aktiviert, wenn der gemessene Wert den Auslösewert erreicht oder überschreitet. Die Regel wird nur dann deaktiviert, wenn der Wert den Resetwert überschreitet. Dies kann dazu beitragen, das Klappern von Relais und das schnelle Ein- und Ausschalten von Regeln zu verhindern.");
//'Timer 1' and 'Timer 2' rules allow timed operation, this rule is active when the number of minutes past midnight has been reached, for instance setting 'Timer 1' trigger to 495 and reset to 555 would switch on at 8:15am and off at 9:15am. This only works if connected to internet for regular time updates.
$("#rt4").text("Die Regeln 'Timer 1' und 'Timer 2' ermöglichen einen zeitgesteuerten Betrieb. Diese Regel ist aktiv, wenn die Anzahl der Minuten nach Mitternacht (00:00Uhr) erreicht wurde. Wenn Sie beispielsweise den Auslöser 'Timer 1' auf 495 setzen und 'Timer 2' auf 555 zurücksetzen, wird dies um 8:15 Uhr eingeschaltet und um 9:15 Uhr ab. Dies funktioniert nur, wenn Sie für regelmäßige Zeitaktualisierungen mit dem Internet verbunden sind.");
//Minutes since midnight now is: 
$("#rt5").text("Zeit nach Mitternacht in Minuten: ");
//Emergency stop is triggered by connector J1, once triggered controller needs to be reset to disable.
$("#rt6").text("Der Not-Stop wird durch Stecker J1 ausgelöst, einmal ausgelöst muss der Controller resettet werden, um sie zu deaktivieren.");


//Rule
$("#rf1").text("Regel");
//Trigger value
$("#rf2").text("Auslösewert");
//Reset value
$("#rf3").text("Resetwert");
//Relay state
$("#rf4").text("Relaisstatus");


//Emergency Stop
$("label[for='rule0value']").text("Not-Aus");
//Internal BMS error 
$("label[for='rule1value']").text("Interner fehler im BMS");
//Current monitoring over current (Amps)
$("label[for='rule2value']").text("maximaler Strom (A)");
//Individual cell over voltage (mV)
$("label[for='rule3value']").text("Zellenüberspannung (mV)");
//Cell under voltage (mV)
$("label[for='rule4value']").text("Zellenunterspannung (mV)");
//Module over temperature (internal) °C
$("label[for='rule5value']").text("Modul Übertemperatur (intern) °C");
//Module under temperature (internal)
$("label[for='rule6value']").text("Modul Untertemperatur (intern)");
//Cell over temperature (external)
$("label[for='rule7value']").text("Zellen Übertemperatur (extern)");
//Cell under temperature (external)
$("label[for='rule8value']").text("Zellen Untertemperatur (extern)");
//Bank over voltage (mV)
$("label[for='rule9value']").text("Überspannung des Strommonitors (mV)");
//Bank under voltage (mV)
$("label[for='rule10value']").text("Stromwächter unter Spannung (mV)");
//Bank over voltage (mV)
$("label[for='rule11value'").text("Paket Überspannung (mV)");
//Bank under voltage (mV)
$("label[for='rule12value'").text("Paket Unterspannung (mV)");
//Bank range/deviation (mV)
$("label[for='rule13value'").text("Bankbereich/Varianz (mV)");
//Timer 2
$("label[for='rule14value']").text("Timer 2");
//Timer 1
$("label[for='rule15value']").text("Timer 1");
//Relay default
$("label[for='defaultvalue']").text("Relais Grundeinstellung");
//Relay type
$("label[for='relaytype']").text("Relais Typen");


//Bypass over temperature
$("label[for='g1']").text("Bypass Übertemperatur");
//Bypass threshold mV
$("label[for='g2']").text("Bypass-Schwelle mV");

//Packets sent
$("#sent > span.x.t").text("Pakete gesendet:");
//Packets rec'd
$("#received > span.x.t").text("Pakete empfangen:");
//Roundtrip (ms)
$("#roundtrip > span.x.t").text("Rundenzeit (ms):");

//Integration
$("#ip1").text("Einbindung");
//For security, you will need to re-enter the password for the service(s) you want to enable or modify, before you save.
$("#ip2").text("Aus Sicherheitsgründen müssen Sie vor dem Speichern das Kennwort für die Dienste, die Sie aktivieren oder ändern möchten, erneut eingeben.");
//After changes are made, the controller will need to be rebooted, do this manually.
$("#ip3").text("Nachdem Änderungen vorgenommen wurden, muss der Controller neu gestartet werden. Führen Sie dieses manuell durch.");

//PATREON
//Remember, this product is free for personal use, if you would like to make a regular donation to keep the features and improvements flowing, use the Patreon link below. Even just a coffee/beer a month makes a difference. Thank you!
$("#ap4").text("Denken Sie daran, dass dieses Produkt für den persönlichen Gebrauch kostenlos ist. Wenn Sie regelmäßig spenden möchten, um die Funktionen und Verbesserungen aufrechtzuerhalten, verwenden Sie den unten stehenden Patreon-Link. Schon ein Kaffee / Bier im Monat macht den Unterschied. Vielen Dank!");

//WARNING
$("#ap5").text("WARNUNG");
//This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.
$("#ap6").text("Dies ist ein DIY-Produkt/DIY-Lösung. Verwenden Sie dieses Produkt/Lösung daher nicht für sicherheitskritische Systeme oder in Situationen, in denen Lebensgefahr besteht.");
//There is no warranty, it may not work as expected or at all.
$("#ap7").text("Es gibt keine Garantie oder Gewährleistung. Es funktioniert möglicherweise nicht wie erwartet oder überhaupt nicht..");
//The use of this project is done so entirely at your own risk. It may involve electrical voltages which could kill - if in doubt, seek help.
$("#ap8").text("Die Nutzung dieses Projekts erfolgt ausschließlich auf eigene Gefahr und eigenes Risiko. Es kann sich um elektrische Spannungen handeln, die tödlich sein können - im Zweifelsfall bitten sie um Hilfe..");
//The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.
$("#ap8a").text("Die Verwendung dieses Projekts entspricht möglicherweise nicht den örtlichen Gesetzen oder Vorschriften. Fragen Sie im Zweifelsfall nach.");

//Modules & Banks
$("#mb1").text("Module & Banken");
//DIYBMS supports up to 100 modules in total. These modules can be split into banks to support parallel configurations.
$("#mb2").text("DIYBMS unterstützt insgesamt bis zu 100 Module. Diese Module können in Bänke aufgeteilt werden, um parallele Konfigurationen zu unterstützen.");
//Example: You have 16 cells configured as 8 in series and 2 in parallel (8S2P).
$("#mb3").text("Beispiel: Sie haben 16 Zellen, die als 8 in einer Reihe und 2 parallel zusammengestellt sind (8S2P)..");

//diybmsCurrentMonitorPage

//<h1>diyBMS Current &amp; Voltage Monitor</h1>
$("#diybmsCurrentMonitorPage > h1").text("diyBMS Ampere & Spannungsüberwachung");
//<p>Configure the MODBUS connection to the current monitor using the settings below.</p>
//Konfigurieren Sie die MODBUS-Verbindung zum aktuellen Monitor mit den folgenden Einstellungen.

//<label for="CurrentMonEnabled">Enabled</label>
$("label[for='CurrentMonEnabled']").text("aktiviert");

//<p>Configuration options for RS485 interface. Communication is half-duplex.</p>
//Konfigurationsoptionen für die RS485-Schnittstelle. Die Kommunikation erfolgt im Halbduplexbereich.

//<h2>Basic Settings</h2>
//Grundeinstellungen

//<p>Ensure the current shunt parameters match the data sheet for your particular shunt resistor.</p>
//Stellen Sie sicher, dass die aktuellen Shunt-Parameter mit dem Datenblatt für Ihren speziellen Shunt-Widerstand übereinstimmen.

//<p>The current monitor uses a 40.96mV maximum scale, so shunt voltages over this will be scaled down proportionally.</p>
//Der aktuelle Monitor verwendet eine maximale Skalierung von 40,96 mV, sodass die darüber liegenden Shuntspannungen proportional verkleinert werden.

//<label for="shuntmaxcur">Shunt maximum current</label>
$("label[for='shuntmaxcur']").text("Shunt maximaler Strom");

//<label for="shuntmv">Shunt output voltage (mV)</label>
$("label[for='shuntmv']").text("Shunt-Ausgangsspannung (mV)");

//<label for="cmvalid">Values valid?</label>
$("label[for='cmvalid']").text("Werte gültig?");

//<label for="cmtimestampage">Last communication (milliseconds)</label>
$("label[for='cmtimestampage']").text("Letzte Verbindung (Millisekunden)");

//<label for="cmwatchdog">Watchdog counter</label>
$("label[for='cmwatchdog']").text("Watchdog-Zähler");

//<label for="cmtemperature">Die temperature &deg;C</label>
$("label[for='cmtemperature']").text("CPU-Kern Temperatur");

//<label for="cmactualshuntmv">Actual shunt mV reading</label>
$("label[for='cmactualshuntmv']").text("Tatsächlicher Shunt-MV-Messwert");

//<label for="cmcurrentlsb">Current LSB size</label>
$("label[for='cmcurrentlsb']").text("Aktuelle LSB-Größe");

//<label for="cmresistance">Shunt resistance</label>
$("label[for='cmresistance']").text("Shunt-Widerstand");

//<label for="cmmodel">Sensor model</label>
$("label[for='cmmodel']").text("Sensormodell");

//<label for="cmfirmwarev">Firmware version</label>
$("label[for='cmfirmwarev']").text("Firmware version");

//<label for="cmfirmwaredate">Firmware date</label>
$("label[for='cmfirmwaredate']").text("Firmware Datum");

//<label for="cmRelayState">Relay state</label>
$("label[for='cmRelayState']").text("Relaisstatus");

//<label for="cmTemperatureOverLimit">Temperature over limit</label>
$("label[for='cmTemperatureOverLimit']").text("Temperatur über dem Grenzwert");

//<label for="cmCurrentOverLimit">Current over limit</label>
$("label[for='cmCurrentOverLimit']").text("Strom über dem Grenzwert");

//<label for="cmCurrentUnderLimit">Current under limit</label>
$("label[for='cmCurrentUnderLimit']").text("Strom unter dem Grenzwert");

//<label for="cmVoltageOverLimit">Voltage over limit</label>
$("label[for='cmVoltageOverLimit']").text("Spannung über dem Grenzwert");

//<label for="cmVoltageUnderLimit">Voltage under limit</label>
$("label[for='cmVoltageUnderLimit']").text("Spannung unter dem Grenzwert");

//<label for="cmPowerOverLimit">Power over limit</label>
$("label[for='cmPowerOverLimit']").text("Leistungsgrenze");


//<h2>Advanced Current Monitor Settings</h2>
//Erweiterte Stromüberwachungs einstellungen

//<p>If you wish to use the relay control on the shunt monitor, set the parameters here. <u>You should not normally need to change the calibration value.</u></p>
//Wenn Sie die Relaissteuerung am Shunt-Monitor verwenden möchten, stellen Sie hier die Parameter ein. Normalerweise sollten Sie den Kalibrierungswert nicht ändern müssen.

//<p>Current limit is for use whilst discharging the battery, under current limit is used when charging, so different discharge/charge current limits can be used.</u></p>
// I cannot translate meaningfully

//<p>Temperature limit is based on the chip die temperature, which may not match the shunt temperature. Only positive temperature coefficient is supported.</u></p>
// Die Temperaturgrenze basiert auf der CPU-Kern Temperatur, die möglicherweise nicht mit der Shunt-Temperatur übereinstimmt. Es wird nur ein positiver Temperaturkoeffizient unterstützt.

//<p>Relay triggers define which rules are used to trigger the relay into a closed state.</u></p>
//Relaisauslöser definieren, welche Regeln verwendet werden, um das Relais in einen geschlossenen Zustand zu versetzen.

//<label for="cmcalibration">Calibration</label>
$("label[for='cmcalibration']").text("Kalibrierung");

//<label for="cmtemplimit">Temperature limit</label>
$("label[for='cmtemplimit']").text("Temperaturgrenze");

//<label for="cmundervlimit">Under voltage limit</label>
$("label[for='cmundervlimit']").text("Unterspannungsgrenze");

//<label for="cmovervlimit">Over voltage limit</label>
$("label[for='cmovervlimit']").text("Überspannungsgrenze");

//<label for="cmoverclimit">Over current limit</label>
$("label[for='cmoverclimit']").text("Überstrombegrenzung");

//<label for="cmunderclimit">Under current limit</label>
$("label[for='cmunderclimit']").text("Unterstrombegrenzung");

//<label for="cmoverplimit">Over power limit</label>
$("label[for='cmoverplimit']").text("Leistungsgrenze");

//<label for="cmtempcoeff">Temperature coefficient ppm/&deg;C</label>
$("label[for='cmtempcoeff']").text("Temperaturkoeffizient");

//<h2>Current Monitor Relay Settings &amp; Temperature coefficient</h2>
// Strom Überwachungsrelais Einstellungen und Temperaturkoeffizient

//Relay triggers define which rules are used to trigger the relay into a closed state.
//Relaisauslöser definieren, welche Regeln werden verwendet, um das Relais in einen geschlossenen Zustand zu versetzen.

//<label for="TempCompEnabled">Temperature coefficient enabled</label>
$("label[for='TempCompEnabled']").text("Temperaturkoeffizient aktiviert");

//<label for="cmTMPOL">Relay Trigger: Temperature</label>
$("label[for='cmTMPOL']").text("Relaisauslöser: Temperatur");

//<label for="cmCURROL">Relay Trigger: Current over</label>
$("label[for='cmCURROL']").text("Relaisauslöser: Überstrom");

//<label for="cmCURRUL">Relay Trigger: Current under</label>
$("label[for='cmCURRUL']").text("Relaisauslöser: Unterstrom");

//<label for="cmVOLTOL">Relay Trigger: Voltage over</label>
$("label[for='cmVOLTOL']").text("Relaisauslöser: Überspannung");

//<label for="cmVOLTUL">Relay Trigger: Voltage under</label>
$("label[for='cmVOLTUL']").text("Relaisauslöser: Unterspannung");

//<label for="cmPOL">Relay Trigger: Power</label>
$("label[for='cmPOL']").text("Relaisauslöser: Stromversorgung");
