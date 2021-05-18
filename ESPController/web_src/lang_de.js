// This file is for German language translation
// https://www.w3schools.com/tags/ref_language_codes.asp

$("#home").text("Home");
$("#modules").text("Module");
$("#settings").text("Einstellungen");
$("#rules").text("Regeln");
$("#more").text("mehr");

$("#integration").text("Einbindung");
$("#currentmonitor").text("diyBMS Amperemeter");
$("#storage").text("Speicher");
$("#avrprogrammer").text("AVR-Programmierer");
$("#about").text("Informationen");

$("#modulesPage > h1").text("Modul Seiten");

$("#globalConfig > h2").text("Globale Einstellungen");
$("#globalConfig > p").text("Konfigurieren Sie alle Module so, dass sie die folgenden Parameter verwenden:");

$("#mpBank").text("Gruppe");
$("#mpModule").text("Module");
$("#mpVoltage").text("Volt");
$("#mpvmin").text("V. Minimum ");
$("#mptint").text("Temp intern °C");
$("#mptext").text("Temp extern °C");
$("#mpbypass").text("Bypass %");
$("#mpbpc").text("Falsche Paketanzahl");
$("#mppktr").text("Erhaltene Pakete");
$("#mpbal").text("Gleichen Sie den Energieverbrauch aus (mAh)");

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
$("#rt1").text("DIYBMS supports relay modules to safely disconnect chargers, contactors or consumers. The rules allow you to configure the relays for your situation.");
//Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of "X" means don't care/leave at value defined by lower priority rules.
$("#rt2").text("Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of 'X' means don't care/leave at value defined by lower priority rules.");
//Rules are triggered when the relevant value meets or exceeds the 'trigger' value. The rule will only disable when the value then passes the 'reset' value. This can help prevent relay clatter and rules firing on/off rapidly.
$("#rt3").text("Rules are triggered when the relevant value meets or exceeds the 'trigger' value. The rule will only disable when the value then passes the 'reset' value. This can help prevent relay clatter and rules firing on/off rapidly.");
//'Timer 1' and 'Timer 2' rules allow timed operation, this rule is active when the number of minutes past midnight has been reached, for instance setting 'Timer 1' trigger to 495 and reset to 555 would switch on at 8:15am and off at 9:15am. This only works if connected to internet for regular time updates.
$("#rt4").text("Timer 1' and 'Timer 2' rules allow timed operation, this rule is active when the number of minutes past midnight has been reached, for instance setting 'Timer 1' trigger to 495 and reset to 555 would switch on at 8:15am and off at 9:15am. This only works if connected to internet for regular time updates.");
//Minutes since midnight now is: 
$("#rt5").text("Minutes since midnight now is: ");
//Emergency stop is triggered by connector J1, once triggered controller needs to be reset to disable.
$("#rt6").text("Emergency stop is triggered by connector J1, once triggered controller needs to be reset to disable.");


//Rule
$("#rf1").text("Rule");
//Trigger value
$("#rf2").text("Trigger value");
//Reset value
$("#rf3").text("Reset value");
//Relay state
$("#rf4").text("Relay state");


//Emergency Stop
$("label[for='rule1value']").text("Emergency Stop");
//Internal BMS error 
$("label[for='rule2value']").text("Internal BMS error");
//Individual cell over voltage (mV)
$("label[for='rule3value']").text("Individual cell over voltage (mV)");
//Cell under voltage (mV)
$("label[for='rule4value']").text("Cell under voltage (mV)");
//Module over temperature (internal) °C
$("label[for='rule5value']").text("Module over temperature (internal) °C");
//Module under temperature (internal)
$("label[for='rule6value']").text("Module under temperature (internal)");
//Cell over temperature (external)
$("label[for='rule7value']").text("Cell over temperature (external)");
//Cell under temperature (external)
$("label[for='rule8value']").text("Cell under temperature (external)");
//Pack over voltage (mV)
$("label[for='rule9value']").text("Pack over voltage (mV)");
//Pack under voltage (mV)
$("label[for='rule10value']").text("Pack under voltage (mV)");
//Timer 2
$("label[for='rule11value']").text("Timer 2");
//Timer 1
$("label[for='rule12value']").text("Timer 1");
//Relay default
$("label[for='defaultvalue']").text("Relay default");
//Relay type
$("label[for='relaytype']").text("Relay type");


//Bypass over temperature
$("label[for='g1']").text("Bypass over temperature");
//Bypass threshold mV
$("label[for='g2']").text("Bypass threshold mV");

//Packets sent
$("#sent > span.x.t").text("Packets sent:");
//Packets rec'd
$("#received > span.x.t").text("Packets rec'd:");
//Roundtrip (ms)
$("#roundtrip > span.x.t").text("Roundtrip (ms):");

//Integration
$("#ip1").text("Integration");
//For security, you will need to re-enter the password for the service(s) you want to enable or modify, before you save.
$("#ip2").text("For security, you will need to re-enter the password for the service(s) you want to enable or modify, before you save.");
//After changes are made, the controller will need to be rebooted, do this manually.
$("#ip3").text("After changes are made, the controller will need to be rebooted, do this manually.");

//WARNING
$("#ap5").text("WARNING");
//This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.
$("#ap6").text("This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.");
//There is no warranty, it may not work as expected or at all.
$("#ap7").text("There is no warranty, it may not work as expected or at all.");
//The use of this project is done so entirely at your own risk. It may involve electrical voltages which could kill - if in doubt, seek help.
$("#ap8").text("The use of this project is done so entirely at your own risk. It may involve electrical voltages which could kill - if in doubt, seek help.");
//The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.
$("#ap8a").text("The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.");

//Modules & Banks
$("#mb1").text("Modules & Banks");
//DIYBMS supports up to 100 modules in total. These modules can be split into banks to support parallel configurations.
$("#mb2").text("DIYBMS supports up to 100 modules in total. These modules can be split into banks to support parallel configurations.");
//Example: You have 16 cells configured as 8 in series and 2 in parallel (8S2P).
$("#mb3").text("Example: You have 16 cells configured as 8 in series and 2 in parallel (8S2P).");


