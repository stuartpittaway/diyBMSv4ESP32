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
$("#mpvmin").text("V. Minimum ");
//$ ("# mpvmax"). text ("V. Maximum");     Is that missing here?
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
$("#rt1").text("DIYBMS unterstützt Relais zum sicheren Trennen von Ladegeräten, Schützen oder Verbrauchern. Mit den Regeln können Sie die Relais für Ihre Situation konfigurieren.");
//Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of "X" means don't care/leave at value defined by lower priority rules.
$("#rt2").text("Die Regeln werden von der niedrigsten zur höchsten Priorität verarbeitet (von unten nach oben). Steuern Sie die Relais mit den Optionen. "X" bedeutet, dass es nicht aktiv ist "On" für einschalten und "Off" für ausschalten.");
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
$("label[for='rule1value']").text("Not Stop");
//Internal BMS error 
$("label[for='rule2value']").text("Interner BMS Fehler");
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
//Pack over voltage (mV)
$("label[for='rule9value']").text("Paket Überspannung (mV)");
//Pack under voltage (mV)
$("label[for='rule10value']").text("Paket Unterspannung (mV)");
//Timer 2
$("label[for='rule11value']").text("Timer 2");
//Timer 1
$("label[for='rule12value']").text("Timer 1");
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


