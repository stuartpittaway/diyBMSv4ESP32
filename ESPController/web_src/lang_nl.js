// This file is for Dutch language translation
// w3schools.com/tags/ref_language_codes.asp

$("#home").text("Home"); //For me is better not translate "Home"
$("#modules").text("Modules");
$("#settings").text("Instellingen");
$("#rules").text("Regels");
$("#more").text("Meer");

$("#integration").text("Integratie");
$("#currentmonitor").text("diyBMS-currentshunt");
$("#storage").text("opslag");
$("#avrprogrammer").text("AVR programmeermodule");
$("#about").text("Info");

$("#modulesPage > h1").text("Modules");

$("#globalConfig > h2").text("Algemene configuratie");
$("#globalConfig > p").text("Configuratie module parameters");
$("#mpBank").text("Bank");
$("#mpModule").text("Module");
$("#mpVoltage").text("Voltage");
$("#mpvmin").text("V. miniumum");
// nota: aquí seguramente falte  $("#mpvmax").text("V. maximum");
$("#mptint").text("Interne temperatuur °C");
$("#mptext").text("Externe temperatuursensor °C");
$("#mpbypass").text("Bypass %"); //For me is better not translate "Bypass"
$("#mpbpc").text("Corrupte paketten");
$("#mppktr").text("Ontvangen paketten");
$("#mpbal").text("Energieverbruik balancering (mAh)");

$("#error1").text("De controller heeft communicatieproblemen met een module.");
$("#error3").text("De controler is geconfigureerd met meer dan aantal ondersteunde modules.");
$("#error4").text("Wachten op antwoord van de module..");
$("#error5").text("De module heeft 0 volt geregistreerd, controleer de configuratie.");
$("#error6").text("De controller heeft niet genoeg opslagruimte.");
$("#error7").text("Noodstop");
$("#iperror").text("Communicatie met de controller voor statusupdates is niet mogelijk.");
$("#jslibrary").text("De JavaScript-bibliotheek werd niet correct geladen, gelieve te pagina vernieuwen.");
$("#saveerror").text("De instellingen werden niet bewaard.");
$("#savesuccess").text("Instellingen bewaard");

$("#warning1").text("Opgelet: de bypass-spanning van de module is verschillend van de globale instelling");
$("#warning2").text("Opgelet: de bypass-temperatuur van de module is verschillend van de globale instelling");
$("#warning3").text("Opgelet: de modules hebben verschillende software-versies, die kan tot instabiliteit leiden");
$("#warning4").text("Opgelet: de modules hebben verschillende hardware-versies");
$("#warning5").text("Opgelet: logging geactiveerd, SD-kaart echter niet gevonden");
$("#warning6").text("Opgelet: enkele functies zijn uitgeschakeld tijdens de AVR-programmeermodus");

$("label[for='NTPServer']").text("NTP Server");
$("label[for='NTPZoneHour']").text("Tijdzone (Uren)");




//DIYBMS supports relay modules to safely disconnect chargers, contactors or consumers. The rules allow you to configure the relays for your situation.
$("#rt1").text("DIYBMS ondersteunt relais om veilig laders, contactors en verbruikers te schakelen. De regels onderaan laten toe ze instellen voor jouw situatie");
//Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of "X" means don't care/leave at value defined by lower priority rules.
$("#rt2").text("Regels worden gevalueerd van laag naar hoog (van onder naar boven).  Stuur de relais met de opties. X wil zeggen negeer waarde van lagere regeles");
//Rules are triggered when the relevant value meets or exceeds the 'trigger' value. The rule will only disable when the value then passes the 'reset' value. This can help prevent relay clatter and rules firing on/off rapidly.
$("#rt3").text("Regels worden actief wanneer de waarde boven de alarmwaarde gaat en wordt pas terug inactief wanneer de waarde onder de herstelwaarde gaat.  Dit vermijdt onstabiele regelingen.");
//'Timer 1' and 'Timer 2' rules allow timed operation, this rule is active when the number of minutes past midnight has been reached, for instance setting 'Timer 1' trigger to 495 and reset to 555 would switch on at 8:15am and off at 9:15am. This only works if connected to internet for regular time updates.
$("#rt4").text("De regels 'Timer 1' en 'Timer 2' laten een tijdsturing toe, de regel is actief wanneer het aantal minuten na middernacht is bereikt.  Bijvoorbeeld Timer 1 alarmwaarde 495 en herstelwaarde 555 zou activeren om 08:15 en deactiveren om 21:15. Deze regel werkt enkel als er internetverbinding is (NTP). ");
//Minutes since midnight now is: 
$("#rt5").text("Tijd na middernacht in minuten: ");
//Emergency stop is triggered by connector J1, once triggered controller needs to be reset to disable.
$("#rt6").text("Een noodstop wordt geactiveerd via connector J1, eens geactiveerd moet de controller gereset worden");


//Rule
$("#rf1").text("Regel");
//Trigger value
$("#rf2").text("Alarmwaarde");
//Reset value
$("#rf3").text("Herstelwaarde");
//Relay state
$("#rf4").text("Relaisstatus");


//Emergency Stop
$("label[for='rule0value']").text("Noodstop");
//Internal BMS error 
$("label[for='rule1value']").text("Interne BMS fout");
//Current monitoring over current (Amps)
//Individual cell over voltage (mV)
$("label[for='rule3value']").text("Celoverspannig (mV)");
//Cell under voltage (mV)
$("label[for='rule4value']").text("Celonderspannijg (mV)");
//Module over temperature (internal) °C
$("label[for='rule5value']").text("Maximale moduletemperatuur (intern) °C");
//Module under temperature (internal)
$("label[for='rule6value']").text("Minimale moduletemperatuur (intern)");
//Cell over temperature (external)
$("label[for='rule7value']").text("Maximale celtemperatuur (extern)");
//Cell under temperature (external)
$("label[for='rule8value']").text("Minimale celtemperatuur (extern)");
//Bank over voltage (mV)
//Bank under voltage (mV)
//Bank over voltage (mV)
$("label[for='rule11value']").text("Maximale batterijspanning (mV)");
//Bank under voltage (mV)
$("label[for='rule12value']").text("Minimale batterijspanning (mV)");
//Bank range/deviation (mV)
//Timer 2
$("label[for='rule14value']").text("Timer 2");
//Timer 1
$("label[for='rule15value']").text("Timer 1");
//Relay default
$("label[for='defaultvalue']").text("Standaardwaarde relais");
//Relay type
$("label[for='relaytype']").text("Type relais");


//Bypass over temperature
$("label[for='g1']").text("Maximale bypass-temperatuur");
//Bypass threshold mV
$("label[for='g2']").text("Bypass-grenswaarde mV");

//Packets sent
$("#sent > span.x.t").text("Paketten verzonden:");
//Packets rec'd
$("#received > span.x.t").text("Paketten ontvangen:");
//Roundtrip (ms)
$("#roundtrip > span.x.t").text("Roundtrip (ms):");

//Integration
$("#ip1").text("Integratie");
//For security, you will need to re-enter the password for the service(s) you want to enable or modify, before you save.
$("#ip2").text("Paswoorden moeten opnieuw ingegeven worden voor de service(s) die geactiveerd of gewijzigd worden.");
//After changes are made, the controller will need to be rebooted, do this manually.
$("#ip3").text("Na het bewaren van instellingen moet de controller manueel herstart worden");

//PATREON
//Remember, this product is free for personal use, if you would like to make a regular donation to keep the features and improvements flowing, use the Patreon link below. Even just a coffee/beer a month makes a difference. Thank you!
$("#ap4").text("Vergeet niet dat dit een gratis product is voor persoonlijk gebruik.  Om het project te laten groeien en evolueren kan je altijd steunen via de Patreon link hieronder, zelfs een kleine gift maakt het verschil.  Bedankt!");

//WARNING
$("#ap5").text("OPGELET");
//This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.
$("#ap6").text("Dit is een DIY product/oplossing, gebruik het niet in veiligheidskritische systemen of in een situatie waarbij het levensbedreigend kan zijn");
//There is no warranty, it may not work as expected or at all.
$("#ap7").text("Er wordt geen enkele garantie gegeven, het kan mogelijks niet aan de verwachtingen voldoen of totaal niet werken");
//The use of this project is done so entirely at your own risk. It may involve electrical voltages which could kill - if in doubt, seek help.
$("#ap8").text("Het gebruik van dit project is volledig op eigen risico.  De mogelijks gebruikte spanningen zijn potentieel dodelijk - bij twijfel, vraag raad aan een professional.");
//The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.
$("#ap8a").text("Het gebruik van dit project is mogelijk niet conform de lokale wetgeving - bij twijfel, vraag raad aan een professional.");

//Modules & Banks
$("#mb1").text("Modules & Banken");
//DIYBMS supports up to 100 modules in total. These modules can be split into banks to support parallel configurations.
$("#mb2").text("DIYBMS ondersteunt maximaal 100 modules in totaal.  Deze modules kunnen opgesplitst worden in verschillende banken t.b.v. een parallele configuratie");
//Example: You have 16 cells configured as 8 in series and 2 in parallel (8S2P).
$("#mb3").text("Bijvoorbeeld: Je hebt 16 cellen geconfigureerd, 8 in serie en 2 in parallel (8S2P)..");

//diybmsCurrentMonitorPage

//<h1>diyBMS Current &amp; Voltage Monitor</h1>
$("#diybmsCurrentMonitorPage > h1").text("diyBMS Stroom & Spanningsbewaking");
//<p>Configure the MODBUS connection to the current monitor using the settings below.</p>
//Configureer de MODBUS-verbining voor de shuntmonitor met de instellingen hieronnder.

//<label for="CurrentMonEnabled">Enabled</label>
$("label[for='CurrentMonEnabled']").text("geactiveerd");

//<p>Configuration options for RS485 interface. Communication is half-duplex.</p>
//RS485 configuratie-instellingen. De communicatie is half-duplex.

//<h2>Basic Settings</h2>
//Basisinstellingen

//<p>Ensure the current shunt parameters match the data sheet for your particular shunt resistor.</p>
// Let er op dat de current shunt parameters overeenkomen met die van de gebruikte shunt

//<p>The current monitor uses a 40.96mV maximum scale, so shunt voltages over this will be scaled down proportionally.</p>
// De actuele monitor gebruikt een schaal van maximum 40,96mV.  Shuntspanningen zal proportieel geschaald worden.

//<label for="shuntmaxcur">Shunt maximum current</label>
$("label[for='shuntmaxcur']").text("Shunt maximum stroom");

//<label for="shuntmv">Shunt output voltage (mV)</label>
$("label[for='shuntmv']").text("Shunt-uitgangspanning (mV)");

//<label for="cmvalid">Values valid?</label>
$("label[for='cmvalid']").text("Geldige waarde?");

//<label for="cmtimestampage">Last communication (milliseconds)</label>
$("label[for='cmtimestampage']").text("Laatste communicatie (milliseconden)");

//<label for="cmwatchdog">Watchdog counter</label>
$("label[for='cmwatchdog']").text("Watchdog-teller");

//<label for="cmtemperature">Die temperature &deg;C</label>
$("label[for='cmtemperature']").text("CPU-core temperatuur");

//<label for="cmactualshuntmv">Actual shunt mV reading</label>
$("label[for='cmactualshuntmv']").text("Actuele shunt mV waarde");

//<label for="cmcurrentlsb">Current LSB size</label>
$("label[for='cmcurrentlsb']").text("Actuele LSB-grootte");

//<label for="cmresistance">Shunt resistance</label>
$("label[for='cmresistance']").text("Shunt-weerstand");

//<label for="cmmodel">Sensor model</label>
$("label[for='cmmodel']").text("Sensortype");

//<label for="cmfirmwarev">Firmware version</label>
$("label[for='cmfirmwarev']").text("Firmware versie");

//<label for="cmfirmwaredate">Firmware date</label>
$("label[for='cmfirmwaredate']").text("Firmware datum");

//<label for="cmRelayState">Relay state</label>
$("label[for='cmRelayState']").text("Relaisstatus");

//<label for="cmTemperatureOverLimit">Temperature over limit</label>
$("label[for='cmTemperatureOverLimit']").text("Temperatur boven limiet");

//<label for="cmCurrentOverLimit">Current over limit</label>
$("label[for='cmCurrentOverLimit']").text("Stroom boven limiet");

//<label for="cmCurrentUnderLimit">Current under limit</label>
$("label[for='cmCurrentUnderLimit']").text("Stroom onder limiet");

//<label for="cmVoltageOverLimit">Voltage over limit</label>
$("label[for='cmVoltageOverLimit']").text("Spannung boven limiet");

//<label for="cmVoltageUnderLimit">Voltage under limit</label>
$("label[for='cmVoltageUnderLimit']").text("Spannung onder limiet");

//<label for="cmPowerOverLimit">Power over limit</label>
$("label[for='cmPowerOverLimit']").text("Vermogen boven limiet");


//<h2>Advanced Current Monitor Settings</h2>
//Geavanceerde shuntmonitor instellingen

//<p>If you wish to use the relay control on the shunt monitor, set the parameters here. <u>You should not normally need to change the calibration value.</u></p>
//Wenn Sie die Relaissteuerung am Shunt-Monitor verwenden möchten, stellen Sie hier die Parameter ein. Normalerweise sollten Sie den Kalibrierungswert nicht ändern müssen.

//<p>Current limit is for use whilst discharging the battery, under current limit is used when charging, so different discharge/charge current limits can be used.</u></p>
// I cannot translate meaningfully

//<p>Temperature limit is based on the chip die temperature, which may not match the shunt temperature. Only positive temperature coefficient is supported.</u></p>
//De temperatuurslimiet is gebaseerd op de kern-temperatuur, die wijkt mogelijks af van de shunt temperatuur.  Enkel een positive termperatuurscoëfficient is ondersteund

//<p>Relay triggers define which rules are used to trigger the relay into a closed state.</u></p>
//Relais triggers bepalen welke regels gebruikt worden om het relais te sluiten

//<label for="cmcalibration">Calibration</label>
$("label[for='cmcalibration']").text("Calibratie");

//<label for="cmtemplimit">Temperature limit</label>
$("label[for='cmtemplimit']").text("Temperatuurslimmiet");

//<label for="cmundervlimit">Under voltage limit</label>
$("label[for='cmundervlimit']").text("Onderspanningslimiet");

//<label for="cmovervlimit">Over voltage limit</label>
$("label[for='cmovervlimit']").text("Overspanningslimiet");

//<label for="cmoverclimit">Over current limit</label>
$("label[for='cmoverclimit']").text("Maximale stroom");

//<label for="cmunderclimit">Under current limit</label>
$("label[for='cmunderclimit']").text("Minimale stroom");

//<label for="cmoverplimit">Over power limit</label>
$("label[for='cmoverplimit']").text("Maximaal vermogen");

//<label for="cmtempcoeff">Temperature coefficient ppm/&deg;C</label>
$("label[for='cmtempcoeff']").text("Temperatuurscoëfficiënt");

//<h2>Current Monitor Relay Settings &amp; Temperature coefficient</h2>
// Stroombewaking relais-instellingen en temperatuurscoëfficiënt

//Relay triggers define which rules are used to trigger the relay into a closed state.
//Relais triggers bepalen welke regels gebruikt worden om het relais te sluiten

//<label for="TempCompEnabled">Temperature coefficient enabled</label>
$("label[for='TempCompEnabled']").text("temperatuurscoëfficiënt geactiveerd");

//<label for="cmTMPOL">Relay Trigger: Temperature</label>
$("label[for='cmTMPOL']").text("Relaistrigger: Temperatuur");

//<label for="cmCURROL">Relay Trigger: Current over</label>
$("label[for='cmCURROL']").text("Relaistrigger: Overstroom");

//<label for="cmCURRUL">Relay Trigger: Current under</label>
$("label[for='cmCURRUL']").text("Relaistrigger: Onderstroom");

//<label for="cmVOLTOL">Relay Trigger: Voltage over</label>
$("label[for='cmVOLTOL']").text("Relaistrigger: Overspanning");

//<label for="cmVOLTUL">Relay Trigger: Voltage under</label>
$("label[for='cmVOLTUL']").text("Relaistrigger: Onderspanning");

//<label for="cmPOL">Relay Trigger: Power</label>
$("label[for='cmPOL']").text("Relaistrigger: Vermogen");
