// This file is for Croatian language translation
// https://www.w3schools.com/tags/ref_language_codes.asp

$("#home").text("Početak");
$("#modules").text("Moduli");
$("#settings").text("Postavke");
$("#rules").text("Pravila");
$("#more").text("Više");

$("#integration").text("Poveyivanje");
$("#currentmonitor").text("DIYBMS Amperemetar");
$("#storage").text("Spremište");
$("#avrprogrammer").text("AVR-Programator");
$("#about").text("Informacije");

$("#modulesPage > h1").text("Moduli");

$("#globalConfig > h2").text("Globalna Konfiguracija");
$("#globalConfig > p").text("Konfigurirajte sve module na takav način da koristite sljedeće postavke:");

$("#mpBank").text("Grupa");
$("#mpModule").text("Modul");
$("#mpVoltage").text("Napon");
$("#mpvmin").text("V. Minimum");
$("#mpvmax").text("V. Maximum");
$("#mptint").text("Unutarnja Temperatura °C");
$("#mptext").text("Vanjska Temperatura °C");
$("#mpbypass").text("Bypass %");
$("#mpbpc").text("Broj pogrešnih paketa");
$("#mppktr").text("Primljeni paketi");
$("#mpbal").text("Energija potrošena na uravnoteživanje (mAh)");

$("#error1").text("Kontroler ima problema u komunikaciji s modulima.");
$("#error3").text("Kontroler je konfiguriran za upotrebu više modula nego što može podržati.");
$("#error4").text("Čeka se odgovor od modula");
$("#error5").text("Modul je očitao krivi napon (NULL). Provjerite konfiguraciju");
$("#error6").text("Kontroler nema dovoljno memorije.");
$("#error7").text("Not-Stop");
$("#iperror").text("Komunikacija s kontrolerom za dohvat statusa nije moguća.");
$("#jslibrary").text("JavaScript biblioteka nije pravilno učitana. Osvježite stranicu.");
$("#saveerror").text("Postavke nije bilo moguće spremiti.");
$("#savesuccess").text("Postavke spremljene");

$("#warning1").text("Upozorenje: Bypass napon modula razlikuje se od globalne postavke");
$("#warning2").text("Upozorenje: Temperatura bypass modula razlikuje se od globalne postavke");
$("#warning3").text("Upozorenje: Moduli imaju različite verzije koda, što bi moglo dovesti do nestabilnosti");
$("#warning4").text("Upozorenje: Moduli imaju različite verzije hardvera ili ploče");
$("#warning5").text("Upozorenje: Zapisivanje je omogućeno, ali SD kartica nije instalirana ili pronađena");
$("#warning6").text("Neke su značajke onemogućene dok su u AVR načinu programiranja");

$("label[for='NTPServer']").text("NTP Server");
$("label[for='NTPZoneHour']").text("Vremenska zona (Sati)");




//DIYBMS supports relay modules to safely disconnect chargers, contactors or consumers. The rules allow you to configure the relays for your situation.
$("#rt1").text("DIYBMS podržava releje za sigurno odvajanje punjača, sklopnika ili potrošača. Pravila vam omogućuju da konfigurirate releje za svoju situaciju.");
//Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of "X" means don't care/leave at value defined by lower priority rules.
$("#rt2").text("Pravila se obrađuju od najnižeg do najvišeg prioriteta (odozdo prema gore). Upravljajte relejima pomoću opcija. "X" znači da nije aktivan. "On" za uključivanje i "Off" za isključivanje.");
//Rules are triggered when the relevant value meets or exceeds the 'trigger' value. The rule will only disable when the value then passes the 'reset' value. This can help prevent relay clatter and rules firing on/off rapidly.
$("#rt3").text("Pravila se aktiviraju kada izmjerena vrijednost dosegne ili premaši vrijednost okidača. Pravilo se deaktivira samo ako vrijednost premašuje vrijednost resetiranja. To može spriječiti zveckanje releja i brzo uključivanje i isključivanje pravila.");
//'Timer 1' and 'Timer 2' rules allow timed operation, this rule is active when the number of minutes past midnight has been reached, for instance setting 'Timer 1' trigger to 495 and reset to 555 would switch on at 8:15am and off at 9:15am. This only works if connected to internet for regular time updates.
$("#rt4").text("Pravila 'Timer 1' i 'Timer 2' omogućuju rad kontroliran vremenom. Ovo je pravilo aktivno kada broj minuta prođe nakon ponoći (00:00). Na primjer, ako postavite okidač 'Timer 1' na 495 i resetirate 'Timer 2' na 555, on će se uključiti u 8:15 i isključiti u 9:15. To funkcionira samo ako ste povezani s internetom radi redovitih ažuriranja vremena.");
//Minutes since midnight now is: 
$("#rt5").text("Vrijeme nakon ponoći u minutama: ");
//Emergency stop is triggered by connector J1, once triggered controller needs to be reset to disable.
$("#rt6").text("Zaustavljanje u nuždi aktivira konektor J1; nakon što se aktivira, regulator se mora resetirati da bi ga deaktivirao.");


//Rule
$("#rf1").text("Pravilo");
//Trigger value
$("#rf2").text("Vrijednost paljenja");
//Reset value
$("#rf3").text("Vrijednost resetiranja");
//Relay state
$("#rf4").text("Status releja");


//Emergency Stop
$("label[for='rule0value']").text("Not Stop");
//Internal BMS error 
$("label[for='rule1value']").text("Greška unutrašnjeg BMSa");
//Current monitoring over current (Amps)
//Individual cell over voltage (mV)
$("label[for='rule3value']").text("Prenapon pojedine čelije (mV)");
//Cell under voltage (mV)
$("label[for='rule4value']").text("Podnapon pojedine čelije (mV)");
//Module over temperature (internal) °C
$("label[for='rule5value']").text("Prekomjerna temperatura modula (unutarnji) °C");
//Module under temperature (internal)
$("label[for='rule6value']").text("Podtemperatura modula (unutarnji)");
//Cell over temperature (external)
$("label[for='rule7value']").text("Prekomjerna temperatura čelije (vanjski)");
//Cell under temperature (external)
$("label[for='rule8value']").text("Podtemperatura čelije (vanjski)");
//Bank over voltage (mV)
//Bank under voltage (mV)
//Bank over voltage (mV)
$("label[for='rule11value']").text("Prenapon bloka (mV)");
//Bank under voltage (mV)
$("label[for='rule12value']").text("Podnapon bloka (mV)");
//Bank range/deviation (mV)
//Timer 2
$("label[for='rule14value']").text("Vremenski brojač 2");
//Timer 1
$("label[for='rule15value']").text("Vremenski brojač 1");
//Relay default
$("label[for='defaultvalue']").text("Defaultni relej");
//Relay type
$("label[for='relaytype']").text("Tip releja");


//Bypass over temperature
$("label[for='g1']").text("Bypass temp pregrijavanja");
//Bypass threshold mV
$("label[for='g2']").text("Bypass prag mV");

//Packets sent
$("#sent > span.x.t").text("Poslanih paketa:");
//Packets rec'd
$("#received > span.x.t").text("Primljenih paketa:");
//Roundtrip (ms)
$("#roundtrip > span.x.t").text("Kruženje (ms):");

//Integration
$("#ip1").text("Integracija");
//For security, you will need to re-enter the password for the service(s) you want to enable or modify, before you save.
$("#ip2").text("Iz sigurnosnih razloga morate ponovno unijeti lozinku za usluge koje želite omogućiti ili promijeniti prije spremanja.");
//After changes are made, the controller will need to be rebooted, do this manually.
$("#ip3").text("Nakon promjene, kontroler mora biti ponovno pokrenut. Učinite to ručno.");

//WARNING
$("#ap5").text("UPOZORENJE");
//This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.
$("#ap6").text("To je DIY proizvod / DIY rješenje. Stoga nemojte koristiti ovaj proizvod / rješenje za sustave koji su kritični za sigurnost ili u situacijama u kojima postoji opasnost za život");
//There is no warranty, it may not work as expected or at all.
$("#ap7").text("Nema jamstva ili jamstva. Možda neće raditi kako se očekuje ili uopće ne radi..");
//The use of this project is done so entirely at your own risk. It may involve electrical voltages which could kill - if in doubt, seek help.
$("#ap8").text("Korištenje ovog projekta provodi se isključivo na vlastitu odgovornost. Moguče uključuje električne napone koji mogu biti smrtonosni - ako ste u nedoumici, potražite pomoć..");
//The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.
$("#ap8a").text("Korištenje ovog projekta možda neće biti u skladu s lokalnim zakonima ili propisima. Ako ste u nedoumici, pitajte.");

//Modules & Banks
$("#mb1").text("Moduli & Blokovi");
//DIYBMS supports up to 100 modules in total. These modules can be split into banks to support parallel configurations.
$("#mb2").text("DIYBMS podržava ukupno do 100 modula. Ovi moduli mogu se podijeliti na Blokove kako bi podržali paralelne konfiguracije.");
//Example: You have 16 cells configured as 8 in series and 2 in parallel (8S2P).
$("#mb3").text("Na primjer, imate 16 ćelija prikupljenih kao 8 u nizu i 2 paralelno (8S2P)..");


