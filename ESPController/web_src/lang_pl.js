//Grzegorz Mirż 18.12.2023

$("#home").text("Start");
$("#tiles").text("Elementy");
$("#modules").text("Moduły");
$("#rules").text("Reguły");
$("#more").text("Więcej");

$("#settings").text("Ustawienia");
$("#charging").text("Ustawienia ładowania/rozładowywania");
$("#integration").text("Integracje");
$("#currentmonitor").text("DIYBMS Monitoring");
$("#storage").text("Karta SD i Logi");
$("#avrprogrammer").text("Programator AVR");
$("#utility").text("Narzędzia");
$("#about").text("O DIYBMS");

$("#modulesPage > h1").text("Moduły");

$("#globalConfig > h2").text("Ustawienia globalne");
$("#globalConfig > p").text("Ustawienia wspólne dla wszystkich modułów:");

$("#mpBank").text("Bank");
$("#mpModule").text("Moduł");
$("#mpVoltage").text("Napięcie V.");
$("#mpvmin").text("V. Min.");
$("#mpvmax").text("V. Max.");
$("#mptint").text("Temp wewn °C");
$("#mptext").text("Temp zewn °C");
$("#mpbypass").text("Obejście PWM %");
$("#mpbpc").text("Ilość błędnych pakietów");
$("#mppktr").text("Pakiety otrzymane");
$("#mpbal").text("Statysytka balansowania (mAh)");

$("#error1").text("Kontroler ma trudności z komunikacją z modułami ogniw i/lub monitorem prądu.");
$("#error3").text("Kontroler jest skonfigurowany tak, aby wykorzystywać więcej modułów, niż jest w stanie obsłużyć.");
$("#error4").text("Oczekiwanie na odpowiedź modułów.");
$("#error5").text("Moduł zwrócił odczyt napięcia ZERO, sprawdź konfigurację");
$("#error6").text("W kontrolerze zabrakło pamięci.");
$("#error7").text("Awaryjne zatrzymanie!");
$("#iperror").text("Nie można zkomunikować się z kontrolerem w celu aktualizacji statusu.");
$("#genericerror").text("Błąd kontrolera!");
$("#jslibrary").text("Nie udało się poprawnie załadować biblioteki JavaScript. Odśwież stronę.");
$("#saveerror").text("Nie udało się zapisać ustawień.");
$("#savesuccess").text("Ustawienia zapisane");
$("#genericwarning").text("Ostrzeżenie kontrolera");
$("#warning1").text("Ostrzeżenie: napięcie obejścia modułu różni się od ustawienia globalnego");
$("#warning2").text("Ostrzeżenie: Temperatura obejścia modułu różni się od ustawienia globalnego");
$("#warning3").text("Ostrzeżenie: moduły mają mieszane wersje kodu, mogą powodować niestabilność");
$("#warning4").text("Ostrzeżenie: moduły mają mieszane wersje sprzętu/płyt");
$("#warning5").text("Ostrzeżenie: Rejestrowanie włączone, ale karta SD nie została zainstalowana/znaleziona");
$("#warning6").text("Niektóre funkcje są wyłączone, gdy włączony jest tryb programowania AVR");
$("#warning7").text("Ładowanie jest zablokowane (ustawienia ładowania)");
$("#warning8").text("Rozładowanie jest uniemożliwione (ustawienia rozładowania)");
$("#warning9").text("Brak zewnętrznego czujnika temperatury ogniwa");
$("#warningxss").text("Strona internetowa nie jest zsynchronizowana z kontrolerem, odśwież stronę (F5)");

$("label[for='NTPServer']").text("Serwer czasu NTP");
$("label[for='NTPZoneHour']").text("Strefa czasowa NTP (godziny)");


//DIYBMS supports relay modules to safely disconnect chargers, contactors or consumers. The rules allow you to configure the relays for your situation.
$("#rt1").text("DIYBMS obsługuje moduły przekaźnikowe do bezpiecznego odłączania ładowarek, styczników lub odbiorników. Reguły umożliwiają skonfigurowanie przekaźników w zależności od sytuacji.");
//Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of "X" means don't care/leave at value defined by lower priority rules.
$("#rt2").text("Reguły są przetwarzane od najniższego do najwyższego priorytetu (od dołu do góry). Steruj przekaźnikami za pomocą opcji. Wartość „X” oznacza, że nie przejmujesz się/zostaw przy wartości określonej przez reguły o niższym priorytecie.");
//Rules are triggered when the relevant value meets or exceeds the 'trigger' value. The rule will only disable when the value then passes the 'reset' value. This can help prevent relay clatter and rules firing on/off rapidly.
$("#rt3").text("Reguły są uruchamiane, gdy odpowiednia wartość osiąga lub przekracza wartość „wyzwalacza”. Reguła zostanie wyłączona dopiero wtedy, gdy wartość przekroczy wartość „reset”. Może to pomóc w zapobieganiu brzęczeniu przekaźników i szybkiemu włączaniu/wyłączaniu reguł.");
//'Timer 1' and 'Timer 2' rules allow timed operation, this rule is active when the number of minutes past midnight has been reached, for instance setting 'Timer 1' trigger to 495 and reset to 555 would switch on at 8:15am and off at 9:15am. This only works if connected to internet for regular time updates.
$("#rt4").text("Reguły „Timer 1” i „Timer 2” umożliwiają działanie w określonym czasie. Reguła ta jest aktywna po osiągnięciu liczby minut po północy, na przykład ustawienie wyzwalacza „Timer 1” na 495 i zresetowanie do 555 spowoduje włączenie o 8:15 i wyłączenie o 9:15. Działa to tylko po podłączeniu do Internetu w celu regularnych aktualizacji czasu.");
//Minutes since midnight now is: 
$("#rt5").text("Minuty od północy to obecnie: ");
//Emergency stop is triggered by connector J1, once triggered controller needs to be reset to disable.
$("#rt6").text("Zatrzymanie awaryjne jest wyzwalane przez złącze J1. Po uruchomieniu zatrzymania awaryjnego sterownik musi zostać zresetowany.");


//Rule
$("#rf1").text("Reguła");
//Trigger value
$("#rf2").text("Wartość wyzwalająca");
//Reset value
$("#rf3").text("Wartość resetująca");
//Relay state
$("#rf4").text("Stan przekaźnika");


//Emergency Stop
$("label[for='rule0value']").text("Awaryjne zatrzymanie");
//Internal BMS error 
$("label[for='rule1value']").text("Błąd wewnętrzny BMS");
//Current monitoring over current (Amps)
$("label[for='rule2value']").text("maksymalny prąd (A)");
//Individual cell over voltage (mV)
$("label[for='rule3value']").text("Napięcie ogniwa powyżej (mV)");
//Cell under voltage (mV)
$("label[for='rule4value']").text("Napięcie ogniwa poniżej (mV)");
//Module over temperature (internal) °C
$("label[for='rule5value']").text("Temperatura ogniwa powyżej (wewn) °C");
//Module under temperature (internal)
$("label[for='rule6value']").text("Temperatura ogniwa powyżej (wewn)");
//Cell over temperature (external)
$("label[for='rule7value']").text("Temperatura ogniwa powyżej (zewn)");
//Cell under temperature (external)
$("label[for='rule8value']").text("Temperatura ogniwa poniżej (zewn)");
//Bank over voltage (mV)
$("label[for='rule9value']").text("Napięcie maksymalne (mV)");
//Bank under voltage (mV)
$("label[for='rule10value']").text("Napięcie minimalne (mV)");
//Bank over voltage (mV)
$("label[for='rule11value'").text("Napięcie pakietu powyżej (mV)");
//Bank under voltage (mV)
$("label[for='rule12value'").text("Napięcie pakietu poniżej (mV)");
//Bank range/deviation (mV)
$("label[for='rule13value'").text("Zakres różnicy napięć ogniw w pakiecie (mV)");
//Timer 2
$("label[for='rule14value']").text("Timer 2");
//Timer 1
$("label[for='rule15value']").text("Timer 1");
//Relay default
$("label[for='defaultvalue']").text("Domyślne ustawienia przekaźnika");
//Relay type
$("label[for='relaytype']").text("Typ przekaźnika");


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
$("#ip1").text("Integracje");
//For security, you will need to re-enter the password for the service(s) you want to enable or modify, before you save.
$("#ip2").text("Ze względów bezpieczeństwa przed zapisaniem konieczne będzie ponowne wprowadzenie hasła do usługi MQTT, jeśli chcesz ją włączyć lub zmodyfikować.");
//After changes are made, the controller will need to be rebooted, do this manually.
$("#ip3").text("Po dokonaniu zmian należy zrestartować sterownik. Zrób to ręcznie.");

$("#ip4").text("Adres URI powinien być podobny do mqtt://192.168.0.26:1833");

$("#ip5").text("Opcja podstawowych danych komórkowych zmniejsza ilość danych MQTT przesyłanych przez sieć.");

$("#ap1").text("Kod źródłowy i sprzęt");
$("#ap2").text("Filmy");
$("#ap3").text("Patreon (wspieranie)");
//PATREON
//Remember, this product is free for personal use, if you would like to make a regular donation to keep the features and improvements flowing, use the Patreon link below. Even just a coffee/beer a month makes a difference. Thank you!
$("#ap4").text("Pamiętaj, że ten produkt jest bezpłatny do użytku osobistego. Jeśli chcesz regularnie przekazywać darowizny na utrzymanie funkcjonalności i ulepszeń, skorzystaj z poniższego linku Patreon. Nawet kawa/piwo miesięcznie robi różnicę. Dziękuję!");

//WARNING
$("#ap5").text("OSTRZEŻENIE");
//This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.
$("#ap6").text("Jest to produkt/rozwiązanie typu „zrób to sam”, więc nie używaj go w systemach o krytycznym znaczeniu dla bezpieczeństwa ani w żadnej sytuacji, w której mogłoby wystąpić zagrożenie życia.");
//There is no warranty, it may not work as expected or at all.
$("#ap7").text("Nie ma gwarancji na poprawne działanie, może nie działać zgodnie z oczekiwaniami lub wcale.");
//The use of this project is done so entirely at your own risk. It may involve electrical voltages which could kill - if in doubt, seek help.
$("#ap8").text("Korzystanie z tego projektu odbywa się całkowicie na własne ryzyko. Ryzysko wysokiego napięcia elektrycznego, które może zabić. W razie wątpliwości należy zwrócić się o pomoc do specjalisty.");
//The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.
$("#ap8a").text("Korzystanie z tego projektu może być niezgodne z lokalnymi przepisami lub regulacjami – w razie wątpliwości należy zwrócić się o pomoc.");

//Modules & Banks
$("#mb1").text("Moduły i pakiety");
//DIYBMS supports up to 100 modules in total. These modules can be split into banks to support parallel configurations.
$("#mb2").text("DIYBMS obsługuje łącznie do 100 modułów. Moduły te można podzielić na pakiety w celu obsługi konfiguracji równoległych.");
//Example: You have 16 cells configured as 8 in series and 2 in parallel (8S2P).
$("#mb3").text("Przykład: Masz 16 ogniw skonfigurowanych jako 8 szeregowo i 2 równolegle (8S2P).");

//diybmsCurrentMonitorPage

//<h1>diyBMS Current &amp; Voltage Monitor</h1>
$("#diybmsCurrentMonitorPage > h1").text("Monitorowanie prądu (A) i napięcia");
//<p>Configure the MODBUS connection to the current monitor using the settings below.</p>
//Skonfiguruj połączenie MODBUS z bieżącym monitorem, korzystając z poniższych ustawień.

//<label for="CurrentMonEnabled">Enabled</label>
$("label[for='CurrentMonEnabled']").text("Włączony");

//<p>Configuration options for RS485 interface. Communication is half-duplex.</p>
//Opcje konfiguracji interfejsu RS485. Komunikacja odbywa się w trybie półdupleksowym.

//<h2>Basic Settings</h2>
//Ustawienia podstawowe

//<p>Ensure the current shunt parameters match the data sheet for your particular shunt resistor.</p>
//Upewnij się, że aktualne parametry bocznika są zgodne z arkuszem danych konkretnego rezystora bocznikowego.

//<p>The current monitor uses a 40.96mV maximum scale, so shunt voltages over this will be scaled down proportionally.</p>
//Monitor prądu wykorzystuje maksymalną skalę 40,96 mV, więc napięcia bocznikowe powyżej tej wartości będą proporcjonalnie zmniejszane.

//<label for="shuntmaxcur">Shunt maximum current</label>
$("label[for='shuntmaxcur']").text("Maksymalny prąd bocznikowy");

//<label for="shuntmv">Shunt output voltage (mV)</label>
$("label[for='shuntmv']").text("Maksymalne napięcie bocznikowe (mV)");

//<label for="cmvalid">Values valid?</label>
$("label[for='cmvalid']").text("Czy wartości są poprawne?");

//<label for="cmtimestampage">Last communication (milliseconds)</label>
$("label[for='cmtimestampage']").text("Ostatnia komunikacja (milisekundy)");

//<label for="cmwatchdog">Watchdog counter</label>
$("label[for='cmwatchdog']").text("Licznik Watchdoga");

//<label for="cmtemperature">Die temperature &deg;C</label>
$("label[for='cmtemperature']").text("Temperatura CPU &deg;C");

//<label for="cmactualshuntmv">Actual shunt mV reading</label>
$("label[for='cmactualshuntmv']").text("Aktualny odczyt napięcia bocznika mV");

//<label for="cmcurrentlsb">Current LSB size</label>
$("label[for='cmcurrentlsb']").text("Aktualny rozmiar LSB");

//<label for="cmresistance">Shunt resistance</label>
$("label[for='cmresistance']").text("Rezystancja bocznikowa");

//<label for="cmmodel">Sensor model</label>
$("label[for='cmmodel']").text("Model czujnika");

//<label for="cmfirmwarev">Firmware version</label>
$("label[for='cmfirmwarev']").text("Wersja oprogramowania");

//<label for="cmfirmwaredate">Firmware date</label>
$("label[for='cmfirmwaredate']").text("Data oprogramowania");

//<label for="cmRelayState">Relay state</label>
$("label[for='cmRelayState']").text("Stan przekaźnika");

//<label for="cmTemperatureOverLimit">Temperature over limit</label>
$("label[for='cmTemperatureOverLimit']").text("Temperatura powyżej limitu");

//<label for="cmCurrentOverLimit">Current over limit</label>
$("label[for='cmCurrentOverLimit']").text("Prąd powyżej limitu");

//<label for="cmCurrentUnderLimit">Current under limit</label>
$("label[for='cmCurrentUnderLimit']").text("Prąd poniżej limitu");

//<label for="cmVoltageOverLimit">Voltage over limit</label>
$("label[for='cmVoltageOverLimit']").text("Napięcie powyżej limitu");

//<label for="cmVoltageUnderLimit">Voltage under limit</label>
$("label[for='cmVoltageUnderLimit']").text("Napięcie poniżej limitu");

//<label for="cmPowerOverLimit">Power over limit</label>
$("label[for='cmPowerOverLimit']").text("Moc powyżej limitu");


//<h2>Advanced Current Monitor Settings</h2>
//Zaawansowane ustawienia monitora prądu

//<p>If you wish to use the relay control on the shunt monitor, set the parameters here. <u>You should not normally need to change the calibration value.</u></p>
//Jeśli chcesz używać sterowania przekaźnikiem na monitorze bocznikowym, ustaw tutaj parametry. <u>Zwykle zmiana wartości kalibracyjnej nie jest konieczna.

//<p>Current limit is for use whilst discharging the battery, under current limit is used when charging, so different discharge/charge current limits can be used.</u></p>
// Limit prądu obowiązuje podczas rozładowywania akumulatora, prąd poniżej limitu jest używany podczas ładowania, więc można stosować różne limity prądu rozładowania/ładowania.

//<p>Temperature limit is based on the chip die temperature, which may not match the shunt temperature. Only positive temperature coefficient is supported.</u></p>
// Limit temperatury opiera się na temperaturze rdzenia procesora, która może różnić się od temperatury bocznika. Obsługiwany jest tylko dodatni współczynnik temperaturowy.

//<p>Relay triggers define which rules are used to trigger the relay into a closed state.</u></p>
// Wyzwalacze przekaźnika definiują, które reguły są używane do wymuszenia stanu zamkniętego przekaźnika.

//<label for="cmcalibration">Calibration</label>
$("label[for='cmcalibration']").text("Kalibracja");

//<label for="cmtemplimit">Temperature limit</label>
$("label[for='cmtemplimit']").text("Limit temperatury");

//<label for="cmundervlimit">Under voltage limit</label>
$("label[for='cmundervlimit']").text("Limit dolny napięcia");

//<label for="cmovervlimit">Over voltage limit</label>
$("label[for='cmovervlimit']").text("Limit górny napięcia");

//<label for="cmoverclimit">Over current limit</label>
$("label[for='cmoverclimit']").text("Limit górny prądu");

//<label for="cmunderclimit">Under current limit</label>
$("label[for='cmunderclimit']").text("Limit dolny prądu");

//<label for="cmoverplimit">Over power limit</label>
$("label[for='cmoverplimit']").text("Limit mocy maksymalnej");

//<label for="cmtempcoeff">Temperature coefficient ppm/&deg;C</label>
$("label[for='cmtempcoeff']").text("Współczynnik temperatury ppm/&deg;C");

//<h2>Current Monitor Relay Settings &amp; Temperature coefficient</h2>
// Aktualne ustawienia przekaźnika monitorującego i współczynnik temperaturowy

//Relay triggers define which rules are used to trigger the relay into a closed state.
//Wyzwalacze przekaźnika definiują, jakie reguły są używane do wymuszenia stanu zamkniętego przekaźnika.

//<label for="TempCompEnabled">Temperature coefficient enabled</label>
$("label[for='TempCompEnabled']").text("Współczynnik temperaturowy włączony");

//<label for="cmTMPOL">Relay Trigger: Temperature</label>
$("label[for='cmTMPOL']").text("Wyzwalacz przekażnika: Temperatura");

//<label for="cmCURROL">Relay Trigger: Current over</label>
$("label[for='cmCURROL']").text("Wyzwalacz przekażnika: Prąd powyżej");

//<label for="cmCURRUL">Relay Trigger: Current under</label>
$("label[for='cmCURRUL']").text("Wyzwalacz przekażnika: Prąd poniżej");

//<label for="cmVOLTOL">Relay Trigger: Voltage over</label>
$("label[for='cmVOLTOL']").text("Wyzwalacz przekażnika: Napięcie powyżej");

//<label for="cmVOLTUL">Relay Trigger: Voltage under</label>
$("label[for='cmVOLTUL']").text("Wyzwalacz przekażnika: Napięcie poniżej");

//<label for="cmPOL">Relay Trigger: Power</label>
$("label[for='cmPOL']").text("Wyzwalacz przekażnika: Moc");
