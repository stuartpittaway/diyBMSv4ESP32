  // This file is for FRENCH language translation

$("#home").text("Acceuil"); 
$("#modules").text("Modules");
$("#settings").text("Réglages");
$("#rules").text("Régles");
$("#more").text("Plus...");

$("#integration").text("Integration");
$("#currentmonitor").text("Moniteur de courant");
$("#victroncanbus").text("BMS CAN bus emulateur pylontech/victron");
$("#storage").text("Paramétres SDCard et Logg");
$("#avrprogrammer").text("Programmateur de module (AVR)");
$("#about").text("Informations");

$("#modulesPage > h1").text("Modules");

$("#globalConfig > h2").text("Configuration globale");
$("#globalConfig > p").text("Configure tous les modules avec les paramétres suivants:");
$("#mpBank").text("Chaine");
$("#mpModule").text("Cellule");
$("#mpVoltage").text("Tension");
$("#mpvmin").text("U.min");
$("#mpvmax").text("U.max");
$("#mptint").text("Temp PCB °C");
$("#mptext").text("Temp ext °C");
$("#mpbypass").text("Bypass %");
$("#mpbpc").text("Nb erreur paquet");
$("#mppktr").text("Paquets reçus");
$("#mpbal").text("Energie utilisée pour balancing (mAh)");

$("#error1").text("Probleme de communication entre le contrôleur et les modules.");
$("#error3").text("Le contrôleur est configuré pour utiliser plus de modules qu'il ne peut en gérer.");
$("#error4").text("Attente réponse modules");
$("#error5").text("Le module reporte 0V, veuillez controler la configuration");
$("#error6").text("Le contrôleur est à court de mémoire.");
$("#error7").text("Arrêt d'URGENCE");
$("#iperror").text("Impossible de communiquer avec le contrôleur pour les mises à jour d'état.");
$("#jslibrary").text("La bibliothèque Javascript n'a pas pu se charger correctement, veuillez actualiser la page.");
$("#saveerror").text("Échec de l'enregistrement des paramètres.");
$("#savesuccess").text("Paramétres sauvegardés");

$("#warning1").text("Avertissement : La tension de bypass du module est différente du réglage global");
$("#warning2").text("Avertissement : La température de dérivation du module est différente du réglage global");
$("#warning3").text("Avertissement : Les modules ont des versions de code mixtes, ce qui peut entraîner une instabilité");
$("#warning4").text("Avertissement : Les modules ont des versions mixtes de matériel/cartes");
$("#warning5").text("Avertissement : journalisation activée mais carte SD non installée/trouvée");
$("#warning6").text("Certaines fonctionnalités sont désactivées lorsque le mode de programmation AVR est activé");
$("#warning7").text("La page Web n'est pas synchronisée avec le contrôleur, actualisez la page Web (F5)");

$("label[for='NTPServer']").text("Serveur NTP");
$("label[for='NTPZoneHour']").text("Décallage (heure)"); 
$("label[for='NTPZoneMin']").text("Décallage (minute)");
$("label[for='NTPDST']").text("Activation heure d'été");
$("label[for='VoltageHigh']").text("Echelle tension haute graphique");
$("label[for='VoltageLow']").text("Echelle tension basse graphique");




//DIYBMS supports relay modules to safely disconnect chargers, contactors or consumers. The rules allow you to configure the relays for your situation.
$("#rt1").text("DIYBMS prend en charge les modules de relais pour déconnecter en toute sécurité les chargeurs, les contacteurs ou les consommateurs. Les règles permettent de configurer les relais selon votre installation.");
//Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of "X" means don't care/leave at value defined by lower priority rules.
$("#rt2").text("Les règles sont traitées de la priorité la plus faible à la plus élevée (de bas en haut). Contrôler les relais à l'aide des options. La valeur signifie pas d'action/laisser à la valeur définie par des règles de priorité basse..");
//Rules are triggered when the relevant value meets or exceeds the 'trigger' value. The rule will only disable when the value then passes the 'reset' value. This can help prevent relay clatter and rules firing on/off rapidly.
$("#rt3").text("Les règles sont déclenchées lorsque la valeur seuil est atteinte ou dépassée « déclencheur ».La règle ne sera désactivée que lorsque la valeur passera ensuite la valeur (réinitialiser). Cet hysthérésys permet d'éviter les battements relais..");
//'Timer 1' and 'Timer 2' rules allow timed operation, this rule is active when the number of  minutes past midnight has been reached, for instance setting 'Timer 1' trigger to 495 and reset to 555 would switch on at 8:15am and off at 9:15am. This only works if connected to internet for regular time updates.
$("#rt4").text("Les règles 'Tempo 1' et 'Tempo 2' permettent un fonctionnement temporisé, cette règle est active lorsque le nombre de minutes après minuit a été atteint, par exemple, régler le déclencheur 'Tempo 1' sur 495 et réinitialiser sur 555 = actif à 8h15 et innactif à 9h15. Cela ne fonctionne que si vous êtes connecté à Internet pour des mises à jour NTP.");
//Minutes since midnight now is: 
$("#rt5").text("Les minutes depuis minuit sont à cet instant de:");
//Emergency stop is triggered by connector J1, once triggered controller needs to be reset to disable.
$("#rt6").text("L'arrêt d'urgence est déclenché par le connecteur J1, une fois déclenché, le contrôleur doit être réinitialisé pour être désactivé.");


//Rule
$("#rf1").text("Régles");
//Trigger value
$("#rf2").text("Valeur de marche");
//Reset value
$("#rf3").text("Valeur d'arret");
//Relay state
$("#rf4").text("Etat relais");


//Emergency Stop
$("label[for='rule0value']").text("Arret d'urgence");
//Internal BMS error
$("label[for='rule1value']").text("Erreur interne BMS");
//Current monitoring over current (Amps)
$("label[for='rule2value']").text("Surintensité vu par le moniteur de courant (A)");
//Individual cell over voltage (mV)
$("label[for='rule3value']").text("Seuil de surtension cellule individuelle (mV)");
//Cell under voltage (mV)
$("label[for='rule4value']").text("Seuil de soustension cellule individuelle (mV))");
//Module over temperature (internal) °C	= 
$("label[for='rule5value']").text("PCB Surtemperature °C");
//Module under temperature (internal)
$("label[for='rule6value']").text("PCB Soustemperature °C");
//Cell over temperature (external)
$("label[for='rule7value']").text("Cellule Surtemperature (Ext) °C");
//Cell under temperature (external)
$("label[for='rule8value']").text("Cellule Soustemperature (Ext) °C");
//Bank over voltage (mV)
$("label[for='rule9value']").text("Surtention chaîne (mV)");
//Bank under voltage (mV)
$("label[for='rule10value']").text("Soustention chaîne (mV)");
//Bank over voltage (mV)
//Bank under voltage (mV)
//Bank range/deviation (mV)
//Timer 2
$("label[for='rule14value']").text("Tempo 2");
//Timer 1
$("label[for='rule15value']").text("Tempo 1");
//Relay default
$("label[for='defaultvalue']").text("Défault relais");
//Relay type
$("label[for='relaytype']").text("Type de Relais");



//Bypass over temperature
$("label[for='g1']").text("Surtempérature by-pass");
//Bypass threshold mV
$("label[for='g2']").text("Seuil de by-pass mV");

$("#sent > span.x.t").text("Paquets envoyés:");
$("#received > span.x.t").text("Packets reçus:");
$("#roundtrip > span.x.t").text("vitesse com (ms):");

//Integration
$("#ip1").text("Integration");
//For security, you will need to re-enter the password for the service(s) you want to enable or modify, before you save.
$("#ip2").text("Pour des raisons de sécurité, vous devrez ressaisir le mot de passe du ou des services que vous souhaitez activer ou modifier avant d'enregistrer.");
//After changes are made, the controller will need to be rebooted, do this manually.
$("#ip3").text("Une fois les modifications apportées, le contrôleur devra être redémarré, faites-le manuellement.");

//WARNING
$("#ap5").text("! ATTENTION !");
//This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life
$("#ap6").text("Il s'agit d'un produit/solution non professionnels, ne l'utilisez donc pas pour des systèmes critiques pour la sécurité ou dans toute situation où il pourrait y avoir un risque de mort..");
//There is no warranty, it may not work as expected or at all.
$("#ap7").text("Il n'y a aucune garantie, cela peut ne pas fonctionner comme prévu ou pas du tout..");
//The use of this project is done so entirely at your own risk. It may involve electrical voltages which could kill - if in doubt, seek help.
$("#ap8").text("L'utilisation de ce projet se fait entièrement à vos risques et périls. Cela peut impliquer des tensions électriques qui pourraient tuer - en cas de doute, demandez de l'aide.");
//The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.
$("#ap8a").text("L'utilisation de ce projet peut ne pas être conforme aux lois ou réglementations locales - en cas de doute, demandez de l'aide.");

//Modules & Banks
$("#mb1").text("Modules & Chaines");
//DIYBMS supports up to 100 modules in total. These modules can be split into banks to support parallel configurations.
$("#mb2").text("DIYBMS prend en charge jusqu'à 128 modules au total. Ces modules peuvent être divisés en chaines pour prendre en charge des configurations parallèles.");
//Example: You have 16 cells configured as 8 in series and 2 in parallel (8S2P).
$("#mb3").text("Exemple : Vous avez 16 cellules configurées avec 8 en série et 2 en parallèle (8S2P)");
$("#mb4").text("Seule la version 4.4 ou ultérieure du module prend en charge des vitesses de communication plus rapides. Vous devez redémarrer le contrôleur manuellement si vous modifiez la vitesse, et assurez-vous également que tous les modules utilisent le bon firmware.");
                                


