// This file is for SPANISH language translation
// w3schools.com/tags/ref_language_codes.asp

$("#home").text("Home"); //For me is better not translate "Home"
$("#modules").text("Módulos");
$("#settings").text("Ajustes");
$("#rules").text("Reglas");
$("#more").text("Más");

$("#integration").text("Integración");
$("#currentmonitor").text("diyBMS monitoreo de amperaje");
$("#storage").text("almacenamiento de registros");
$("#avrprogrammer").text("programador de AVR");
$("#about").text("Información");

$("#modulesPage > h1").text("Módulos");

$("#globalConfig > h2").text("Configuración global");
$("#globalConfig > p").text("Configuración de todos los módulos conforme a los siguientes parametros");
$("#mpBank").text("Banco");
$("#mpModule").text("Módulo");
$("#mpVoltage").text("Voltaje");
$("#mpvmin").text("V. Mínimo");
$("#mpvmax").text("V. Máximo");
$("#mptint").text("Temp interna °C");
$("#mptext").text("Temp externa °C");
$("#mpbypass").text("Bypass %"); //For me is better not translate "Bypass"
$("#mpbpc").text("Recuento de paquetes incorrectos");
$("#mppktr").text("Paquetes recibidos");
$("#mpbal").text("Energía consumida en balanceo (mAh)");

$("#error1").text("El controlador tiene dificultades para comunicarse con los módulos de monitoreo de celda.");
$("#error3").text("El controlador está configurado para utilizar más módulos de los que puede soportar.");
$("#error4").text("Esperando que los módulos respondan");
$("#error5").text("El módulo ha devuelto la lectura de CERO voltios, verifique la configuración");
$("#error6").text("El controlador se ha quedado sin memoria.");
$("#error7").text("Parada de emergencia");
$("#iperror").text("No se puede comunicar con el controlador para actualizaciones de estado.");
$("#jslibrary").text("La biblioteca de JavaScript no se ha cargado correctamente, refresque la página.");
$("#saveerror").text("No se pudo guardar la configuración.");
$("#savesuccess").text("Ajustes guardados");

$("#warning1").text("Advertencia: El voltaje de bypass del módulo es diferente a la configuración global");
$("#warning2").text("Advertencia: la temperatura de bypass del módulo es diferente a la configuración global");
$("#warning3").text("Advertencia: los módulos tienen versiones mixtas de código, pueden causar inestabilidad");
$("#warning4").text("Advertencia: los módulos tienen versiones mixtas de hardware / placas");
$("#warning5").text("Advertencia: registro habilitado pero tarjeta SD no instalada / encontrada");
$("#warning6").text("Algunas funciones están deshabilitadas mientras el modo de programación AVR está habilitado");

$("label[for='NTPServer']").text("Servidor NTP");
$("label[for='NTPZoneHour']").text("Zona horaria (horas)"); 

//DIYBMS supports relay modules to safely disconnect chargers, contactors or consumers. The rules allow you to configure the relays for your situation.
$("#rt1").text("DIYBMS admite módulos de relé para desconectar de forma segura cargadores, contactores o consumos. Las reglas le permiten configurar los relés para su situación.");
//Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of "X" means don't care/leave at value defined by lower priority rules.
$("#rt2").text("Las reglas se procesan desde la prioridad más baja hasta la más alta (de abajo hacia arriba). Controle los relés usando las opciones. Un valor de 'X' significa que no se activará/desactivará con en el valor definido por las reglas de menor prioridad.");
//Rules are triggered when the relevant value meets or exceeds the 'trigger' value. The rule will only disable when the value then passes the 'reset' value. This can help prevent relay clatter and rules firing on/off rapidly.
$("#rt3").text("Las reglas se activan cuando el valor relevante alcanza o supera el valor de 'activación'. La regla solo se desactivará cuando el valor pase el valor de 'reseteo'. Esto puede ayudar a evitar el traqueteo de los relés y a que las reglas se activen o desactiven rápidamente.");
//'Timer 1' and 'Timer 2' rules allow timed operation, this rule is active when the number of minutes past midnight has been reached, for instance setting 'Timer 1' trigger to 495 and reset to 555 would switch on at 8:15am and off at 9:15am. This only works if connected to internet for regular time updates.
$("#rt4").text("Las reglas 'Temporizador 1' y 'Temporizador 2' permiten el funcionamiento temporizado, esta regla está activa cuando se alcanza el número de minutos después de la medianoche, por ejemplo, si se configura el disparador del 'Temporizador 1' en 495 y se restablece en 555, se encendería a las 8:15 a. M. y sale a las 9:15 am. Esto solo funciona si está conectado a Internet para actualizaciones periódicas.");
//Minutes since midnight now is: 
$("#rt5").text("Minutos desde la medianoche ahora es:");
//Emergency stop is triggered by connector J1, once triggered controller needs to be reset to disable.
$("#rt6").text("La parada de emergencia se activa mediante el conector J1, una vez activado, el controlador debe reiniciarse para deshabilitarlo.");


//Rule
$("#rf1").text("Regla");
//Trigger value
$("#rf2").text("Valor de activación");
//Reset value
$("#rf3").text("Valor de reseteo");
//Relay state
$("#rf4").text("Estado del relé");


//Emergency Stop
$("label[for='rule0value']").text("Parada de emergencia");
//Internal BMS error
$("label[for='rule1value']").text("Error interno de BMS");
//Current monitoring over current (Amps)
$("label[for='rule2value']").text("Sobrecorriente del monitor de corriente (A)");
//Individual cell over voltage (mV)
$("label[for='rule3value']").text("Celda individual por encima de voltaje (mV)");
//Cell under voltage (mV)
$("label[for='rule4value']").text("Celda individual por debajo de voltaje (mV)");
//Module over temperature (internal) °C
$("label[for='rule5value']").text("Módulo por encima de temperatura (interna) °C");
//Module under temperature (internal)
$("label[for='rule6value']").text("Módulo por debajo de temperatura (interna) °C");
//Cell over temperature (external)
$("label[for='rule7value']").text("Celda por encima de temperatura (externa) °C");
//Cell under temperature (external)
$("label[for='rule8value']").text("Celda por debajo de temperatura (externa) °C");
//Current monitor over voltage (mV)
$("label[for='rule9value']").text("Monitor de corriente por encima de voltaje (mV)");
//Current monitor under voltage (mV)
$("label[for='rule10value']").text("Monitor de corriente por debajo de voltaje (mV)");
//Bank over voltage (mV)
$("label[for='rule11value']").text("Banco por encima de voltaje (mV)");
//Bank under voltage (mV)
$("label[for='rule12value']").text("Banco por debajo de voltaje (mV)");
//Bank range/deviation (mV)
$("label[for='rule13value']").text("Rango/desviación del banco (mV)");
//Timer 2
$("label[for='rule14value']").text("Temporizador 2");
//Timer 1
$("label[for='rule15value']").text("Temporizador 1");



//Bypass over temperature
$("label[for='g1']").text("Sobretemperatura de bypass");
//Bypass threshold mV
$("label[for='g2']").text("Umbral de bypass mV");

$("#sent > span.x.t").text("Paquetes enviados:");
$("#received > span.x.t").text("Paquetes recibidos:");
$("#roundtrip > span.x.t").text("Ida y vuelta (ms):");

//Integration
$("#ip1").text("Integración");
//For security, you will need to re-enter the password for the service(s) you want to enable or modify, before you save.
$("#ip2").text("Por seguridad, deberá volver a introducir la contraseña de los servicios que desee habilitar o modificar antes de guardar.");

//WARNING
$("#ap5").text("ADVERTENCIA");
//This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.
$("#ap6").text("Este es un producto/solución DIY (hágalo usted mismo), así que no lo utilice en sistemas críticos para la seguridad ni en ninguna situación en la que pueda existir riesgo para la vida.");
//There is no warranty, it may not work as expected or at all.
$("#ap7").text("No hay garantía; puede que no funcione como se espera o que no funcione en absoluto.");
//The use of this project is done so entirely at your own risk. It may involve electrical voltages which could kill - if in doubt, seek help.
$("#ap8").text("El uso de este proyecto es enteramente bajo su propio riesgo. Puede implicar tensiones eléctricas que podrían matar; en caso de duda, busque ayuda.");
//The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.
$("#ap8a").text("Es posible que el uso de este proyecto no cumpla las leyes o normativas locales; en caso de duda, busque ayuda.");

//Modules & Banks
$("#mb1").text("Módulos y bancos");
//DIYBMS supports up to 100 modules in total. These modules can be split into banks to support parallel configurations.
$("#mb2").text("DIYBMS admite hasta 100 módulos en total. Estos módulos se pueden dividir en bancos para admitir configuraciones en paralelo.");
//Example: You have 16 cells configured as 8 in series and 2 in parallel (8S2P).
$("#mb3").text("Ejemplo: tiene 16 celdas configuradas como 8 en serie y 2 en paralelo (8S2P).");

