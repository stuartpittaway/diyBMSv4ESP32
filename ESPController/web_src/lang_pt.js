// This file is for Portuguese language translation
// w3schools.com/tags/ref_language_codes.asp

$("#home").text("Home"); //For me is better not translate "Home"
$("#modules").text("Módulos");
$("#settings").text("Ajustes");
$("#rules").text("Regras");
$("#more").text("Mais");

$("#integration").text("Integração");
$("#currentmonitor").text("Monitor de corrente diyBMS");
$("#storage").text("almacenamiento de registros");
$("#avrprogrammer").text("programador de AVR");
$("#about").text("Informação");

$("#modulesPage > h1").text("Módulos");

$("#globalConfig > h2").text("Configuração global");
$("#globalConfig > p").text("Configuração de todos os módulos conforme os siguintes parâmetros");
$("#mpBank").text("Banco");
$("#mpModule").text("Módulo");
$("#mpVoltage").text("Voltagem");
$("#mpvmin").text("V. Mínima");
// nota: aquí seguramente falte  $("#mpvmax").text("V. Máxima");
$("#mptint").text("Temp interna °C");
$("#mptext").text("Temp externa °C");
$("#mpbypass").text("Bypass %"); //For me is better not translate "Bypass"
$("#mpbpc").text("Contagem de pacotes incorrectos");
$("#mppktr").text("Pacotes recebidos");
$("#mpbal").text("Energia consumida a balacear (mAh)");

$("#error1").text("O controlador tem dificultades para comunicar com os módulos de monitorização de célula.");
$("#error3").text("O controlador está configurado para utilizar mais módulos que os que pode soportar.");
$("#error4").text("Esperando que os módulos respondam");
$("#error5").text("O módulo devolveu a leitura de ZERO volts, verifique a configuracão");
$("#error6").text("O controlador ficou sem memória disponível.");
$("#error7").text("Paragem de emergência");
$("#iperror").text("Não se conseguiu comunicar com o controlador para actualizações de estado.");
$("#jslibrary").text("A biblioteca de JavaScript não se carregou correctamente, refresque a página.");
$("#saveerror").text("Erro ao guardar a configuracão.");
$("#savesuccess").text("Configuracão guardada com sucesso");

$("#warning1").text("Aviso: A tensão de bypass do módulo é diferente da configuracão global");
$("#warning2").text("Aviso: A temperatura de bypass do módulo é diferente da configuracão global");
$("#warning3").text("Aviso: Os módulos têm versões diferentes de código, pode causar instabilidade");
$("#warning4").text("Aviso: Os módulos têm versões diferentes de hardware / placas");
$("#warning5").text("Aviso: registo ativado mas cartão SD não instalado / encontrado");
$("#warning6").text("Algumas funções estão desativadas enquanto o modo de programação AVR está ativo");

$("label[for='NTPServer']").text("Servidor NTP");
$("label[for='NTPZoneHour']").text("Zona horária (hour)"); 

//DIYBMS supports relay modules to safely disconnect chargers, contactors or consumers. The rules allow you to configure the relays for your situation.
$("#rt1").text("DIYBMS suporta módulos de relé para desligar carregadores, contatores ou consumidores com segurança. As regras permitem que configure os relés para a sua situação.");
//Rules are processed from lowest priorty to highest (bottom up). Control the relays using the options. A value of "X" means don't care/leave at value defined by lower priority rules.
$("#rt2").text("As regras são processadas da prioridade mais baixa para a mais alta (de baixo para cima). Controle os relés usando as opções. Um valor de "X" significa deixar no valor definido por regras de prioridade mais baixa.");
//Rules are triggered when the relevant value meets or exceeds the 'trigger' value. The rule will only disable when the value then passes the 'reset' value. This can help prevent relay clatter and rules firing on/off rapidly.
$("#rt3").text("As regras são acionadas quando o valor relevante iguala ou excede o valor 'acionador'. A regra só desliga quando o valor passar o valor de 'reset'. Isso pode ajudar a evitar o ruído do relé e as regras ligando / desligando rapidamente.");
//'Timer 1' and 'Timer 2' rules allow timed operation, this rule is active when the number of minutes past midnight has been reached, for instance setting 'Timer 1' trigger to 495 and reset to 555 would switch on at 8:15am and off at 9:15am. This only works if connected to internet for regular time updates.
$("#rt4").text("As regras do 'Temporizador 1' e 'Temporizador 2' permitem a operação cronometrada, esta regra está ativa quando o número de minutos após a meia-noite for atingido, por exemplo, definir o gatilho do 'Cronómetro 1' para 495 e reset para 555 ligaria às 8h15 e desligava às 9h15. Isso só funciona se estiver conectado à Internet para atualizações regulares de horário.");
//Minutes since midnight now is: 
$("#rt5").text("Minutos desde a meia-noite, agora são:");
//Emergency stop is triggered by connector J1, once triggered controller needs to be reset to disable.
$("#rt6").text("A paragem de emergência é acionada pelo conector J1, uma vez acionado o controlador precisa ser reiniciado para desativar.");


//Rule
$("#rf1").text("Regras");
//Trigger value
$("#rf2").text("Valor de activação");
//Reset value
$("#rf3").text("Valor de desativação");
//Relay state
$("#rf4").text("Estado do relé");


//Emergency Stop
$("label[for='rule0value']").text("Paragem de emergência");
//Internal BMS error= 
$("label[for='rule1value']").text("Erro interno do BMS");
//Current monitoring over current (Amps)
//Individual cell over voltage (mV)
$("label[for='rule3value']").text("Sobretensão de célula individual (mV)");
//Cell under voltage (mV)
$("label[for='rule4value']").text("Subtensão de célula individual  (mV)");
//Module over temperature (internal) °C	= 
$("label[for='rule5value']").text("Módulo acima da temperatura (interna) °C");
//Module under temperature (internal)
$("label[for='rule6value']").text("Módulo abaixo da temperatura (interna) °C");
//Cell over temperature (external)
$("label[for='rule7value']").text("Célula acima da temperatura (externa) °C");
//Cell under temperature (external)
$("label[for='rule8value']").text("Célula abaixo da temperatura (externa) °C");
//Bank over voltage (mV)
$("label[for='rule9value']").text("Sobretensão do conjunto (mV)");
//Bank under voltage (mV)
$("label[for='rule10value']").text("Subtensão do conjunto (mV)");
//Bank over voltage (mV)
//Bank under voltage (mV)
//Bank range/deviation (mV)
//Timer 2
$("label[for='rule14value']").text("Temporizador 2");
//Timer 1
$("label[for='rule15value']").text("Temporizador 1");
//Relay default
$("label[for='defaultvalue']").text("Configuração por defeito do Relé");
//Relay type = 
$("label[for='relaytype']").text("Tipo de Relé");



//Bypass over temperature
$("label[for='g1']").text("Bypass acima da temperatura");
//Bypass threshold mV
$("label[for='g2']").text("Bypass threshold mV"); // need some context in order to translate this

$("#sent > span.x.t").text("Pacotes enviados:");
$("#received > span.x.t").text("Pacotes recebidos:");
$("#roundtrip > span.x.t").text("Ida e volta (ms):");

//Integration
$("#ip1").text("Integração");
//For security, you will need to re-enter the password for the service(s) you want to enable or modify, before you save.
$("#ip2").text("Por segurança, precisará inserir novamente a senha do(s) serviço(s) que deseja ativar ou modificar antes de salvar.");
//After changes are made, the controller will need to be rebooted, do this manually.
$("#ip3").text("Após as alterações serem feitas, o controlador precisará ser reinicializado, faça isso manualmente.");

//WARNING
$("#ap5").text("AVISO!");
//This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.
$("#ap6").text("Este é um produto/solução faça-você-mesmo, não o use para sistemas críticos de segurança ou em qualquer situação onde possa haver risco de vida.");
//There is no warranty, it may not work as expected or at all.
$("#ap7").text("Não há qualquer garantia, pode não funcionar como o esperado ou nem funcionar de todo.");
//The use of this project is done so entirely at your own risk. It may involve electrical voltages which could kill - if in doubt, seek help.
$("#ap8").text("A utilização deste projeto é feita por sua própria conta e risco. Pode envolver tensões elétricas que podem matar - em caso de dúvida, procure ajuda.");
//The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.
$("#ap8a").text("O uso deste projeto pode não estar em conformidade com as leis ou regulamentações locais - em caso de dúvida, procure ajuda.");

//Modules & Banks
$("#mb1").text("Módulos e bancos");
//DIYBMS supports up to 100 modules in total. These modules can be split into banks to support parallel configurations.
$("#mb2").text("DIYBMS suporta até 100 módulos no total. Esses módulos podem ser divididos em bancos para suportar configurações paralelas.");
//Example: You have 16 cells configured as 8 in series and 2 in parallel (8S2P).
$("#mb3").text("Exemplo: pode ter 16 células configuradas como 8 em série e 2 em paralelo (8S2P)");

