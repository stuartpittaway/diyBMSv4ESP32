function identifyModule(button, bank, module) {
  //Populate settings div
  $.getJSON("identifyModule.json",{ b:bank, m:module }, function(data) { }).fail(function() {$("#iperror").show();});
}

function configureModule(button, bank, module) {
  //Select correct row in table
  $(button).parent().parent().parent().find(".selected").removeClass("selected");
  $(button).parent().parent().addClass("selected");

  //Populate settings div
  $("#settingConfig h2").html("Settings for module bank:"+bank+" module:"+module);
  $("#settingConfig").show();

  $.getJSON( "modules.json",  {b:bank,m:module},
    function(data) {

      var div=$("#settingConfig .settings");
      $('#b').val(data.settings.bank);
      $('#m').val(data.settings.module);

      if (data.settings.Cached==true){
        $('#Version').val(data.settings.ver);
        $('#BypassOverTempShutdown').val(data.settings.BypassOverTempShutdown);
        $('#BypassThresholdmV').val(data.settings.BypassThresholdmV);
        $('#Calib').val(data.settings.Calib.toFixed(4));
        $('#ExtBCoef').val(data.settings.ExtBCoef);
        $('#IntBCoef').val(data.settings.IntBCoef);
        $('#LoadRes').val(data.settings.LoadRes.toFixed(2));
        $('#mVPerADC').val(data.settings.mVPerADC.toFixed(2));
        $('#movetobank').val(data.settings.bank);

        $('#settingsForm').show();
        $('#waitforsettings').hide();
      } else {
        //Data not ready yet
        $('#settingsForm').hide();
        $('#waitforsettings').show();
        //Call back in 5 seconds to refresh page - this is a bad idea!
        //setTimeout(configureModule, 5000, button, bank, module);
      }
    }).fail(function() {
     $("#iperror").show();
  });
}

function queryBMS() {

  $.getJSON( "monitor.json", function( jsondata ) {
    var labels = [];
    var cells = [];
    var bank = [];
    var voltages = [];
    var voltagesmin = [];
    var voltagesmax = [];
    var tempint = [];
    var tempext = [];

    var badpktcount = [];

    var voltage = [0.0, 0.0, 0.0, 0.0];
    //Not currently supported
    var current = [0.0, 0.0, 0.0, 0.0];

    var bankmin = [5000,5000,5000,5000];
    var bankmax = [0.0,0.0,0.0,0.0];

    var minVoltage=2.5;
    var maxVoltage=4.5;

    for (var bankNumber = 0; bankNumber < jsondata.banks; bankNumber++) {
        //Need to cater for banks of cells
        $.each(jsondata.bank[bankNumber], function(index, value) {
            
          //Should be in stylesheet....
            var color = value.bypass ? "#B44247" : "#55a1ea";

            var v = (parseFloat(value.v) / 1000.0);

            //Auto scale graph is outside of normal bounds
            if (v>maxVoltage) { maxVoltage=v;}
            if (v<minVoltage) { minVoltage=v;}

            voltages.push({ value: v, itemStyle: { color: color } });
            voltagesmin.push((parseFloat(value.minv) / 1000.0));
            voltagesmax.push((parseFloat(value.maxv) / 1000.0));

            //TODO: This looks incorrect needs to take into account bank/cell configs
            bank.push(bankNumber);
            cells.push(index);
            badpktcount.push(value.badpkt);
            labels.push(bankNumber + "/" + index);

            color = value.bypasshot ? "#B44247" : "#55a1ea";
            tempint.push({ value: value.int, itemStyle: { color: color } });
            tempext.push({value: (value.ext == -40 ? 0 : value.ext), itemStyle: { color: "#55a1ea" } });

            var bIndex=jsondata.parallel ? bankNumber:0;
            voltage[bIndex] += v;
            if (value.v<bankmin[bIndex]) {bankmin[bIndex]=value.v;}
            if (value.v>bankmax[bIndex]) {bankmax[bIndex]=value.v;}
        });
    }

    //Ignore and hide any errors which are zero
    if (jsondata.monitor.badcrc==0) { $("#badcrc").hide(); } else { $("#badcrc .v").html(jsondata.monitor.badcrc);$("#badcrc").show();}
    if (jsondata.monitor.ignored==0) { $("#ignored").hide(); } else { $("#ignored .v").html(jsondata.monitor.ignored);$("#ignored").show();}

    if (jsondata.monitor.sent==0) { $("#sent").hide(); } else { $("#sent .v").html(jsondata.monitor.sent);$("#sent").show();}
    if (jsondata.monitor.received==0) { $("#received").hide(); } else { $("#received .v").html(jsondata.monitor.received);$("#received").show();}
    if (jsondata.monitor.roundtrip==0) { $("#roundtrip").hide(); } else { $("#roundtrip .v").html(jsondata.monitor.roundtrip);$("#roundtrip").show();}

    for (var bankNumber = 0; bankNumber < 4; bankNumber++) {
      if (voltage[bankNumber]>0) {
        $("#voltage"+(bankNumber+1)+" .v").html(voltage[bankNumber].toFixed(2)+"V");
        var range=bankmax[bankNumber]-bankmin[bankNumber];
        $("#range"+(bankNumber+1)+" .v").html(range+"mV");

        $("#voltage"+(bankNumber+1)).show();
        $("#range"+(bankNumber+1)).show();
      } else {
        $("#voltage"+(bankNumber+1)).hide();
        $("#range"+(bankNumber+1)).hide();
      }
    }

    //Not currently supported
    $("#current").hide();
    $("#current .v").html(current[0].toFixed(2));

    if (jsondata.monitor.commserr==true) {
      $("#commserr").show();
    } else {
      $("#commserr").fadeOut();
    }

    $("#info").show();

    $("#iperror").hide();

    if($('#modulesPage').is(':visible')){
        var tbody=$("#modulesRows");

        if ($('#modulesRows div').length!=cells.length) {
            $("#settingConfig").hide();

            //Add rows if they dont exist (or incorrect amount)
            $(tbody).find("div").remove();

            $.each(cells, function( index, value ) {
                $(tbody).append("<div><span>"
                +bank[index]
                +"</span><span>"+value+"</span><span></span><span class='hide'></span><span class='hide'></span>"
                +"<span class='hide'></span><span class='hide'></span><span class='hide'></span>"
                +"<span><button type='button' onclick='return identifyModule(this,"+bank[index]+","+value+");'>Identify</button></span>"
                +"<span><button type='button' onclick='return configureModule(this,"+bank[index]+","+value+");'>Configure</button></span></div>")
            });
        }

        var rows=$(tbody).find("div");

        $.each(cells, function( index, value ) {
            var columns=$(rows[index]).find("span");

            //$(columns[0]).html(value);
            $(columns[2]).html(voltages[index].value.toFixed(3));
            $(columns[3]).html(voltagesmin[index].toFixed(3));
            $(columns[4]).html(voltagesmax[index].toFixed(3));
            $(columns[5]).html(tempint[index].value);
            $(columns[6]).html(tempext[index]);
            $(columns[7]).html(badpktcount[index]);
        });
    }


    if($('#homePage').is(':visible')){

      
      if (g1==null) {
        // based on prepared DOM, initialize echarts instance
        g1 = echarts.init(document.getElementById('graph1'));

        var labelOption = {
            normal: {
                show: true,
                position: 'insideBottom',
                distance: 15,
                align: 'left',
                verticalAlign: 'middle',
                rotate: 90,
                formatter: '{c}V',
                fontSize: 24, color: '#eeeeee',fontFamily: 'Inconsolata'                  
            }
        };

        var labelOption3 = {
            normal: {
                show: true,
                position: 'top',
                distance:5,
                formatter: '{c}V',
                fontSize: 14, color: '#c1bdbd',fontFamily: 'Inconsolata'
            }
        };

        var labelOption4 = {
            normal: {
                show: true,
                position: 'bottom',
                distance:5,
                formatter: '{c}V',
                fontSize: 14, color: '#807d7d',fontFamily: 'Inconsolata'
            }
        };

        var labelOption2 = {
              normal: {
                  show: true,
                  position: 'insideBottom',
                  distance: 15,
                  align: 'left',
                  verticalAlign: 'middle',
                  rotate: 90,
                  formatter: '{c}°C',
                  fontSize: 20, color: '#eeeeee'
                  ,fontFamily: 'Inconsolata'
              }
          };

        
        // specify chart configuration item and data
        var option = {
    //            color: ['#c1bdbd', '#c1bdbd', '#c1bdbd'],
          tooltip: { trigger:'axis', show:true },
            legend: { data:['Voltage'], show:false },
            xAxis: [
              {gridIndex:0,type:'category',axisLine:{lineStyle:{color:'#c1bdbd'}} }
            ,{gridIndex:1,type:'category',axisLine:{lineStyle:{color:'#c1bdbd'}} }
            ],
            yAxis: [
              {gridIndex:0,name:'Volts',type:'value',min:minVoltage,max:maxVoltage
              ,interval:0.25,position:'left'
              ,axisLine:{lineStyle:{color:'#c1bdbd'}}
              ,axisLabel:{formatter:'{value}V'}              
            },
              
              {gridIndex:1,name:'Temperature',type:'value',interval:10,position:'left'
              ,axisLine:{lineStyle:{color:'#c1bdbd'}}
              ,axisLabel:{ formatter: '{value}°C' }              
            } 
            ]
            ,series: [{ name: 'Voltage', type: 'bar', data: [], label:labelOption }
                ,{name:'Min V', type:'line', data: [], label: labelOption4,symbolSize:20,symbol:['circle'], itemStyle:{normal:{lineStyle:{color:'transparent',type:'dotted'}} } }
                ,{name:'Max V', type:'line', data: [], label: labelOption3,symbolSize:20,symbol:['triangle'], itemStyle:{normal:{lineStyle:{color:'transparent',type:'dotted'}} } }
                ,{xAxisIndex:1, yAxisIndex:1, name:'BypassTemperature',type:'bar', data: [], label: labelOption2 }
                ,{xAxisIndex:1, yAxisIndex:1, name:'CellTemperature',type:'bar',data: [], label: labelOption2 }
            ],
            grid: [
              {containLabel:false, left:'5%', right:'5%', bottom:'32%'}
             ,{containLabel:false, left:'5%', right:'5%', top:'78%'}
            ],
        };

        // use configuration item and data specified to show chart
        g1.setOption(option);
      }


      if (g1!=null) {
          g1.setOption({
              xAxis: { data: labels },
              yAxis: [ {gridIndex:0,min:minVoltage,max:maxVoltage}]
              ,series: [{ name: 'Voltage', data: voltages }
              ,{ name: 'Min V', data: voltagesmin }
              ,{ name: 'Max V', data: voltagesmax }
              ,{ name: 'BypassTemperature', data: tempint }
              ,{ name: 'CellTemperature', data: tempext }]
          });
        }
    }//end homepage visible

    //Call again in a few seconds
    setTimeout(queryBMS, 4000);

    $("#loading").hide();

  }).fail(function() {
     $("#iperror").show();
      //Try again in a few seconds (2 seconds if errored)
      setTimeout(queryBMS, 2000);
      $("#loading").hide();
  });
}

$(window).on('resize', function(){ if(g1 != null && g1 != undefined && $('#homePage').is(':visible')){g1.resize();}});

