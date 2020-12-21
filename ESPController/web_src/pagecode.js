const INTERNALWARNINGCODE = {
    NoWarning: 0,
    ModuleInconsistantBypassVoltage: 1,
    ModuleInconsistantBypassTemperature: 2,
    ModuleInconsistantCodeVersion: 3,
    ModuleInconsistantBoardRevision: 4
}
Object.freeze(INTERNALWARNINGCODE);

const INTERNALERRORCODE =
{
    NoError: 0,
    CommunicationsError: 1,
    ModuleCountMismatch: 2,
    TooManyModules: 3,
    WaitingForModulesToReply: 4,
    ZeroVoltModule: 5,
    ControllerMemoryError: 6

};
Object.freeze(INTERNALERRORCODE);

function identifyModule(button, cellid) {
    $.getJSON("identifyModule.json", { c: cellid }, function (data) { }).fail(function () { $("#iperror").show(); });
}

function configureModule(button, cellid, attempts) {
    $('#loading').show();

    //Select correct row in table
    $(button).parent().parent().parent().find(".selected").removeClass("selected");
    $(button).parent().parent().addClass("selected");

    $.getJSON("modules.json", { c: cellid },
        function (data) {
            var div = $("#settingConfig .settings");
            $('#c').val(data.settings.id);
            $('#m').val(data.settings.id);

            if (data.settings.Cached == true) {
                var currentReading = parseFloat($("#modulesRows > tr.selected > td:nth-child(3)").text());
                $("#ActualVoltage").val(currentReading.toFixed(3));

                $("#settingConfig h2").html("Settings for module bank:" + data.settings.bank + " module:" + data.settings.module);

                //Populate settings div
                $('#ModuleId').val(data.settings.id);
                $('#Version').val(data.settings.ver.toString() + '/' + data.settings.code.toString(16));
                $('#BypassOverTempShutdown').val(data.settings.BypassOverTempShutdown);
                $('#BypassThresholdmV').val(data.settings.BypassThresholdmV);
                $('#Calib').val(data.settings.Calib.toFixed(4));
                $('#ExtBCoef').val(data.settings.ExtBCoef);
                $('#IntBCoef').val(data.settings.IntBCoef);
                $('#LoadRes').val(data.settings.LoadRes.toFixed(2));
                $('#mVPerADC').val(data.settings.mVPerADC.toFixed(2));

                $("#settingConfig").show();
                $('#loading').hide();
            } else {
                //Data not ready yet, we will have to try again soon
                $('#settingConfig').hide();

                if (attempts > 0) {
                    //Call back to refresh page, only try for a limited number of attempts
                    attempts--;
                    setTimeout(configureModule, 1500, button, cellid, attempts);
                }
            }
        }).fail(function () {
            $("#iperror").show();
        });
}

function queryBMS() {
    $.getJSON("monitor2.json", function (jsondata) {
        var labels = [];
        var cells = [];
        var bank = [];
        var voltages = [];
        var voltagesmin = [];
        var voltagesmax = [];
        var tempint = [];
        var tempext = [];
        var pwm = [];


        var minVoltage = DEFAULT_GRAPH_MIN_VOLTAGE;
        var maxVoltage = DEFAULT_GRAPH_MAX_VOLTAGE;

        var bankNumber = 0;
        var cellsInBank = 0;

        // Need one color for each pack, could make it colourful I suppose :-)
        const colours = [
            '#55a1ea', '#33628f', '#55a1ea', '#33628f',
            '#55a1ea', '#33628f', '#55a1ea', '#33628f',
            '#55a1ea', '#33628f', '#55a1ea', '#33628f',
            '#55a1ea', '#33628f', '#55a1ea', '#33628f',
        ]

        const red = '#B44247'

        var markLineData = [];

        markLineData.push({ name: 'avg', type: 'average', lineStyle: { color: '#ddd', width: 2, type: 'dotted', opacity: 0.3 }, label: { distance: [10, 0], position: 'start' } });
        markLineData.push({ name: 'min', type: 'min', lineStyle: { color: '#ddd', width: 2, type: 'dotted', opacity: 0.3 }, label: { distance: [10, 0], position: 'start' } });
        markLineData.push({ name: 'max', type: 'max', lineStyle: { color: '#ddd', width: 2, type: 'dotted', opacity: 0.3 }, label: { distance: [10, 0], position: 'start' } });

        var xAxis = 0;
        for (let index = 0; index < jsondata.banks; index++) {
            markLineData.push({ name: "Bank " + index, xAxis: xAxis });
            xAxis += jsondata.seriesmodules;
        }

        if (jsondata.voltages) {
            for (let i = 0; i < jsondata.voltages.length; i++) {
                labels.push(bankNumber + "/" + i);

                // Make different banks different colours (stripes)
                var stdcolor = colours[bankNumber];
                // Red
                var color = jsondata.bypass[i] == 1 ? red : stdcolor;

                var v = (parseFloat(jsondata.voltages[i]) / 1000.0);
                voltages.push({ value: v, itemStyle: { color: color } });

                //Auto scale graph is outside of normal bounds
                if (v > maxVoltage) { maxVoltage = v; }
                if (v < minVoltage) { minVoltage = v; }

                voltagesmin.push((parseFloat(jsondata.minvoltages[i]) / 1000.0));
                voltagesmax.push((parseFloat(jsondata.maxvoltages[i]) / 1000.0));

                bank.push(bankNumber);
                cells.push(i);

                
                cellsInBank++;
                if (cellsInBank == jsondata.seriesmodules) {
                    cellsInBank = 0;
                    bankNumber++;
                }

                color = jsondata.bypasshot[i] == 1 ? red : stdcolor;
                tempint.push({ value: jsondata.inttemp[i], itemStyle: { color: color } });
                tempext.push({ value: (jsondata.exttemp[i] == -40 ? 0 : jsondata.exttemp[i]), itemStyle: { color: stdcolor } });
                pwm.push({ value: jsondata.bypasspwm[i] == 0 ? null : Math.trunc(jsondata.bypasspwm[i]/255*100) });
            }
        }

        //Scale down for low voltages
        if (minVoltage < 2.5) { minVoltage = 0; }

        if (jsondata) {
            //Ignore and hide any errors which are zero
            if (jsondata.badcrc == 0) { $("#badcrc").hide(); } else { $("#badcrc .v").html(jsondata.badcrc); $("#badcrc").show(); }
            if (jsondata.ignored == 0) { $("#ignored").hide(); } else { $("#ignored .v").html(jsondata.ignored); $("#ignored").show(); }
            if (jsondata.sent == 0) { $("#sent").hide(); } else { $("#sent .v").html(jsondata.sent); $("#sent").show(); }
            if (jsondata.received == 0) { $("#received").hide(); } else { $("#received .v").html(jsondata.received); $("#received").show(); }
            if (jsondata.roundtrip == 0) { $("#roundtrip").hide(); } else { $("#roundtrip .v").html(jsondata.roundtrip); $("#roundtrip").show(); }
            if (jsondata.oos == 0) { $("#oos").hide(); } else { $("#oos .v").html(jsondata.oos); $("#oos").show(); }
        }

        if (jsondata.bankv) {
            for (var bankNumber = 0; bankNumber < jsondata.bankv.length; bankNumber++) {
                $("#voltage" + bankNumber + " .v").html((parseFloat(jsondata.bankv[bankNumber]) / 1000.0).toFixed(2) + "V");
                $("#range" + bankNumber + " .v").html(jsondata.voltrange[bankNumber] + "mV");
                $("#voltage" + bankNumber).show();
                $("#range" + bankNumber).show();
                //$("#bank" + (bankNumber )).show();
            }

            for (var bankNumber = jsondata.bankv.length; bankNumber < MAXIMUM_NUMBER_OF_BANKS; bankNumber++) {
                //$("#bank" + (bankNumber )).hide();
                $("#voltage" + bankNumber).hide();
                $("#range" + bankNumber).hide();
            }
        }

        //Not currently supported
        if (jsondata.current) {
            if (jsondata.current[0] == null) {
                $("#current").hide();
            } else {
                $("#current .v").html((parseFloat(jsondata.current[0]) / 1000.0).toFixed(2));
                $("#current").show();
            }
        }

        //Needs increasing when more warnings are added
        for (let warning = 1; warning <= 4; warning++) {
            if (jsondata.warnings.includes(warning)) {
                $("#warning" + warning).show();
            } else {
                $("#warning" + warning).hide();
            }
        }

        //Needs increasing when more errors are added
        for (let error = 1; error <= 6; error++) {
            if (jsondata.errors.includes(error)) {
                $("#error" + error).show();

                if (error == INTERNALERRORCODE.ModuleCountMismatch) {
                    $("#missingmodule1").html(jsondata.modulesfnd);
                    $("#missingmodule2").html(jsondata.banks * jsondata.seriesmodules);
                }
            } else {
                $("#error" + error).hide();
            }
        }

        $("#info").show();
        $("#iperror").hide();

        if ($('#modulesPage').is(':visible')) {
            //The modules page is visible
            var tbody = $("#modulesRows");

            if ($('#modulesRows tr').length != cells.length) {
                $("#settingConfig").hide();

                //Add rows if they dont exist (or incorrect amount)
                $(tbody).find("tr").remove();

                $.each(cells, function (index, value) {
                    $(tbody).append("<tr><td>"
                        + bank[index]
                        + "</td><td>" + value + "</td><td></td><td class='hide'></td><td class='hide'></td>"
                        + "<td class='hide'></td><td class='hide'></td><td class='hide'></td><td class='hide'></td><td class='hide'></td><td class='hide'></td>"
                        + "<td><button type='button' onclick='return identifyModule(this," + index + ");'>Identify</button>"
                        + "<button type='button' onclick='return configureModule(this," + index + ",10);'>Configure</button></td></tr>")
                });
            }

            var rows = $(tbody).find("tr");

            $.each(cells, function (index, value) {
                var columns = $(rows[index]).find("td");
                $(columns[2]).html(voltages[index].value.toFixed(3));
                $(columns[3]).html(voltagesmin[index].toFixed(3));
                $(columns[4]).html(voltagesmax[index].toFixed(3));
                $(columns[5]).html(tempint[index].value);
                $(columns[6]).html(tempext[index].value);
                $(columns[7]).html(pwm[index].value);
                //$(columns[8]).html(badpktcount[index]);
                //$(columns[9]).html(pktrecvd[index]);
                //$(columns[10]).html(balcurrent[index]);
            });

            //As the module page is open, we refresh the last 3 columns using seperate JSON web service to keep the monitor2.json 
            //packets as small as possible


            $.getJSON("monitor3.json", function (jsondata) {              
                var tbody = $("#modulesRows");
                var rows = $(tbody).find("tr");
                $.each(cells, function (index, value) {
                    var columns = $(rows[index]).find("td");
                    $(columns[8]).html(jsondata.badpacket[index]);
                    $(columns[9]).html(jsondata.pktrecvd[index]);
                    $(columns[10]).html(jsondata.balcurrent[index]);
                });
            });
        }


        if ($('#homePage').is(':visible')) {
            if (window.g1 == null && $('#graph1').css('display') != 'none') {
                // based on prepared DOM, initialize echarts instance
                window.g1 = echarts.init(document.getElementById('graph1'))

                // specify chart configuration item and data
                var option = {
                    tooltip: {
                        show: true,
                        axisPointer: {
                            type: 'cross',
                            label: {
                                backgroundColor: '#6a7985'
                            }
                        }
                    },
                    legend: {
                        show: false
                    },
                    xAxis: [{
                        gridIndex: 0,
                        type: 'category',
                        axisLine: {
                            lineStyle: {
                                color: '#c1bdbd'
                            }
                        }
                    }, {
                        gridIndex: 1,
                        type: 'category',
                        axisLine: {
                            lineStyle: {
                                color: '#c1bdbd'
                            }
                        }
                    }],
                    yAxis: [{
                        id: 0,
                        gridIndex: 0,
                        name: 'Volts',
                        type: 'value',
                        min: 2.5,
                        max: 4.5,
                        interval: 0.25,
                        position: 'left',
                        axisLine: {
                            lineStyle: {
                                color: '#c1bdbd'
                            }
                        },
                        axisLabel: {
                            formatter: function (value, index) {
                                return value.toFixed(2);
                            }
                        }
                    },
                    {
                        id: 1,
                        gridIndex: 0,
                        name: 'Bypass',
                        type: 'value',
                        min: 0,
                        max: 100,
                        interval: 10,
                        position: 'right',
                        axisLabel: { formatter: '{value}%' },
                        splitLine: { show: false },
                        axisLine: { lineStyle: { type: 'dotted', color: '#c1bdbd' } },
                        axisTick: { show: false }
                    },
                    {
                        id: 2,
                        gridIndex: 1,
                        name: 'Temperature',
                        type: 'value',
                        interval: 10,
                        position: 'left',
                        axisLine: {
                            lineStyle: {
                                color: '#c1bdbd'
                            }
                        },
                        axisLabel: {
                            formatter: '{value}°C'
                        }
                    }],
                    series: [
                        {
                            xAxisIndex: 0,
                            name: 'Voltage',
                            yAxisIndex: 0,
                            type: 'bar',
                            data: [],

                            markLine: {
                                silent: true,
                                symbol: 'none',
                                lineStyle: { width: 5, type: 'dashed', opacity: 0.1 },
                                label: { show: true, distance: [0, 0], formatter: '{b}' },
                                data: markLineData
                            },
                            itemStyle: { color: '#55a1ea', barBorderRadius: [8, 8, 0, 0] },
                            label: {
                                normal: {
                                    show: true,
                                    position: 'insideBottom',
                                    distance: 10,
                                    align: 'left',
                                    verticalAlign: 'middle',
                                    rotate: 90,
                                    formatter: '{c}V',
                                    fontSize: 24,
                                    color: '#eeeeee',
                                    fontFamily: 'Fira Code'
                                }
                            }
                        }, {
                            xAxisIndex: 0,
                            name: 'Min V',
                            yAxisIndex: 0,
                            type: 'line',
                            data: [],
                            label: {
                                normal: {
                                    show: true,
                                    position: 'bottom',
                                    distance: 5,
                                    formatter: '{c}V',
                                    fontSize: 14,
                                    color: '#eeeeee',
                                    fontFamily: 'Fira Code'
                                }
                            },
                            symbolSize: 16,
                            symbol: ['circle'],
                            itemStyle: {
                                normal: {
                                    color: "#c1bdbd",
                                    lineStyle: {
                                        color: 'transparent'
                                    }
                                }
                            }
                        }
                        , {
                            xAxisIndex: 0,
                            name: 'Max V',
                            yAxisIndex: 0,
                            type: 'line',
                            data: [],
                            label: {
                                normal: {
                                    show: true,
                                    position: 'top',
                                    distance: 5,
                                    formatter: '{c}V',
                                    fontSize: 14,
                                    color: '#c1bdbd',
                                    fontFamily: 'Fira Code'
                                }
                            },
                            symbolSize: 16,
                            symbol: ['arrow'],
                            itemStyle: {
                                normal: {
                                    color: "#c1bdbd",
                                    lineStyle: {
                                        color: 'transparent'
                                    }
                                }
                            }
                        }

                        , {
                            xAxisIndex: 0,
                            name: 'Bypass',
                            yAxisIndex: 1,
                            type: 'line',
                            data: [],
                            label: {
                                normal: {
                                    show: true,
                                    position: 'right',
                                    distance: 5,
                                    formatter: '{c}%',
                                    fontSize: 14,
                                    color: '#f0e400',
                                    fontFamily: 'Fira Code'
                                }
                            },
                            symbolSize: 16,
                            symbol: ['square'],
                            itemStyle: { normal: { color: "#f0e400", lineStyle: { color: 'transparent' } } }
                        }

                        , {
                            xAxisIndex: 1,
                            yAxisIndex: 2,
                            name: 'BypassTemperature',
                            type: 'bar',
                            data: [],
                            itemStyle: {
                                color: '#55a1ea',
                                barBorderRadius: [8, 8, 0, 0]
                            },
                            label: {
                                normal: {
                                    show: true,
                                    position: 'insideBottom',
                                    distance: 8,
                                    align: 'left',
                                    verticalAlign: 'middle',
                                    rotate: 90,
                                    formatter: '{c}°C',
                                    fontSize: 20,
                                    color: '#eeeeee',
                                    fontFamily: 'Fira Code'
                                }
                            }
                        }

                        , {
                            xAxisIndex: 1,
                            yAxisIndex: 2,
                            name: 'CellTemperature',
                            type: 'bar',
                            data: [],
                            itemStyle: {
                                color: '#55a1ea',
                                barBorderRadius: [8, 8, 0, 0]
                            },
                            label: {
                                normal: {
                                    show: true,
                                    position: 'insideBottom',
                                    distance: 8,
                                    align: 'left',
                                    verticalAlign: 'middle',
                                    rotate: 90,
                                    formatter: '{c}°C',
                                    fontSize: 20,
                                    color: '#eeeeee',
                                    fontFamily: 'Fira Code'
                                }
                            }

                        }
                    ],
                    grid: [
                        {
                            containLabel: false,
                            left: '4%',
                            right: '4%',
                            bottom: '30%'

                        }, {
                            containLabel: false,
                            left: '4%',
                            right: '4%',
                            top: '76%'
                        }]
                };

                // use configuration item and data specified to show chart
                g1.setOption(option);

            }

            if (window.g2 == null && $('#graph2').css('display') != 'none') {
                window.g2 = echarts.init(document.getElementById('graph2'));

                var Option3dBar = {
                    tooltip: {},
                    visualMap: { max: 4, inRange: { color: ['#313695', '#4575b4', '#74add1', '#abd9e9', '#e0f3f8', '#ffffbf', '#fee090', '#fdae61', '#f46d43', '#d73027', '#a50026'] } },
                    xAxis3D: { type: 'category', data: [], name: 'Cell', nameTextStyle: { color: '#ffffff' } },
                    yAxis3D: { type: 'category', data: [], name: 'Bank', nameTextStyle: { color: '#ffffff' } },
                    zAxis3D: { type: 'value', name: 'Voltage', nameTextStyle: { color: '#ffffff' } },
                    grid3D: {
                        boxWidth: 200,
                        boxDepth: 80,
                        viewControl: {
                            // projection: 'orthographic'
                        },
                        light: {
                            main: {
                                intensity: 1.2,
                                shadow: true
                            },
                            ambient: {
                                intensity: 0.3
                            }
                        }
                    },
                    series: [{
                        type: 'bar3D',
                        data: [],
                        shading: 'lambert',
                        label: { textStyle: { fontSize: 16, borderWidth: 1, color: '#ffffff' } },

                        emphasis: {
                            label: {
                                textStyle: {
                                    fontSize: 16,
                                    color: '#aaa'
                                }
                            },
                            itemStyle: { color: '#fff' }
                        }
                    }]
                };

                g2.setOption(Option3dBar);
            }


            if (window.g1 != null && $('#graph1').css('display') != 'none') {
                g1.setOption({
                    markLine: { data: markLineData },
                    xAxis: { data: labels },
                    yAxis: [{ gridIndex: 0, min: minVoltage, max: maxVoltage }]
                    , series: [{ name: 'Voltage', data: voltages }
                        , { name: 'Min V', data: voltagesmin }
                        , { name: 'Max V', data: voltagesmax }
                        , { name: 'Bypass', data: pwm }
                        , { name: 'BypassTemperature', data: tempint }
                        , { name: 'CellTemperature', data: tempext }]
                });
            }



            if (window.g2 != null && $('#graph2').css('display') != 'none') {
                //Format the data to show as 3D Bar chart
                var cells3d = [];
                var banks3d = [];

                for (var seriesmodules = 0; seriesmodules < jsondata.seriesmodules; seriesmodules++) {
                    cells3d.push({ value: 'Cell ' + seriesmodules, textStyle: { color: '#ffffff' } });
                }

                var data3d = [];
                var cell = 0;
                for (var bankNumber = 0; bankNumber < jsondata.banks; bankNumber++) {
                    banks3d.push({ value: 'Bank ' + bankNumber, textStyle: { color: '#ffffff' } });
                    //Build up 3d array for cell data
                    for (var seriesmodules = 0; seriesmodules < jsondata.seriesmodules; seriesmodules++) {
                        data3d.push({ value: [seriesmodules, bankNumber, voltages[cell].value], itemStyle: voltages[cell].itemStyle });
                        cell++;
                    }
                }

                g2.setOption({
                    xAxis3D: { data: cells3d },
                    yAxis3D: { data: banks3d },
                    zAxis3D: { min: minVoltage, max: maxVoltage },
                    series: [{ data: data3d }]

                    , grid3D: {
                        boxWidth: 20 * jsondata.seriesmodules > 200 ? 200 : 20 * jsondata.seriesmodules,
                        boxDepth: 20 * jsondata.banks > 100 ? 100 : 20 * jsondata.banks
                    }
                }

                );
            }

        }//end homepage visible

        //Call again in a few seconds
        setTimeout(queryBMS, 4000);

        $("#loading").hide();

    }).fail(function () {
        $("#iperror").show();
        //Try again in a few seconds (2 seconds if errored)
        setTimeout(queryBMS, 2000);
        $("#loading").hide();
        //Dim the main home page graph
        $("#homePage").css({ opacity: 0.1 });
    });
}

$(window).on('resize', function () {
    if (g1 != null && g1 != undefined && $('#homePage').is(':visible')) { g1.resize(); }
    if (g2 != null && g2 != undefined && $('#homePage').is(':visible')) { g2.resize(); }
});

$(function () {
    $("#loading").show();

    //Populate all the setting rules with relay select lists
    $.each($(".settings table tbody tr td:empty"), function (index, value) {
        $.each([1, 2, 3, 4], function (index1, relay) {
            $(value).append('<select id="rule' + (index + 1) + 'relay' + relay + '" name="rule' + (index + 1) + 'relay' + relay + '"><option>On</option><option>Off</option><option>X</option></select>');
        });
    }
    );

    for (var n = 1; n <= 32; n++) {
        $("#totalSeriesModules").append('<option>' + n + '</option>')
    }
    for (var n = MAXIMUM_NUMBER_OF_BANKS - 1; n >= 0; n--) {
        $("#totalBanks").prepend('<option>' + (n + 1) + '</option>')
        $("#info").prepend('<div id="range' + n + '" class="stat"><span class="x t">Range ' + n + ':</span><span class="x v"></span></div>');
        $("#info").prepend('<div id="voltage' + n + '" class="stat"><span class="x t">Voltage ' + n + ':</span><span class="x v"></span></div>');

        $("#voltage" + n).hide();
        $("#range" + n).hide();
    }



    $("#graph1").show();
    $("#graph2").hide();
    $("#graphOptions a").click(function (event) {
        event.preventDefault();
        if ($(event.target).text() == "2D") {
            $("#graph1").show();
            $("#graph2").hide();

            //Hide 3d graph
            //if (window.g2 != null) {window.g2.clear();            }
        } else {
            $("#graph1").hide();
            $("#graph2").show();

            //Hide 2d graph
            //if (window.g1 != null) {window.g1.clear();            }
        }
        $(window).trigger('resize');
    });



    $('#CalculateCalibration').click(function () {
        var currentReading = parseFloat($("#modulesRows > tr.selected > td:nth-child(3)").text());
        var currentCalib = parseFloat($("#Calib").val());
        var actualV = parseFloat($("#ActualVoltage").val());
        var result = (currentCalib / currentReading) * actualV;
        $("#Calib").val(result.toFixed(4));
        return true;
    });

    $("#home").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        $(".page").hide();
        $("#homePage").show();
        return true;
    });

    $("#about").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        $(".page").hide();

        $.getJSON("settings.json",
            function (data) {
                $("#aboutPage").show();
            }).fail(function () { }
            );

        return true;
    });

    $("#modules").click(function () {
        $("#loading").show();

        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        $(".page").hide();

        //Remove existing table
        $("#modulesRows").find("tr").remove();

        $("#settingConfig").hide();

        $.getJSON("settings.json",
            function (data) {
                $("#g1").val(data.settings.bypassovertemp);
                $("#g2").val(data.settings.bypassthreshold);

                $("#modulesPage").show();
            }).fail(function () { }
            );
        return true;
    });

    $("#settings").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        $(".page").hide();

        $("#banksForm").hide();
        $("#settingsPage").show();

        $("#VoltageHigh").val(DEFAULT_GRAPH_MAX_VOLTAGE.toFixed(2));
        $("#VoltageLow").val(DEFAULT_GRAPH_MIN_VOLTAGE.toFixed(2));

        $.getJSON("settings.json",
            function (data) {

                $("#NTPServer").val(data.settings.NTPServerName);
                $("#NTPZoneHour").val(data.settings.TimeZone);
                $("#NTPZoneMin").val(data.settings.MinutesTimeZone);
                $("#NTPDST").prop("checked", data.settings.DST);

                var d = new Date(1000 * data.settings.now);
                $("#timenow").html(d.toJSON());

                $("#totalSeriesModules").val(data.settings.totalseriesmodules);
                $("#totalBanks").val(data.settings.totalnumberofbanks);

                $("#banksForm").show();
            }).fail(function () { }
            );

        return true;
    });


    $("#rules").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        $(".page").hide();

        $("#rulesForm").hide();
        $("#rulesPage").show();

        $.getJSON("rules.json",
            function (data) {
                //Rules have loaded

                //Default relay settings
                $.each(data.relaydefault, function (index2, value2) {
                    var relay_value = "X";
                    if (value2 === true) { relay_value = "On"; }
                    if (value2 === false) { relay_value = "Off"; }
                    $("#defaultrelay" + (index2 + 1)).val(relay_value);
                });

                //Default relay settings
                $.each(data.relaytype, function (index2, value2) {
                    $("#relaytype" + (index2 + 1)).val(value2);
                });

                $("#minutesnow").html(data.timenow);

                if (data.OutputsEnabled) {
                    $("#ExternalIO").hide();
                } else { $("#ExternalIO").show(); }

                //Loop through each rule updating the page
                var i = 1;
                var allrules = $(".settings table tbody tr td label");
                $.each(data.rules, function (index, value) {
                    $("#rule" + (index + 1) + "value").val(value.value);
                    $("#rule" + (index + 1) + "hysteresis").val(value.hysteresis);

                    //Highlight rules which are active
                    if (value.triggered) {
                        $(allrules[index]).addClass("triggered")
                    } else {
                        $(allrules[index]).removeClass("triggered")
                    }

                    $(allrules[index]).removeClass("disablerule");

                    $.each(value.relays, function (index2, value2) {
                        var relay_value = "X";
                        if (value2 === true) { relay_value = "On"; }
                        if (value2 === false) { relay_value = "Off"; }

                        $("#rule" + (index + 1) + "relay" + (index2 + 1)).val(relay_value);

                    });
                });

                if (data.ControlState != 0xff) {
                    //Controller is not in running state yet, so some rules are disabled
                    $.each([2, 3, 4, 5, 6, 7], function (index, value) {
                        $(allrules[value]).addClass("disablerule");
                    });
                }

                $("#rulesForm").show();
            }).fail(function () { }
            );

        return true;
    });

    $("#integration").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        $(".page").hide();
        $("#integrationPage").show();

        $("#mqttForm").hide();
        $("#influxForm").hide();

        $.getJSON("integration.json",
            function (data) {

                $("#mqttEnabled").prop("checked", data.mqtt.enabled);
                $("#mqttTopic").val(data.mqtt.topic);
                $("#mqttServer").val(data.mqtt.server);
                $("#mqttPort").val(data.mqtt.port);
                $("#mqttUsername").val(data.mqtt.username);
                $("#mqttPassword").val("");

                $("#influxEnabled").prop("checked", data.influxdb.enabled);
                $("#influxServer").val(data.influxdb.server);
                $("#influxPort").val(data.influxdb.port);
                $("#influxDatabase").val(data.influxdb.database);
                $("#influxUsername").val(data.influxdb.username);
                $("#influxPassword").val("");

                $("#mqttForm").show();
                $("#influxForm").show();
            }).fail(function () { }
            );

        return true;
    });

    $("form").unbind('submit').submit(function (e) {
        e.preventDefault();

        $.ajax({
            type: $(this).attr('method'),
            url: $(this).attr('action'),
            data: $(this).serialize(),
            success: function (data) {
                $("#savesuccess").show().delay(2000).fadeOut(500);
            },
            error: function (data) {
                $("#saveerror").show().delay(2000).fadeOut(500);
            },
        });
    });

    $("#settingsForm").unbind('submit').submit(function (e) {
        e.preventDefault();

        $.ajax({
            type: $(this).attr('method'),
            url: $(this).attr('action'),
            data: $(this).serialize(),
            success: function (data) {
                $('#settingConfig').hide();
                $("#savesuccess").show().delay(2000).fadeOut(500);
            },
            error: function (data) {
                $("#saveerror").show().delay(2000).fadeOut(500);
            },
        });
    });

    $("#displaySettingForm").unbind('submit').submit(function (e) {
        e.preventDefault();

        $.ajax({
            type: $(this).attr('method'),
            url: $(this).attr('action'),
            data: $(this).serialize(),
            success: function (data) {
                DEFAULT_GRAPH_MAX_VOLTAGE = parseFloat($("#VoltageHigh").val());
                DEFAULT_GRAPH_MIN_VOLTAGE = parseFloat($("#VoltageLow").val());
                $("#savesuccess").show().delay(2000).fadeOut(500);
            },
            error: function (data) {
                $("#saveerror").show().delay(2000).fadeOut(500);
            },
        });
    });

    $("#mqttEnabled").change(function () {
        if ($(this).is(":checked")) {
            $("#mqttForm").removeAttr("novalidate");
        } else {
            $("#mqttForm").attr("novalidate", "");
        }
    });

    $("#influxEnabled").change(function () {
        if ($(this).is(":checked")) {
            $("#influxForm").removeAttr("novalidate");
        } else {
            $("#influxForm").attr("novalidate", "");
        }
    });

    $.ajaxSetup({
        beforeSend: function (xhr, settings) { settings.data += '&xss=' + XSS_KEY; }
    });

    //$(document).ajaxStart(function(){ }); 
    //$(document).ajaxStop(function(){ });

    $("#homePage").show();

    //On page ready
    queryBMS();
});
