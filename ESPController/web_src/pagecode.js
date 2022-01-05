const INTERNALRULENUMBER = {
    EmergencyStop: 0,
    BMSError: 1,
    CurrentMonitorOverCurrentAmps: 2,
    Individualcellovervoltage: 3,
    Individualcellundervoltage: 4,
    ModuleOverTemperatureInternal: 5,
    ModuleUnderTemperatureInternal: 6,
    IndividualcellovertemperatureExternal: 7,
    IndividualcellundertemperatureExternal: 8,
    PackOverVoltage: 9,
    PackUnderVoltage: 10,
    Timer2: 11,
    Timer1: 12
}
Object.freeze(INTERNALRULENUMBER);

const INTERNALWARNINGCODE = {
    NoWarning: 0,
    ModuleInconsistantBypassVoltage: 1,
    ModuleInconsistantBypassTemperature: 2,
    ModuleInconsistantCodeVersion: 3,
    ModuleInconsistantBoardRevision: 4,
    LoggingEnabledNoSDCard: 5,
    AVRProgrammingMode: 6,
    XSSKEYSync: 7
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
    ControllerMemoryError: 6,
    EmergencyStop: 7
};
Object.freeze(INTERNALERRORCODE);


function switchPage(newPage) {
    $(".page").hide();
    $(newPage).show();
    $("#myNav").height("0%");
}
function identifyModule(button, cellid) {
    $.getJSON("identifyModule.json", { c: cellid }, function (data) { }).fail(function () { $("#iperror").show(); });
}

function refreshCurrentMonitorValues() {
    $.getJSON("currentmonitor.json",
        function (data) {
            $("#CurrentMonEnabled").prop("checked", data.enabled);
            $("#modbusAddress").val(data.address);

            $("#shuntmaxcur").val(data.shuntmaxcur);
            $("#shuntmv").val(data.shuntmv);

            $("#cmvalid").val(data.valid);

            $("#cmbatterycapacity").val(data.batterycapacity);
            $("#cmfullchargevolt").val(data.fullchargevolt.toFixed(2));
            $("#cmtailcurrent").val(data.tailcurrent.toFixed(2));
            $("#cmchargeefficiency").val(data.chargeefficiency.toFixed(1));

            $("#cmtimestampage").val(data.timestampage);
            $("#cmtemperature").val(data.temperature);
            $("#cmwatchdog").val(data.watchdog);
            $("#cmactualshuntmv").val(data.actualshuntmv);
            $("#cmcurrentlsb").val(data.currentlsb);
            $("#cmresistance").val(data.resistance);
            $("#cmcalibration").val(data.calibration);

            $("#cmtemplimit").val(data.templimit);

            $("#cmundervlimit").val(data.undervlimit);
            $("#cmovervlimit").val(data.overvlimit);

            $("#cmoverclimit").val(data.overclimit);
            $("#cmunderclimit").val(data.underclimit);

            $("#cmoverplimit").val(data.overplimit);
            //Temperature coefficient
            $("#cmtempcoeff").val(data.tempcoeff);

            $("#cmmodel").val(data.model.toString(16));
            $("#cmfirmwarev").val(data.firmwarev.toString(16));

            var d = new Date(data.firmwaredate * 1000);
            $("#cmfirmwaredate").val(d.toString());


            $("#TempCompEnabled").prop("checked", data.TempCompEnabled);
            $("#cmTemperatureOverLimit").val(data.TMPOL);

            $("#cmCurrentOverLimit").val(data.CURROL);
            $("#cmCurrentUnderLimit").val(data.CURRUL);
            $("#cmVoltageOverLimit").val(data.VOLTOL);
            $("#cmVoltageUnderLimit").val(data.VOLTUL);
            $("#cmPowerOverLimit").val(data.POL);

            $("#cmRelayState").val(data.RelayState ? "CLOSED" : "OPEN");

            $("#cmTMPOL").prop("checked", data.T_TMPOL);
            $("#cmCURROL").prop("checked", data.T_CURROL);
            $("#cmCURRUL").prop("checked", data.T_CURRUL);
            $("#cmVOLTOL").prop("checked", data.T_VOLTOL);
            $("#cmVOLTUL").prop("checked", data.T_VOLTUL);
            $("#cmPOL").prop("checked", data.T_POL);


            if (data.enabled) {
                $("#currentmonadvanced").show();
                $("#currentmonbasic").show();
                $("#currentmonrelay").show();
            } else {
                $("#currentmonadvanced").hide();
                $("#currentmonbasic").hide();
                $("#currentmonrelay").hide();
            }

        }).fail(function () { }
        );

}

function showFailure() {
    $.notify($("#saveerror").text(), { className: 'error', autoHideDelay: 15000 });
}

function showSuccess() {
    $.notify($("#savesuccess").text(), { className: 'success' });
}

function currentmonitorSubmitForm(form) {
    $.ajax({
        type: $(form).attr('method'),
        url: $(form).attr('action'),
        data: $(form).serialize(),
        success: function (data) {
            $("#currentmonadvanced").hide();
            $("#currentmonbasic").hide();
            $("#currentmonrelay").hide();

            showSuccess();

            //Show spinner for 6 seconds, then refresh page
            $("#loading").show().delay(6000).hide("fast", function () {
                $("#currentmonitor").click();
            });
        },
        error: function (data) {
            showFailure();
        },
    });
}

function avrProgrammingStatsUpdate(attempts) {
    $.getJSON("avrstatus.json",
        function (data) {
            console.log(data);

            if (data.inprogress == 0) {
                //Finished or aborted with error, update display
                $("#avrinfo").empty();
                if (data.result == 0) {
                    $("#avrinfo").html('Success, took ' + data.duration + 'ms, ' + data.size + ' bytes.');
                } else {

                    let errmessage = "";
                    switch (data.result) {
                        case 1:
                            errmessage = "WRONG_DEVICE_ID"
                            break;
                        case 2:
                            errmessage = "COMMIT_FAIL"
                            break;
                        case 3:
                            errmessage = "FAILED_ENTER_PROG_MODE"
                            break;
                        case 4:
                            errmessage = "FAIL_VERIFY"
                            break;
                        case 5:
                            errmessage = "FAIL_PROG_FUSE"
                            break;
                        case 6:
                            errmessage = "INCORRECT_PAGE_CONFIG"
                            break;
                    }

                    $("#avrinfo").html('Failed, error code ' + data.result + ' ' + errmessage);
                }

            } else {
                $("#avrinfo").html("In progress...");

                if (attempts > 0) {
                    attempts--;
                    setTimeout(avrProgrammingStatsUpdate, 1000, attempts);
                }
            }


        }).fail(function () {
            $("#iperror").show();
        });

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
                $('#Version').html(data.settings.ver.toString() + ' / <a href="https://github.com/stuartpittaway/diyBMSv4ESP32/commit/' + data.settings.code.toString(16) + '" rel="noreferrer" target="_blank">' + data.settings.code.toString(16) + '</a>');
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

function secondsToHms(seconds) {
    seconds = Number(seconds);
    var d = Math.floor(seconds / (3600 * 24));
    var h = Math.floor(seconds % (3600 * 24) / 3600);
    var m = Math.floor(seconds % 3600 / 60);
    var s = Math.floor(seconds % 60);

    var dDisplay = d > 0 ? d + "d" : "";
    var hDisplay = h > 0 ? h + "h" : "";
    var mDisplay = m > 0 ? m + "m" : "";
    var sDisplay = h > 0 ? "" : (s > 0 ? s + "s" : "");
    return dDisplay + hDisplay + mDisplay + sDisplay;
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

        markLineData.push({ name: 'avg', type: 'average', lineStyle: { color: '#ddd', width: 2, type: 'dotted', opacity: 0.3 }, label: { distance: [10, 0], position: 'start', color: "#eeeeee", textBorderColor: '#313131', textBorderWidth: 2 } });
        markLineData.push({ name: 'min', type: 'min', lineStyle: { color: '#ddd', width: 2, type: 'dotted', opacity: 0.3 }, label: { distance: [10, 0], position: 'start', color: "#eeeeee", textBorderColor: '#313131', textBorderWidth: 2 } });
        markLineData.push({ name: 'max', type: 'max', lineStyle: { color: '#ddd', width: 2, type: 'dotted', opacity: 0.3 }, label: { distance: [10, 0], position: 'start', color: "#eeeeee", textBorderColor: '#313131', textBorderWidth: 2 } });

        var xAxis = 0;
        for (let index = 0; index < jsondata.banks; index++) {
            markLineData.push({ name: "Bank " + index, xAxis: xAxis, lineStyle: { color: colours[index], width: 4, type: 'dashed', opacity: 0.5 }, label: { show: true, distance: [0, 0], formatter: '{b}', color: '#eeeeee', textBorderColor: colours[index], textBorderWidth: 2 } });
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

                if (jsondata.minvoltages) {
                    voltagesmin.push((parseFloat(jsondata.minvoltages[i]) / 1000.0));
                }
                if (jsondata.maxvoltages) {
                    voltagesmax.push((parseFloat(jsondata.maxvoltages[i]) / 1000.0));
                }

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
                pwm.push({ value: jsondata.bypasspwm[i] == 0 ? null : Math.trunc(jsondata.bypasspwm[i] / 255 * 100) });
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

            
            if (jsondata.can_fail == 0) { $("#canfail").hide(); } else { $("#canfail .v").html(jsondata.can_fail); $("#canfail").show(); }

            if (jsondata.can_sent == 0) { $("#cansent").hide(); } else { $("#cansent .v").html(jsondata.can_sent); $("#cansent").show(); }
            if (jsondata.can_rec == 0) { $("#canrecd").hide(); } else { $("#canrecd .v").html(jsondata.can_rec); $("#canrecd").show(); }

            if (jsondata.qlen == 0) { $("#qlen").hide(); } else { $("#qlen .v").html(jsondata.qlen); $("#qlen").show(); }

            $("#uptime .v").html(secondsToHms(jsondata.uptime)); $("#uptime").show();

            if (jsondata.activerules == 0) {
                $("#activerules").hide();
            } else {
                $("#activerules").html(jsondata.activerules);
                $("#activerules").show(400);
            }
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

        if (jsondata.sec) {
            if (!XSS_KEY.endsWith(jsondata.sec)) {
                if ($("#warning7").data("notify") == undefined) {
                    $("#warning7").data("notify", 1);
                    $.notify($("#warning7").text(), { autoHide: false, globalPosition: 'top left', className: 'error' });
                }
            }
        }

        if (jsondata.current) {
            if (jsondata.current[0] == null) {
                $("#current").hide();
                $("#shuntv").hide();
                $("#soc").hide();
                $("#amphout").hide();
                $("#amphin").hide();
                $("#power").hide();
            } else {
                var data = jsondata.current[0];

                $("#current .v").html(parseFloat(data.c).toFixed(2) + "A");
                $("#current").show();

                $("#shuntv .v").html(parseFloat(data.v).toFixed(2) + "V");
                $("#shuntv").show();

                $("#soc .v").html(parseFloat(data.soc).toFixed(2) + "%");
                $("#soc").show();

                $("#power .v").html(parseFloat(data.p) + "W");
                $("#power").show();

                $("#amphout .v").html((parseFloat(data.mahout) / 1000).toFixed(3));
                $("#amphout").show();

                $("#amphin .v").html((parseFloat(data.mahin) / 1000).toFixed(3));
                $("#amphin").show();
            }
        }




        //Needs increasing when more warnings are added
        if (jsondata.warnings) {
            for (let warning = 1; warning <= 6; warning++) {
                if (jsondata.warnings.includes(warning)) {
                    //Once a warning has triggered, hide it from showing in the future
                    if ($("#warning" + warning).data("notify") == undefined) {
                        $("#warning" + warning).data("notify", 1);
                        $.notify($("#warning" + warning).text(), { autoHideDelay: 15000, globalPosition: 'top left', className: 'warn' });
                    }
                }
                //else {
                //$("#warning" + warning).hide();
                //}
            }
        }

        //Needs increasing when more errors are added
        if (jsondata.errors) {
            for (let error = 1; error <= 7; error++) {
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
                if (voltagesmin.length > 0) {
                    $(columns[3]).html(voltagesmin[index].toFixed(3));
                } else {
                    $(columns[3]).html("n/a");
                }
                if (voltagesmax.length > 0) {
                    $(columns[4]).html(voltagesmax[index].toFixed(3));
                } else {
                    $(columns[4]).html("n/a");
                }
                $(columns[5]).html(tempint[index].value);
                $(columns[6]).html(tempext[index].value);
                $(columns[7]).html(pwm[index].value);
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

            if (window.g2 == null && $('#graph2').css('display') != 'none' && window.Graph3DAvailable === true) {
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

        $("#homePage").css({ opacity: 1.0 });
        $("#loading").hide();
        //Call again in a few seconds
        setTimeout(queryBMS, 4000);

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
    $("#avrprogconfirm").hide();

    $("#more").on("click"
        , function (e) {
            e.preventDefault();
            $("#myNav").height("90%");
        });

    $("#closebtn").on("click"
        , function (e) {
            e.preventDefault();
            $("#myNav").height("0%");
        });

    //Populate all the setting rules with relay select lists
    $.each($(".settings table tbody tr td:empty"), function (index, value) {
        $.each([1, 2, 3, 4], function (index1, relay) {
            $(value).append('<select id="rule' + (index) + 'relay' + relay + '" name="rule' + (index) + 'relay' + relay + '"><option>On</option><option>Off</option><option>X</option></select>');
        });
    }
    );

    $("#labelMaxModules").text(MAXIMUM_NUMBER_OF_SERIES_MODULES);


    for (var n = 1; n <= MAXIMUM_NUMBER_OF_SERIES_MODULES; n++) {
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
        } else {
            $("#graph1").hide();
            $("#graph2").show();
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
        switchPage("#homePage");
        return true;
    });

    $("#about").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        switchPage("#aboutPage");

        $.getJSON("settings.json",
            function (data) {
                $("#MinFreeHeap").html(data.settings.MinFreeHeap);
                $("#FreeHeap").html(data.settings.FreeHeap);
                $("#HeapSize").html(data.settings.HeapSize);
                $("#SdkVersion").html(data.settings.SdkVersion);
                $("#HostName").html("<a href='http://" + data.settings.HostName + "'>" + data.settings.HostName + "</a>");
            }).fail(function () { }
            );

        return true;
    });

    $("#modules").click(function () {
        $("#loading").show();

        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        //Remove existing table
        $("#modulesRows").find("tr").remove();

        $("#settingConfig").hide();

        switchPage("#modulesPage");

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

        $("#banksForm").hide();
        $("#settingsPage").show();

        $("#VoltageHigh").val(DEFAULT_GRAPH_MAX_VOLTAGE.toFixed(2));
        $("#VoltageLow").val(DEFAULT_GRAPH_MIN_VOLTAGE.toFixed(2));

        switchPage("#settingsPage");

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

                $("#baudrate").empty();
                $("#baudrate").append('<option value="2400">Standard</option>')
                $("#baudrate").append('<option value="5000">5K</option>')
                $("#baudrate").append('<option value="9600">9K6</option>')

                $("#baudrate").val(data.settings.baudrate);


                $("#banksForm").show();
            }).fail(function () { }
            );

        return true;
    });


    $("#rules").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");

        $("#rulesForm").hide();

        switchPage("#rulesPage");

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


                //Loop through each rule updating the page
                var i = 1;
                var allrules = $(".settings table tbody tr td label");
                $.each(data.rules, function (index, value) {
                    $("#rule" + (index) + "value").val(value.value);
                    $("#rule" + (index) + "hysteresis").val(value.hysteresis);

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

                        $("#rule" + (index) + "relay" + (index2 + 1)).val(relay_value);
                    });
                });

                if (data.ControlState != 0xff) {
                    //Controller is not in running state yet, so some rules are disabled
                    $.each([INTERNALRULENUMBER.Individualcellovervoltage,
                    INTERNALRULENUMBER.Individualcellundervoltage,
                    INTERNALRULENUMBER.ModuleOverTemperatureInternal,
                    INTERNALRULENUMBER.ModuleUnderTemperatureInternal,
                    INTERNALRULENUMBER.IndividualcellovertemperatureExternal,
                    INTERNALRULENUMBER.IndividualcellundertemperatureExternal], function (index, value) {
                        $(allrules[value]).addClass("disablerule");
                    });
                }

                $("#rulesForm").show();
            }).fail(function () { }
            );

        return true;
    });

    $("#currentmonitor").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");

        switchPage("#diybmsCurrentMonitorPage");


        $.getJSON("rs485settings.json",
            function (data) {
                $("#rs485baudrate").val(data.baudrate);
                $("#rs485databit").val(data.databits);
                $("#rs485parity").val(data.parity);
                $("#rs485stopbit").val(data.stopbits);
            }).fail(function () { }
            );

        refreshCurrentMonitorValues();

        return true;
    });

    $("#victroncanbus").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");

        $.getJSON("victron.json",
            function (data) {
                $("#VictronEnabled").prop("checked", data.victron.enabled);

                for (let index = 0; index < data.victron.cvl.length; index++) {
                    $("#cvl" + index).val((data.victron.cvl[index] / 10).toFixed(2));
                    $("#ccl" + index).val((data.victron.ccl[index] / 10).toFixed(2));
                    $("#dcl" + index).val((data.victron.dcl[index] / 10).toFixed(2));
                }

                switchPage("#victroncanbusPage");
            }).fail(function () { }
            );

        return true;
    });


    $("#currentmonrefresh").click(function (e) {
        e.preventDefault();
        refreshCurrentMonitorValues();
    });

    $("#integration").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");

        switchPage("#integrationPage");

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
                $("#influxUrl").val(data.influxdb.url);
                $("#influxDatabase").val(data.influxdb.bucket);
                $("#influxToken").val(data.influxdb.apitoken);
                $("#influxOrgId").val(data.influxdb.orgid);

                $("#mqttForm").show();
                $("#influxForm").show();
            }).fail(function () { }
            );

        return true;
    });


    $("#mount").click(function () {
        $.ajax({
            type: 'post',
            url: 'sdmount.json',
            data: 'mount=1',
            success: function (data) {
                //Refresh the storage page
                $("#storage").trigger("click");
            },
            error: function (data) {
                showFailure();
            },
        });
    });


    $("#savewifi").click(function () {
        $.ajax({
            type: 'post',
            url: 'wificonfigtofile.json',
            data: 'save=1',
            success: function (data) {
                //Refresh the storage page
                showSuccess();
            },
            error: function (data) {
                showFailure();
            },
        });
    });

    $("#saveconfig").click(function () {
        $.ajax({
            type: 'post',
            url: 'saveconfigtofile.json',
            data: 'save=1',
            success: function (data) {
                //Refresh the storage page
                showSuccess();
                $("#storage").trigger("click");
            },
            error: function (data) {
                showFailure();
            },
        });
    });


    $("#unmount").click(function () {
        $.ajax({
            type: 'post',
            url: 'sdunmount.json',
            data: 'unmount=1',
            success: function (data) {
                //Refresh the storage page
                $("#storage").trigger("click");
            },
            error: function (data) {
                showFailure();
            },
        });
    });


    $("#AVRProgDisable").click(function () {
        $.ajax({
            type: 'post',
            url: 'disableavrprog.json',
            data: { 'enable': 0 },
            timeout: 10000
        })
            .done(
                function (data) {
                    $("#AVRProgEnable").prop('disabled', false).css({ opacity: 1.0 });
                    $("#AVRProgDisable").prop('disabled', true).css({ opacity: 0.25 });
                    $("#ProgAVR").prop('disabled', true).css({ opacity: 0.25 });
                    $("#ProgAVRCancel").prop('disabled', true).css({ opacity: 0.25 });
                    //Allow warning to trigger again
                    $("#warning7").removeData("notify");
                })
            .fail(function (data) {
                $("#avrinfo").html("Failed");
            })
            .always(function (data) {
                //$("#ProgAVR").prop('disabled', false).css({ opacity: 1.0 });
                //$("#ProgAVRCancel").prop('disabled', false).css({ opacity: 1.0 });
            });

        return true;
    });

    $("#AVRProgEnable").click(function () {
        $.ajax({
            type: 'post',
            url: 'enableavrprog.json',
            data: { 'enable': 1 },
            timeout: 10000
        })
            .done(
                function (data) {
                    $("#AVRProgEnable").prop('disabled', true).css({ opacity: 0.25 });
                    $("#AVRProgDisable").prop('disabled', false).css({ opacity: 1.0 });
                    $("#ProgAVR").prop('disabled', false).css({ opacity: 1.0 });
                    $("#ProgAVRCancel").prop('disabled', false).css({ opacity: 1.0 });
                })
            .fail(function (data) {
                $("#avrinfo").html("Failed");
            })
            .always(function (data) {
                //$("#ProgAVR").prop('disabled', false).css({ opacity: 1.0 });
                //$("#ProgAVRCancel").prop('disabled', false).css({ opacity: 1.0 });
            });

        return true;
    });

    $("#ProgAVRCancel").click(function () {
        $("#avrprogconfirm").hide();
        $("#avrinfo").empty();
        return true;
    });

    $("#ProgAVR").click(function () {
        $("#ProgAVR").prop('disabled', true).css({ opacity: 0.25 });
        $("#ProgAVRCancel").prop('disabled', true).css({ opacity: 0.25 });
        $("#avrinfo").empty();

        $.ajax({
            type: 'post',
            url: 'avrprog.json',
            data: { file: $("#selectedavrindex").val() },
            //Wait up to 30 seconds
            timeout: 30000
        })
            .done(
                function (data) {
                    $("#avrinfo").html(data.message);
                    if (data.started) {
                        setTimeout(avrProgrammingStatsUpdate, 250, 20);
                    }
                })
            .fail(function (data) {
                $("#avrinfo").html("Failed");
            })
            .always(function (data) {
                $("#ProgAVR").prop('disabled', false).css({ opacity: 1.0 });
                $("#ProgAVRCancel").prop('disabled', false).css({ opacity: 1.0 });
            });

        return true;
    });


    $("#avrprogrammer").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        switchPage("#avrprogPage");

        $.getJSON("avrstorage.json",
            function (data) {
                $("#avrprog").empty();
                $("#avrprogconfirm").hide();
                $("#selectedavrindex").val("");

                if (data.ProgModeEnabled == 1) {
                    //Programming mode enabled
                    $("#AVRProgEnable").prop('disabled', true).css({ opacity: 0.25 });
                    $("#AVRProgDisable").prop('disabled', false).css({ opacity: 1.0 });

                    $("#ProgAVR").prop('disabled', false).css({ opacity: 1.0 });
                    $("#ProgAVRCancel").prop('disabled', false).css({ opacity: 1.0 });
                } else {
                    $("#AVRProgEnable").prop('disabled', false).css({ opacity: 1.0 });
                    $("#AVRProgDisable").prop('disabled', true).css({ opacity: 0.25 });
                    $("#ProgAVR").prop('disabled', true).css({ opacity: 0.25 });
                    $("#ProgAVRCancel").prop('disabled', true).css({ opacity: 0.25 });
                }

                if (data.avrprog.avrprog) {
                    $.each(data.avrprog.avrprog, function (index, value) {

                        var li = document.createElement("li");
                        $("#avrprog").append(li);

                        var aref = $("<a href='#' data-index='" + index + "'>" + value.board + " (" + value.ver + ")</a>").on("click",

                            function (event) {
                                event.preventDefault();
                                $("#avrprogconfirm").show();
                                $("#selectedavrindex").val($(this).data("index"));
                            }
                        );

                        $(li).appendTo()
                        $(li).append(aref);
                    });
                }
            }).fail(function () { }
            );

        return true;
    });



    $("#storage").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        switchPage("#storagePage");

        $.getJSON("storage.json",
            function (data) {

                //Allow warning to trigger again
                $("#warning5").removeData("notify");

                $("#loggingEnabled").prop("checked", data.storage.logging);
                $("#loggingFreq").val(data.storage.frequency);

                if (data.storage.sdcard.available) {
                    $("#sdcardmissing").hide();
                } else { $("#sdcardmissing").show(); }

                $("#sdcard_total").html(Number(data.storage.sdcard.total).toLocaleString());
                $("#sdcard_used").html(Number(data.storage.sdcard.used).toLocaleString());

                if (data.storage.sdcard.total > 0) {
                    $("#sdcard_used_percent").html(((data.storage.sdcard.used / data.storage.sdcard.total) * 100).toFixed(1));
                }
                else { $("#sdcard_used_percent").html("0"); }

                $("#sdcardfiles").empty();
                if (data.storage.sdcard.files) {
                    $.each(data.storage.sdcard.files, function (index, value) {
                        if (value != null) {
                            $("#sdcardfiles").append("<li><a href='download?type=sdcard&file=" + encodeURI(value) + "'>" + value + "</a></li>");
                        }
                    });
                }

                $("#flash_total").html(Number(data.storage.flash.total).toLocaleString());
                $("#flash_used").html(Number(data.storage.flash.used).toLocaleString());

                if (data.storage.flash.total > 0) {
                    $("#flash_used_percent").html(((data.storage.flash.used / data.storage.flash.total) * 100).toFixed(1));
                }
                else { $("#flash_used_percent").html("0"); }

                $("#flashfiles").empty();
                if (data.storage.flash.files) {
                    $.each(data.storage.flash.files, function (index, value) {
                        if (value != null) {
                            $("#flashfiles").append("<li><a href='download?type=flash&file=" + encodeURI(value) + "'>" + value + "</a></li>");
                        }
                    });
                }

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
                showSuccess();
            },
            error: function (data) {
                showFailure();
            },
        });
    });


    $("#diybmsCurrentMonitorForm2").unbind('submit').submit(function (e) {
        e.preventDefault();
        currentmonitorSubmitForm(this);
    });
    $("#diybmsCurrentMonitorForm3").unbind('submit').submit(function (e) {
        e.preventDefault();
        currentmonitorSubmitForm(this);
    });
    $("#diybmsCurrentMonitorForm4").unbind('submit').submit(function (e) {
        e.preventDefault();
        currentmonitorSubmitForm(this);
    });
    $("#diybmsCurrentMonitorForm1").unbind('submit').submit(function (e) {
        e.preventDefault();
        currentmonitorSubmitForm(this);
    });

    /*
        $("#victronForm1").unbind('submit').submit(function (e) {
            e.preventDefault();        
        });
    */

    $("#globalSettingsForm").unbind('submit').submit(function (e) {
        e.preventDefault();

        $.ajax({
            type: $(this).attr('method'),
            url: $(this).attr('action'),
            data: $(this).serialize(),
            success: function (data) {
                showSuccess();

                //Allow warning to trigger again
                $("#warning1").removeData("notify");
                $("#warning2").removeData("notify");
                $("#warning3").removeData("notify");
                $("#warning4").removeData("notify");
            },
            error: function (data) {
                showFailure();
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
                showSuccess();

                //Allow warning to trigger again
                $("#warning1").removeData("notify");
                $("#warning2").removeData("notify");
                $("#warning3").removeData("notify");
                $("#warning4").removeData("notify");

            },
            error: function (data) {
                showFailure();
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
                showSuccess();
            },
            error: function (data) {
                showFailure();
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


    $(".stat").mouseenter(function () {
        $(this).addClass("hover");
    }).mouseleave(function () {
        $(this).removeClass("hover");
    });

    //$(document).ajaxStart(function(){ });
    //$(document).ajaxStop(function(){ });

    $("#homePage").show();

    //On page ready
    queryBMS();
}); // end $(function ()
