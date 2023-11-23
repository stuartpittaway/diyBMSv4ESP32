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
    CurrentMonitorOverVoltage: 9,
    CurrentMonitorUnderVoltage: 10,
    BankOverVoltage: 11,
    BankUnderVoltage: 12,
    BankRange: 13,
    Timer2: 14,
    Timer1: 15
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


// TILE_IDS holds an array of the statistic panels on the page
// this array drives the ability to toggle visibility
// DO NOT MODIFY THE ORDER/SEQUENCE - ADD NEW ITEMS AT THE END/USE EMPTY STRINGS
// IF YOU NEED TO DELETE - REPLACE VALUE WITH null
// THEY ARE ARRANGED IN SUB-ARRAY OF 16 ITEMS - CORRESPONDING TO A 16 BIT UNSIGNED VALUE
// PAD ARRAYS TO 16 ITEMS
const TILE_IDS = [
    ["voltage0", "range0", "voltage1", "range1", "voltage2", "range2", "voltage3", "range3", "voltage4", "range4", "voltage5", "range5", "voltage6", "range6", "voltage7", "range7"],
    ["voltage8", "range8", "voltage9", "range9", "voltage10", "range10", "voltage11", "range11", "voltage12", "range12", "voltage13", "range13", "voltage14", "range14", "voltage15", "range15"],
    ["soc", "current", "shuntv", "power", "amphout", "amphin", "damphout", "damphin", "oos", "badcrc", "ignored", "canfail", "sent", "received", "roundtrip", "uptime"],
    ["qlen", "cansent", "canrecd", "dyncvolt", "dynccurr", "graphOptions", "time100", "time20", "time10", "celltemp", "canrecerr", "chgmode", null, null, null, null]
];
Object.freeze(TILE_IDS);

var tileconfig = [];

var timer_postTileVisibiltity = null;

function upload_file() {

    let data = document.getElementById("uploadfile_sel").files[0];
    xhr = new XMLHttpRequest();
    xhr.open("POST", "/uploadfile", true);
    xhr.setRequestHeader('X-Requested-With', 'XMLHttpRequest');
    xhr.onreadystatechange = function () {
        if (xhr.readyState === XMLHttpRequest.DONE) {
            var status = xhr.status;
            if (status >= 200 && status < 400) {
                //Refresh the storage page
                $("#storage").trigger("click");
                $.notify("File upload success", { autoHide: true, globalPosition: 'top right', className: 'success' });
            } else {
                $.notify("File upload failed", { autoHide: true, globalPosition: 'top right', className: 'error' });
            }
            $("#progress").hide();
        }
    };
    xhr.send(data);
    return false;

}

function upload_firmware() {
    $("#progress").show();
    $("#status_div").text("Upload in progress");
    let data = document.getElementById("file_sel").files[0];
    xhr = new XMLHttpRequest();
    xhr.open("POST", "/ota", true);
    xhr.setRequestHeader('X-Requested-With', 'XMLHttpRequest');
    xhr.upload.addEventListener("progress", function (event) {
        if (event.lengthComputable) {
            document.getElementById("progress").style.width = (event.loaded / event.total) * 100 + "%";
        }
    });
    xhr.onreadystatechange = function () {
        if (xhr.readyState === XMLHttpRequest.DONE) {
            var status = xhr.status;
            if (status >= 200 && status < 400) {
                $("#status_div").text("Upload accepted. BMS will reboot.");
            } else {
                $("#status_div").text("Upload rejected!");
            }
            $("#progress").hide();
        }
    };
    xhr.send(data);
    return false;
}


function CalculateChargeCurrent(value1, value2, highestCellVoltage, maximumchargecurrent, kneemv, cellmaxmv) {
    if (highestCellVoltage < kneemv) {
        // Voltage is below the knee voltage, so use full current
        return maximumchargecurrent;
    }

    var knee_voltage = 0 / 100.0;
    var at_knee = Math.pow(value1, knee_voltage * Math.pow(knee_voltage, value2));

    var target_cell_voltage = (cellmaxmv - kneemv) / 100.0;
    var at_target_cell_voltage = Math.pow(value1, target_cell_voltage * Math.pow(target_cell_voltage, value2));

    var actual_cell_voltage = (highestCellVoltage - kneemv) / 100.0;
    var at_actual_cell_voltage = Math.pow(value1, actual_cell_voltage * Math.pow(actual_cell_voltage, value2));

    var percent = 1 - (at_actual_cell_voltage / at_knee) / at_target_cell_voltage;

    if (percent < 0.01) {
        percent = 0.01;
    }

    return Math.min(maximumchargecurrent, (maximumchargecurrent * percent));
}

function DrawChargingGraph() {


    var xaxisvalues = [];
    var yaxisvalues = [];

    const chargecurrent = parseFloat($("#chargecurrent").val());
    const cellminmv = parseInt($("#cellminmv").val());
    const cellmaxmv = parseInt($("#cellmaxmv").val());
    const kneemv = parseInt($("#kneemv").val());
    const cellmaxspikemv = parseInt($("#cellmaxspikemv").val());

    const value1 = parseFloat($("#cur_val1").val());
    const value2 = parseFloat($("#cur_val2").val());

    for (let voltage = cellminmv; voltage <= cellmaxspikemv; voltage += 5) {
        xaxisvalues.push(voltage);
        yaxisvalues.push(CalculateChargeCurrent(value1, value2, voltage, chargecurrent, kneemv, cellmaxmv));
    }

    /*
        if (window.g3 != null) {
            window.g3.dispose();
            window.g3 = null;
        }
    */
    if (window.g3 == null) {
        window.g3 = echarts.init(document.getElementById('graph3'))

        var option;

        option = {

            tooltip: {
                trigger: 'axis',
                axisPointer: {
                    type: 'cross',
                    label: {
                        backgroundColor: '#6a7985'
                    }
                }
            },
            grid: {
                left: '2%',
                right: '2%',
                bottom: '2%',
                containLabel: true
            },
            xAxis: [
                {
                    type: 'category',
                    boundaryGap: false,
                    data: xaxisvalues,
                    axisLabel: {
                        fontSize: '12',
                        fontFamily: 'monospace',
                        formatter: '{value}mV',
                        color: '#ffffff'
                    }
                }
            ],
            yAxis: [
                {
                    type: 'value',
                    axisLabel: {
                        fontSize: '12',
                        fontFamily: 'monospace',
                        formatter: '{value}A',
                        color: '#ffffff'
                    },
                    splitNumber: 10
                }
            ],
            series: [
                {
                    name: 'Current',
                    type: 'line',
                    areaStyle: {},
                    data: yaxisvalues
                }
            ]
        };

        option && window.g3.setOption(option);
    } else {
        //Update the values
        window.g3.setOption({
            xAxis: { data: xaxisvalues },
            series: { data: yaxisvalues }
        });

    }
}

function switchPage(newPage) {
    $(".page").hide();
    $(newPage).show();
    $("#myNav").height("0%");
}

function identifyModule(button, cellid) {
    $.getJSON("/api/identifyModule", { c: cellid }, function (data) { showActionSuccess(); }).fail(function () { $("#iperror").show(); });
}

function restoreconfig(location, filename) {
    let isConfirmed = confirm("Are you sure you wish to restore '" + filename + "' configuration file?");

    if (isConfirmed) {

        alert("Note: Passwords are not restored, you will need to manually change these in the integration page if required.");

        $.ajax({
            type: 'POST',
            url: '/post/restoreconfig',
            data: $.param({ flashram: location, filename: filename }),
            success: function (data) {
                showSuccess();
                alert("Restore complete, you should now reboot the controller.");
            },
            error: function (data) {
                showFailure();
            },
        });

    }
}

function refreshCurrentMonitorValues() {
    $.getJSON("/api/currentmonitor",
        function (data) {
            $("#CurrentMonEnabled").prop("checked", data.enabled);
            $("#modbusAddress").val(data.address);
            $("#CurrentMonDev").val(data.devicetype);

            $("#shuntmaxcur").val(data.shuntmaxcur);
            $("#shuntmv").val(data.shuntmv);
            $("#cmvalid").val(data.valid);
            $("#cmtimestampage").val(data.timestampage);

            //INA228 or INA229 based devices
            if (data.devicetype == 0 || data.devicetype == 2) {
                $("#cmbatterycapacity").val(data.batterycapacity);
                $("#cmfullchargevolt").val(data.fullchargevolt.toFixed(2));
                $("#cmtailcurrent").val(data.tailcurrent.toFixed(2));
                $("#cmchargeefficiency").val(data.chargeefficiency.toFixed(1));

                $("#cmtemperature").val(data.temperature);
                $("#cmwatchdog").val(data.watchdog);
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

                if (data.RelayState == null) {
                    $("#cmRelayState").val("N/A");
                } else {
                    $("#cmRelayState").val(data.RelayState ? "CLOSED" : "OPEN");
                }

                $("#cmTMPOL").prop("checked", data.T_TMPOL);
                $("#cmCURROL").prop("checked", data.T_CURROL);
                $("#cmCURRUL").prop("checked", data.T_CURRUL);
                $("#cmVOLTOL").prop("checked", data.T_VOLTOL);
                $("#cmVOLTUL").prop("checked", data.T_VOLTUL);
                $("#cmPOL").prop("checked", data.T_POL);
            }

            if (data.enabled) {
                $("#currentmonbasic").show();
                if (data.devicetype == 0 || data.devicetype == 2) {
                    //DIYBMS Current Monitor (internal/external)
                    $("#currentmonadvanced").show();
                    $("#currentmonrelay").show();
                } else {
                    //PZEM-017
                    $("#currentmonadvanced").hide();

                    $("#currentmonrelay").hide();
                }
            } else {
                $("#currentmonadvanced").hide();
                $("#currentmonbasic").hide();
                $("#currentmonrelay").hide();
            }

        }).fail(function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
        );

}

// Show and hide tiles based on bit pattern in tileconfig array
function refreshVisibleTiles() {
    for (i = 0; i < TILE_IDS.length; i++) {
        var tc = TILE_IDS[i];
        var value = tileconfig[i];
        for (var a = tc.length - 1; a >= 0; a--) {
            var visible = (value & 1) == 1 ? true : false;
            value = value >>> 1;
            if (tc[a] != null && tc[a] != undefined && tc[a] != "") {
                var obj = $("#" + tc[a]);
                if (visible) {
                    //Only show if we have not force hidden it
                    if (obj.hasClass(".hide") == false) {
                        obj.addClass("vistile").show();
                    }
                } else {
                    obj.hide();
                }
            }
        }
    }
}


//Determine which tiles are visible and store config on controller
//as bitmap pattern
function postTileVisibiltity() {
    $(".stat.vistile.hide").removeClass("vistile");

    var newconfig = [];
    for (var index = 0; index < tileconfig.length; index++) {
        newconfig.push(0);
    }

    for (var i = 0; i < TILE_IDS.length; i++) {
        var tc = TILE_IDS[i];
        var value = 0;
        var v = 0x8000;
        for (var a = 0; a < tc.length; a++) {
            if (tc[a] != null && tc[a] != undefined && tc[a] != "") {
                if ($("#" + tc[a]).hasClass("vistile")) {
                    value = value | v;
                }
            }
            //Right shift onto next item
            v = v >>> 1;
        }
        newconfig[i] = value;
    }

    var diff = false;
    for (var index = 0; index < tileconfig.length; index++) {
        if (tileconfig[index] != newconfig[index]) {
            tileconfig[index] = newconfig[index];
            diff = true;
        }
    }

    if (diff) {
        $.ajax({
            type: 'POST',
            url: '/post/visibletiles',
            //This is crappy, but ESP isn't great at handling POST array values
            data: $.param({ v0: tileconfig[0], v1: tileconfig[1], v2: tileconfig[2], v3: tileconfig[3], v4: tileconfig[4] }),
            success: function (data) {
                //refresh if needed
                if ($('#home').hasClass('active')) {
                    refreshVisibleTiles();
                }
            },
            error: function (data) {
                showFailure();
            },
        });
    }

}


function loadVisibleTileData() {
    //Only refresh if array is empty
    if (tileconfig.length == 0) {

        $.getJSON("/api/tileconfig",
            function (data) {
                for (i = 0; i < data.tileconfig.values.length; i++) {
                    tileconfig[i] = data.tileconfig.values[i];
                }
                refreshVisibleTiles();
            }).fail(
                function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
            );
    }
}


function showFailure() {
    $.notify($("#saveerror").text(), { className: 'error', autoHideDelay: 15000 });
}

function showSuccess() {
    $.notify($("#savesuccess").text(), { className: 'success' });
}
function showActionSuccess() {
    $.notify($("#actionsuccess").text(), { className: 'success' });
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
    $.getJSON("/api/avrstatus",
        function (data) {
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

    $.getJSON("/api/modules", { c: cellid },
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

                if (data.settings.Prohibited) {
                    $('#ActualVoltage').attr('disabled', 'disabled');
                    $('#Calib').attr('disabled', 'disabled');
                    $('#BypassThresholdmV').attr('disabled', 'disabled');
                    $('#BypassOverTempShutdown').attr('disabled', 'disabled');
                    $('#CalculateCalibration').hide();
                } else {
                    $('#ActualVoltage').removeAttr('disabled');
                    $('#Calib').removeAttr('disabled');
                    $('#BypassThresholdmV').removeAttr('disabled');
                    $('#BypassOverTempShutdown').removeAttr('disabled');
                    $('#CalculateCalibration').show();
                }

                if (data.settings.ver === 490) {
                    $('#ParasiteVoltage').val(data.settings.Parasite);
                    $('#FanSwitchOnT').val(data.settings.FanSwitchOnT);
                    $('#RelayMinV').val(data.settings.RelayMinV);
                    $('#RelayRange').val(data.settings.RelayRange);

                    $('#RunAwayMinmV').val(data.settings.RunAwayMinmV);
                    $('#RunAwayDiffmV').val(data.settings.RunAwayDiffmV);

                    $('#v490_1').show();
                    $('#v490_2').show();
                    $('#v490_3').show();
                    $('#v490_4').show();

                    if (data.settings.Prohibited) {
                        $('#FanSwitchOnT').attr('disabled', 'disabled');
                        $('#RelayMinV').attr('disabled', 'disabled');
                        $('#RelayRange').attr('disabled', 'disabled');
                        $('#RunAwayMinmV').attr('disabled', 'disabled');
                        $('#RunAwayDiffmV').attr('disabled', 'disabled');
                    } else {
                        $('#FanSwitchOnT').removeAttr('disabled');
                        $('#RelayMinV').removeAttr('disabled');
                        $('#RelayRange').removeAttr('disabled');
                        $('#RunAwayMinmV').removeAttr('disabled');
                        $('#RunAwayDiffmV').removeAttr('disabled');
                    }

                } else {
                    $('#v490_1').hide();
                    $('#v490_2').hide();
                    $('#v490_3').hide();
                    $('#v490_4').hide();
                    $('#v490_5').hide();
                    $('#v490_6').hide();
                }

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

    if (seconds < 0) {
        return "";
    }

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
    $.getJSON("/api/monitor2", function (jsondata) {
        var labels = [];
        var cells = [];
        var bank = [];
        var voltages = [];
        var voltagesmin = [];
        var voltagesmax = [];
        var tempint = [];
        var tempext = [];
        var pwm = [];

        var minVoltage = DEFAULT_GRAPH_MIN_VOLTAGE / 1000.0;
        var maxVoltage = DEFAULT_GRAPH_MAX_VOLTAGE / 1000.0;

        var minExtTemp = 999;
        var maxExtTemp = -999;

        var bankNumber = 0;
        var cellsInBank = 0;

        // Need one color for each bank, could make it colourful I suppose :-)
        const colours = [
            '#55a1ea', '#33628f', '#498FD0', '#6D8EA0',
            '#55a1ea', '#33628f', '#498FD0', '#6D8EA0',
            '#55a1ea', '#33628f', '#498FD0', '#6D8EA0',
            '#55a1ea', '#33628f', '#498FD0', '#6D8EA0',
        ]

        const red = '#B44247'

        const highestCell = '#8c265d'
        const lowestCell = '#b6a016'

        var markLineData = [];

        markLineData.push({ name: 'avg', type: 'average', lineStyle: { color: '#ddd', width: 2, type: 'dotted', opacity: 0.3 }, label: { distance: [10, 0], position: 'start', color: "#eeeeee", textBorderColor: '#313131', textBorderWidth: 2 } });
        //markLineData.push({ name: 'min', type: 'min', lineStyle: { color: '#ddd', width: 2, type: 'dotted', opacity: 0.3 }, label: { distance: [10, 0], position: 'start', color: "#eeeeee", textBorderColor: '#313131', textBorderWidth: 2 } });
        //markLineData.push({ name: 'max', type: 'max', lineStyle: { color: '#ddd', width: 2, type: 'dotted', opacity: 0.3 }, label: { distance: [10, 0], position: 'start', color: "#eeeeee", textBorderColor: '#313131', textBorderWidth: 2 } });

        var xAxis = 0;
        for (let index = 0; index < jsondata.banks; index++) {
            markLineData.push({ name: "Bank " + index, xAxis: xAxis, lineStyle: { color: colours[index], width: 4, type: 'dashed', opacity: 0.5 }, label: { show: true, distance: [0, 0], formatter: '{b}', color: '#eeeeee', textBorderColor: colours[index], textBorderWidth: 2 } });
            xAxis += jsondata.seriesmodules;
        }

        if (jsondata.voltages) {
            //Clone array of voltages
            tempArray = [];
            for (i = 0; i < jsondata.voltages.length; i++) {
                tempArray[i] = jsondata.voltages[i];
            }

            //Split voltages into banks
            sorted_voltages = [];
            for (i = 0; i < jsondata.banks; i++) {
                unsorted = tempArray.splice(0, jsondata.seriesmodules);
                sorted_voltages.push(unsorted.sort());
            }

            for (let i = 0; i < jsondata.voltages.length; i++) {
                labels.push(bankNumber + "/" + i);

                // Make different banks different colours (stripes)
                var stdcolor = colours[bankNumber];

                var color = stdcolor;

                //Highlight lowest cell voltage in this bank
                if (jsondata.voltages[i] === sorted_voltages[bankNumber][0]) {
                    color = lowestCell;
                }
                //Highlight highest cell voltage in this bank
                if (jsondata.voltages[i] === sorted_voltages[bankNumber][jsondata.seriesmodules - 1]) {
                    color = highestCell;
                }
                // Red
                if (jsondata.bypass[i] === 1) {
                    color = red;
                }

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
                var exttemp = (jsondata.exttemp[i] == -40 ? 0 : jsondata.exttemp[i]);
                tempext.push({ value: exttemp, itemStyle: { color: stdcolor } });

                if (jsondata.exttemp[i] != null) {
                    if (exttemp > maxExtTemp) {
                        maxExtTemp = exttemp;
                    }
                    if (exttemp < minExtTemp) {
                        minExtTemp = exttemp;
                    }
                }


                pwm.push({ value: jsondata.bypasspwm[i] == 0 ? null : Math.trunc(jsondata.bypasspwm[i] / 255 * 100) });
            }
        }

        //Scale down for low voltages
        if (minVoltage < 0) { minVoltage = 0; }

        if (jsondata) {
            $("#badcrc .v").html(jsondata.badcrc);
            $("#ignored .v").html(jsondata.ignored);
            $("#sent .v").html(jsondata.sent);
            $("#received .v").html(jsondata.received);
            $("#roundtrip .v").html(jsondata.roundtrip);
            $("#oos .v").html(jsondata.oos);
            $("#canfail .v").html(jsondata.can_fail);
            $("#canrecerr .v").html(jsondata.can_r_err);
            $("#cansent .v").html(jsondata.can_sent);
            $("#canrecd .v").html(jsondata.can_rec);
            $("#qlen .v").html(jsondata.qlen);
            $("#uptime .v").html(secondsToHms(jsondata.uptime));
            if (minExtTemp == 999 || maxExtTemp == -999) {
                $("#celltemp .v").html("");
            } else {
                $("#celltemp .v").html(minExtTemp + "/" + maxExtTemp + "&deg;C");
            }

            if (jsondata.activerules == 0) {
                $("#activerules").hide();
            } else {
                $("#activerules").html(jsondata.activerules);
                $("#activerules").show(400);
            }

            if (jsondata.dyncv) {
                $("#dyncvolt .v").html(parseFloat(jsondata.dyncv / 10).toFixed(2) + "V");
            } else { $("#dyncvolt .v").html(""); }

            if (jsondata.dyncc) {
                $("#dynccurr .v").html(parseFloat(jsondata.dyncc / 10).toFixed(2) + "A");
            } else { $("#dynccurr .v").html(""); }



            switch (jsondata.cmode) {
                case 0: $("#chgmode .v").html("Standard"); break;
                case 1: $("#chgmode .v").html("Absorb " + secondsToHms(jsondata.ctime)); break;
                case 2: $("#chgmode .v").html("Float " + secondsToHms(jsondata.ctime)); break;
                case 3: $("#chgmode .v").html("Dynamic"); break;
                case 4: $("#chgmode .v").html("Stopped"); break;
                default: $("#chgmode .v").html("Unknown");
            }
        }

        if (jsondata.bankv) {
            for (var bankNumber = 0; bankNumber < jsondata.bankv.length; bankNumber++) {
                $("#voltage" + bankNumber + " .v").html((parseFloat(jsondata.bankv[bankNumber]) / 1000.0).toFixed(2) + "V");
                $("#range" + bankNumber + " .v").html(jsondata.voltrange[bankNumber] + "mV");
                $("#voltage" + bankNumber).removeClass("hide");
                $("#range" + bankNumber).removeClass("hide");
            }

            for (var bankNumber = jsondata.bankv.length; bankNumber < MAXIMUM_NUMBER_OF_BANKS; bankNumber++) {
                $("#voltage" + bankNumber).hide().addClass("hide");
                $("#range" + bankNumber).hide().addClass("hide");
            }
        }


        if (jsondata.current) {
            if (jsondata.current[0] == null) {
                $("#current .v").html("");
                $("#shuntv .v").html("");
                $("#soc .v").html("");
                $("#power .v").html("");
                $("#amphout .v").html("");
                $("#amphin .v").html("");
                $("#damphout .v").html("");
                $("#damphin .v").html("");
                $("#time100 .v").html("");
                $("#time10 .v").html("");
                $("#time20 .v").html("");
            } else {
                var data = jsondata.current[0];
                $("#current .v").html(parseFloat(data.c).toFixed(2) + "A");
                $("#shuntv .v").html(parseFloat(data.v).toFixed(2) + "V");
                $("#soc .v").html(parseFloat(data.soc).toFixed(2) + "%");
                $("#power .v").html(parseFloat(data.p) + "W");
                $("#amphout .v").html((parseFloat(data.mahout) / 1000).toFixed(3));
                $("#amphin .v").html((parseFloat(data.mahin) / 1000).toFixed(3));
                $("#damphout .v").html((parseFloat(data.dmahout) / 1000).toFixed(3));
                $("#damphin .v").html((parseFloat(data.dmahin) / 1000).toFixed(3));

                if (data.time100 > 0) {
                    $("#time100 .v").html(secondsToHms(data.time100));
                } else { $("#time100 .v").html("&infin;"); }
                if (data.time20 > 0) {
                    $("#time20 .v").html(secondsToHms(data.time20));
                } else { $("#time20 .v").html("&infin;"); }
                if (data.time10 > 0) {
                    $("#time10 .v").html(secondsToHms(data.time10));
                } else { $("#time10 .v").html("&infin;"); }
            }
        }

        //Loop size needs increasing when more warnings are added
        if (jsondata.warnings) {
            for (let warning = 1; warning <= 9; warning++) {
                if (jsondata.warnings.includes(warning)) {
                    //Once a warning has triggered, hide it from showing in the future
                    if ($("#warning" + warning).data("notify") == undefined) {
                        $("#warning" + warning).data("notify", 1);
                        $.notify($("#warning" + warning).text(), { autoHideDelay: 15000, globalPosition: 'top left', className: 'warn' });
                    }
                }
            }

            //Allow charge/discharge warnings to reappear
            if (jsondata.warnings.includes(7) == false) {
                $("#warning7").removeData("notify");
            }
            if (jsondata.warnings.includes(8) == false) {
                $("#warning8").removeData("notify");
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

            //As the module page is open, we refresh the last 3 columns using seperate JSON web service to keep the monitor2
            //packets as small as possible

            $.getJSON("/api/monitor3", function (jsondata) {
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
                        show: true, axisPointer: {
                            type: 'cross', label: {
                                backgroundColor: '#6a7985'
                            }
                        }
                    },
                    legend: {
                        show: false
                    },
                    xAxis: [{
                        gridIndex: 0, type: 'category', axisLine: {
                            lineStyle: {
                                color: '#c1bdbd'
                            }
                        }
                    }, {
                        gridIndex: 1, type: 'category', axisLine: {
                            lineStyle: { color: '#c1bdbd' }
                        }
                    }],
                    yAxis: [{
                        id: 0, gridIndex: 0, name: 'Volts', type: 'value', min: 2.5, max: 4.5, interval: 0.25, position: 'left',
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
                        gridIndex: 0, name: 'Bypass', type: 'value', min: 0,
                        max: 100, interval: 10, position: 'right',
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
                            lineStyle: { color: '#c1bdbd' }
                        },
                        axisLabel: { formatter: '{value}°C' }
                    }],
                    series: [
                        {
                            xAxisIndex: 0,
                            name: 'Voltage',
                            yAxisIndex: 0,
                            type: 'bar',
                            data: [],
                            markLine: {
                                silent: true, symbol: 'none', data: markLineData
                            },
                            itemStyle: { color: '#55a1ea', barBorderRadius: [8, 8, 0, 0] },
                            label: {
                                normal: {
                                    show: true, position: 'insideBottom', distance: 10, align: 'left', verticalAlign: 'middle', rotate: 90, formatter: '{c}V', fontSize: 24, color: '#eeeeee', fontFamily: 'Share Tech Mono'
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
                                    show: true, position: 'bottom', distance: 5, formatter: '{c}V', fontSize: 14, color: '#eeeeee', fontFamily: 'Share Tech Mono'
                                }
                            },
                            symbolSize: 16,
                            symbol: ['circle'],
                            itemStyle: {
                                normal: {
                                    color: "#c1bdbd", lineStyle: { color: 'transparent' }
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
                                    show: true, position: 'top', distance: 5, formatter: '{c}V', fontSize: 14, color: '#c1bdbd', fontFamily: 'Share Tech Mono'
                                }
                            },
                            symbolSize: 16,
                            symbol: ['arrow'],
                            itemStyle: {
                                normal: {
                                    color: "#c1bdbd", lineStyle: { color: 'transparent' }
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
                                    show: true, position: 'right', distance: 5, formatter: '{c}%', fontSize: 14, color: '#f0e400', fontFamily: 'Share Tech Mono'
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
                                color: '#55a1ea', barBorderRadius: [8, 8, 0, 0]
                            },
                            label: {
                                normal: {
                                    show: true, position: 'insideBottom', distance: 8,
                                    align: 'left', verticalAlign: 'middle',
                                    rotate: 90, formatter: '{c}°C', fontSize: 20, color: '#eeeeee', fontFamily: 'Share Tech Mono'
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
                                color: '#55a1ea', barBorderRadius: [8, 8, 0, 0]
                            },
                            label: {
                                normal: {
                                    show: true, position: 'insideBottom', distance: 8,
                                    align: 'left', verticalAlign: 'middle', rotate: 90,
                                    formatter: '{c}°C', fontSize: 20, color: '#eeeeee', fontFamily: 'Share Tech Mono'
                                }
                            }

                        }
                    ],
                    grid: [
                        {
                            containLabel: false, left: '4%', right: '4%', bottom: '30%'

                        }, {
                            containLabel: false, left: '4%', right: '4%', top: '76%'
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
        setTimeout(queryBMS, 3500);

        loadVisibleTileData();

    }).fail(function (jqXHR, textStatus, errorThrown) {

        if (jqXHR.status == 400 && jqXHR.responseJSON.error === "Invalid cookie") {
            if ($("#warningXSS").data("notify") == undefined) {
                $("#warningXSS").data("notify", 1);
                $.notify($("#warningXSS").text(), { autoHide: false, globalPosition: 'top left', className: 'error' });
            }
        } else {
            //Other type of error
            $("#iperror").show();
            //Try again in a few seconds (2 seconds if errored)
            setTimeout(queryBMS, 2000);
            $("#loading").hide();
        }
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
    $(".stat").hide();

    if (window.Graph3DAvailable === true) {
        //Re-show this as pagecode would have hidden it
        $("#graphOptions").show();
    }

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
            $(value).append('<select id="rule' + (index) + 'relay' + relay + '" class="rule" name="rule' + (index) + 'relay' + relay + '"><option>On</option><option>Off</option><option>X</option></select>');
        });
    }
    );

    $(".rule").on("change", function () {
        var origv = $(this).attr("data-origv")
        if (origv !== this.value) {
            $(this).addClass("modified");
        } else {
            $(this).removeClass("modified");
        }
    });

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
        $(".graphs").show();
        $("#graph1").show();
        $("#graph2").hide();
        $("#info .stat").not(".vistile").hide();
        $("#info .vistile").not(".hide").show();
        switchPage("#homePage");
        g1.resize();
        refreshVisibleTiles();

        if (window.Graph3DAvailable === true) {
            $('#graphOptions').show();
        }

        return true;
    });

    $("#tiles").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        //Use the home page for the grid, but hide graphs
        switchPage("#homePage");
        $(".graphs").hide();
        $("#graph1").hide();
        $("#graph2").hide();
        //Show all statistic panels (which are not forced hidden)
        $("#info .stat").not(".hide").show();
        return true;
    });


    $("#history").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        switchPage("#historyPage");

        $.getJSON("/api/history",
            function (data) {
                $("#historyTable tbody").empty();

                for (var index = 0; index < data.time.length; index++) {
                    var dt = new Date(data.time[index] * 1000).toLocaleString();
                    var newRowContent = "<tr><td>" + dt + "</td>"
                        + "<td>" + data.voltage[index] + "</td>"
                        + "<td>" + data.current[index] + "</td>"
                        + "<td>" + data.stateofcharge[index] + "</td>"
                        + "<td>" + data.milliamphour_in[index] + "</td>"
                        + "<td>" + data.milliamphour_out[index] + "</td>"
                        + "<td>" + data.highestBankRange[index] + "</td>"
                        + "<td>" + data.address_LowCellV[index] + "</td>"
                        + "<td>" + data.lowestCellVoltage[index] + "</td>"
                        + "<td>" + data.address_HighCellV[index] + "</td>"
                        + "<td>" + data.highestCellVoltage[index] + "</td>"
                        + "<td>" + data.lowestBankVoltage[index] + "</td>"
                        + "<td>" + data.highestBankVoltage[index] + "</td>"
                        + "<td>" + data.lowestExternalTemp[index] + "</td>"
                        + "<td>" + data.highestExternalTemp[index] + "</td>"
                        + "</tr>";
                    $("#historyTable tbody").append(newRowContent);
                }

            }).fail(function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
            );

        return true;
    });


    $("#diagbutton").click(function () {
        $.getJSON("/api/diagnostic",
            function (data) {

                data.diagnostic.tasks.sort((a, b) => a.hwm - b.hwm);

                $("#MinFreeHeap").html(data.diagnostic.MinFreeHeap);
                $("#FreeHeap").html(data.diagnostic.FreeHeap);
                $("#HeapSize").html(data.diagnostic.HeapSize);
                $("#SdkVersion").html(data.diagnostic.SdkVersion);
                $("#NumberRunningTasks").html(data.diagnostic.numtasks);
                $("#tasks").empty();
                $("#tasks").append("<thead><tr><th>Number</th><th>Name</th><th>Stack High Watermark</th></tr></thead>");

                $.each(data.diagnostic.tasks, function (index, value) {
                    $("#tasks").append("<tr><td>" + index + "</td><td>" + value.name + "</td><td>" + value.hwm + "</td></tr>");
                });

                if (data.diagnostic.coredump) {
                    let cd = data.diagnostic.coredump;
                    $("#coredumptask").html(cd.exc_task + ", cause:0x" + cd.exc_cause);
                    $("#backtrace").empty();
                    var text = "Guru Meditation Error:" + cd.exc_task + "\n";
                    text += "PC: 0x" + cd.exc_pc + "\n";

                    text += "EXC_A: ";
                    for (let index = 0; index < cd.exc_a.length; index++) {
                        text += "0x" + cd.exc_a[index] + " ";
                    }
                    text += "\n";

                    if (cd.epcx) {
                        text += "EPCX: ";
                        for (let index = 0; index < cd.epcx.length; index++) {
                            text += "0x" + cd.epcx[index] + " ";
                        }
                        text += "\n";
                    }

                    text += "EXCCAUSE: 0x" + cd.exc_cause + "\n";
                    text += "EXCVADDR: 0x" + cd.exc_vaddr + "\n";
                    text += "EXCTCB: 0x" + cd.exc_tcb + "\n";
                    text += "EPCX_REG_BITS: 0x" + cd.epcx_reg_bits + "\n";
                    text += "DUMPVER: " + cd.dumpver + "\n";
                    text += "CORRUPTED: " + cd.bt_corrupted + "\n";

                    text += "ELF file SHA256: " + cd.app_elf_sha256 + "\n\nBacktrace: ";

                    for (let index = 0; index < cd.bt_depth; index++) {
                        text += "0x" + cd.backtrace[index] + ":0x0 ";
                    }

                    $("#backtrace").text(text);
                }

                $("#diagnostics").show();

            }).fail(function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
            );

        return true;
    });

    $("#about").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        switchPage("#aboutPage");

        $("#diagnostics").hide();

        $.getJSON("/api/settings",
            function (data) {
                $("#HostName").html("<a href='http://" + data.settings.HostName + "'>" + data.settings.HostName + "</a>");
            }).fail(function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
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

        $.getJSON("/api/settings",
            function (data) {
                $("#g1").val(data.settings.bypassovertemp);
                $("#g2").val(data.settings.bypassthreshold);

                $("#modulesPage").show();
            }).fail(function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
            );
        return true;
    });

    $("#settings").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");

        $("#banksForm").hide();
        $("#settingsPage").show();

        $("#VoltageHigh").val(DEFAULT_GRAPH_MAX_VOLTAGE);
        $("#VoltageLow").val(DEFAULT_GRAPH_MIN_VOLTAGE);

        switchPage("#settingsPage");

        $.getJSON("/api/settings",
            function (data) {

                $("#NTPServer").val(data.settings.NTPServerName);
                $("#NTPZoneHour").val(data.settings.TimeZone);
                $("#NTPZoneMin").val(data.settings.MinutesTimeZone);
                $("#NTPDST").prop("checked", data.settings.DST);

                $("#timenow").html(data.settings.datetime);

                $("#totalSeriesModules").val(data.settings.totalseriesmodules);
                $("#totalBanks").val(data.settings.totalnumberofbanks);

                $("#baudrate").empty();
                $("#baudrate").append('<option value="2400">Standard</option>')
                $("#baudrate").append('<option value="5000">5K</option>')
                $("#baudrate").append('<option value="9600">9K6</option>')
                $("#baudrate").append('<option value="10000">10K</option>')

                $("#baudrate").val(data.settings.baudrate);

                $("#interpacketgap").empty();
                for (let index = 2000; index < 10000; index += 500) {
                    $("#interpacketgap").append('<option value="' + index + '">' + index + '</option>')
                }

                $("#interpacketgap").val(data.settings.interpacketgap);
                $("#banksForm").show();



                if (data.settings.hasOwnProperty("run_ip")) {
                    $("#run_ip").val(data.settings.run_ip);
                    $("#run_netmask").val(data.settings.run_netmask);
                    $("#run_gw").val(data.settings.run_gw);
                }

                if (data.settings.hasOwnProperty("run_dns1")) {
                    $("#run_dns1").val(data.settings.run_dns1);
                }
                if (data.settings.hasOwnProperty("run_dns2")) {
                    $("#run_dns2").val(data.settings.run_dns2);
                }
                if (data.settings.hasOwnProperty("HostName")) {
                    $("#run_hostname").val(data.settings.HostName);
                }

                // Manual IP settings
                if (data.settings.man_ip === "0.0.0.0") {
                    $("#new_ip").val(data.settings.run_ip);
                    $("#new_gw").val(data.settings.run_gw);
                    $("#new_netmask").val(data.settings.run_netmask);
                    $("#new_dns1").val(data.settings.run_dns1);
                    $("#new_dns2").val(data.settings.run_dns2);
                } else {
                    $("#new_ip").val(data.settings.man_ip);
                    $("#new_gw").val(data.settings.man_gw);
                    $("#new_netmask").val(data.settings.man_netmask);
                    $("#new_dns1").val(data.settings.man_dns1);
                    $("#new_dns2").val(data.settings.man_dns2);
                }

                $("#networkForm").show();

                
                $("#rssi_now").val(data.wifi.rssi);
                $("#bssid").val(data.wifi.bssid);
                $("#ssid").val(data.wifi.ssid);
                
                $("#rssi_low").val(data.wifi.rssi_low);
                $("#sta_start").val(data.wifi.sta_start);
                $("#sta_connected").val(data.wifi.sta_connected);
                $("#sta_disconnected").val(data.wifi.sta_disconnected);
                $("#sta_lost_ip").val(data.wifi.sta_lost_ip);
                $("#sta_got_ip").val(data.wifi.sta_got_ip);             

            }).fail(function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
            );

        return true;
    });

    function loadRelayRules() {
        $.getJSON("/api/rules",
            function (data) {
                //Rules have loaded
                $("#minutesnow").html(data.timenow);

                //Default relay settings
                $.each(data.relaydefault, function (index2, value2) {
                    var relay_value = "X";
                    if (value2 === true) { relay_value = "On"; }
                    if (value2 === false) { relay_value = "Off"; }
                    $("#defaultrelay" + (index2 + 1)).val(relay_value).attr("data-origv", relay_value).removeClass("modified");
                });

                //Default relay settings
                $.each(data.relaytype, function (index2, value2) {
                    $("#relaytype" + (index2 + 1)).val(value2).attr("data-origv", value2).removeClass("modified");
                });

                //Loop through each rule updating the page
                var i = 1;
                var allrules = $(".settings table tbody tr td label");
                $.each(data.rules, function (index, value) {
                    $("#rule" + (index) + "value").val(value.value).attr("data-origv", value.value).removeClass("modified");

                    $("#rule" + (index) + "hyst").val(value.hysteresis).attr("data-origv", value.hysteresis).removeClass("modified");

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

                        $("#rule" + (index) + "relay" + (index2 + 1)).val(relay_value).attr("data-origv", relay_value).removeClass("modified");
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
            }).fail(function () { $.notify("Read request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
            );
    }

    $("#rules").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");

        $("#rulesForm").hide();

        switchPage("#rulesPage");

        loadRelayRules();
        return true;
    });

    $("#currentmonitor").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");

        switchPage("#diybmsCurrentMonitorPage");


        $.getJSON("/api/rs485settings",
            function (data) {
                $("#rs485baudrate").val(data.baudrate);
                $("#rs485databit").val(data.databits);
                $("#rs485parity").val(data.parity);
                $("#rs485stopbit").val(data.stopbits);
            }).fail(function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
            );

        refreshCurrentMonitorValues();

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

        $.getJSON("/api/integration",
            function (data) {

                $("#mqttEnabled").prop("checked", data.mqtt.enabled);
                $("#mqttBasicReporting").prop("checked", data.mqtt.basiccellreporting);
                $("#mqttTopic").val(data.mqtt.topic);
                $("#mqttUri").val(data.mqtt.uri);
                $("#mqttUsername").val(data.mqtt.username);
                $("#mqttPassword").val("");

                $("#mqttConnected").val(data.mqtt.connected);
                $("#mqttErrConnCount").val(data.mqtt.err_conn_count);
                $("#mqttErrTransCount").val(data.mqtt.err_trans_count);
                $("#mqttConnCount").val(data.mqtt.conn_count);
                $("#mqttDiscCount").val(data.mqtt.disc_count);

                $("#influxEnabled").prop("checked", data.influxdb.enabled);
                $("#influxUrl").val(data.influxdb.url);
                $("#influxDatabase").val(data.influxdb.bucket);
                $("#influxToken").val(data.influxdb.apitoken);
                $("#influxOrgId").val(data.influxdb.orgid);
                $("#influxFreq").val(data.influxdb.frequency);

                $("#haUrl").val(window.location.origin+"/ha");
                $("#haAPI").val(data.ha.api);                

                $("#haForm").show();
                $("#mqttForm").show();
                $("#influxForm").show();
            }).fail(function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
            );

        return true;
    });



    $("#mount").click(function () {
        $.ajax({
            type: 'POST',
            url: '/post/sdmount',
            data: $.param({ mount: 1 }),
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
            type: 'POST',
            url: '/post/wificonfigtofile',
            data: $.param({ save: 1 }),
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
            type: 'POST',
            url: '/post/saveconfigtofile',
            data: $.param({ save: 1 }),
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
            type: 'POST',
            url: '/post/sdunmount',
            data: $.param({ unmount: 1 }),
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
            type: 'POST',
            url: '/post/disableavrprog',
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
                    $("#warningXSS").removeData("notify");
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
            type: 'POST',
            url: '/post/enableavrprog',
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
            type: 'POST',
            url: '/post/avrprog',
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

    $("#utility").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        switchPage("#utilityPage");

        $("#setsoc").val(75.5);
    });

    $("#avrprogrammer").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        switchPage("#avrprogPage");

        $.getJSON("/api/avrstorage",
            function (data) {
                $("#avrprog").empty();
                $("#avrprogconfirm").hide();
                $("#selectedavrindex").val("");

                if (data.ProgModeEnabled == true) {
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
            }).fail(function () { $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' }); }
            );

        return true;
    });


    $("#charging").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        switchPage("#chargingPage");
        $.getJSON("/api/chargeconfig",
            function (data) {

                $("#canbusprotocol").val(data.chargeconfig.canbusprotocol);
                $("#canbusinverter").val(data.chargeconfig.canbusinverter);
                $("#canbusbaud").val(data.chargeconfig.canbusbaud);
                $("#nominalbatcap").val(data.chargeconfig.nominalbatcap);

                $("#chargevolt").val((data.chargeconfig.chargevolt / 10.0).toFixed(1));
                $("#chargecurrent").val((data.chargeconfig.chargecurrent / 10.0).toFixed(1));
                $("#dischargecurrent").val((data.chargeconfig.dischargecurrent / 10.0).toFixed(1));
                $("#dischargevolt").val((data.chargeconfig.dischargevolt / 10.0).toFixed(1));

                $("#cellminmv").val(data.chargeconfig.cellminmv);
                $("#cellmaxmv").val(data.chargeconfig.cellmaxmv);
                $("#kneemv").val(data.chargeconfig.kneemv);
                $("#sensitivity").val((data.chargeconfig.sensitivity / 10.0).toFixed(1));

                $("#cur_val1").val((data.chargeconfig.cur_val1 / 10.0).toFixed(1));
                $("#cur_val2").val((data.chargeconfig.cur_val2 / 10.0).toFixed(1));

                $("#cellmaxspikemv").val(data.chargeconfig.cellmaxspikemv);

                $("#chargetemplow").val(data.chargeconfig.chargetemplow);
                $("#chargetemphigh").val(data.chargeconfig.chargetemphigh);
                $("#dischargetemplow").val(data.chargeconfig.dischargetemplow);
                $("#dischargetemphigh").val(data.chargeconfig.dischargetemphigh);

                $("#absorptimer").val(data.chargeconfig.absorptimer);
                $("#floattimer").val(data.chargeconfig.floattimer);
                $("#socresume").val(data.chargeconfig.socresume);
                $("#floatvolt").val((data.chargeconfig.floatvolt / 10.0).toFixed(1));


                $("#stopchargebalance").prop("checked", data.chargeconfig.stopchargebalance);
                $("#socoverride").prop("checked", data.chargeconfig.socoverride);
                $("#socforcelow").prop("checked", data.chargeconfig.socforcelow);
                $("#dynamiccharge").prop("checked", data.chargeconfig.dynamiccharge);
                $("#preventcharging").prop("checked", data.chargeconfig.preventcharging);
                $("#preventdischarge").prop("checked", data.chargeconfig.preventdischarge);

                DrawChargingGraph();

            }).fail(function () {
                $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' });
            }
            );

        return true;
    });
    $("#storage").click(function () {
        $(".header-right a").removeClass("active");
        $(this).addClass("active");
        switchPage("#storagePage");

        $.getJSON("/api/storage",
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
                            link = "<a href='download?type=sdcard&file=" + encodeURI(value) + "'>" + value + "</a>";
                            if (value.endsWith(".json") && value.startsWith("backup_config_")) {
                                link += "<button class='small' onclick='restoreconfig(0,\"" + encodeURI(value) + "\")'>Restore</button>";
                            }
                            $("#sdcardfiles").append("<li>" + link + "</li>");
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
                            link = "<a href='download?type=flash&file=" + encodeURI(value) + "'>" + value + "</a>";
                            if (value.endsWith(".json") && (value.startsWith("cfg_") || value.startsWith("upld_"))) {
                                link += "<button class='small' onclick='restoreconfig(1,\"" + encodeURI(value) + "\")'>Restore</button>";
                            }
                            $("#flashfiles").append("<li>" + link + "</li>");
                        }
                    });
                }

            }).fail(function () {
                $.notify("Request failed", { autoHide: true, globalPosition: 'top right', className: 'error' });
            }
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
    
    $("#haForm").unbind('submit').submit(function (e) {
        e.preventDefault();

        $.ajax({
            type: $(this).attr('method'),
            url: $(this).attr('action'),
            data: $("#haAPI").serialize(),
            success: function (data) {
                showSuccess();
                $("#integration").click();
            },
            error: function (data) {
                showFailure();
            },
        });
    });
    

    $("#rulesForm").unbind('submit').submit(function (e) {
        e.preventDefault();
        $.ajax({
            type: $(this).attr('method'),
            url: $(this).attr('action'),
            // Filter to only elements which have changed
            data: $("#rulesForm input.rule.modified, #rulesForm select.rule.modified").serialize(),
            success: function (data) {
                showSuccess();
                //Force reload of values to refresh the input boxes/attributes
                loadRelayRules();
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

    $.ajaxPrefilter(function (options, originalOptions, jqXHR) {
        if (originalOptions.type.toUpperCase() !== 'POST' || options.type.toUpperCase() !== 'POST') {
            return;
        }

        if (options.data.length > 0) {
            options.data += '&';
        }

        options.data += $.param({ xss: XSS_KEY });
    });

    $(".stat").mouseenter(function () {
        $(this).addClass("hover");
    }).mouseleave(function () {
        $(this).removeClass("hover");
    });

    $(".stat").click(function () {
        if ($(this).hasClass("vistile") == false && $(this).hasClass(".hide") == false) {
            $(this).addClass("vistile");
        } else {
            $(this).removeClass("vistile");
        }

        clearTimeout(timer_postTileVisibiltity);
        timer_postTileVisibiltity = setTimeout(postTileVisibiltity, 3500);
    });

    $("#file_sel").change(function () { upload_firmware(); });
    $("#uploadfw").click(function () { $("#file_sel").click(); });

    $("#uploadfile").click(function () { $("#uploadfile_sel").click(); });
    $("#uploadfile_sel").change(function () { upload_file(); });

    $("#progress").hide();

    $("#homePage").show();

    $("#usedhcp").click(function () {
        $("#new_ip").val("");
        $("#new_netmask").val("");
        $("#new_gw").val("");
        $("#new_dns1").val("");
        $("#new_dns2").val("");
        $("#usedhcpsubmit").click();
    });


    //Redraw graph if one of the values changes (and focus lost)
    $("#chargecurrent")
        .add("#cellminmv")
        .add("#cellmaxmv")
        .add("#kneemv")
        .add("#cellmaxspikemv")
        .add("#cur_val1")
        .add("#cur_val2").focusout(function () {
            DrawChargingGraph();
        });

    $("#chargecurrent")
        .add("#cellminmv")
        .add("#cellmaxmv")
        .add("#kneemv")
        .add("#cellmaxspikemv")
        .add("#cur_val1")
        .add("#cur_val2").change(function () {
            DrawChargingGraph();
        });


    //On page ready
    queryBMS();


    //Automatically open correct sub-page based on hash
    var hash = $(location).attr('hash');
    switch (hash) {
        case "#tiles":
        case "#modules":
        case "#rules":
        case "#settings":
        case "#charging":
        case "#integration":
        case "#currentmonitor":
        case "#storage":
        case "#avrprogrammer":
        case "#utility":
        case "#history":
            $(hash).click();
            break;
    }

}); // end $(function ()
