// This file is for JAPANESE language translation
// https://www.w3schools.com/tags/ref_language_codes.asp

/*
Elements which contain a child element that is filled in at runtime are
deliberately NOT translated here, because .text() would delete that child:

  #mb2             contains <span id="labelMaxModules">
  #error2          contains <span id="missingmodule1"> and <span id="missingmodule2">
  #genericerror    contains <span id="genericerrcode">
  #genericwarning  contains <span id="genericwarningcode">
  #ap4             contains the Patreon <a><img></a> link

The following ids appear more than once in default.htm, so a "#id" selector is
ambiguous and they are left alone as well: #mb1, #b9, #b10, #b11

#in7 is the one exception - its only child is <i>username:password</i>, which is
static text, so the sentence is translated with the code inline and the italic
styling is lost.  No data is lost.
*/

// ---------------------------------------------------------------- navigation
$("#home").text("ホーム");
$("#modules").text("モジュール");
$("#settings").text("設定");
$("#rules").text("ルール");
$("#more").text("その他");

$("#integration").text("外部連携");
$("#currentmonitor").text("電圧・電流モニタ");
$("#storage").text("ストレージとログ");
$("#avrprogrammer").text("AVR ライタ");
$("#about").text("このシステムについて");

// -------------------------------------------------------------- page headings
$("#modulesPage > h1").text("モジュール");
$("#settingsPage > h1").text("設定");
$("#historyPage > h1").text("履歴");
$("#rulesPage > h1").text("ルール");
$("#diybmsCurrentMonitorPage > h1").text("電圧・電流モニタ");
$("#utilityPage > h1").text("ユーティリティ");
$("#storagePage > h1").text("ストレージとログ");
$("#avrprogPage > h1").text("AVR ライタ");
$("#aboutPage > h1").text("このシステムについて");

// ------------------------------------------------------- errors and warnings
$("#error1").text("コントローラがセルモジュールまたは電流モニタと通信できていません。");
$("#error3").text("コントローラが扱える上限を超える数のモジュールが設定されています。");
$("#error4").text("モジュールの応答を待っています");
$("#error5").text("モジュールが 0V を返しました。設定を確認してください");
$("#error6").text("コントローラのメモリが不足しました。");
$("#error7").text("緊急停止");

$("#warning1").text("警告: モジュールのバイパス電圧が全体設定と異なります");
$("#warning2").text("警告: モジュールのバイパス温度が全体設定と異なります");
$("#warning3").text("警告: モジュールのファームウェア版が混在しています。動作が不安定になる可能性があります");
$("#warning4").text("警告: モジュールのハードウェア／基板の版が混在しています");
$("#warning5").text("警告: ログが有効ですが SD カードが見つかりません");
$("#warning6").text("AVR 書き込みモード中は一部の機能が停止します");
$("#warning7").text("充電が禁止されています（充電設定による）");

$("#iperror").text("コントローラと通信できないため状態を更新できません。");
$("#jslibrary").text("Javascript ライブラリの読み込みに失敗しました。ページを再読み込みしてください。");
$("#saveerror").text("設定の保存に失敗しました。");
$("#savesuccess").text("設定を保存しました");
$("#sdcardmissing").text("SD カードが装着されていない、または現在利用できません");

// ------------------------------------------------------------- status tiles
$("#uptime > span.x.t").text("稼働時間:");
$("#received > span.x.t").text("受信パケット:");
$("#sent > span.x.t").text("送信パケット:");
$("#badcrc > span.x.t").text("CRC エラー:");
$("#ignored > span.x.t").text("無視した要求:");
$("#oos > span.x.t").text("順序エラー:");
$("#roundtrip > span.x.t").text("一巡時間 (ms):");
$("#qlen > span.x.t").text("送信キュー長:");
$("#current > span.x.t").text("電流:");
$("#power > span.x.t").text("電力:");
$("#shuntv > span.x.t").text("シャント電圧:");
$("#amphin > span.x.t").text("充電 Ah:");
$("#amphout > span.x.t").text("放電 Ah:");
$("#cansent > span.x.t").text("CAN 送信:");
$("#canrecd > span.x.t").text("CAN 受信:");
$("#canfail > span.x.t").text("CAN 送信エラー:");
$("#graphOptions > span.x.t").text("グラフ:");

// ------------------------------------------------------------- modules page
$("#globalConfig > h2").text("全体設定");
$("#gc1").text("すべてのモジュールに次の設定を適用します:");
$("#globalSettingsButton").text("設定を保存");
$("#settingConfig > h2").text("モジュール個別の設定");

$("#mpBank").text("バンク");
$("#mpModule").text("モジュール");
$("#mpVoltage").text("電圧");
$("#mpvmin").text("最低電圧");
$("#mpvmax").text("最高電圧");
$("#mptint").text("基板温度 °C");
$("#mptext").text("外部温度 °C");
$("#mpbypass").text("バイパス PWM %");
$("#mpbpc").text("不正パケット数");
$("#mppktr").text("受信パケット数");
$("#mpbal").text("バランス放電量");

$("label[for='g1']").text("バイパス過温度");
$("label[for='g2']").text("バイパス開始電圧 mV");
$("label[for='ModuleId']").text("モジュール ID");
$("label[for='Version']").text("モジュール版数");
$("label[for='BypassOverTempShutdown']").text("バイパス過温度 (°C)");
$("label[for='BypassThresholdmV']").text("バイパス開始電圧 (mV)");
$("label[for='Calib']").text("校正倍率");
$("label[for='ActualVoltage']").text("計算補助 － 実測電圧");
$("#CalculateCalibration").text("計算");
$("label[for='ExtBCoef']").text("外部温度センサ B 定数");
$("label[for='IntBCoef']").text("内部温度センサ B 定数");
$("label[for='LoadRes']").text("バランス抵抗値");
$("label[for='mVPerADC']").text("ADC 1 カウントあたりの mV");
$("label[for='ParasiteVoltage']").text("センサの寄生電圧 (mV)");
$("label[for='FanSwitchOnT']").text("ファン ON 温度 (°C)");
$("label[for='RelayMinV']").text("バランスリレー 最低電圧 (mV)");
$("label[for='RelayRange']").text("バランスリレー 最小電圧差 (mV)");
$("label[for='RunAwayMinmV']").text("暴走バランス セル最低電圧 (mV)");
$("label[for='RunAwayDiffmV']").text("暴走バランス 最小電圧差 (mV)");

// ------------------------------------------------------------ settings page
$("#mb3").text("例: 16 セルを 8 直列 2 並列 (8S2P) で構成している場合。");
$("#mb4").text("通信速度を上げられるのはハードウェア版 4.4 以降のモジュールだけです。速度を変えたらコントローラを再起動し、全モジュールが対応するファームウェアで動いていることを確認してください。");
$("label[for='totalSeriesModules']").text("直列セル数 (例: 8S)");
$("label[for='totalBanks']").text("並列バンク数 (例: 2P)");
$("label[for='baudrate']").text("通信速度");
$("label[for='interpacketgap']").text("パケット間隔 (ms)");
$("#banksForm > div > button").text("モジュールとバンクの設定を保存");
$("#settingsForm > div > button").text("設定を保存");

$("#wifi1").text("WiFi 統計");
$("#wifi2").text("WiFi の不具合を切り分けるために、ESP32 が処理した WiFi イベントの回数を表示しています");
$("#settingsPage > div:nth-child(5) > h2").text("時刻同期 (NTP)");
$("#settingsPage > div:nth-child(6) > h2").text("表示設定");

$("#mb5").text("Wi-Fi「ステーション」側インタフェースの情報:");
$("#mb6").text("Wi-Fi のアクセスポイント／SSID を変更したい場合は、コントローラを再起動し、USB シリアルケーブルとコンソールから設定してください。");
$("#mb7").text("Wi-Fi「ステーション」側には固定 IP アドレスを指定できます。保存したあと、反映するにはコントローラを再起動してください。");
$("#mb8").text("自動設定に戻すには「DHCP を使う」を押してください。こちらも再起動が必要です。");
$("#usedhcp").text("DHCP を使う");
$("#usedhcpsubmit").text("ネットワーク設定を保存");

$("label[for='run_hostname']").text("ホスト名");
$("label[for='run_ip']").text("IP アドレス");
$("label[for='run_netmask']").text("サブネットマスク");
$("label[for='run_gw']").text("ゲートウェイ");
$("label[for='run_dns1']").text("優先 DNS");
$("label[for='run_dns2']").text("代替 DNS");
$("label[for='new_ip']").text("IP アドレス");
$("label[for='new_netmask']").text("サブネットマスク");
$("label[for='new_gw']").text("ゲートウェイ");
$("label[for='new_dns1']").text("優先 DNS");
$("label[for='new_dns2']").text("代替 DNS");
$("label[for='ssid']").text("SSID");
$("label[for='bssid']").text("BSSID");
$("label[for='rssi_now']").text("現在の RSSI (dBm)");
$("label[for='rssi_low']").text("RSSI 低下イベント");
$("label[for='sta_start']").text("STA 開始");
$("label[for='sta_connected']").text("STA 接続");
$("label[for='sta_disconnected']").text("STA 切断");
$("label[for='sta_lost_ip']").text("STA IP 喪失");
$("label[for='sta_got_ip']").text("STA IP 取得");

$("label[for='NTPServer']").text("NTP サーバ");
$("label[for='NTPZoneHour']").text("時差 (時)");
$("label[for='NTPZoneMin']").text("時差 (分)");
$("label[for='NTPDST']").text("夏時間を有効にする");
$("#ntpForm > div > button").text("NTP 設定を保存");

$("label[for='Language']").text("言語");
$("label[for='VoltageHigh']").text("グラフ電圧範囲 上限 mV");
$("label[for='VoltageLow']").text("グラフ電圧範囲 下限 mV");
$("#displaySettingForm > div > button").text("表示設定を保存");

$("#savewifi").text("WiFi 設定を保存");
$("#saveconfig").text("設定を書き出し");
$("#restartControllerForm > div > button").text("コントローラを再起動");
$("#resetCountersForm > div > button").text("カウンタをリセット");

// --------------------------------------------------------------- rules page
$("#rt1").text("diyBMS は充電器・コンタクタ・負荷を安全に切り離すためのリレーモジュールに対応しています。ルールで自分の構成に合わせてリレーの動作を決められます。");
$("#rt2").text("ルールは優先度の低いものから高いものへ（表の下から上へ）処理されます。リレーの動作は選択肢で指定します。「X」は「指定なし＝優先度の低いルールが決めた状態のまま」という意味です。");
$("#rt3").text("ルールは対象の値がトリガ値以上になったときに発動します。解除されるのは値がリセット値を越えたときだけです。これによりリレーが細かく開閉し続けるのを防げます。");
$("#rt4").text("「タイマ 1」「タイマ 2」は時刻による動作です。午前 0 時からの経過分数が指定値に達すると有効になります。例えばタイマ 1 のトリガを 495、リセットを 555 にすると 8:15 に入り 9:15 に切れます。インターネットに接続して時刻を取得できている場合のみ動作します。");
$("#rt5").text("現在の午前 0 時からの経過分数:");
$("#rt6").text("緊急停止はコネクタ J1 で発動します。一度発動するとコントローラを再起動するまで解除されません。");

$("#rf1").text("ルール");
$("#rf2").text("トリガ値");
$("#rf3").text("リセット値");
$("#rf4").text("リレー状態");

$("label[for='rule0value']").text("緊急停止");
$("label[for='rule1value']").text("BMS 内部エラー");
$("label[for='rule2value']").text("電流モニタ 過電流 (A)");
$("label[for='rule3value']").text("セル単体 過電圧 (mV)");
$("label[for='rule4value']").text("セル 低電圧 (mV)");
$("label[for='rule5value']").text("モジュール 過温度（基板内部） °C");
$("label[for='rule6value']").text("モジュール 低温度（基板内部） °C");
$("label[for='rule7value']").text("セル 過温度（外部センサ） °C");
$("label[for='rule8value']").text("セル 低温度（外部センサ） °C");
$("label[for='rule9value']").text("電流モニタ 過電圧 (mV)");
$("label[for='rule10value']").text("電流モニタ 低電圧 (mV)");
$("label[for='rule11value']").text("バンク 過電圧 (mV)");
$("label[for='rule12value']").text("バンク 低電圧 (mV)");
$("label[for='rule13value']").text("バンク電圧差 (mV)");
$("label[for='rule14value']").text("タイマ 2");
$("label[for='rule15value']").text("タイマ 1");

$("label[for='defaultvalue']").text("リレー既定状態");
$("label[for='relaytype']").text("リレー種別");

// -------------------------------------------------------------- integration
$("#ip1").text("外部連携");
$("#ip2").text("安全のため、MQTT を有効にする場合や設定を変更する場合は、保存する前にパスワードを入力し直す必要があります。");
$("#ip4").text("URI は mqtt://192.168.0.26:1833 のような形式です");

$("label[for='mqttEnabled']").text("有効");
$("label[for='mqttUri']").text("URI");
$("label[for='mqttPort']").text("ポート");
$("label[for='mqttUsername']").text("ユーザ名");
$("label[for='mqttPassword']").text("パスワード");
$("label[for='mqttTopic']").text("トピック");
$("label[for='mqttBasicReporting']").text("セルの基本データのみ送信する");
$("#ip5").text("基本データのみにすると、ネットワークに流れる MQTT のデータ量を減らせます。");
$("label[for='mqttConnected']").text("接続状態");
$("label[for='mqttErrConnCount']").text("接続エラー回数");
$("label[for='mqttErrTransCount']").text("転送エラー回数");
$("label[for='mqttConnCount']").text("接続回数");
$("label[for='mqttDiscCount']").text("切断回数");
$("#mqttForm > div > button").text("MQTT 設定を保存");

$("#in1").text("InfluxDB");

$("#in2").text("API バージョン 2.X");
$("#in4").text("HTTP のみ対応しています。ローカルサーバの場合はドメイン名ではなく IP アドレスを指定してください。");
$("#in5").text("API バージョン 1.X");
$("#in7").text("v1 で使う場合は、ユーザ名とパスワードを username:password の形式にしてトークン欄に入れてください。");
$("#in8").text("organisation（組織）は使われないので、そのまま \"organisation\" と入れてください。");
$("#in9").text("URL は上の v2 の例と同じです。");

$("label[for='influxEnabled']").text("有効");
$("label[for='influxFreq']").text("記録間隔 (秒)");
$("label[for='influxUrl']").text("InfluxDB 書き込み API の URL");
$("label[for='influxToken']").text("API 認証トークン");
$("label[for='influxOrgId']").text("InfluxDB 2 の組織 ID");
$("label[for='influxDatabase']").text("バケット名またはデータベース名");
$("#influxForm > div > button").text("Influx 設定を保存");

$("#ha1").text("Home Assistant Web API");
$("label[for='haUrl']").text("エンドポイント API の URL");
$("label[for='haAPI']").text("API 認証トークン");

// ------------------------------------------------------------ current monitor
$("#diybmsCurrentMonitorPage > div:nth-child(2) > h2").text("接続");
$("#currentmonbasic > h2").text("基本設定");
$("#currentmonadvanced > h2").text("詳細設定");
$("#currentmonrefresh").text("値を再取得");

$("#b1").text("下の設定で電流モニタとの MODBUS 接続を設定します。");
$("#b2").text("RS485 インタフェースの設定です。通信は半二重です。");
$("#b3").text("シャント抵抗のパラメータが、お使いのシャントのデータシートと一致しているか確認してください。");
$("#b4").text("diyBMS の電流モニタはフルスケール 40.96mV です。これを超えるシャント電圧は比例して縮小されます。");
$("#b5").text("diyBMS の MODBUS シャントモニタ側のリレー制御を使う場合はここで設定します。この設定はコントローラ内蔵の電流モニタには影響しません。");
$("#b6").text("過電流側は放電中、低電流側は充電中に使われるので、放電と充電で別々の電流制限を設定できます。");
$("#b7").text("温度制限はチップのダイ温度が基準で、シャント本体の温度とは一致しないことがあります。正の温度係数のみ対応しています。");
$("#b8").text("リレートリガは、どのルールが成立したときにリレーを閉じるかを決めます。");
$("#b14").text("PZEM-017 は 9600,8,None,2 のシリアル設定を使います");
$("#b15").text("PZEM-017 は 75mV シャントを使います。この機器では充電率とアラームには対応していません。対応するシャントは 50, 100, 200, 300A のみです。");

$("label[for='CurrentMonEnabled']").text("有効");
$("label[for='CurrentMonDev']").text("機器");
$("label[for='modbusAddress']").text("Modbus アドレス");
$("label[for='shuntmv']").text("シャント出力電圧 (mV)");
$("label[for='shuntmaxcur']").text("シャント定格電流");
$("#diybmsCurrentMonitorForm1 > div > button").text("接続設定を保存");

$("label[for='cmbatterycapacity']").text("電池容量 (Ah)");
$("label[for='cmfullchargevolt']").text("満充電電圧");
$("label[for='cmtailcurrent']").text("テール電流 (A)");
$("label[for='cmchargeefficiency']").text("充電効率 %");
$("label[for='cmvalid']").text("値は有効か");
$("label[for='cmtimestampage']").text("最終通信からの経過 (ミリ秒)");
$("label[for='cmwatchdog']").text("ウォッチドッグ回数");
$("label[for='cmtemperature']").text("ダイ温度 °C");
$("label[for='cmresistance']").text("シャント抵抗値");
$("label[for='cmmodel']").text("センサ型番");
$("label[for='cmfirmwarev']").text("ファームウェア版数");
$("label[for='cmfirmwaredate']").text("ファームウェア日付");
$("label[for='cmRelayState']").text("リレー状態");
$("label[for='cmTemperatureOverLimit']").text("温度上限超過");
$("label[for='cmCurrentOverLimit']").text("電流上限超過");
$("label[for='cmCurrentUnderLimit']").text("電流下限未満");
$("label[for='cmVoltageOverLimit']").text("電圧上限超過");
$("label[for='cmVoltageUnderLimit']").text("電圧下限未満");
$("label[for='cmPowerOverLimit']").text("電力上限超過");

$("label[for='cmcalibration']").text("校正値");
$("label[for='cmtemplimit']").text("温度制限");
$("label[for='cmundervlimit']").text("低電圧しきい値");
$("label[for='cmovervlimit']").text("過電圧しきい値");
$("label[for='cmoverclimit']").text("過電流しきい値");
$("label[for='cmunderclimit']").text("低電流しきい値");
$("label[for='cmoverplimit']").text("過電力しきい値");
$("label[for='cmtempcoeff']").text("温度係数 ppm/°C");
$("#diybmsCurrentMonitorForm3 > div > button").text("詳細設定を保存");

$("label[for='TempCompEnabled']").text("温度係数を有効にする");
$("label[for='cmTMPOL']").text("リレートリガ: 温度");
$("label[for='cmCURROL']").text("リレートリガ: 過電流");
$("label[for='cmCURRUL']").text("リレートリガ: 低電流");
$("label[for='cmVOLTOL']").text("リレートリガ: 過電圧");
$("label[for='cmVOLTUL']").text("リレートリガ: 低電圧");
$("label[for='cmPOL']").text("リレートリガ: 電力");
$("#diybmsCurrentMonitorForm4 > div > button").text("リレートリガを保存");

$("label[for='rs485baudrate']").text("ボーレート");
$("label[for='rs485databit']").text("データビット");
$("label[for='rs485parity']").text("パリティ");
$("label[for='rs485stopbit']").text("ストップビット");
$("#RS485Form > div > button").text("RS485 設定を保存");

// ------------------------------------------------- charge/discharge (CAN/RS485)
$("#v0").text("充放電の設定");
$("#v1").text("この機能を使うと、diyBMS が電池と BMS の情報を各種のサードパーティ製プロトコルで CANBUS / RS485 機器に渡せます。");
$("#v2").text("この設定を使うには、CANBUS (RS485) 経由で充放電パラメータを受け取れるインバータ／充電器が必要です。Pylontech は通常 500k baud で動作します。Victron は機種によって別の速度を使うことがあります。");
$("#v3").text("温度制御には diyBMS モジュールの外部温度センサを使います。0°C 以下では充電できない LiFePO4 セルで特に有用です。");
$("#v4").text("CAN 接続の両端に終端抵抗を入れるのを忘れないでください。コントローラ側はジャンパ JP1 をはんだで短絡すれば終端できます。");
$("#v7").text("外部のインバータ／充電器と確実に連携するには、電流シャント／電流モニタが必要です。");

$("label[for='protocol']").text("エミュレートするプロトコル");
$("label[for='canbusinverter']").text("インバータのメーカー／機種");
$("label[for='canbusbaud']").text("CANBUS ボーレート");

$("#charging").text("充放電の設定");
$("label[for='nominalbatcap']").text("公称容量 (Ah)");
$("label[for='expected_cycles']").text("想定サイクル寿命");
$("label[for='eol_capacity']").text("寿命とみなす容量 (%)");
$("label[for='total_ah_charge']").text("積算充電量 (Ah)");
$("label[for='total_ah_discharge']").text("積算放電量 (Ah)");
$("label[for='estimate_bat_cycle']").text("消費したサイクル数（推定）");
$("label[for='stateofhealth']").text("劣化状態 SOH (%)");

$("label[for='chargevolt']").text("充電電圧 (V)");
$("label[for='chargecurrent']").text("充電電流の上限 (A)");
$("label[for='chargetemplow']").text("充電可能温度 下限 (°C)");
$("label[for='chargetemphigh']").text("充電可能温度 上限 (°C)");
$("label[for='absorptimer']").text("吸収充電タイマ (分)");
$("label[for='floatvolt']").text("フロート電圧 (V)");
$("label[for='floattimer']").text("フロートタイマ (分)");
$("label[for='socresume']").text("充電を再開する SoC");
$("label[for='dynamiccharge']").text("動的充電制御を有効にする");
$("label[for='cellmaxmv']").text("充電目標のセル最高電圧 (mV)");
$("label[for='cellmaxspikemv']").text("セル電圧スパイクの上限 (mV)");
$("label[for='kneemv']").text("充電カーブが直線でなくなる電圧［ニー］ (mV)");
$("label[for='sensitivity']").text("電圧感度");
$("label[for='cur_val1']").text("充電電流 設定値 1");
$("label[for='cur_val2']").text("充電電流 設定値 2");

$("label[for='dischargevolt']").text("放電の下限電圧 (V)");
$("label[for='dischargecurrent']").text("放電電流の上限 (A)");
$("label[for='dischargetemplow']").text("放電可能温度 下限 (°C)");
$("label[for='dischargetemphigh']").text("放電可能温度 上限 (°C)");
$("label[for='cellminmv']").text("セル単体の最低電圧 (mV)");
$("label[for='stopchargebalance']").text("セルのバランス動作中は充電を止める");

$("label[for='socoverride']").text("SoC を手動で指定する（既定はオフ）");
$("label[for='socforcelow']").text("SoC を強制的に低く扱う（既定はオフ）");
$("label[for='preventcharging']").text("充電を禁止する");
$("label[for='preventdischarge']").text("放電を禁止する");
$("label[for='setsoc']").text("SoC を設定 %");

// ------------------------------------------------------------ storage & logging
$("#storagePage > div:nth-child(2) > h2").text("SD カード");
$("#storagePage > div:nth-child(3) > h2").text("内蔵フラッシュ");
$("#storagePage > div:nth-child(4) > h2").text("ログ");
$("#b12").text("セルのデータと出力状態は SD カードにログファイルとして保存できます。");
$("#mount").text("マウント");
$("#unmount").text("アンマウント");
$("#uploadfile").text("ファイルをアップロード");
$("label[for='loggingEnabled']").text("SD カードへのログを有効にする");
$("label[for='loggingFreq']").text("セルデータの記録間隔 (秒)");
$("#loggingForm > div > button").text("ログ設定を保存");

// -------------------------------------------------------------- AVR programmer
$("#avrprogPage > div > p:nth-child(2)").text("手順:");
$("#avrprogPage > div > ol > li:nth-child(1)").text("「書き込みモードを有効にする」を押します。SD カードへのログと TFT タッチ操作が停止します。");
$("#avrprogPage > div > ol > li:nth-child(2)").text("書き込むファイルを選びます");
$("#avrprogPage > div > ol > li:nth-child(3)").text("警告をよく読みます");
$("#avrprogPage > div > ol > li:nth-child(4)").text("書き込む機器をコントローラの ISP コネクタに接続します");
$("#avrprogPage > div > ol > li:nth-child(5)").text("「書き込み実行」を押します");
$("#avrprogPage > div > ol > li:nth-child(6)").text("完了したら機器を取り外します");
$("#avrprogPage > div > ol > li:nth-child(7)").text("必要なら次の機器を書き込みます");
$("#avrprogPage > div > ol > li:nth-child(8)").text("書き込みモードを無効にします");

$("#avrprogconfirm > ul > li:nth-child(1)").text("書き込む前に、モジュール／機器から電池と通信ケーブルをすべて外してください。");
$("#avrprogconfirm > ul > li:nth-child(2)").text("フラットケーブルまたは変換基板でモジュールをコントローラの ISP ポートに接続します。切り欠きがモジュール側の 1 番ピンと合っていることを確認してください。");
$("#avrprogconfirm > ul > li:nth-child(3)").text("「書き込み実行」を押すと書き込みが始まります");
$("#avrprogconfirm > ul > li:nth-child(4)").text("書き込みが終わったらすぐにケーブルを外してください");

$("#AVRProgEnable").text("書き込みモードを有効にする");
$("#AVRProgDisable").text("書き込みモードを無効にする");
$("#ProgAVR").text("書き込み実行");
$("#ProgAVRCancel").text("中止");

// --------------------------------------------------------------------- about
$("#ap1").text("ソースコードとハードウェア");
$("#ap2").text("動画");
$("#ap5").text("警告");
$("#ap6").text("これは DIY の製品／仕組みです。安全上重要なシステムや、人命に関わる可能性のある用途には使用しないでください。");
$("#ap7").text("いかなる保証もありません。期待どおりに動かないことも、まったく動かないこともあります。");
$("#ap8").text("このプロジェクトの利用は完全に自己責任です。死に至る可能性のある電圧を扱うことがあります。少しでも不安があれば詳しい人に相談してください。");
$("#ap8a").text("このプロジェクトの利用は、お住まいの地域の法令や規制に適合しない場合があります。不安があれば詳しい人に相談してください。");
$("#ap9").text("ライセンス");
$("#ap10").text("この作品は Creative Commons 表示 - 非営利 - 継承 2.0 UK: England & Wales ライセンスの下で提供されています。");
$("#ap11").text("プラットフォームとバージョン");
$("#ap12").text("コントローラのファームウェア更新");
$("#uploadfw").text("新しいファームウェアをアップロード");
$("#ap13").text("診断");
$("#diagbutton").text("診断");

// -------------------------------------------------------- misc tabs / warnings
$("#utility").text("ユーティリティ");
$("#tiles").text("タイル");
$("#history").text("履歴");
$("#b13").text("導入と設定の解説動画（YouTube）");
$("#warning8").text("放電が禁止されています（放電設定による）");
$("#warning9").text("外部セル温度センサが見つかりません");
$("#warningXSS").text("Web ページの内容がコントローラとずれています。ページを再読み込み (F5) してください");
