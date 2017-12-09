/*
||
|| @version  1.0
|| @author  Jin zhouyun
|| @contact 2435575291@qq.com
|| State Machine Implementation
|| @description
||
*/

#include <SPI.h>
#include <SD.h>
#include <WiFi101.h>
#include <ArduinoJson.h>
#include "aWOT.h"
#include "FBD.h"
#include "PubSubClient.h"
#include "FiniteStateMachine.h"
#include <Wire.h>
#include "RTClib.h"

#define VBATPIN A7

File myFile;
String SensorJson = "{ \"DIOSetting\":[0, 0, 0, 0, 0],\
   \"ADCSetting\" : [\
  { \"Enable\":0,\"SetPointHigh\" : 2.5, \"SetPointLow\" : 1.0 },\
  { \"Enable\":0,\"SetPointHigh\" : 2.5, \"SetPointLow\" : 1.0 },\
  { \"Enable\":0,\"SetPointHigh\" : 2.5, \"SetPointLow\" : 1.0 },\
  { \"Enable\":0,\"SetPointHigh\" : 2.5, \"SetPointLow\" : 1.0 },\
  { \"Enable\":0,\"SetPointHigh\" : 2.5, \"SetPointLow\" : 1.0 },\
  { \"Enable\":0,\"SetPointHigh\" : 2.5, \"SetPointLow\" : 1.0 }\
  ]}";

String NetworkJson = "{\"ssid\":\"Aurora Inn Staff\",\"pass\":\"LittlePigLittlePigLetMeIn\",\"ipaddr\":\"\",\"subnet\":\"\",\"gateway\":\"\",\"dns\":\"\",\"MqttUrl\":\"test.mosquitto.org\",\"MqttPort\":\"1883\"}";

#define SD_CHIPSEL 10

static char mqtt_msg[1024];

// 0 - OFF, 1 - Idle, 2 - ON, 
#define STATUS_OFF 0
#define STATUS_IDLE 1
#define STATUS_ON 2
#define DEBOUNCE 200

#define DI_IDLE 2
#define DI_OFF 0
#define DI_ON 1

#define DICOUNT 5
#define ADCCOUNT 6
bool bSensorUsed[ADCCOUNT];
bool bDigitalUsed[DICOUNT]; // D5, D9, D11, D12, D13

							// A0~A5, We have to add debounce, too, becuase, ADC value will be present error value, 200ms debounce
uint8_t SensorCurrStatus[ADCCOUNT];
uint8_t SensorLastStatus[ADCCOUNT];
uint8_t SensorRealStatus[ADCCOUNT];

// default value of setpoints
static double SetPointLow[ADCCOUNT];
static double SetPointHigh[ADCCOUNT];

uint8_t ADCPins[ADCCOUNT] = { A0, A1, A2, A3, A4, A5 };

static uint32_t nLastTime[6];

#define D5 5
#define D9 9
#define D11 11
#define D12 12
#define D13 13

uint8_t bPreD5 = DI_IDLE;
uint8_t bPreD9 = DI_IDLE;
uint8_t bPreD11 = DI_IDLE;
uint8_t bPreD12 = DI_IDLE;
uint8_t bPreD13 = DI_IDLE;

static bool bWaitSetting = true;

RTC_PCF8523 rtc;

// This is AP mode time
const uint32_t nPreAPTime = 30000; // 30 sec

const uint16_t nMqttDisconTime = 30000; // Mqtt reconnect interval, 30 sec
const uint16_t nWifiConnInterval = 10000; // Hotsopt Reconnect interval, 10sec

static TON MqttDisconTON, WifiConnTON;
static Rtrg MqttDisconTrg, WifiConnTrg;

const uint16_t nBatteryHighTime = 30000; // Battery High Voltage Condition time, 
										 // if Battery voltage is higher than 4.2V for 30sec, it will off charger
const uint16_t nBatteryLowTime = 30000; // Battery Low Voltage Condition time, 
										// if Battery voltage is lower than 3.5V for 30sec, it will turn on  charger

static TON BattLowTON, BattHighTON;
static Rtrg BattLowTrg, BattHighTrg;

String IPAddrSetting = "";
String SubnetSetting = "";
String GateWaySetting = "";
String DnsSetting = "";

String DeviceID = "MiRemote00001";

void InitVars()
{
	for (uint8_t idx = 0; idx < 6; idx++)
	{
		SetPointLow[idx] = 125;
		SetPointHigh[idx] = 400;
		SensorRealStatus[idx] = STATUS_IDLE;
		SensorLastStatus[idx] = STATUS_IDLE;
		SensorCurrStatus[idx] = STATUS_IDLE;

		bSensorUsed[idx] = false;
		bDigitalUsed[idx] = false;
	}
}

void initFBD()
{
	MqttDisconTON.PRE = 0;
	MqttDisconTON.IN = 0;
	MqttDisconTON.Q = 0;
	MqttDisconTON.ET = 0;
	MqttDisconTON.PT = nMqttDisconTime;

	MqttDisconTrg.IN = 0;
	MqttDisconTrg.PRE = 0;
	MqttDisconTrg.Q = 0;


	WifiConnTON.IN = 0;
	WifiConnTON.Q = 0;
	WifiConnTON.PRE = 0;
	WifiConnTON.PT = nWifiConnInterval;
	WifiConnTON.ET = 0;

	WifiConnTrg.IN = 0;
	WifiConnTrg.PRE = 0;
	WifiConnTrg.Q = 0;

	BattLowTON.IN = 0;
	BattLowTON.PRE = 0;
	BattLowTON.Q = 0;
	BattLowTON.PT = nBatteryLowTime;
	BattLowTON.ET = 0;

	BattLowTrg.IN = 0;
	BattLowTON.Q = 0;
	BattLowTON.PRE = 0;

	BattHighTON.IN = 0;
	BattHighTON.PRE = 0;
	BattHighTON.Q = 0;
	BattHighTON.PT = nBatteryHighTime;
	BattHighTON.ET = 0;

	BattHighTrg.IN = 0;
	BattHighTrg.Q = 0;
	BattHighTrg.PRE = 0;
}

void APModeEnter()
{
	listNetworks();
	InitSoftAP();
	InitWebServer();
}

void ConnectEnter();
void ConnectingUpdate();

void STAEnter();
void STAUpdate();

State Startup = State(NULL);  //no operation
State PreAPState = State(APModeEnter, NULL, NULL);
State APState = State(NULL);  //no operation
State Connecting = State(ConnectEnter, ConnectingUpdate, NULL);
State STAState = State(STAEnter, STAUpdate, NULL);  //no operation

FSM stateMachine = FSM(Startup); //initialize state machine, start in state: noop

WebApp app;
WiFiServer *server;

String AvailableSSIDs = "";
// 
char AP_SSID[32];
char AP_PASS[32];

// Update these with values suitable for your network.

char ssid[50]; //  your network SSID (name)
char pass[50];    // your network password (use for WPA, or use as key for WEP)
char mqttserver[50]; // You can input ip address example 192.168.2.24
int status = WL_IDLE_STATUS;

String Strssid = "";
String Strpass = "";
String StrMqttUrl = "";
String StrMqttPort = "";

const char mqtt_OutTopic[] = "home/featherout";
const char mqtt_InputTopic[] = "home/featherinput";

// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

WiFiClient Wifi101client;
PubSubClient client(Wifi101client);

// Callback function
void callback(char* topic, byte* payload, unsigned int length) {

	// Message Processing
	String StrPacket = "";
	String StrPre = "";
	for (uint8_t idx = 0; idx < length; idx++)
		StrPre += (char)payload[idx];

	Serial.println("Mqtt message received!");
	Serial.println(StrPre);

	// Remove \r, \n
	for (uint8_t idx = 0; idx < StrPre.length(); idx++)
	{
		char ch = StrPre.charAt(idx);
		if (ch != 0x0D && ch != 0x0A)
			StrPacket += (char)ch;
	}

	// Command format is ID=Feather00001,Cmd=S,L0=1.0,H0=2.5,L1=1.0,H1=2.5,L2=1.0,H2=2.5,L3=1.0,H3=2.5,L4=1.0,H4=2.5,L5=1.0,H5=2.5
	// Check Device ID
	if (GetQueryValue("ID", StrPacket) == DeviceID)
	{
		// Command Check
		if (GetQueryValue("Cmd", StrPacket) == "S")
		{
			double ntemp = 0;
			ntemp = GetQueryValue("L0", StrPacket).toFloat();
			SetPointLow[0] = ntemp;

			ntemp = GetQueryValue("H0", StrPacket).toFloat();
			SetPointHigh[0] = ntemp;

			if ((SetPointLow[0] != 0 && SetPointHigh[0] != 0) && (SetPointLow[0] < SetPointHigh[0]))
				bSensorUsed[0] = true;
			else
				bSensorUsed[0] = false;

			ntemp = GetQueryValue("L1", StrPacket).toFloat();
			SetPointLow[1] = ntemp;

			ntemp = GetQueryValue("H1", StrPacket).toFloat();
			SetPointHigh[1] = ntemp;

			if ((SetPointLow[1] != 0 && SetPointHigh[1] != 0) && (SetPointLow[1] < SetPointHigh[1]))
				bSensorUsed[1] = true;
			else
				bSensorUsed[1] = false;

			ntemp = GetQueryValue("L2", StrPacket).toFloat();
			SetPointLow[2] = ntemp;

			ntemp = GetQueryValue("H2", StrPacket).toFloat();
			SetPointHigh[2] = ntemp;

			if ((SetPointLow[2] != 0 && SetPointHigh[2] != 0) && (SetPointLow[2] < SetPointHigh[2]))
				bSensorUsed[2] = true;
			else
				bSensorUsed[2] = false;

			ntemp = GetQueryValue("L3", StrPacket).toFloat();
			SetPointLow[3] = ntemp;

			ntemp = GetQueryValue("H3", StrPacket).toFloat();
			SetPointHigh[3] = ntemp;

			if ((SetPointLow[3] != 0 && SetPointHigh[3] != 0) && (SetPointLow[3] < SetPointHigh[3]))
				bSensorUsed[3] = true;
			else
				bSensorUsed[3] = false;

			ntemp = GetQueryValue("L4", StrPacket).toFloat();
			SetPointLow[4] = ntemp;

			ntemp = GetQueryValue("H4", StrPacket).toFloat();
			SetPointHigh[4] = ntemp;

			if ((SetPointLow[4] != 0 && SetPointHigh[4] != 0) && (SetPointLow[4] < SetPointHigh[4]))
				bSensorUsed[4] = true;
			else
				bSensorUsed[4] = false;

			ntemp = GetQueryValue("L5", StrPacket).toFloat();
			SetPointLow[5] = ntemp;

			ntemp = GetQueryValue("H5", StrPacket).toFloat();
			SetPointHigh[5] = ntemp;

			if ((SetPointLow[5] != 0 && SetPointHigh[5] != 0) && (SetPointLow[5] < SetPointHigh[5]))
				bSensorUsed[5] = true;
			else
				bSensorUsed[5] = false;

			Serial.println("I have modified setting values!");

			for (uint8_t idx2 = 0; idx2 < 6; idx2++)
			{
				Serial.print("ADC Channel");
				Serial.print(idx2, DEC);
				if (bSensorUsed[idx2])
					Serial.print(" Enabled. ");
				else
					Serial.print(" Diabled. ");
				Serial.print("SetPintLow");
				Serial.print(" is ");
				Serial.print(SetPointLow[idx2]);
				Serial.print(", SetPintHigh");
				Serial.print(String(idx2));
				Serial.print(" is ");
				Serial.println(SetPointHigh[idx2]);
			}
			SendToMqtt("I have received vaild setting values!");
			SaveSensorSetting();
			bWaitSetting = false;
		}
		else if (GetQueryValue("Cmd", StrPacket) == "D")
		{ // 
		  // Command format is ID=Feather00001,Cmd=D,CH0=1,CH1=1,CH2=1,CH3=1,CH4=1
			uint16_t ntemp = 0;
			ntemp = GetQueryValue("CH0", StrPacket).toInt();
			if (ntemp != bDigitalUsed[0])
			{
				bDigitalUsed[0] = ntemp;
				if (bDigitalUsed[0])// if it is enabled by command
					bPreD5 = DI_IDLE;
			}

			ntemp = GetQueryValue("CH1", StrPacket).toInt();
			if (ntemp != bDigitalUsed[1])
			{
				bDigitalUsed[1] = ntemp;
				if (bDigitalUsed[1])// if it is enabled by command
					bPreD9 = DI_IDLE;
			}

			ntemp = GetQueryValue("CH2", StrPacket).toInt();
			if (ntemp != bDigitalUsed[2])
			{
				bDigitalUsed[3] = ntemp;
				if (bDigitalUsed[2])// if it is enabled by command
					bPreD11 = DI_IDLE;
			}

			ntemp = GetQueryValue("CH3", StrPacket).toInt();
			if (ntemp != bDigitalUsed[3])
			{
				bDigitalUsed[4] = ntemp;
				if (bDigitalUsed[3])// if it is enabled by command
					bPreD12 = DI_IDLE;
			}

			ntemp = GetQueryValue("CH5", StrPacket).toInt();
			if (ntemp != bDigitalUsed[4])
			{
				bDigitalUsed[5] = ntemp;
				if (bDigitalUsed[4])// if it is enabled by command
					bPreD13 = DI_IDLE;
			}
			SaveSensorSetting();
			SendToMqtt("I have received Digital input setting message!");
		}
		else
		{ //  if needs, it can be send response
		}
	}
}

void WifiSetup(Request &req, Response &res)
{
	char QueryName[10];
	char QueryValue[50];
	req.postParam(QueryName, 10, QueryValue, 50);
	Serial.print("Selecetd SSID is ");
	Serial.println(QueryValue);
	Strssid = String(QueryValue);

	req.postParam(QueryName, 10, QueryValue, 50);
	Serial.print("Selecetd Password is ");
	Serial.println(QueryValue);
	Strpass = String(QueryValue);

	String tempstr;
	req.postParam(QueryName, 10, QueryValue, 50);
	Serial.print("IP Address Setting is ");
	IPAddrSetting = String(QueryValue);

	Serial.println(IPAddrSetting);

	req.postParam(QueryName, 10, QueryValue, 50);
	Serial.print("Subnet Address Setting is ");
	SubnetSetting = String(QueryValue);
	Serial.println(SubnetSetting);

	req.postParam(QueryName, 10, QueryValue, 50);
	Serial.print("Default Gateway Setting is ");
	GateWaySetting = String(QueryValue);

	Serial.println(GateWaySetting);

	req.postParam(QueryName, 10, QueryValue, 50);
	Serial.print("DNS Server Setting is ");
	DnsSetting = String(QueryValue);
	Serial.println(DnsSetting);

	if (req.postParam(QueryName, 10, QueryValue, 50))
	{// Manual
		Serial.println("Manual IP Setting");
	}
	else
	{
		IPAddrSetting = "";
		SubnetSetting = "";
		GateWaySetting = "";
		DnsSetting = "";
	}

	res.success("text/html");
	res.printP("<!DOCTYPE html>\
    <html lang='en'>\
      <head>\
      <meta charset='utf-8'>\
      <meta name='viewport' content='width=device-width, initial-scale=1'>\
      <title>Feather M0 WIFI Web Server</title>");
	if (!stateMachine.isInState(APState))
		res.printP("<link rel = 'stylesheet' href = 'http://netdna.bootstrapcdn.com/bootstrap/3.1.1/css/bootstrap.min.css'>");
	res.printP("<style>body{padding-top:25px;padding-bottom:20px}.header{border-bottom:1px solid #e5e5e5;margin-bottom:0;color:#D4AC0D}\
        .jumbotron{text-align:center}.marketing{margin:40px 0}\
        .arduino h4{font-size:20px;color:#27AE60;margin-top:5px;padding-right:10px;padding-left:0; display:inline-block;}\
        .arduino h5{font-size:20px;color:#F1C40F;margin-top:5px;padding-right:0;padding-left:0px; display:inline-block;}\
        .arduino h6{font-size:16px;color:#27AE60;margin-top:5px;padding-right:0;padding-left:0px; display:inline-block;}\
        .clear{ clear:both;}\
        .align-center {text-align:center;}\
      </style>\
      </head>\
      <body style='background-color:#EBDEF0'>\
      <div class='container align-center'>\
        <div class='header'>\
        <h1>Welcome to Feather Sensor Setting</h1>\
        </div>\
        <div class='row arduino'>\
        <div class='col-lg-12'>\
          <div class='clear'></div>\
          <br>\
            <div class='clear'></div>\
            <h5>WiFi Setting is changed!</h5>\
            <div class='clear'></div>\
            <h4>Ad hoc SSID is </h4><h5>");
	res.print(Strssid);
	res.printP(".</h5><br / ><br / ><br / >\
            <h4>IP Address</h4><h5>");
	res.print(IPAddrSetting);
	res.printP("</h5><br / >\
            <h4>Subnet Mask</h4><h5>");
	res.print(SubnetSetting);
	res.printP("</h5><br / >\
            <h4>Default Gateway</h4><h5>");
	res.print(GateWaySetting);
	res.printP("</h5><br / >\
            <h4>DNS Server</h4><h5>");
	res.print(DnsSetting);
	res.printP("</h5><br / >\
            <div class='clear'></div>\
            <br><br>\
            <h6> Copyright Aurora Inn 2017 </h6>\
        </div>\
        </div>\
      </div>\
      </body></html>");

	SaveNetworkSetting();

	stateMachine.transitionTo(Connecting);
}

void MqttSetup(Request &req, Response &res)
{
	char QueryName[10];
	char QueryValue[50];
	req.postParam(QueryName, 10, QueryValue, 50);
	Serial.print("Mqtt Server URL is ");
	Serial.println(QueryValue);
	StrMqttUrl = String(QueryValue);
	req.postParam(QueryName, 10, QueryValue, 50);
	Serial.print("Mqtt Port is ");
	Serial.println(QueryValue);
	StrMqttPort = String(QueryValue);
	MqttConnect();

	res.success("text/html");
	res.printP("<!DOCTYPE html>\
      <html lang='en'>\
        <head>\
        <meta charset='utf-8'>\
        <meta name='viewport' content='width=device-width, initial-scale=1'>\
        <title>Feather M0 WIFI Web Server</title>\
        <link rel='stylesheet' href='http://netdna.bootstrapcdn.com/bootstrap/3.1.1/css/bootstrap.min.css'>\
        <style>body{padding-top:25px;padding-bottom:20px}.header{border-bottom:1px solid #e5e5e5;margin-bottom:0;color:#D4AC0D}\
          .jumbotron{text-align:center}.marketing{margin:40px 0}\
          .arduino h4{font-size:20px;color:#27AE60;margin-top:10px;padding-right:10px;padding-left:0; display:inline-block;}\
          .arduino h5{font-size:20px;color:#F1C40F;margin-top:10px;padding-right:0;padding-left:0px; display:inline-block;}\
          .arduino h6{font-size:16px;color:#27AE60;margin-top:10px;padding-right:0;padding-left:0px; display:inline-block;}\
          .clear{ clear:both;}\
          .align-center {text-align:center;}\
        </style>\
        </head>\
        <body style='background-color:#EBDEF0'>\
        <div class='container align-center'>\
          <div class='header'>\
          <h1>Welcome to Feather Sensor Setting</h1>\
          </div>\
          <div class='row arduino'>\
          <div class='col-lg-12'>\
            <div class='clear'></div>\
            <br>\
            <br>\
            <div class='clear'></div>\
            <h4>STA Info</h4><h6><a href='/WiFiSetting'>Change Setting</a></h6>\
            <br />\
            <h4>SSID:</h4>\
            <h5>");
	String httpContent = Strssid;
	res.print(httpContent);

	res.printP("</h5>&nbsp;&nbsp;&nbsp;\
            <h4>Local IP address:</h4>\
            <h5>");
	httpContent = IPtoString(WiFi.localIP());
	res.print(httpContent);
	res.printP("</h5>\
            <div class='clear'></div>\
            <br>\
            <form action='/MQTT' method='POST'>\
              <div class='container'>\
                <div class='row arduino'>\
                  <div class='col-lg-12'>\
                    <div class='header'>\
                    </div>\
                    <div class='header'></div>\
                    <div class='clear'></div>\
                    <h4>MQTT Setting</h4>\
                    <div class='clear'></div>\
                    <h4>Server:</h6>&nbsp\
                    <h5><input type='text' name='server' value='");
	res.print(StrMqttUrl);
	res.printP("' maxlength='32' size='15'></h5>\
                    &nbsp&nbsp&nbsp&nbsp\
                    <h4>Port:</h4>&nbsp\
                    <h5><input type='text' name='Port' value='");
	res.print(StrMqttPort);
	res.printP("' maxlength='4' size='1'></h5>\
                    <h4><input type='submit' value='Connect'></h4><br />\
                    <h4>Status:</h4><h5>");
	if (client.connected())
		res.print("Connected");
	else
		res.print("Disconnected");

	res.printP("</h5>\
                  </div>\
                </div>\
              </div>\
            </form>\
            <br>\
            <form action='/Time' method='POST'>\
              <div class='container'>\
                <div class='row arduino'>\
                  <div class='col-lg-12'>\
                    <div class='header'></div>\
                    <h4>Datetime Setting</h4>\
                    <div class='clear'></div>\
                    <h4>Current DateTime:</h4>\
                    <h5>\
                      <input type='datetime-local' name='currtime' value='");
	res.print(RTCtoString());
	res.printP("' step='1'>\
                    </h5>\
                    <h4><input type='submit' value='Set'></h4>\
                  </div>\
                </div>\
              </div>\
            </form>\
            <div class='clear'></div>\
            <br><br>\
              <h6> Copyright Aurora Inn 2017 </h6>\
          </div>\
          </div>\
        </div>\
        </body>\
      </html>");

	SaveNetworkSetting();
}

void TimeProc(Request &req, Response &res)
{
	char VarName[10];
	char date[20];
	bool bres = req.postParam(VarName, 10, date, 20);
	String strDate = String(date);

	SettimeFromString(strDate);
	res.success("text/html");
	res.printP("<!DOCTYPE html>\
  <html lang='en'>\
    <head>\
    <meta charset='utf-8'>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>Feather M0 WIFI Web Server</title>\
    <link rel='stylesheet' href='http://netdna.bootstrapcdn.com/bootstrap/3.1.1/css/bootstrap.min.css'>\
    <style>body{padding-top:25px;padding-bottom:20px}.header{border-bottom:1px solid #e5e5e5;margin-bottom:0;color:#D4AC0D}\
      .jumbotron{text-align:center}.marketing{margin:40px 0}\
      .arduino h4{font-size:20px;color:#27AE60;margin-top:10px;padding-right:10px;padding-left:0; display:inline-block;}\
      .arduino h5{font-size:20px;color:#F1C40F;margin-top:10px;padding-right:0;padding-left:0px; display:inline-block;}\
      .arduino h6{font-size:16px;color:#27AE60;margin-top:10px;padding-right:0;padding-left:0px; display:inline-block;}\
      .clear{ clear:both;}\
      .align-center {text-align:center;}\
    </style>\
    </head>\
    <body style='background-color:#EBDEF0'>\
    <div class='container align-center'>\
      <div class='header'>\
      <h1>Welcome to Feather Sensor Setting</h1>\
      </div>\
      <div class='row arduino'>\
      <div class='col-lg-12'>\
        <div class='clear'></div>\
        <br>\
        <br>\
        <div class='clear'></div>\
        <h4>STA Info</h4><h6><a href='/WiFiSetting'>Change Setting</a></h6>\
        <br />\
        <h4>SSID:</h4>\
        <h5>");
	String httpContent = Strssid;
	res.print(httpContent);

	res.printP("</h5>&nbsp;&nbsp;&nbsp;\
        <h4>Local IP address:</h4>\
        <h5>");
	httpContent = IPtoString(WiFi.localIP());
	res.print(httpContent);
	res.printP("</h5>\
        <div class='clear'></div>\
        <br>\
        <form action='/MQTT' method='POST'>\
          <div class='container'>\
            <div class='row arduino'>\
              <div class='col-lg-12'>\
                <div class='header'>\
                </div>\
                <div class='header'></div>\
                <div class='clear'></div>\
                <h4>MQTT Setting</h4>\
                <div class='clear'></div>\
                <h4>Server:</h6>&nbsp\
                <h5><input type='text' name='server' value='");
	res.print(StrMqttUrl);
	res.printP("' maxlength='15' size='10'></h5>\
                &nbsp&nbsp&nbsp&nbsp\
                <h4>Port:</h4>&nbsp\
                <h5><input type='text' name='Port' value='");
	res.print(StrMqttPort);
	res.printP("' maxlength='4' size='1'></h5>\
                <h4><input type='submit' value='Connect'></h4><br />\
                <h4>Status:</h4><h5>Connected</h5>\
              </div>\
            </div>\
          </div>\
        </form>\
        <br>\
        <form action='/Time' method='POST'>\
          <div class='container'>\
            <div class='row arduino'>\
              <div class='col-lg-12'>\
                <div class='header'></div>\
                <h4>Datetime Setting</h4>\
                <div class='clear'></div>\
                <h4>Current DateTime:</h4>\
                <h5>\
                  <input type='datetime-local' name='currtime' value='");
	res.print(RTCtoString());
	res.printP("' step='1'>\
                </h5>\
                <h4><input type='submit' value='Set'></h4>\
              </div>\
            </div>\
          </div>\
        </form>\
        <div class='clear'></div>\
        <br><br>\
          <h6> Copyright Aurora Inn 2017 </h6>\
      </div>\
      </div>\
    </div>\
    </body>\
  </html>");
}

// define a handler function
void indexCmd(Request &req, Response &res) {
	if (stateMachine.isInState(PreAPState))
		stateMachine.immediateTransitionTo(APState);

	if (stateMachine.isInState(APState))
	{
		res.success("text/html");
		res.printP("<!DOCTYPE html>\
      <html lang='en'>\
        <head>\
        <meta charset='utf-8'>\
        <meta name='viewport' content='width=device-width, initial-scale=1'>\
        <title>Feather M0 WIFI Web Server</title>\
        <style>body{padding-top:25px;padding-bottom:20px}.header{border-bottom:1px solid #e5e5e5;margin-bottom:0;color:#D4AC0D}\
          .jumbotron{text-align:center}.marketing{margin:40px 0}\
          .arduino h4{font-size:20px;color:#27AE60;margin-top:10px;padding-right:10px;padding-left:0; display:inline-block;}\
          .arduino h5{font-size:20px;color:#F1C40F;margin-top:10px;padding-right:0;padding-left:0px; display:inline-block;}\
          .arduino h6{font-size:16px;color:#27AE60;margin-top:10px;padding-right:0;padding-left:0px; display:inline-block;}\
          .clear{ clear:both;}\
          .align-center {text-align:center;}\
        </style>\
        </head>\
        <body style='background-color:#EBDEF0'>\
        <div class='container align-center'>\
          <div class='header'>\
          <h1>Welcome to Feather Sensor Setting</h1>\
          </div>\
          <div class='row arduino'>\
          <div class='col-lg-12'>\
            <div class='clear'></div>\
            <br>\
            <br>\
            <div class='clear'></div>\
            <h4>STA Info</h4><h6><a href='/WiFiSetting'>Change Setting</a></h6>\
            <br />\
            <h4>SSID:</h4>\
            <h5>");
		String httpContent = Strssid;
		res.print(httpContent);

		res.printP("</h5>&nbsp;&nbsp;&nbsp;\
            <h4>Local IP address:</h4>\
            <h5>");
		httpContent = IPtoString(WiFi.localIP());
		res.print(httpContent);
		res.printP("</h5>\
            <br><br>\
            <div class='clear'></div>\
            <br><br>\
              <h6> Copyright Aurora Inn 2017 </h6>\
          </div>\
          </div>\
        </div>\
        </body>\
      </html>");
	}
	else
	{
		res.success("text/html");
		res.printP("<!DOCTYPE html>\
      <html lang='en'>\
        <head>\
        <meta charset='utf-8'>\
        <meta name='viewport' content='width=device-width, initial-scale=1'>\
        <title>Feather M0 WIFI Web Server</title>\
        <link rel='stylesheet' href='http://netdna.bootstrapcdn.com/bootstrap/3.1.1/css/bootstrap.min.css'>\
        <style>body{padding-top:25px;padding-bottom:20px}.header{border-bottom:1px solid #e5e5e5;margin-bottom:0;color:#D4AC0D}\
          .jumbotron{text-align:center}.marketing{margin:40px 0}\
          .arduino h4{font-size:20px;color:#27AE60;margin-top:10px;padding-right:10px;padding-left:0; display:inline-block;}\
          .arduino h5{font-size:20px;color:#F1C40F;margin-top:10px;padding-right:0;padding-left:0px; display:inline-block;}\
          .arduino h6{font-size:16px;color:#27AE60;margin-top:10px;padding-right:0;padding-left:0px; display:inline-block;}\
          .clear{ clear:both;}\
          .align-center {text-align:center;}\
        </style>\
        </head>\
        <body style='background-color:#EBDEF0'>\
        <div class='container align-center'>\
          <div class='header'>\
          <h1>Welcome to Feather Sensor Setting</h1>\
          </div>\
          <div class='row arduino'>\
          <div class='col-lg-12'>\
            <div class='clear'></div>\
            <br>\
            <br>\
            <div class='clear'></div>\
            <h4>STA Info</h4><h6><a href='/WiFiSetting'>Change Setting</a></h6>\
            <br />\
            <h4>SSID:</h4>\
            <h5>");
		String httpContent = Strssid;
		res.print(httpContent);

		res.printP("</h5>&nbsp;&nbsp;&nbsp;\
            <h4>Local IP address:</h4>\
            <h5>");
		httpContent = IPtoString(WiFi.localIP());
		res.print(httpContent);
		res.printP("</h5>\
            <div class='clear'></div>\
            <br>\
            <form action='/MQTT' method='POST'>\
              <div class='container'>\
                <div class='row arduino'>\
                  <div class='col-lg-12'>\
                    <div class='header'>\
                    </div>\
                    <div class='header'></div>\
                    <div class='clear'></div>\
                    <h4>MQTT Setting</h4>\
                    <div class='clear'></div>\
                    <h4>Server:</h6>&nbsp\
                    <h5><input type='text' name='server' value='");
		res.print(StrMqttUrl);
		res.printP("' maxlength='32' size='15'></h5>\
                    &nbsp&nbsp&nbsp&nbsp\
                    <h4>Port:</h4>&nbsp\
                    <h5><input type='text' name='Port' value='");
		res.print(StrMqttPort);
		res.printP("' maxlength='4' size='1'></h5>\
                    <h4><input type='submit' value='Connect'></h4><br />\
                    <h4>Status:</h4><h5>");
		if (client.connected())
			res.print("Connected");
		else
			res.print("Disconnected");

		res.printP("</h5>\
                  </div>\
                </div>\
              </div>\
            </form>\
            <br>\
            <form action='/Time' method='POST'>\
              <div class='container'>\
                <div class='row arduino'>\
                  <div class='col-lg-12'>\
                    <div class='header'></div>\
                    <h4>Datetime Setting</h4>\
                    <div class='clear'></div>\
                    <h4>Current DateTime:</h4>\
                    <h5>\
                      <input type='datetime-local' name='currtime' value='");
		res.print(RTCtoString());
		Serial.println(RTCtoString());
		res.printP("' step='1'>\
                    </h5>\
                    <h4><input type='submit' value='Set'></h4>\
                  </div>\
                </div>\
              </div>\
            </form>\
            <div class='clear'></div>\
            <br><br>\
              <h6> Copyright Aurora Inn 2017 </h6>\
          </div>\
          </div>\
        </div>\
        </body>\
      </html>");
	}
}

void SettimeFromString(String str)
{ // 2000-01-01T00%3A00%3A00
	Serial.println(str);

	uint16_t nYear = str.substring(0, 4).toInt();
	uint8_t nMonth = str.substring(5, 7).toInt();
	uint8_t nDay = str.substring(8, 10).toInt();
	uint8_t nHour = str.substring(11, 13).toInt();
	uint8_t nMinute = str.substring(14, 16).toInt();
	uint8_t nSecond = str.substring(17, 19).toInt();
	rtc.adjust(DateTime(nYear, nMonth, nDay, nHour, nMinute, nSecond));
}

void WiFiSetting(Request &req, Response &res) {
	if (!stateMachine.isInState(APState))
	{
		listNetworks();
	}

	char json[256];
	AvailableSSIDs.toCharArray(json, AvailableSSIDs.length() + 1);
	StaticJsonBuffer<1024> jsonBuffer;
	JsonArray& root = jsonBuffer.parseArray(json);
	int numberOfElements = root.size();

	res.success("text/html");
	res.printP("<!DOCTYPE html>\
    <html lang='en'>\
      <head>\
      <meta charset='utf-8'>\
      <meta name='viewport' content='width=device-width, initial-scale=1'>\
      <title>Feather M0 WIFI Web Server</title>");
	if (!stateMachine.isInState(APState))
		res.printP("<link rel = 'stylesheet' href = 'http://netdna.bootstrapcdn.com/bootstrap/3.1.1/css/bootstrap.min.css'>");
	res.printP("<style>body{padding-top:25px;padding-bottom:20px}.header{border-bottom:1px solid #e5e5e5;margin-bottom:0;color:#D4AC0D}\
        .jumbotron{text-align:center}.marketing{margin:40px 0}\
        .arduino h4{font-size:20px;color:#27AE60;margin-top:5px;padding-right:10px;padding-left:0; display:inline-block;}\
        .arduino h5{font-size:20px;color:#F1C40F;margin-top:5px;padding-right:0;padding-left:0px; display:inline-block;}\
        .arduino h6{font-size:16px;color:#27AE60;margin-top:5px;padding-right:0;padding-left:0px; display:inline-block;}\
        .clear{ clear:both;}\
        .align-center {text-align:center;}\
      </style>\
      </head>\
      <body style='background-color:#EBDEF0'>\
      <script>\
      function EnableControls() {\
        if(document.getElementById('manual').checked == true)\
        {\
          document.getElementById('ipaddr').removeAttribute('disabled');\
          document.getElementById('subnet').removeAttribute('disabled');\
          document.getElementById('gateway').removeAttribute('disabled');\
          document.getElementById('dns').removeAttribute('disabled');\
        }\
        else\
        {\
          document.getElementById('ipaddr').setAttribute('disabled', true);\
          document.getElementById('subnet').setAttribute('disabled', true);\
          document.getElementById('gateway').setAttribute('disabled', true);\
          document.getElementById('dns').setAttribute('disabled', true);\
        }\
      }\
      </script><div class='container align-center'>\
        <div class='header'>\
        <h1>Welcome to Feather Sensor Setting</h1>\
        </div>\
        <div class='row arduino'>\
        <div class='col-lg-12'>\
          <div class='clear'></div>\
          <br>\
        <form action='/Wifi' method='POST'>\
          <div class='container'>\
            <div class='row arduino'>\
              <div class='col-lg-12'>\
                <div class='clear'></div>\
                <h4>Select WiFi AP which you connect</h4>\
                <div class='clear'></div>\
                <h4>Select SSID:</h4>\
                <h5><select name='SSID'>");
	for (uint8_t idx = 0; idx < numberOfElements; idx++)
	{
		JsonObject& ItemObj = root[idx];
		res.print("<option value='");
		const char *ssid = ItemObj["ssid"];
		res.printP(ssid);
		res.printP("'>");
		res.printP(ssid);
		res.print("</option>");
	}
	res.printP("</select>\
                </h5>&nbsp&nbsp&nbsp&nbsp\
                <h6>Password:<input type='password' name='PASS' maxlength='15' size='10'></h6>\
                <h6><input type='submit' value='Apply'></h6>\
              </div>\
            </div>\
          </div>\
          <div class='clear'></div>\
          <div class='header'>\
          </div>\
          <br />\
          <h4>Available AP List</h4>\
          <br>\
          <div class='container'>\
            <div class='row arduino'>\
              <div class='col-lg-12'>\
                <div class='clear'></div>\
                <br />");
	for (uint8_t idx = 0; idx < numberOfElements; idx++)
	{
		const char *ssid = root[idx]["ssid"];
		const char *enctype = root[idx]["enctype"];
		const char *signal = root[idx]["Signal"];
		const char *MAC = root[idx]["bssid"];
		res.printP("<h4>SSID:</h4><h5>");
		res.printP(ssid);
		res.printP("</h5>&nbsp&nbsp&nbsp&nbsp");

		res.printP("<h4>Encryption type:</h4><h5>");
		res.printP(enctype);
		res.printP("</h5>&nbsp&nbsp&nbsp&nbsp");

		res.printP("<h4>Signal Strength:</h4><h5>");
		res.printP(signal);
		res.printP("dB</h5>&nbsp&nbsp&nbsp&nbsp");

		res.printP("<h4>Mac Addr:</h4><h5>");
		res.printP(MAC);
		res.printP("</h5>&nbsp&nbsp&nbsp&nbsp<br/><br/>");
	}
	res.printP("<h4>Manual Static IP Address Settings</h4>\
                <div class='clear'></div>");
	if (IPAddrSetting.length() > 0)
	{
		res.printP("<h6 style='width:140px;'>IP address:</h6><h5><input type='text' name='ipaddr' maxlength='15' size='10' value='");
		res.print(IPAddrSetting);
		res.printP("'></h5><br />\
                <h6 style='width:140px;'>Subnet mask:</h6><h5><input type='text' name='subnet' maxlength='15' size='10' value='");
		res.print(SubnetSetting);
		res.printP("'></h5><br />\
                <h6 style='width:140px;'>Default gateway:</h6><h5><input type='text' name='gateway' maxlength='15' size='10' value='");
		res.print(GateWaySetting);
		res.printP("'></h5><br />\
                <h6 style='width:140px;'>DNS Server:</h6><h5><input type='text' name='dnsserver' maxlength='15' size='10' value='");
		res.print(DnsSetting);
		res.printP("'></h5><br />");
	}
	else
	{
		res.printP("<h6 style='width:140px;'>IP address:</h6><h5><input id ='ipaddr' type='text' name='ipaddr' maxlength='15' size='10' disabled='true'></h5><br />\
          <h6 style='width:140px;'>Subnet mask:</h6><h5><input id ='subnet' type='text' name='subnet' maxlength='15' size='10' disabled='true'></h5><br />\
          <h6 style='width:140px;'>Default gateway:</h6><h5><input id ='gateway' type='text' name='gateway' maxlength='15' size='10' disabled='true'></h5><br />\
          <h6 style='width:140px;'>DNS Server:</h6><h5><input id ='dns' type='text' name='dnsserver' maxlength='15' size='10' disabled='true'></h5><br />");
	}

	res.printP("<h6><input id='manual' type='checkbox' name='manual' value='manual' onchange='EnableControls()'");
	if (IPAddrSetting.length() > 0)
		res.printP("checked = 'checked'");
	res.printP("> Manual Settings<br></h6>\
              </div>\
            </div>\
          </div>\
        </form>\
        <div class='clear'></div>\
          <div class='header'>\
          </div>\
          <br />");
	res.printP("<div class='clear'></div>\
          <br><br>\
            <h6> Copyright Aurora Inn 2017 </h6>\
        </div>\
        </div>\
      </div>\
      </body></html>");

}

/**/
void InitSoftAP()
{
	WiFi.disconnect();
	uint8_t mac[6];
	WiFi.macAddress(mac);
	String strtmp = F("MiRemote_");
	for (uint8_t idx = 0; idx < 6; idx++)
	{
		if (mac[5 - idx] < 16)
			strtmp += "0";
		strtmp += String(mac[5 - idx], HEX);
	}

	strtmp.toUpperCase();

	strtmp.toCharArray(AP_SSID, strtmp.length() + 1);

	// Create open network. We can change this line in order to add WEP
	status = WiFi.beginAP(AP_SSID);
	if (status != WL_AP_LISTENING) {
		Serial.println("Creating access point failed");
		// don't continue
		while (true);
	}

	Serial.println(F("Soft AP is created!!!"));
}

/**/
void setup()
{
	InitPorts();
	initFBD();
	InitVars();

	Serial.begin(9600);
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB port only
	}

	// Configure Wifi101
	WiFi.setPins(8, 7, 4, 2);
	// check for the presence of the shield:
	if (WiFi.status() == WL_NO_SHIELD) {
		Serial.println("WiFi shield not present");
		// don't continue:
		while (true);
	}
	WiFi.lowPowerMode();

	if (!rtc.begin()) {
		Serial.println("Couldn't find RTC");
		while (1);
	}

	Serial.println("Initializing SD card...");
	if (!SD.begin(10))
	{
		Serial.println("initialization failed!");
		return;
	}

	if (!LoadSensorSetting())
		InitSensorSetting();

	if (!LoadNetworkSetting())
		InitNetworkSetting();

	stateMachine.transitionTo(PreAPState);

}

/**/
void InitWebServer()
{
	// start the web server on port 80
	server = new WiFiServer(80);

	server->begin();

	// mount the handler to the default router
	app.get("/", &indexCmd);
	app.get("/WiFiSetting", &WiFiSetting);
	app.post("/Time", &TimeProc);
	app.post("/MQTT", &MqttSetup);
	app.post("/Wifi", &WifiSetup);
}

/*
*/
void printWiFiStatus() {
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print your WiFi shield's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.print(rssi);
	Serial.println(" dBm");
	// print where to go in a browser:
	Serial.print("To see this page in action, open a browser to http://");
	Serial.println(ip);
}

void loop()
{
	WiFiClient Webclient = server->available();
	if (Webclient) {
		app.process(&Webclient);
	}

	if (stateMachine.isInState(PreAPState))
	{
		if (stateMachine.timeInCurrentState() > nPreAPTime)
			stateMachine.transitionTo(Connecting);
	}

	for (uint8_t idx = 0; idx < 6; idx++)
	{
		uint16_t nADC = analogRead(ADCPins[idx]);
		double fAdc = (double)map(nADC, 0, 1024, 0, 330) / (double)100.0;
		if (fAdc <= SetPointLow[idx])
			SensorCurrStatus[idx] = STATUS_OFF;
		else if (fAdc > SetPointLow[idx] && fAdc < SetPointHigh[idx])
			SensorCurrStatus[idx] = STATUS_IDLE;
		else if (fAdc >= SetPointHigh[idx])
			SensorCurrStatus[idx] = STATUS_ON;
		if (SensorCurrStatus[idx] != SensorLastStatus[idx])
			nLastTime[idx] = millis();
		SensorLastStatus[idx] = SensorCurrStatus[idx];
		if ((millis() - nLastTime[idx]) > DEBOUNCE)
		{
			if (SensorCurrStatus[idx] != SensorRealStatus[idx])
			{
				SensorRealStatus[idx] = SensorCurrStatus[idx];
				String msg = "ADC";
				msg += String(idx);
				if (SensorRealStatus[idx] == STATUS_ON)
				{ // On event triggered.
					msg += " ON!, ";
				}
				else if (SensorRealStatus[idx] == STATUS_OFF)
				{ // Off event triggered.
					msg += " OFF!, ";
				}
				msg += String(fAdc, 2);
				msg += "V.";
				if (bSensorUsed[idx])
				{
					SendToMqtt(msg);
					Serial.println(msg);
				}

			}
		}
	}

	// Digital Input
	if (bPreD5 != digitalRead(D5))
	{
		String msg = "";
		bPreD5 = digitalRead(D5);
		if (bPreD5 == HIGH)
			msg = "D5 changed to High!";
		else
			msg = "D5 changed to Low!";

		if (bDigitalUsed[0]) // When its setting is true, it will send mqtt message
			SendToMqtt(msg);
	}

	if (bPreD9 != digitalRead(D9))
	{
		bPreD9 = digitalRead(D9);
		String msg = "";
		if (bPreD9 == HIGH)
			msg = "D9 changed to High!";
		else
			msg = "D9 changed to Low!";
		if (bDigitalUsed[1]) // When its setting is true, it will send mqtt message
			SendToMqtt(msg);
	}

	if (bPreD11 != digitalRead(D11))
	{
		String msg = "";
		bPreD11 = digitalRead(D11);
		if (bPreD11 == HIGH)
			msg = "D11 changed to High!";
		else
			msg = "D11 changed to Low!";
		if (bDigitalUsed[2]) // When its setting is true, it will send mqtt message
			SendToMqtt(msg);
	}

	if (bPreD12 != digitalRead(D12))
	{
		String msg = "";
		bPreD12 = digitalRead(D12);
		if (bPreD12 == HIGH)
			msg = "D12 changed to High!";
		else
			msg = "D12 changed to Low!";
		if (bDigitalUsed[3]) // When its setting is true, it will send mqtt message
			SendToMqtt(msg);
	}
	if (bPreD13 != digitalRead(D13))
	{
		String msg;
		bPreD13 = digitalRead(D13);
		if (bPreD13 == HIGH)
			msg = "D13 changed to High!";
		else
			msg = "D13 changed to Low!";
		if (bDigitalUsed[4]) // When its setting is true, it will send mqtt message
			SendToMqtt(msg);

	}

	// FSM management
	stateMachine.update();

	// Mqtt Proc
	if (client.connected())
		client.loop();


	// Battery Sensing Part
	float BattVolt = BattVoltage();

	BattHighTON.IN = BattVolt > 4.0F;
	TONFunc(&BattHighTON);
	BattHighTrg.IN = BattHighTON.Q;
	RTrgFunc(&BattHighTrg);
	if (BattHighTrg.Q)
	{
		Serial.println(F("Battery High Trigeered! I have turned off Charger."));
	}

	BattLowTON.IN = BattVolt < 3.2F;
	TONFunc(&BattLowTON);
	BattLowTrg.IN = BattLowTON.Q;
	RTrgFunc(&BattLowTrg);
	if (BattLowTrg.Q)
	{
		Serial.println(F("Battery Low Trigeered! I have turned on Charger."));

	}
}

void listNetworks() {
	// scan for nearby networks:
	Serial.println("** Scan Networks **");
	int numSsid = WiFi.scanNetworks();
	if (numSsid == -1)
	{
		Serial.println("Couldn't get a WiFi connection");
		while (true);
	}

	// print the list of networks seen:
	Serial.print("number of available networks: ");
	Serial.println(numSsid);

	StaticJsonBuffer<1024> jsonBuffer;
	AvailableSSIDs = "";

	JsonArray& rootArray = jsonBuffer.createArray();
	for (uint8_t idx = 0; idx < numSsid; idx++)
	{
		String strtemp;
		JsonObject& Item = rootArray.createNestedObject();
		Item["ssid"] = String(WiFi.SSID(idx));
		Item["enctype"] = GetEncryptionType(WiFi.encryptionType(idx));
		Item["Signal"] = WiFi.RSSI(idx);
		byte bssid[6];
		Item["bssid"] = GetBSSID(WiFi.BSSID(idx, bssid));
	}
	rootArray.printTo(AvailableSSIDs);
	Serial.println(AvailableSSIDs);
}

String GetBSSID(byte bssid[]) {
	String result = "";
	if (bssid[5] < 16)
		result += "0";
	result += String(bssid[5], HEX);
	result += ":";

	if (bssid[4] < 16)
		result += "0";
	result += String(bssid[4], HEX);
	result += ":";

	if (bssid[3] < 16)
		result += "0";
	result += String(bssid[3], HEX);
	result += ":";

	if (bssid[2] < 16)
		result += "0";
	result += String(bssid[2], HEX);
	result += ":";

	if (bssid[1] < 16)
		result += "0";
	result += String(bssid[1], HEX);
	result += ":";

	if (bssid[0] < 16)
		result += "0";
	result += String(bssid[0], HEX);

	return result;
}

String GetEncryptionType(int thisType)
{
	String result = "";
	switch (thisType) {
	case ENC_TYPE_WEP:
		result = "WEP";
		break;
	case ENC_TYPE_TKIP:
		result = "WPA";
		break;
	case ENC_TYPE_CCMP:
		result = "WPA2";
		break;
	case ENC_TYPE_NONE:
		result = "None";
		break;
	case ENC_TYPE_AUTO:
		result = "Auto";
		break;
	}
	return result;
}

void printEncryptionType(int thisType) {
	// read the encryption type and print out the name:
	switch (thisType) {
	case ENC_TYPE_WEP:
		Serial.print("WEP");
		break;
	case ENC_TYPE_TKIP:
		Serial.print("WPA");
		break;
	case ENC_TYPE_CCMP:
		Serial.print("WPA2");
		break;
	case ENC_TYPE_NONE:
		Serial.print("None");
		break;
	case ENC_TYPE_AUTO:
		Serial.print("Auto");
		break;
	}
}

void print2Digits(byte thisByte) {
	if (thisByte < 0xF) {
		Serial.print("0");
	}
	Serial.print(thisByte, HEX);
}


void ConnectWIFI()
{
	WiFi.disconnect();
	delay(1000);

	if (IPAddrSetting == "")
	{
		WiFi.dhcp();
	}
	else
	{
		IPAddress localip, subnet, gateway, dnsserver;
		localip.fromString(IPAddrSetting);
		subnet.fromString(SubnetSetting);
		gateway.fromString(GateWaySetting);
		dnsserver.fromString(DnsSetting);
		WiFi.config(localip); // , subnet, gateway, dnsserver);
	}

	Serial.print("Attempting to connect to SSID: ");
	Serial.println(ssid);

	Strssid.toCharArray(ssid, Strssid.length() + 1);
	Strpass.toCharArray(pass, Strpass.length() + 1);
	// Connect to WPA/WPA2 network. Change this line if using open or WEP network:
	status = WiFi.begin(ssid, pass);
}

void ConnectEnter()
{
	//if (status != WL_CONNECTED)
	ConnectWIFI();
}

void ConnectingUpdate()
{
	if (WiFi.status() == WL_CONNECTED)
	{
		printWiFiStatus();
		stateMachine.transitionTo(STAState);
	}
	WifiConnTON.IN = (WifiConnTON.Q == 0) && WiFi.status() != WL_CONNECTED;
	TONFunc(&WifiConnTON);
	WifiConnTrg.IN = WifiConnTON.Q;
	RTrgFunc(&WifiConnTrg);
	if (WifiConnTrg.Q)
	{
		ConnectWIFI();
	}
}

void STAUpdate()
{

	if (WiFi.status() != WL_CONNECTED)
	{
		Serial.println("I have lost WIFI Connection!, so I will enter into Connecting Status!");
		stateMachine.transitionTo(Connecting);
	}

	MqttDisconTON.IN = (MqttDisconTON.Q == 0 && client.connected() == false);
	TONFunc(&MqttDisconTON);
	MqttDisconTrg.IN = MqttDisconTON.Q;
	RTrgFunc(&MqttDisconTrg);
	if (MqttDisconTrg.Q)
	{
		//Serial.println(stateMachine.timeInCurrentState());
		MqttConnect();
	}
}


String IPtoString(IPAddress address)
{
	return String(address[0]) + "." +
		String(address[1]) + "." +
		String(address[2]) + "." +
		String(address[3]);
}

String RTCtoString()
{
	DateTime now = rtc.now();
	String strTime = "";

	strTime += String(now.year(), DEC);
	strTime += "-";
	if (now.month() < 10)
		strTime += "0";
	strTime += String(now.month());
	strTime += "-";
	if (now.day() < 10)
		strTime += "0";
	strTime += String(now.day());
	strTime += "T";
	if (now.hour() < 10)
		strTime += "0";
	strTime += String(now.hour());
	strTime += ":";
	if (now.minute() < 10)
		strTime += "0";
	strTime += String(now.minute());
	strTime += ":";
	if (now.second() < 10)
		strTime += "0";
	strTime += String(now.second());
	//Serial.println(strTime);
	return strTime;
}

void STAEnter()
{
	MqttConnect();
}

void MqttConnect() {

	if (client.connected())
		client.disconnect();

	StrMqttUrl.toCharArray(mqttserver, StrMqttUrl.length() + 1);
	Serial.print("Mqtt Url is ");
	Serial.println(mqttserver);
	client.setServer(mqttserver, StrMqttPort.toInt());
	client.setCallback(callback);

	Serial.print("Attempting MQTT connection...");

	// Create a random client ID
	String clientId = "Feather-";
	clientId += String(random(0xffff), HEX);

	// Attempt to connect
	if (client.connect(clientId.c_str()))
	{ // Success
		Serial.println("connected");
		client.subscribe(mqtt_InputTopic);

		String msg = "I have connecetd.";
		SendToMqtt(msg);

		{
			StaticJsonBuffer<200> jsonBuffer;
			JsonObject& root = jsonBuffer.createObject();
			root["ssid"] = Strssid;
			root["pass"] = Strpass;
			root["ipaddr"] = IPAddrSetting;
			root["subnet"] = SubnetSetting;
			root["gateway"] = GateWaySetting;
			root["dns"] = DnsSetting;

			String strJSON;
			root.printTo(strJSON);
			strJSON = "Network Setting->" + strJSON;
			SendToMqtt(strJSON);
		}

		{
			StaticJsonBuffer<1024> jsonBuffer1;

			JsonObject& root1 = jsonBuffer1.createObject();
			JsonArray& DIOSetting = root1.createNestedArray("DIOSetting");
			for (uint8_t idx = 0; idx < 6; idx++)
				DIOSetting.add((uint8_t)bDigitalUsed[idx]);

			JsonArray& ADCSetting = root1.createNestedArray("ADCSetting");
			for (uint8_t idx = 0; idx < 6; idx++)
			{
				JsonObject& Object = ADCSetting.createNestedObject();
				Object["E"] = (uint8_t)bSensorUsed[idx];
				Object["H"] = SetPointHigh[idx];
				Object["L"] = SetPointLow[idx];
			}
			String strJSON1;
			root1.printTo(strJSON1);
			strJSON1 = "Sensor Setting->" + strJSON1;
			Serial.println(strJSON1);
			SendToMqtt(strJSON1);
		}

		{
			for (uint8_t idx = 0; idx < 6; idx++)
			{
				String msg = "ADC";
				msg += String(idx);
				if (SensorRealStatus[idx] == STATUS_ON)
				{ // On event triggered.
					msg += " ON! ";
				}
				else if (SensorRealStatus[idx] == STATUS_OFF)
				{ // Off event triggered.
					msg += " OFF! ";
				}
				uint16_t nADC = analogRead(ADCPins[idx]);
				double fAdc = (double)map(nADC, 0, 1024, 0, 330) / (double)100.0;

				msg += String(fAdc, 2);
				msg += "V.";
				if (bSensorUsed[idx] && SensorRealStatus[idx] != STATUS_IDLE)
				{
					delay(100);
					SendToMqtt(msg);
					Serial.println(msg);
				}
			}

			// Digital Input
			{
				String msg = "";
				bool bD5 = digitalRead(D5);
				if (bD5 == HIGH)
					msg = F("D5 Current Status High!");
				else
					msg = F("D5 Current Status Low!");
				Serial.println(msg);
				if (bDigitalUsed[0]) // When its setting is true, it will send mqtt message
					SendToMqtt(msg);
			}

			{
				bool bD9 = digitalRead(D9);
				String msg = "";
				if (bD9 == HIGH)
					msg = "D9 Current Status High!";
				else
					msg = "D9 Current Status Low!";
				if (bDigitalUsed[1]) // When its setting is true, it will send mqtt message
					SendToMqtt(msg);
			}

			{
				String msg = "";
				bool bD11 = digitalRead(D11);
				if (bD11 == HIGH)
					msg = "D11 Current Status High!";
				else
					msg = "D11 Current Status Low!";
				if (bDigitalUsed[3]) // When its setting is true, it will send mqtt message
					SendToMqtt(msg);
			}

			{
				String msg = "";
				bool bD12 = digitalRead(D12);
				if (bD12 == HIGH)
					msg = "D12 Current Status High!";
				else
					msg = "D12 Current Status Low!";
				if (bDigitalUsed[4]) // When its setting is true, it will send mqtt message
					SendToMqtt(msg);
			}

			{
				String msg;
				bool bD13 = digitalRead(D13);
				if (bD13 == HIGH)
					msg = "D13 Current Status High!";
				else
					msg = "D13 Current Status Low!";
				if (bDigitalUsed[5]) // When its setting is true, it will send mqtt message
					SendToMqtt(msg);
			}
		}

		delay(500);
		// Send Battery Voltage Info
		float BattVolt = BattVoltage();
		msg = "Battery Voltage : ";
		msg += String(BattVolt, 2);
		msg += ".";
		SendToMqtt(msg);

	}
	else
	{ // Failed
		Serial.print("failed, rc=");
		Serial.println(client.state());
	}
}

String GetQueryValue(String QueryName, String Query)
{
	String result = "";
	QueryName += "=";
	int nFirstPoint = Query.indexOf(QueryName);

	if (nFirstPoint < 0)
		return "";
	nFirstPoint += (QueryName.length() - 1);
	int nEndPoint = Query.indexOf(",", nFirstPoint + 1);
	if (nEndPoint < 0)
		result = Query.substring(nFirstPoint + 1, Query.length());
	else
		result = Query.substring(nFirstPoint + 1, nEndPoint);
	return result;
}

void SendToMqtt(String msg)
{
	if (!client.connected())
		return;

	String sendmsg = DeviceID;
	sendmsg += ", ";
	sendmsg += RTCtoString();
	sendmsg += ":";
	sendmsg += msg;

	sendmsg += "\n";
	sendmsg.toCharArray(mqtt_msg, sendmsg.length() + 1);
	client.publish(mqtt_OutTopic, mqtt_msg);
}

bool SaveSensorSetting()
{

	StaticJsonBuffer<1024> jsonBuffer;

	JsonObject& root = jsonBuffer.createObject();
	JsonArray& DIOSetting = root.createNestedArray("DIOSetting");
	for (uint8_t idx = 0; idx < DICOUNT; idx++)
		DIOSetting.add((uint8_t)bDigitalUsed[idx]);

	JsonArray& ADCSetting = root.createNestedArray("ADCSetting");
	for (uint8_t idx = 0; idx < ADCCOUNT; idx++)
	{
		JsonObject& Object = ADCSetting.createNestedObject();
		//Object.set("SetPointHigh", SetPointHigh[idx], 2);
		Object["Enable"] = (uint8_t)bSensorUsed[idx];
		Object["SetPointHigh"] = SetPointHigh[idx];
		Object["SetPointLow"] = SetPointLow[idx];
	}
	String strJSON;
	root.printTo(strJSON);
	Serial.println(strJSON);

	// Save setting string into file
	SD.remove("Sensor.txt");
	myFile = SD.open("Sensor.txt", FILE_WRITE);
	if (myFile)
	{
		myFile.print(strJSON);
		// close the file:
		myFile.close();
	}
	else
	{
		// if the file didn't open, print an error:
		Serial.println("error opening Sensor.txt");
	}

	return true;
}

void InitSensorSetting()
{
	// Save setting string into file
	myFile = SD.open("Sensor.txt", FILE_WRITE);
	if (myFile)
	{
		myFile.print(SensorJson);
		// close the file:
		myFile.close();
	}
	else
	{
		// if the file didn't open, print an error:
		Serial.println("error opening Sensor.txt");
	}

	StaticJsonBuffer<1024> jsonBuffer;

	JsonObject& root = jsonBuffer.parseObject(SensorJson);

	JsonArray& DIOSetting = root["DIOSetting"].asArray();
	for (uint8_t idx = 0; idx < DICOUNT; idx++)
	{
		bDigitalUsed[idx] = DIOSetting[idx].as<uint8_t>();
		Serial.print("Digital Input ");
		Serial.print(idx);
		Serial.print(" setting is ");
		Serial.println(bDigitalUsed[idx]);
	}

	JsonArray& ADCSetting = root["ADCSetting"].asArray();

	for (uint8_t idx = 0; idx < ADCCOUNT; idx++)
	{
		JsonObject& Object = ADCSetting.createNestedObject();
		bSensorUsed[idx] = (uint8_t)ADCSetting[idx]["Enable"];
		SetPointHigh[idx] = (double)ADCSetting[idx]["SetPointHigh"];
		SetPointLow[idx] = (double)ADCSetting[idx]["SetPointLow"];

		Serial.print("ADC Sesnsor ");
		Serial.print(idx);
		Serial.print(" Enable ");
		Serial.print(bSensorUsed[idx]);
		Serial.print(", SetPointHigh is ");
		Serial.print(SetPointHigh[idx], 2);
		Serial.print(", SetPointLow is ");
		Serial.println(SetPointLow[idx], 2);
	}
}



bool LoadSensorSetting()
{

	// Load setting string from file
	myFile = SD.open("Sensor.txt");
	String FileContent = "";
	if (myFile)
	{
		// read from the file until there's nothing else in it:
		while (myFile.available())
		{
			char temp = myFile.read();
			FileContent += temp;
			Serial.write(temp);
		}
		// close the file:
		myFile.close();
	}
	else
	{
		// if the file didn't open, print an error:
		Serial.println("error opening Sensor.txt");
		return false;
	}

	StaticJsonBuffer<1024> jsonBuffer;

	JsonObject& root = jsonBuffer.parseObject(FileContent);

	if (root == JsonObject::invalid())
		return false;

	JsonArray& DIOSetting = root["DIOSetting"].asArray();
	if (DIOSetting.size() != DICOUNT)
		return false;
	for (uint8_t idx = 0; idx < DICOUNT; idx++)
	{
		bDigitalUsed[idx] = DIOSetting[idx].as<uint8_t>();
		Serial.print("Digital Input ");
		Serial.print(idx);
		Serial.print(" setting is ");
		Serial.println(bDigitalUsed[idx]);
	}

	JsonArray& ADCSetting = root["ADCSetting"].asArray();
	if (ADCSetting.size() != ADCCOUNT)
		return false;

	for (uint8_t idx = 0; idx < ADCCOUNT; idx++)
	{
		JsonObject& Object = ADCSetting.createNestedObject();
		bSensorUsed[idx] = (uint8_t)ADCSetting[idx]["Enable"];
		SetPointHigh[idx] = (double)ADCSetting[idx]["SetPointHigh"];
		SetPointLow[idx] = (double)ADCSetting[idx]["SetPointLow"];

		Serial.print("ADC Sesnsor ");
		Serial.print(idx);
		Serial.print(" Enable ");
		Serial.print(bSensorUsed[idx]);
		Serial.print(", SetPointHigh is ");
		Serial.print(SetPointHigh[idx], 2);
		Serial.print(", SetPointLow is ");
		Serial.println(SetPointLow[idx], 2);
	}

	return true;
}



void InitNetworkSetting()
{
	Serial.println("Init network Setting");
	// Save setting string into file
	myFile = SD.open("Network.txt", FILE_WRITE);
	if (myFile)
	{
		myFile.print(NetworkJson);
		// close the file:
		myFile.close();
	}
	else
	{
		// if the file didn't open, print an error:
		Serial.println("error opening Network.txt");
	}

	StaticJsonBuffer<512> jsonBuffer;

	JsonObject& root = jsonBuffer.parseObject(NetworkJson);

	const char *ssid = root["ssid"];
	Strssid = String(ssid);
	const char *pass = root["pass"];
	Strpass = String(pass);
	const char *ipaddr = root["ipaddr"];
	IPAddrSetting = String(ipaddr);
	const char *subnet = root["subnet"];
	SubnetSetting = String(subnet);
	const char *gateway = root["gateway"];
	GateWaySetting = String(gateway);
	const char *dns = root["dns"];
	DnsSetting = String(dns);
	const char *MqttUrl = root["MqttUrl"];
	StrMqttUrl = String(MqttUrl);
	const char *MqttPort = root["MqttPort"];
	StrMqttPort = String(MqttPort);

	Serial.println(Strssid);
	Serial.println(Strpass);
	Serial.println(IPAddrSetting);
	Serial.println(SubnetSetting);
	Serial.println(GateWaySetting);
	Serial.println(DnsSetting);
	Serial.println(StrMqttUrl);
	Serial.println(StrMqttPort);

}

bool LoadNetworkSetting()
{
	Serial.println("Load network Setting");
	// Load setting string from file
	myFile = SD.open("Network.txt");
	String FileContent = "";
	if (myFile)
	{
		// read from the file until there's nothing else in it:
		while (myFile.available())
		{
			char temp = myFile.read();
			FileContent += temp;
			Serial.write(temp);
		}
		// close the file:
		myFile.close();
	}
	else
	{
		// if the file didn't open, print an error:
		Serial.println("error opening NetworkSetting.txt");
		return false;
	}

	StaticJsonBuffer<512> jsonBuffer;
	JsonObject& root = jsonBuffer.parseObject(FileContent);

	if (root == JsonObject::invalid())
		return false;

	const char *ssid = root["ssid"];
	Strssid = String(ssid);
	const char *pass = root["pass"];
	Strpass = String(pass);
	const char *ipaddr = root["ipaddr"];
	IPAddrSetting = String(ipaddr);
	const char *subnet = root["subnet"];
	SubnetSetting = String(subnet);
	const char *gateway = root["gateway"];
	GateWaySetting = String(gateway);
	const char *dns = root["dns"];
	DnsSetting = String(dns);
	const char *MqttUrl = root["MqttUrl"];
	StrMqttUrl = String(MqttUrl);
	const char *MqttPort = root["MqttPort"];
	StrMqttPort = String(MqttPort);

	Serial.println(Strssid);
	Serial.println(Strpass);
	Serial.println(IPAddrSetting);
	Serial.println(SubnetSetting);
	Serial.println(GateWaySetting);
	Serial.println(DnsSetting);
	Serial.println(StrMqttUrl);
	Serial.println(StrMqttPort);
	return true;
}

bool SaveNetworkSetting()
{
	StaticJsonBuffer<256> jsonBuffer;

	JsonObject& root = jsonBuffer.createObject();
	root["ssid"] = Strssid;
	root["pass"] = Strpass;
	root["ipaddr"] = IPAddrSetting;
	root["subnet"] = SubnetSetting;
	root["gateway"] = GateWaySetting;
	root["dns"] = DnsSetting;
	root["MqttUrl"] = StrMqttUrl;
	root["MqttPort"] = StrMqttPort;

	String strJSON;
	root.printTo(strJSON);
	Serial.println(strJSON);
	// Save setting string into file
	SD.remove("Network.txt");
	myFile = SD.open("Network.txt", FILE_WRITE);
	if (myFile)
	{
		myFile.print(strJSON);
		// close the file:
		myFile.close();
	}
	else
	{
		// if the file didn't open, print an error:
		Serial.println("error opening Network.txt");
	}

	return true;
}

/**/
void InitPorts()
{
	pinMode(D5, INPUT_PULLUP);
	pinMode(D9, INPUT_PULLUP);
	pinMode(D11, INPUT_PULLUP);
	pinMode(D12, INPUT_PULLUP);
	pinMode(D13, INPUT_PULLUP);
}

float BattVoltage()
{
	float measuredvbat = analogRead(VBATPIN);
	measuredvbat *= 2;    // we divided by 2, so multiply back
	measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredvbat /= 1024; // convert to voltage
						  //Serial.print("VBat: "); Serial.println(measuredvbat);
	return measuredvbat;
}
