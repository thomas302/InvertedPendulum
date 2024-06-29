#include "Motor.h"
#include <ESP32Encoder.h>

#define PendulumLength 24;
//        Forward Reverse, Enable, Enc1, Enc2, mode
Motor m(33, 25, 32, 34, 35, 0);

ESP32Encoder enc; 

hw_timer_t *Timer0_Cfg = NULL;

double start;

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <ArduinoJson.h>

const char* ssid = "Casadevelez";
const char* password = "Nosoup4you!";
const char* hostname = "esp32-server";

AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Json Variable to Hold Sensor Readings
JsonDocument readings;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void notifyClients(String sensorReadings) {
  ws.textAll(sensorReadings);
}

String getPosition(){
  //readings["KP"] = m.mPID->GetKp();
  //readings["KI"] = m.mPID->GetKi();
  //readings["KD"] = m.mPID->GetKd();
  //readings["KF"] = m.kF;
  //readings["setpoint"] = m.setpoint;
  //readings["pOut"] = m.output;
  //readings["position"] = m.encoder.getCount();
  String jsonString = "{\"position\":" + String(m.encoder.getCount(),10) + "}";
  //serializeJson(readings, jsonString);
  return jsonString;
}

String getSetpoint(){
  //readings["KP"] = m.mPID->GetKp();
  //readings["KI"] = m.mPID->GetKi();
  //readings["KD"] = m.mPID->GetKd();
  //readings["KF"] = m.kF;
  //readings["setpoint"] = m.setpoint;
  //readings["pOut"] = m.output;
  //readings["position"] = m.encoder.getCount();
  String jsonString = "{\"setpoint\":" + String(m.setpoint, 10) + "}";
  //serializeJson(readings, jsonString);
  return jsonString;
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  JsonDocument buffer;
  DeserializationError e =  deserializeJson(buffer, data);
  String val;

  switch(e.code()){
      case e.Ok:        
        val = serializeJsonPretty(buffer, val);
        Serial.print(val);
        if(buffer.containsKey("getReadings")){
          String vals = getPosition();
          notifyClients(vals);
          Serial.println(vals);
        }
        if(buffer.containsKey("getSetpoint")){
          notifyClients(getSetpoint());
          Serial.println(getSetpoint());
        }
        else if(buffer.containsKey("write")){
          double kp, ki, kd, kf, setpoint, output;
          if(buffer.containsKey("KP")){
            kp = buffer["KP"].as<double>();
          }else kp = m.mPID->GetKp();

          if(buffer.containsKey("KI")){
            ki = buffer["KI"].as<double>();
          }else ki = m.mPID->GetKi();

          if(buffer.containsKey("Kd")){
            kd = buffer["KD"].as<double>();
          }else kd = m.mPID->GetKd();

          if(buffer.containsKey("KF")){
            kf = buffer["KF"].as<double>();
          }else kf = m.kF;

          if(buffer.containsKey("setpoint")){
            setpoint = buffer["setpoint"].as<double>();
          }else setpoint = m.setpoint;

          /*if(buffer.containsKey("output")){
            output = buffer["output"].as<double>();
          }else output = m.output;*/
        }
      break;
      default:
        Serial.print("Error: ");
        Serial.println(e.c_str());
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}


void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void server_setup(){
  initWiFi();
  initWebSocket();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "Server Exists", "text/html");
  });

  server.begin();
}

/*void IRAM_ATTR Timer0_ISR()
{
    m.mPID->calculateNow();
    m.writeOutput();
    Serial.println("run");
}
/*
void setup() {
  analogWriteResolution(10);
  m.configPIDF(1/80000.0,0,1/30000,0);
  m.setPID_Enabled(true);
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  server_setup();
  //m = Motor(33, 25, 32, 34, 35, 0);
  start = millis();
  Timer0_Cfg = timerBegin(0, 80, true);
  m.setpoint = 50000;
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 1000, true);
}

void loop() {
  if(millis()-start > 500){
    notifyClients(getPosition());
    start = millis();
    Serial.println("written");
  }
}
*/
//hw_timer_t *Timer0_Cfg = NULL;

const int LED = 2;
void IRAM_ATTR Timer0_ISR()
{

    m.updatePIDNow();
    //Serial.println("run");
    digitalWrite(LED, !digitalRead(LED));
}
void setup()
{
    Serial.begin(115200);
    m.setSetpoint(50000);

    server_setup();
    pinMode(LED, OUTPUT);

    m.configPIDF(1/80000.0,0,1/30000,0);

    m.mPID->Initialize();

    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 50*1000, true);
    timerAlarmEnable(Timer0_Cfg);
    start = millis();
}
void loop()
{
  m.updateInput();
  m.writeOutput();
  //Serial.println(m.output);
    // Do Nothing!
  if(millis()-start > 500){\
    notifyClients(getPosition());
    start = millis();   
  }
}