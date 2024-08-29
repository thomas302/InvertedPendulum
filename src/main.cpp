#include "Motor.h"
#include <ESP32Encoder.h>
#include <WiFi.h>
//#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include "LittleFS.h"
#include <ArduinoJson.h>

Motor m(33, 25, 32, 34, 35, 0);

ESP32Encoder enc; 

hw_timer_t *Timer0_Cfg = NULL;
hw_timer_t *Timer1_Cfg = NULL;

double start;

const char* ssid     = "InvPendulum";
const char* password = "password";
//const char* hostname = "motor-pid-server";

JsonDocument buffer;
DeserializationError e;
String val;

AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Initialize WiFi
void initWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  //WiFi.setHostname(hostname);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println("");
  //delay (1000);

  Serial.println(WiFi.localIP());

}

void notifyClients(String sensorReadings) {
  ws.textAll(sensorReadings);
}

String getPosition(){
  String jsonString = "{\"position\":" + String(m.input,10) + "}";
  return jsonString;
}

String getSetpoint(){
  String jsonString = "{\"setpoint\":" + String(m.setpoint, 10) + "}";
  return jsonString;
}

String getOutputs(){
  String jsonString = "{\"position\":" + String((int) m.input,10);
  jsonString = jsonString + ",\"output\":"+ String(m.output, 5) + "}";
  return jsonString;
}

String getPIDF(){
  String jsonString = "{\"KP\":" + String(m.mPID->GetKp(),20);
  jsonString = jsonString + ",\"KI\":" + String(m.mPID->GetKi(),20);
  jsonString = jsonString + ",\"KD\":" + String(m.mPID->GetKd(),20);
  jsonString = jsonString + ",\"KF\":" + String(m.kF,5);
  jsonString = jsonString + "}";
  return jsonString;
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  e = deserializeJson(buffer, data);

  switch(e.code()){
      case e.Ok:        
        val = serializeJson(buffer, val);
        if(buffer.containsKey("getReadings")){
          notifyClients(getPosition());
          Serial.println(getPosition());
        }
        if(buffer.containsKey("getSetpoint")){
          notifyClients(getSetpoint());
          Serial.println(getSetpoint());
        }
        if(buffer.containsKey("getPIDF")){
          notifyClients(getPIDF());
          Serial.println(getPIDF());
        }
        else if(buffer.containsKey("write")){
          Serial.print("writing");
          double kp, ki, kd, kf, setpoint, output;
          bool pidfUpdate = false;
          if(buffer.containsKey("KP")){
            kp = buffer["KP"].as<double>();
            Serial.print(" KP: ");
            Serial.println(kp,10);
            pidfUpdate = true;
          }else kp = m.mPID->GetKp();

          if(buffer.containsKey("KI")){
            ki = buffer["KI"].as<double>();
            Serial.print(" KI: ");
            Serial.println(ki,10);
            pidfUpdate = true;
          }else ki = m.mPID->GetKi();

          if(buffer.containsKey("KD")){
            kd = buffer["KD"].as<double>();
            Serial.print(" KD: ");
            Serial.println(kd,10);
            pidfUpdate = true;
          }else kd = m.mPID->GetKd();

          if(buffer.containsKey("KF")){
            kf = buffer["KF"].as<double>();
            Serial.print(" KF: ");
            Serial.println(kf,10);
            pidfUpdate = true;
          }else kf = m.kF;

          if (pidfUpdate){
            m.configPIDF(kp, ki, kd, kf);
          }

          if(buffer.containsKey("setpoint")){
            setpoint = buffer["setpoint"].as<double>();
            Serial.println(setpoint);
            m.setpoint = setpoint;
          }else setpoint = m.setpoint;
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
    request->send(200, "text/html", "Server Exists");
  });

  server.begin();
}

const int LED = 2;
void IRAM_ATTR Timer0_ISR()
{
    m.updatePIDNow();
    //digitalWrite(LED, !digitalRead(LED));
}

bool writePos = false;
void IRAM_ATTR Timer1_ISR(){
  writePos = true;
}

ESP32Encoder pendulumEnc;

void setup()
{
    Serial.begin(115200);
    m.setSetpoint(50000);
    pinMode(LED, OUTPUT);

    m.configPIDF(1/80000.0,0,1/30000,0);

    m.mPID->Initialize();

    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 10*1000, true);
    timerAlarmEnable(Timer0_Cfg);

    server_setup();

    Timer1_Cfg = timerBegin(1, 80, true);
    timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
    timerAlarmWrite(Timer1_Cfg, 500*1000, true);
    timerAlarmEnable(Timer1_Cfg);


    pinMode(16, INPUT);
    pinMode(17, INPUT);

    pendulumEnc.attachFullQuad(16, 17);

}

void loop()
{
  m.updateInput();
  m.writeOutput();
  Serial.println(pendulumEnc.getCount());
    // Do Nothing!
  if(writePos){
    notifyClients(getOutputs());
    writePos = false;  
  }
}