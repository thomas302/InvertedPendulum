#include "Motor.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <WiFi.h>
//#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include "LittleFS.h"
#include <ArduinoJson.h>

ESP32Encoder pendEnc, motorEnc;

Motor* m;

hw_timer_t *Timer0_Cfg = NULL;
hw_timer_t *Timer1_Cfg = NULL;

double start;

const char* ssid     = "InvPendulum";
const char* password = "password";
//const char* hostname = "motor-pid-server";



String getPosition(){
  String jsonString = "{\"position\":" + String(m->input,10) + "}";
  return jsonString;
}

String getSetpoint(){
  String jsonString = "{\"setpoint\":" + String(m->setpoint, 10) + "}";
  return jsonString;
}

String getOutputs(){
  String jsonString = "{\"position\":" + String((int) m->input,10);
  jsonString = jsonString + ",\"output\":"+ String(m->output, 5) + "}";
  return jsonString;
}

String getPIDF(){
  String jsonString = "{\"KP\":" + String(m->mPID->GetKp(),20);
  jsonString = jsonString + ",\"KI\":" + String(m->mPID->GetKi(),20);
  jsonString = jsonString + ",\"KD\":" + String(m->mPID->GetKd(),20);
  jsonString = jsonString + ",\"KF\":" + String(m->kF,5);
  jsonString = jsonString + "}";
  return jsonString;
}

void IRAM_ATTR updatePID()
{
    m->updatePIDNow();
}

bool writePos = false;
void IRAM_ATTR updateServerOut(){
  writePos = true;
}

void setup()
{
  pinMode(16, INPUT);
  pinMode(17, INPUT);

  pinMode(34, INPUT);
  pinMode(35, INPUT);

  pendEnc.attachFullQuad(16,17);

  motorEnc.attachFullQuad(34, 35);

  m = new Motor(33, 25, 32, &pendEnc, &motorEnc);

  Serial.begin(115200);

  m->setSetpoint(25000);

  m->configPIDF(.0001,0,0,0);

  m->mPID->Initialize();

  // Sets timer to update pid on 10ms loop time
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &updatePID, true);
  timerAlarmWrite(Timer0_Cfg, 10*1000, true);
  timerAlarmEnable(Timer0_Cfg);

  server_setup();

  // Sets timer to update server output every 500 ms.
  Timer1_Cfg = timerBegin(1, 80, true);
  timerAttachInterrupt(Timer1_Cfg, &updateServerOut, true);
  timerAlarmWrite(Timer1_Cfg, 500*1000, true);
  timerAlarmEnable(Timer1_Cfg);

}

void loop()
{
  m->updateInput();
  m->writeOutputESC();
  
    m->debugInfo();
    writePos = false;
  
}
