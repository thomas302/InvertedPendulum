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

//const char* hostname = "motor-pid-server";
const double belt_pitch = 0.2; // (cm/tooth)
const double num_pulley_teeth = 60; 
const double pi_val = 3.1415926535; 
const double num_encoder_ticks = 4096; // (ticks/rev)
double cart_pos;

void write_cart_position() {
  cart_pos = belt_pitch * num_pulley_teeth / (pi_val * num_encoder_ticks); // (cm)
  Serial.print("cm:");
  Serial.println(cart_pos);
}


void IRAM_ATTR updatePID()
{
    m->update_input();
    m->update_PID();
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

    m = new Motor(33, 25, 32, &motorEnc);

    Serial.begin(115200);

    m->set_setpoint(25000);

    m->config_PIDF(.0001,0,0,0);

    // Sets timer to update pid on 10ms loop time
    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &updatePID, true);
    timerAlarmWrite(Timer0_Cfg, 10*1000, true);
    timerAlarmEnable(Timer0_Cfg);

    // Sets timer to update server output every 500 ms.
    Timer1_Cfg = timerBegin(1, 80, true);
    timerAttachInterrupt(Timer1_Cfg, &updateServerOut, true);
    timerAlarmWrite(Timer1_Cfg, 500*1000, true);
    timerAlarmEnable(Timer1_Cfg);

}

void loop()
{
    write_cart_position();
    //m->debugInfo();
    m->write_output();
    writePos = false;
}

