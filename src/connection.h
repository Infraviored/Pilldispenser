#ifndef CONNECTION_H
#define CONNECTION_H

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

extern WiFiClient espClient;
extern PubSubClient client;

void flashLed(int repetitions, int on_duration, int off_duration = 0);
void setup_wifi();
void reconnect();

#endif // CONNECTION_H
