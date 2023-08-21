#include <Arduino.h>
#include "config.h"
#include "connection.h"

// Global object instantiation
WiFiClient espClient;
PubSubClient client(espClient);

void flashLed(int repetitions, int on_duration, int off_duration)
{
  for (int i = 0; i < repetitions; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED
    delay(on_duration);
    digitalWrite(LED_BUILTIN, LOW); // Turn off the LED
    delay(off_duration);
  }
  }

  void setup_wifi()
  {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.print(".");
      flashLed(1, 100, 500); // Blink once every 500ms while trying to connect to WiFi
    }

    flashLed(3, 100, 100); // Blink 3 times if connected to WiFi
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

  void reconnect()
  {
    while (!client.connected())
    {
      Serial.print("Attempting MQTT connection... ");
      if (client.connect(MQTT_DEVICE_NAME, mqtt_user, mqtt_pass))
      {
        Serial.println("connected");
        flashLed(3, 300, 300); // Flash the built-in LED 3 times after connecting

        // Directly use the global constants
        client.subscribe(commandTopic);
        client.publish(stateTopic, "ready");
      }
      else
      {
        Serial.print("Failed to connect, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        flashLed(2, 100, 400); // Blink twice if failed to connect
        if (client.state() == -2)
        {
          Serial.println("Network unreachable, redoing WiFi setup...");
          WiFi.disconnect(); // Disconnect from the WiFi network
          delay(1000);       // Wait for a while
          setup_wifi();      // Redo the WiFi setup
        }
        delay(2000);
      }
    }
  }
  
