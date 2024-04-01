#include <Arduino.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "connection.h"
#include "Breather.h"
// Function prototypes
void dispense(int *servo);
void shakeBottom(int *servo);
void moveTo(int *servo, int position, int duration);
void handleMqttMessage(char* topic, byte* payload, unsigned int length);
void detect_collection(); 
int step_value = 0;
int collection_value = 0;

bool uncollected = false;
int detectionCount = 0;
unsigned long firstDetectionTime = 0;
unsigned long secondDetectionTime = 0;

// Declare a global servo object
Servo currentServo;

#define RED_LED_PIN 4
#define WHITE_LED_PIN 5

Breather redBreather(RED_LED_PIN);
Breather whiteBreather(WHITE_LED_PIN);

void start_breathing(Breather &breather1, int cycleDuration1, Breather *breather2 = nullptr, int cycleDuration2 = 0, unsigned long offset2 = 0)
{
  breather1.startBreathing(cycleDuration1);
  if (breather2)
  {
    delay(offset2);
    breather2->startBreathing(cycleDuration2);
  }
}

void breathe_once(Breather &breather1, int cycleDuration1, unsigned long startDelay1 = 0, Breather *breather2 = nullptr, int cycleDuration2 = 0, unsigned long startDelay2 = 0)
{
  breather1.breatheOnce(cycleDuration1, startDelay1);

  if (breather2)
  {
    breather2->breatheOnce(cycleDuration2, startDelay2);
  }

  unsigned long endTime = millis() + cycleDuration1 + startDelay1;
  if (breather2)
  {
    unsigned long breather2EndTime = millis() + cycleDuration2 + startDelay2;
    if (breather2EndTime > endTime)
    {
      endTime = breather2EndTime;
    }
  }

  while (millis() < endTime)
  {
    breather1.update();
    if (breather2)
      breather2->update();
    delay(10); // Small delay to avoid CPU hogging
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting up...");

  pinMode(SENSOR_PIN, INPUT);


  redBreather.initialize();
  whiteBreather.initialize();

  breathe_once(redBreather, 1000, 0, &whiteBreather, 1000, 500);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(handleMqttMessage);

  setup_wifi();
  reconnect();



  Serial.println("Ready to give drugs!");
  breathe_once(redBreather, 2100, 0, &whiteBreather, 2100, 1000);
}


void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  redBreather.update();
  whiteBreather.update();
  client.loop();
  if (uncollected) {
    detect_collection();
  }
  delay(20);
}

void handleMqttMessage(char *topic, byte *payload, unsigned int length)
{
  Serial.println("Command received");
  String message = "";
  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }

  // Use the global constant for topic comparison
  if (strcmp(topic, commandTopic) == 0)
  {
    if (message == "dispense_zinc")
    {
      Serial.println("Dispense zinc");
      client.publish(stateTopic, "dispensing Zinc");
      dispense(zinc);
    }
    else if (message == "dispense_omega")
    {
      client.publish(stateTopic, "dispensing Omega");
      Serial.println("Dispense omega");
      dispense(omega);
    }
    else if (message == "dispense_magnesium")
    {
      client.publish(stateTopic, "dispensing Magnesium");
      Serial.println("Dispense magnesium");
      dispense(magnesium);
    }
    else if (message == "reset_error")
    {
      Serial.println("Reset the red blinker");
      redBreather.stopBreathing();
      client.publish(stateTopic, "ready");
    }
    else
    {
      Serial.println("Command unknown");
    }
  }
}

void moveTo(int *servo, int position, int duration){
  int oldPosition = servo[4];
  int deltaPos = abs(oldPosition - position);
  currentServo.attach(servo[0], lowestPWM, highestPWM);

  if (duration == 0 || deltaPos*maxRate > duration){ //no duration given or duration smaller than minimal Time

  currentServo.write(position);
  delay(maxRate*deltaPos);
  }

  else {
    if (oldPosition <= position){
      for (int pos = oldPosition; pos <= position; pos += 1) {
        currentServo.write(pos);
        delay(duration/deltaPos);
      }

    } else{
      for (int pos = oldPosition; pos >= position; pos -= 1) {
        currentServo.write(pos);
        delay(duration/deltaPos);
      }
    }
  }
  currentServo.detach();
  servo[4] = position;
}



void shakeBottom(int *servo){
  int lowpos = servo[1];
  int highpos = lowpos - servo[5];
  for (int j = 0; j < 8; j++) {
    moveTo(servo, lowpos, 0);
    moveTo(servo, highpos, 0);
  }
}

void detect_collection()
{
  step_value = analogRead(SENSOR_PIN);
  if (step_value < 14)
  {
    step_value = 0;
  }
  collection_value = 10 * step_value + collection_threshold_factor * collection_value;
  Serial.println(collection_value);
  if(collection_value > collection_threshold)
  {
    Serial.println("Collection detected");
    client.publish("pilldispenser/state", "ready");
    uncollected = false;
    collection_value = 0;
    redBreather.stopBreathing();
    whiteBreather.stopBreathing();
  }
}

bool dropSense(int *servo) {
  //pre drop location is LARGER
  int preDropLocation = servo[2];
  int dropLocation = servo[3];
  int duration = servo[6];

  // Move to pre drop location
  moveTo(servo, preDropLocation, 800);

  // Attach servo
  currentServo.attach(servo[0], lowestPWM, highestPWM);

  // Calculate distance to drop location
  float deltaPos = abs(preDropLocation - dropLocation);

  bool dropped = false;

  // First loop: Read and move for 1/2 second
  for (int i = 0; i < 20; i++) {
    // Read sensor value and move servo
    int sensorValue = analogRead(SENSOR_PIN);
    Serial.println(sensorValue);
    int pos = preDropLocation - deltaPos * ((float)i / (float)20);
    currentServo.write(pos);

    // Check if threshold is reached
    if (sensorValue > drop_threshold) {
      Serial.println("DROP");
      dropped = true;
      break;
    }
  }

  // Second loop: Read only, Skip when already dropped.
  if (!dropped) {
    for (int i = 0; i <= (duration*2); i++) {
      int sensorValue = analogRead(SENSOR_PIN);
      Serial.println(sensorValue);

      // Check if threshold is reached
      if (sensorValue > drop_threshold) {
        Serial.println("DROP");
        dropped = true;
        break;
      }
    }
  }

  // Detach servo and return result
  currentServo.detach();
  if (dropped) {
    return true;
  } else {
    Serial.println('-');
    return false;
  }
}

  void dispense(int *servo)
  {
        for (int i = 0; i < 5; i++)
    {
      moveTo(servo, servo[1], 0);
      shakeBottom(servo);

      if (dropSense(servo))
      {
        Serial.println("Dispensed!");
        uncollected = true;
        start_breathing(redBreather, 4000, &whiteBreather, 2000, 1000);
        client.publish("pilldispenser/state", "Pills ready");
        delay(100);
        return;
      }
      client.loop();
    }
    Serial.println("Error: Pill dispenser empty!");
    client.publish("pilldispenser/state", "Dispense Error");
    redBreather.startBreathing(200);
  }
