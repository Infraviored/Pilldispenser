#include <Arduino.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "config.h"
// Function prototypes
void dispense(int *servo);
void shakeBottom(int *servo);
void moveTo(int *servo, int position, int duration);
void handleMqttMessage(char* topic, byte* payload, unsigned int length);
void flashLed(int repetitions, int on_duration, int off_duration);
void setup_wifi();
void reconnect();
void detect_collection(); 
void detect_collection2();
int step_value = 0;
int collection_value = 400000; 

bool uncolleted = false;
int detectionCount = 0;
unsigned long firstDetectionTime = 0;
unsigned long secondDetectionTime = 0;

WiFiClient espClient;
PubSubClient client(espClient);

// Declare a global servo object
Servo currentServo;
class Breather
{
private:
  int pin;
  int maxBrightness = 255;
  unsigned long phaseShift;
  unsigned long halfPeriod;
  unsigned long previousTime = 0;

public:
  enum BreatheState
  {
    OFF,
    BREATHING_CONTINUOUSLY,
    BREATHE_ONCE
  } state = OFF;

  Breather(int pin) : pin(pin) {}

  void initialize()
  {
    pinMode(pin, OUTPUT);
    analogWrite(pin, 0); // LED OFF for a clear start
    phaseShift = 0;      // Reset phase shift
  }

  void update()
  {
    if (state != OFF)
    {
      unsigned long time = (millis() - phaseShift) % (2 * halfPeriod);

      if (state == BREATHE_ONCE && previousTime > time) // means it has looped over
      {
        state = OFF;
        analogWrite(pin, 0); // LED OFF
        Serial.println(getName() + " finished a cycle and set to OFF state.");
        return;
      }

      int brightness;
      if (time < halfPeriod)
      {
        brightness = maxBrightness * time / halfPeriod;
      }
      else
      {
        brightness = maxBrightness * (2 * halfPeriod - time) / halfPeriod;
      }
      analogWrite(pin, brightness); // Adjust the brightness

      previousTime = time;
    }
    else
    {
      analogWrite(pin, 0); // LED OFF
    }
  }

  void startBreathing(int cycleDuration, unsigned long delay = 0)
  {
    state = BREATHING_CONTINUOUSLY;
    halfPeriod = cycleDuration / 2;
    phaseShift = millis() - delay;
  }

  void stopBreathing()
  {
    state = OFF;
    analogWrite(pin, 0); // LED OFF
  }

  void breatheOnce(int cycleDuration, unsigned long startDelay = 0)
  {
    analogWrite(pin, 0); // Ensure the LED starts from 0
    state = BREATHE_ONCE;
    halfPeriod = cycleDuration / 2;
    phaseShift = millis() + startDelay - halfPeriod; // Adjust phaseShift to account for delay
    Serial.println(getName() + " set to BREATHE_ONCE state with a start delay of " + String(startDelay));
  }

  String getName() const
  {
    return "Breather at pin " + String(pin);
  }
};

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

  // Use the new setup_wifi function for WiFi connection
  setup_wifi();

  // Use the new reconnect function for MQTT connection
  reconnect();



  Serial.println("Ready to give drugs!");
  breathe_once(redBreather, 2100, 0, &whiteBreather, 2100, 1000);
}



void loop()
{
  redBreather.update();
  whiteBreather.update();
  client.loop();
  if (uncolleted) {
    detect_collection();
  }
}
void detect_collection(){
  step_value = analogRead(SENSOR_PIN);
  collection_value = 100*step_value+0.999*collection_value;
  if (collection_value > 550000){
    Serial.println("Collection detected");
    uncolleted = false;
    collection_value = 400000;
    redBreather.stopBreathing();
    whiteBreather.stopBreathing();
  }
}

void handleMqttMessage(char* topic, byte* payload, unsigned int length) {
  Serial.println("Command received");
  String message = "";
  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  if (strcmp(topic, "pilldispenser/commands") == 0) {
    if (message == "dispense_zinc") {
      Serial.println("Dispense zinc");
      client.publish("pilldispenser/state", "dispensing Zinc");
      dispense(zinc);
    } else if (message == "dispense_omega") {
      client.publish("pilldispenser/state", "dispensing Omega");
      Serial.println("Dispense omega");
      dispense(omega);
    } else if (message == "dispense_magnesium") {
      client.publish("pilldispenser/state", "dispensing Magnesium");
      Serial.println("Dispense magnesium");
      dispense(magnesium);
    } else if (message == "reset_error") {
      Serial.println("Reset the red blinker");
      redBreather.stopBreathing();
      client.publish("pilldispenser/state", "ready");
    } else {
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

  int loopdelay = 2;
  int maxLoopStep = duration / loopdelay;

  // First loop: Read and move for 1/2 second
  for (int i = 0; i < 20; i++) {
    // Read sensor value and move servo
    int sensorValue = analogRead(SENSOR_PIN);
    int pos = preDropLocation - deltaPos * ((float)i / (float)20);
    currentServo.write(pos);

    // Check if threshold is reached
    if (sensorValue > threshold) {
      Serial.println("DROP");
      dropped = true;
      break;
    }
  }

  // Second loop: Read only, Skip when already dropped.
  if (!dropped) {
    for (int i = 0; i <= maxLoopStep; i++) {
      int sensorValue = analogRead(SENSOR_PIN);

      // Check if threshold is reached
      if (sensorValue > threshold) {
        Serial.println("DROP");
        dropped = true;
        break;
      }

      delay(loopdelay);
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
        uncolleted = true;
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

  void flashLed(int repetitions, int on_duration, int off_duration = 0)
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
      if (client.connect("pilldispenser", mqtt_user, mqtt_pass))
      {
        Serial.println("connected");
        flashLed(3, 300, 300); // Flash the built-in LED 3 times after connecting
        client.subscribe("pilldispenser/commands");
        client.publish("pilldispenser/state", "ready");
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