#include "config.h"
#include <ESP8266WiFi.h>

//! IMPORTANT! RENAME!
const char *MQTT_DEVICE_NAME = "pilldispenser";

// Construct topics using String and then convert back to const char*
String sCommandTopic = String(MQTT_DEVICE_NAME) + "/commands";
String sStateTopic = String(MQTT_DEVICE_NAME) + "/state";
const char *commandTopic = sCommandTopic.c_str();
const char *stateTopic = sStateTopic.c_str();

// Pin to which the sensor is connected
const int SENSOR_PIN = A0;
const int RED_LED_PIN = D2;
const int WHITE_LED_PIN = D1;

// Threshold value for triggering the sensor
const int drop_threshold = 20;
const int collection_threshold = 1200;
const float collection_threshold_factor = 0.93;

const int lowestPWM = 415;
const int highestPWM = 2575;

const int rotationBeforeDrop = 15;

const int maxRate = 3;




// DISPENSER DEFINITIONS

//                   PIN         LOWEST      PREDROP     DROP        LASTPOS     SHAKEDIST   FALLDURATION
int zinc[] =        {D5,         180,        30,         2,          0,          22,         2400};
int omega[] =       {D6,         180,        40,         0,          0,          30,         1600};
int magnesium[] =   {D7,         180,        30,         12,         0,          30,         1200};

// PIN: Where the Servo PWM Pin is connected.
// LOWEST: Lowest Position  of the disk in degrees. 180 normally is the lowest Point, 0 is the highest.
// PREDROP: The Location to where the hole of the disk moves before the dropping sequence (with detection) is started. This should be chosen s.t. the Pill is just before the hole but can not fall out yet.
// DROP: The Loctaion where the drop happens. This is normally close to 0 degrees (the uppermost position).
// SHAKEDIST: from the LOWEST Point, how far up the disk should be moved during one shaking motion.
// DURATION: How long the dropping sequence with detection should take. The further up the Dispenser is, the longer the time it takes to fall down, the more DURATION is needed.




int order[] = {*zinc, *omega, *magnesium};