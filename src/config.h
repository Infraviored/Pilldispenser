#ifndef CONFIG_H
#define CONFIG_H

extern const char *MQTT_DEVICE_NAME;
extern const char *commandTopic;
extern const char *stateTopic;

extern const int SENSOR_PIN;
extern const int RED_LED_PIN;
extern const int WHITE_LED_PIN;

extern const int drop_threshold;
extern const int collection_threshold;
extern const float collection_threshold_factor;
extern const int lowestPWM;
extern const int highestPWM;
extern const int rotationBeforeDrop;
extern const int maxRate;

extern const char* ssid;
extern const char* password;
extern const char* mqtt_server;
extern const int mqtt_port;
extern const char* mqtt_user;
extern const char* mqtt_pass;



extern int zinc[];
extern int omega[];
extern int magnesium[];

extern int order[];

#endif