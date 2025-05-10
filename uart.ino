#include <Arduino.h>
#include "esp32-hal-ledc.h"
#include <cstdlib>
#include <string.h>
#include <time.h>

// Libraries for MQTT client and WiFi connection
#include <WiFi.h>
#include <mqtt_client.h>

// Azure IoT SDK for C includes
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>

// Additional sample headers
#include "AzIoTSasToken.h"
#include "SerialLogger.h"
#include "iot_configs.h"

#include <ArduinoJson.h> // Thư viện JSON
#include <HardwareSerial.h> // Để dùng Serial2

// FreeRTOS Configuration
#define WIFI_TASK_STACK_SIZE      3072
#define MQTT_TASK_STACK_SIZE      3072
#define TEST_DATA_TASK_STACK_SIZE 4096  // Renamed from UART_TASK_STACK_SIZE

// H-Bridge motor control pins
#define MOTOR_ENB_PIN     21  // Enable pin for motor (PWM)
#define MOTOR_IN3_PIN     23  // Input 1 for H-bridge
#define MOTOR_IN4_PIN     22  // Input 2 for H-bridge

// Valve pins
#define VALVE1_PIN        5
#define VALVE2_PIN        17
#define VALVE3_PIN        16
#define VALVE4_PIN        4
#define VALVE5_PIN        0 

// PWM Configuration for motor
#define MOTOR_PWM_CHANNEL 0
#define MOTOR_PWM_FREQ    5000
#define MOTOR_PWM_RESOLUTION 8
#define MOTOR_SPEED       200  // PWM value from 0 to 255

// Test configuration
#define TEST_DATA_INTERVAL 30000  // Generate test data every 30 seconds
#define PUMP_DURATION     15000   // 15 seconds pump duration

// Telemetry frequency in milliseconds
#define TELEMETRY_FREQUENCY_MILLISECS 30000

// Azure IoT Configuration
#define AZURE_SDK_CLIENT_USER_AGENT "c%2F" AZ_SDK_VERSION_STRING "(ard;esp32)"
#define sizeofarray(a) (sizeof(a) / sizeof(a[0]))
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define MQTT_QOS1 1
#define DO_NOT_RETAIN_MSG 0
#define SAS_TOKEN_DURATION_IN_MINUTES 60
#define UNIX_TIME_NOV_13_2017 1510592825
#define PST_TIME_ZONE -8
#define PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF 1
#define GMT_OFFSET_SECS (PST_TIME_ZONE * 3600)
#define GMT_OFFSET_SECS_DST ((PST_TIME_ZONE + PST_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF) * 3600)

#define E32_RX_PIN 32 // RX của ESP32 nhận từ TX của S3
#define E32_TX_PIN 33 

// Translate iot_configs.h defines into variables
static const char* ssid = IOT_CONFIG_WIFI_SSID;
static const char* password = IOT_CONFIG_WIFI_PASSWORD;
static const char* host = IOT_CONFIG_IOTHUB_FQDN;
static const char* mqtt_broker_uri = "mqtts://" IOT_CONFIG_IOTHUB_FQDN;
static const char* device_id = IOT_CONFIG_DEVICE_ID;
static const int mqtt_port = AZ_IOT_DEFAULT_MQTT_CONNECT_PORT;

// MQTT client & Azure Hub Client
static esp_mqtt_client_handle_t mqtt_client;
static az_iot_hub_client client;

static char mqtt_client_id[128];
static char mqtt_username[128];
static char mqtt_password[200];
static uint8_t sas_signature_buffer[256];
static unsigned long next_telemetry_send_time_ms = 0;
static char telemetry_topic[128];
static uint32_t telemetry_send_count = 0;
static String telemetry_payload = "{}";
int position ;
StaticJsonDocument<512> jsonDoc;

#define INCOMING_DATA_BUFFER_SIZE 128
static char incoming_data[INCOMING_DATA_BUFFER_SIZE];

// Semaphores
SemaphoreHandle_t wifiSemaphore = NULL;
SemaphoreHandle_t mqttSemaphore = NULL;
SemaphoreHandle_t dataSemaphore = NULL;  // Renamed from uartSemaphore

// Control variables
static bool pump_active = false;
static int active_valve = 0;
static unsigned long pump_start_time = 0;
static unsigned long next_test_data_time_ms = 0;

// Data structure for snore data
// typedef struct {
//     char timestamp[32];
//     int snore_intensity;
//     float duration;
//     int position;
// } SnoreData;

// static SnoreData currentSnoreData;

// SAS Token (Only if not using X.509)
#ifndef IOT_CONFIG_USE_X509_CERT
static AzIoTSasToken sasToken(
    &client,
    AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_KEY),
    AZ_SPAN_FROM_BUFFER(sas_signature_buffer),
    AZ_SPAN_FROM_BUFFER(mqtt_password));
#endif

// Function Prototypes
void wifiTask(void* pvParameters);
void mqttTask(void* pvParameters);
void testDataTask(void* pvParameters);  // Renamed from uartTask
void closeAllValves();
void setupHardware();
void connectToWiFi();
void initializeTime();
void initializeIoTHubClient();
int initializeMqttClient();
void inflateAirPillow(int position);
void deflateAirPillow();
void motorSetToInflate();
void motorSetToDeflate();
void motorStop();
void startMotor(uint8_t power);
//String createTelemetryPayload(const SnoreData* data);
// void sendTelemetry(const SnoreData* data);
// void generateTestData(SnoreData* data);
// void getCurrentTimeString(char* timeStr, size_t maxSize);

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial2.begin(115200, SERIAL_8N1, E32_RX_PIN, E32_TX_PIN);

    
    Logger.Info("Initializing system...");
    
    // Create semaphores
    wifiSemaphore = xSemaphoreCreateBinary();
    mqttSemaphore = xSemaphoreCreateBinary();
    dataSemaphore = xSemaphoreCreateBinary();
    
    // Initialize hardware
    setupHardware();
    
    // Connect to network and initialize Azure IoT
    connectToWiFi();
    initializeTime();
    initializeIoTHubClient();
    initializeMqttClient();
    
    // Create Azure IoT Hub telemetry topic string
    az_span telemetry_topic_span = AZ_SPAN_LITERAL_FROM_STR("devices/");
    az_span device_id_span = az_span_create((uint8_t*)device_id, strlen(device_id));
    az_span telemetry_topic_event_span = AZ_SPAN_LITERAL_FROM_STR("/messages/events/");
    az_span telemetry_topic_buffer_span = az_span_create((uint8_t*)telemetry_topic, sizeof(telemetry_topic));
    az_span_copy(telemetry_topic_buffer_span, telemetry_topic_span);
    az_span remainder = az_span_copy(telemetry_topic_buffer_span, telemetry_topic_span);
    remainder = az_span_copy(remainder, device_id_span);
    remainder = az_span_copy(remainder, telemetry_topic_event_span);
    az_span_copy_u8(remainder, 0); // Null-terminate with a single byte
    
    // Initialize current time for telemetry and test data
    next_telemetry_send_time_ms = millis();
    next_test_data_time_ms = millis();
    
    // Initialize default snore data
    // getCurrentTimeString(currentSnoreData.timestamp, sizeof(currentSnoreData.timestamp));
    // currentSnoreData.snore_intensity = 0;
    // currentSnoreData.duration = 0.0;
    // currentSnoreData.position = 0;
    
    // Create tasks with appropriate priorities
    xTaskCreate(wifiTask, "WiFiTask", WIFI_TASK_STACK_SIZE, NULL, 3, NULL);
    //xTaskCreate(mqttTask, "MQTTTask", MQTT_TASK_STACK_SIZE, NULL, 2, NULL);
//    xTaskCreate(testDataTask, "TestDataTask", TEST_DATA_TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreatePinnedToCore(
        uartTask,           // Hàm thực thi
        "UART_Task",        // Tên task
        4096,               // Stack size
        NULL,               // Tham số truyền vào
        1,                  // Priority
        NULL,               // Task handle
        1                   // Core dùng (core 1)
    );
    
    xSemaphoreGive(dataSemaphore);
    
    Logger.Info("System initialization complete");
}

void loop() {
    // Main loop is empty as tasks handle all the work
    delay(1000);
}

void setupHardware() {
    Logger.Info("Initializing H-Bridge and valves...");
    
    // Initialize PWM for motor control using standard Arduino functions
    pinMode(MOTOR_ENB_PIN, OUTPUT);
    
    // Set H-bridge control pins as outputs
    pinMode(MOTOR_IN3_PIN, OUTPUT);
    pinMode(MOTOR_IN4_PIN, OUTPUT);
    
    // Set all valve pins as outputs
    pinMode(VALVE1_PIN, OUTPUT);
    pinMode(VALVE2_PIN, OUTPUT);
    pinMode(VALVE3_PIN, OUTPUT);
    pinMode(VALVE4_PIN, OUTPUT);
    pinMode(VALVE5_PIN, OUTPUT);
    
    // Initialize motor to OFF state
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, LOW);
    ledcWrite(MOTOR_PWM_CHANNEL, 0);
    
    // Close all valves
    closeAllValves();
    
    Logger.Info("Hardware initialization complete");
}

void closeAllValves() {
    digitalWrite(VALVE1_PIN, LOW);
    digitalWrite(VALVE2_PIN, LOW);
    digitalWrite(VALVE3_PIN, LOW);
    digitalWrite(VALVE4_PIN, LOW);
    digitalWrite(VALVE5_PIN, LOW);
    active_valve = 0;
    
    Logger.Info("All valves closed");
}

// WiFi connection
void connectToWiFi() {
    Logger.Info("Connecting to WIFI SSID " + String(ssid));
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);
    
    int wifi_retry = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_retry < 10) {
        delay(500);
        Serial.print(".");
        wifi_retry++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Logger.Info("WiFi connected, IP address: " + WiFi.localIP().toString());
        xSemaphoreGive(wifiSemaphore);
    } else {
        Logger.Error("Failed to connect to WiFi");
    }
}

void initializeTime() {
    Logger.Info("Setting time using SNTP");
    
    configTime(GMT_OFFSET_SECS, GMT_OFFSET_SECS_DST, NTP_SERVERS);
    time_t now = time(NULL);
    while (now < UNIX_TIME_NOV_13_2017) {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println("");
    Logger.Info("Time initialized!");
}

// // Get current time as string
// void getCurrentTimeString(char* timeStr, size_t maxSize) {
//     time_t now;
//     struct tm timeinfo;
    
//     time(&now);
//     localtime_r(&now, &timeinfo);
    
//     strftime(timeStr, maxSize, "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
// }

String getCurrentTimestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      return "TIME_ERROR";
    }
    char timeStringBuff[30]; // Đủ chỗ cho YYYY-MM-DDTHH:MM:SSZ
    // strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%SZ", &timeinfo); // UTC time
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%S", &timeinfo); // Local time (theo offset)
    return String(timeStringBuff);
  }

// MQTT Event Handler
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    (void)handler_args;
    (void)base;
    (void)event_id;
    
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
#else // ESP_ARDUINO_VERSION_MAJOR
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
#endif // ESP_ARDUINO_VERSION_MAJOR
    switch (event->event_id) {
        int i, r;
        
        case MQTT_EVENT_ERROR:
            Logger.Info("MQTT event MQTT_EVENT_ERROR");
            break;
        case MQTT_EVENT_CONNECTED:
            Logger.Info("MQTT event MQTT_EVENT_CONNECTED");
            
            r = esp_mqtt_client_subscribe(mqtt_client, AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC, 1);
            if (r == -1) {
                Logger.Error("Could not subscribe for cloud-to-device messages.");
            } else {
                Logger.Info("Subscribed for cloud-to-device messages; message id:" + String(r));
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            Logger.Info("MQTT event MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            Logger.Info("MQTT event MQTT_EVENT_SUBSCRIBED");
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            Logger.Info("MQTT event MQTT_EVENT_UNSUBSCRIBED");
            break;
        case MQTT_EVENT_PUBLISHED:
            Logger.Info("MQTT event MQTT_EVENT_PUBLISHED");
            break;
        case MQTT_EVENT_DATA:
            Logger.Info("MQTT event MQTT_EVENT_DATA");
            
            for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->topic_len; i++) {
                incoming_data[i] = event->topic[i];
            }
            incoming_data[i] = '\0';
            Logger.Info("Topic: " + String(incoming_data));
            
            for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->data_len; i++) {
                incoming_data[i] = event->data[i];
            }
            incoming_data[i] = '\0';
            Logger.Info("Data: " + String(incoming_data));
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            Logger.Info("MQTT event MQTT_EVENT_BEFORE_CONNECT");
            break;
        default:
            Logger.Error("MQTT event UNKNOWN");
            break;
    }
    
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#else // ESP_ARDUINO_VERSION_MAJOR
    return ESP_OK;
#endif // ESP_ARDUINO_VERSION_MAJOR
}

void initializeIoTHubClient() {
    az_iot_hub_client_options options = az_iot_hub_client_options_default();
    options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);
    
    if (az_result_failed(az_iot_hub_client_init(
            &client,
            az_span_create((uint8_t*)host, strlen(host)),
            az_span_create((uint8_t*)device_id, strlen(device_id)),
            &options))) {
        Logger.Error("Failed initializing Azure IoT Hub client");
        return;
    }
    
    size_t client_id_length;
    if (az_result_failed(az_iot_hub_client_get_client_id(
            &client, mqtt_client_id, sizeof(mqtt_client_id) - 1, &client_id_length))) {
        Logger.Error("Failed getting client id");
        return;
    }
    
    if (az_result_failed(az_iot_hub_client_get_user_name(
            &client, mqtt_username, sizeofarray(mqtt_username), NULL))) {
        Logger.Error("Failed to get MQTT clientId, return code");
        return;
    }
    
    Logger.Info("Client ID: " + String(mqtt_client_id));
    Logger.Info("Username: " + String(mqtt_username));
}

int initializeMqttClient() {
#ifndef IOT_CONFIG_USE_X509_CERT
    if (sasToken.Generate(SAS_TOKEN_DURATION_IN_MINUTES) != 0) {
        Logger.Error("Failed generating SAS token");
        return 1;
    }
#endif
    
    esp_mqtt_client_config_t mqtt_config;
    memset(&mqtt_config, 0, sizeof(mqtt_config));
    
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    mqtt_config.broker.address.uri = mqtt_broker_uri;
    mqtt_config.broker.address.port = mqtt_port;
    mqtt_config.credentials.client_id = mqtt_client_id;
    mqtt_config.credentials.username = mqtt_username;
    
#ifdef IOT_CONFIG_USE_X509_CERT
    Logger.Info("MQTT client using X509 Certificate authentication");
    mqtt_config.credentials.authentication.certificate = IOT_CONFIG_DEVICE_CERT;
    mqtt_config.credentials.authentication.certificate_len = (size_t)sizeof(IOT_CONFIG_DEVICE_CERT);
    mqtt_config.credentials.authentication.key = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
    mqtt_config.credentials.authentication.key_len = (size_t)sizeof(IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY);
#else // Using SAS key
    mqtt_config.credentials.authentication.password = (const char*)az_span_ptr(sasToken.Get());
#endif
    
    mqtt_config.session.keepalive = 30;
    mqtt_config.session.disable_clean_session = 0;
    mqtt_config.network.disable_auto_reconnect = false;
    mqtt_config.broker.verification.certificate = (const char*)ca_pem;
    mqtt_config.broker.verification.certificate_len = (size_t)ca_pem_len;
#else // ESP_ARDUINO_VERSION_MAJOR  
    mqtt_config.uri = mqtt_broker_uri;
    mqtt_config.port = mqtt_port;
    mqtt_config.client_id = mqtt_client_id;
    mqtt_config.username = mqtt_username;
    
#ifdef IOT_CONFIG_USE_X509_CERT
    Logger.Info("MQTT client using X509 Certificate authentication");
    mqtt_config.client_cert_pem = IOT_CONFIG_DEVICE_CERT;
    mqtt_config.client_key_pem = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
#else // Using SAS key
    mqtt_config.password = (const char*)az_span_ptr(sasToken.Get());
#endif
    
    mqtt_config.keepalive = 30;
    mqtt_config.disable_clean_session = 0;
    mqtt_config.disable_auto_reconnect = false;
    mqtt_config.event_handle = mqtt_event_handler;
    mqtt_config.user_context = NULL;
    mqtt_config.cert_pem = (const char*)ca_pem;
#endif // ESP_ARDUINO_VERSION_MAJOR
    
    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    
    if (mqtt_client == NULL) {
        Logger.Error("Failed creating mqtt client");
        return 1;
    }
    
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
#endif // ESP_ARDUINO_VERSION_MAJOR
    
    esp_err_t start_result = esp_mqtt_client_start(mqtt_client);
    
    if (start_result != ESP_OK) {
        Logger.Error("Could not start mqtt client; error code:" + start_result);
        return 1;
    } else {
        Logger.Info("MQTT client started");
        xSemaphoreGive(mqttSemaphore);
        return 0;
    }
}

// Motor and valve control functions
void motorSetToInflate() {
    digitalWrite(MOTOR_IN3_PIN, HIGH);
    digitalWrite(MOTOR_IN4_PIN, LOW);
    Logger.Info("Motor set to inflate");
}

void motorSetToDeflate() {
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, HIGH);
    Logger.Info("Motor set to deflate");
}

void motorStop() {
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, LOW);
    ledcWrite(MOTOR_PWM_CHANNEL, 0);
    
    // Close valve 5 whenever motor stops
    digitalWrite(VALVE5_PIN, LOW);
    
    pump_active = false;
    
    Logger.Info("Motor stopped and valve 5 closed");
}

void startMotor(uint8_t power = MOTOR_SPEED) {
    // Open valve 5 whenever motor starts
    digitalWrite(VALVE5_PIN, HIGH);
    
    // Start the motor
    ledcWrite(MOTOR_PWM_CHANNEL, power);
    
    pump_active = true;
    pump_start_time = millis();
    
    Logger.Info("Motor started with valve 5 open, power: " + String(power));
}

void inflateAirPillow(int position) {
    if (position < 1 || position > 4) {
        Logger.Error("Invalid position for air pillow inflation: " + String(position));
        return;
    }
    
    // Close any previously open valves
    closeAllValves();
    
    // Set active valve based on position
    active_valve = position;
    
    // Open the valve corresponding to the position
    switch (position) {
        case 1: 
            digitalWrite(VALVE1_PIN, HIGH); 
            Logger.Info("Opened valve 1");
            break;
        case 2: 
            digitalWrite(VALVE2_PIN, HIGH); 
            Logger.Info("Opened valve 2");
            break;
        case 3: 
            digitalWrite(VALVE3_PIN, HIGH); 
            Logger.Info("Opened valve 3");
            break;
        case 4: 
            digitalWrite(VALVE4_PIN, HIGH); 
            Logger.Info("Opened valve 4");
            break;
    }
    
    // Set motor direction for inflation
    motorSetToInflate();
    
    // Start the motor
    startMotor();
    
    Logger.Info("Inflating air pillow at position: " + String(position));
}

void deflateAirPillow() {
    // Set motor direction for deflation
    motorSetToDeflate();
    
    // Start the motor
    startMotor();
    
    // Wait for some deflation to occur
    delay(3000);
    
    // Close all valves and stop the motor
    closeAllValves();
    motorStop();
    
    Logger.Info("Deflated air pillow");
}

// Hàm tạo payload JSON đúng định dạng ở đây



// Send telemetry to Azure IoT Hub
void sendTelemetry(String json) {
    if (WiFi.status() != WL_CONNECTED) {
        Logger.Error("WiFi not connected, skipping telemetry");
        return;
    }
    
    Logger.Info("Sending telemetry...");
    
    String payload = json;  // load json created 
    Logger.Info("Payload: " + payload);
    
    int payload_size = payload.length();
    
    if (xSemaphoreTake(mqttSemaphore, pdMS_TO_TICKS(2000)) == pdTRUE) {
        if (esp_mqtt_client_publish(
            mqtt_client,
            telemetry_topic,
            payload.c_str(),
            payload_size,
            MQTT_QOS1,
            DO_NOT_RETAIN_MSG) == 0) {
            Logger.Error("Failed publishing telemetry");
        } else {
            Logger.Info("Telemetry published successfully");
            telemetry_send_count++;
        }
        xSemaphoreGive(mqttSemaphore);
    } else {
        Logger.Error("Failed to get MQTT semaphore for telemetry");
    }
    Logger.Info("\nda goi data");
}

// WiFi Task: Manages WiFi connection
void wifiTask(void* pvParameters) {
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            Logger.Info("WiFi disconnected, attempting reconnection");
            connectToWiFi();
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // Check WiFi every 10 seconds
    }
}

// MQTT Task: Manages MQTT connection and token renewal
// void mqttTask(void* pvParameters) {
//     for (;;) {
//         if (xSemaphoreTake(mqttSemaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
//             #ifndef IOT_CONFIG_USE_X509_CERT
//             if (sasToken.IsExpired()) {
//                 Logger.Info("SAS token expired; generating new token");
//                 sasToken.Generate(SAS_TOKEN_DURATION_IN_MINUTES);
//                 esp_mqtt_client_destroy(mqtt_client);
//                 initializeMqttClient();
//             }
//             #endif
            
//             // Check if it's time to send telemetry data
//             unsigned long current_time = millis();
//             if (current_time - next_telemetry_send_time_ms >= TELEMETRY_FREQUENCY_MILLISECS) {
//                 next_telemetry_send_time_ms = current_time;
                
//                 // Send the last received/generated data
//                 if (xSemaphoreTake(dataSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
//                     sendTelemetry(&currentSnoreData);
//                     xSemaphoreGive(dataSemaphore);
//                 }
//             }
            
//             xSemaphoreGive(mqttSemaphore);
//         }
        
//         // Check if we need to stop the pump after the duration
//         if (pump_active && (millis() - pump_start_time >= PUMP_DURATION)) {
//             motorStop();
//             closeAllValves();
//             Logger.Info("Pump duration reached, stopping pump");
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
//     }
// }


void uartTask(void* pvParameters) {
    while (1) {
      if (Serial2.available()) {
        delay(50); // Đợi buffer ổn định
        int available_bytes = Serial2.available();
  
        if (available_bytes == 1) {
          uint8_t signal = Serial2.read();
          Serial.print("Received Signal: ");
          Serial.println(signal);
  
          if (signal == 1) {
            Serial.println("Snoring continues, waiting for the next 20 seconds...");
          } else if (signal == 0) {
            Serial.println("stop pumping");
  
            String snoring_endtime = getCurrentTimestamp();
            jsonDoc["snoringEndtime"] = snoring_endtime;
            jsonDoc["intervention"]["endTime"] = snoring_endtime;
  
            String finalJson;
            serializeJson(jsonDoc, finalJson);
            Serial.println("Final JSON: " + finalJson);
  
            // Cờ báo JSON đã sẵn sàng
            //json_ready_to_send = true;
  
            // Gửi dữ liệu ngay
            sendTelemetry(finalJson);
            inflateAirPillow(position);
          } else {
            Serial.println("Invalid signal received.");
          }
  
          while (Serial2.available()) {
            Serial2.read(); // Clear buffer
          }
        } else {
          // Nhận chuỗi mới (deviceId,score,position)
          String receivedData = Serial2.readStringUntil('\n');
          if (receivedData.length() == 0) {
            Serial.println("Empty data received.");
            continue;
          }
  
          Serial.println("Received Data: " + receivedData);
          int firstComma = receivedData.indexOf(',');
          int secondComma = receivedData.indexOf(',', firstComma + 1);
          if (firstComma == -1 || secondComma == -1) {
            Serial.println("Invalid data format (missing fields)");
            continue;
          }
  
          String device_id = receivedData.substring(0, firstComma);
          String scoreStr = receivedData.substring(firstComma + 1, secondComma);
          String positionStr = receivedData.substring(secondComma + 1);
          int score = scoreStr.toInt();
        position = positionStr.toInt();
  
          jsonDoc.clear();
          jsonDoc["deviceId"] = device_id;
          String current_time_stamp = getCurrentTimestamp();
          jsonDoc["timestamp"] = current_time_stamp;
          jsonDoc["snoringLevel"] = score;
          JsonObject intervention = jsonDoc.createNestedObject("intervention");
          intervention["active"] = true;
          intervention["startTime"] = current_time_stamp;
  
          String updatedJson;
          serializeJson(jsonDoc, updatedJson);
          Serial.println("Updated JSON: " + updatedJson);
          Serial.println("position at: " + positionStr);
  
          Serial2.println(updatedJson);
        }
      }
      vTaskDelay(10 / portTICK_PERIOD_MS); // Nhường CPU cho task khác
    }
  }
  

