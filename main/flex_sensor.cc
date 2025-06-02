#include "flex_sensor.h"
#include "driver/adc.h"
#include "esp_log.h"
#include <math.h>

#define FLEX_THRESHOLD 100
static const char *TAG = "FLEX_SENSOR";

// ADC configuration
static adc1_channel_t adc1_pins[3] = {ADC1_CHANNEL_3, ADC1_CHANNEL_4, ADC1_CHANNEL_6};  // GPIO39, GPIO32, GPIO34
static adc2_channel_t adc2_pin = ADC2_CHANNEL_6;  // GPIO14
static int baseline[4] = {0};
static int last_position = 0;

void flex_sensor_init(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Configure ADC1 channels
    for (int i = 0; i < 3; i++) {
        adc1_config_channel_atten(adc1_pins[i], ADC_ATTEN_DB_11);
        baseline[i] = adc1_get_raw(adc1_pins[i]);
    }
    
    // Configure ADC2 channel
    adc2_config_channel_atten(adc2_pin, ADC_ATTEN_DB_11);
    int value = 0;
    adc2_get_raw(adc2_pin, ADC_WIDTH_BIT_12, &value);
    baseline[3] = value;
    
    ESP_LOGI(TAG, "Initialized baseline for 4 flex sensors");
}

int flex_sensor_check(void)
{
    int current_position = 0;
    
    // Check ADC1 channels
    for (int i = 0; i < 3; i++) {
        int value = adc1_get_raw(adc1_pins[i]);
        if (abs(value - baseline[i]) > FLEX_THRESHOLD) {
            current_position = i + 1;
            break;
        }
    }
    
    // If no ADC1 sensor activated, check ADC2
    if (current_position == 0) {
        int value = 0;
        adc2_get_raw(adc2_pin, ADC_WIDTH_BIT_12, &value);
        if (abs(value - baseline[3]) > FLEX_THRESHOLD) {
            current_position = 4;
        }
    }
    
    // Only print if position changed
    if (current_position != last_position) {
        if (current_position > 0) {
            printf(">> Current active sensor: %d\n", current_position);
        }
        last_position = current_position;
    }
    
    return current_position;
}
