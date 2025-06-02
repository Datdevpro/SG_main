#include "flex_sensor.h"
#include "driver/adc.h"
#include "esp_log.h"
#include <math.h>

#define FLEX_THRESHOLD 100
static const char *TAG = "FLEX_SENSOR";

// Tách riêng ADC1 và ADC2
static adc1_channel_t adc1_pins[3] = {ADC1_CHANNEL_3, ADC1_CHANNEL_4, ADC1_CHANNEL_6};
static adc2_channel_t adc2_pin = ADC2_CHANNEL_6;
static int baseline[4] = {0};
static int position = 0;

void flex_sensor_init(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Cấu hình và đọc ADC1
    for (int i = 0; i < 3; i++) {
        adc1_config_channel_atten(adc1_pins[i], ADC_ATTEN_DB_11);
        baseline[i] = adc1_get_raw(adc1_pins[i]);
    }
    
    // Cấu hình và đọc ADC2
    adc2_config_channel_atten(adc2_pin, ADC_ATTEN_DB_11);
    int value = 0;
    adc2_get_raw(adc2_pin, ADC_WIDTH_BIT_12, &value);
    baseline[3] = value;
    
    ESP_LOGI(TAG, "Initialized baseline for 4 flex sensors.");
}

int flex_sensor_check(void)
{
    // Kiểm tra ADC1
    for (int i = 0; i < 3; i++) {
        int value = adc1_get_raw(adc1_pins[i]);
        if (abs(value - baseline[i]) > FLEX_THRESHOLD) {
            if (position != i + 1) {
                position = i + 1;
                ESP_LOGI(TAG, "Sensor %d activated (value: %d)", position, value);
            }
            return position;
        }
    }
    
    // Kiểm tra ADC2
    int value = 0;
    adc2_get_raw(adc2_pin, ADC_WIDTH_BIT_12, &value);
    if (abs(value - baseline[3]) > FLEX_THRESHOLD) {
        if (position != 4) {
            position = 4;
            ESP_LOGI(TAG, "Sensor 4 activated (value: %d)", value);
        }
        return position;
    }
    
    return 0; // Không có cảm biến nào bị tác động
}

