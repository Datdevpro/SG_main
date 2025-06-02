#include "command_responder.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "data_sender.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "flex_sensor.h"
#include "esp_log.h"
// The default implementation writes out the name of the recognized command
// to the error console. Real applications will want to take some custom
// action instead, and should implement their own versions of this function.

static const char *TAG = "COMMAND";

uint64_t millis() {   // add deplay 2s when detected snoring for 
                      //prevent multiple event in a few seconds
  return esp_timer_get_time() / 1000;
}


static bool snore_detected = false; 
static unsigned last_snore_time = 0; // Timestamp of the last snore detection
bool waiting_for_second_snore = false;
static unsigned wait_snore_start_time = 0;
static int last_position = -1;
static bool sensor_initialized = false;


void RespondToCommand(int32_t current_time, const char* found_command,
                      int16_t score, bool is_new_command) {
    // Initialize flex sensor if not done yet
    if (!sensor_initialized) {
        flex_sensor_init();
        sensor_initialized = true;
    }

    // Check flex sensor with proper position validation
    int position = flex_sensor_check();
    if (position > 0) {  // Only consider valid positions (> 0)
        if (position != last_position) {  // Only print when position changes
            printf(">> Current active sensor: %d\n", position);
            last_position = position;
        }
    } else {
        last_position = -1;  // Reset last position when no valid position detected
    }

    // Check if the detected command is "snore" and it's a new command 
    if (is_new_command && !snore_detected) {
        if (!snore_detected || (millis() - last_snore_time > 20500)) {
            if (position <= 0) {  // Changed condition to match the example
                MicroPrintf("No flex sensor activated");
                return;
            }
            
            MicroPrintf("Heard %s (%d) @%dms at position %d", found_command, score, current_time, position);
            send_snore_json(score, position);
            MicroPrintf("sent data");
            snore_detected = true;
            last_snore_time = millis();
            MicroPrintf("Go to count down and Start Pumping");
        }
    }
  
    // After 20 seconds, check if snore is still detected
    if (snore_detected && (millis() - last_snore_time > 20500)) {
        if (!waiting_for_second_snore) {
            // Start the 3-second waiting window
            wait_snore_start_time = millis();
            waiting_for_second_snore = true;
            MicroPrintf("Waiting 5 seconds for another snore...");
        } else {
            // In 3-second window now
            if ((millis() - wait_snore_start_time) <= 5000) {
                if (is_new_command) {
                    MicroPrintf("Snore detected within 5 seconds, continue pumping.");
                    send_pumping_signal(1, position);  
                    last_snore_time = millis();  // Reset timer
                    waiting_for_second_snore = false;
                }
            } else {
                // 3 seconds passed, no new snore
                MicroPrintf("No snore in 5 seconds, stop pumping.");
                send_pumping_signal(0, position);
                snore_detected = false;
                waiting_for_second_snore = false;
                MicroPrintf("Waiting for snore...");
            }
        }
    }
}
  
