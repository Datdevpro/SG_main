
#include "command_responder.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "data_sender.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
// The default implementation writes out the name of the recognized command
// to the error console. Real applications will want to take some custom
// action instead, and should implement their own versions of this function.

uint64_t millis() {   // add deplay 2s when detected snoring for 
                      //prevent multiple event in a few seconds
  return esp_timer_get_time() / 1000;
}


static bool snore_detected = false; 
static unsigned last_snore_time = 0; // Timestamp of the last snore detection
bool waiting_for_second_snore = false;
static unsigned wait_snore_start_time = 0;



void RespondToCommand(int32_t current_time, const char* found_command,
                      int16_t score, bool is_new_command) {
      
  // Check if the detected command is "snore" and it's a new command 
  // here is the start of snoring detection period
  if (is_new_command && strcmp(found_command, "snoring" ) == 0 && !snore_detected) {
    if (!snore_detected || (millis() - last_snore_time > 20000)) {
      MicroPrintf("Heard %s (%d) @%dms", found_command, score, current_time);
      send_snore_json(score, 1);
      MicroPrintf("sent data");
      snore_detected = true;
      last_snore_time = millis();
      MicroPrintf("Go to count down and Start Pumping");
    }
  }
  
  // After 20 seconds, check if snore is still detected
  if (snore_detected && (millis() - last_snore_time > 20000)) {
    if (!waiting_for_second_snore) {
      // Start the 3-second waiting window
      wait_snore_start_time = millis();
      waiting_for_second_snore = true;
      MicroPrintf("Waiting 3 seconds for another snore...");
    } else {
      // In 3-second window now
      if ((millis() - wait_snore_start_time) <= 5000) {
        if (is_new_command ) {
          MicroPrintf("Snore detected within 3 seconds, continue pumping.");
          send_continue_pumping_signal();
          last_snore_time = millis();  // Reset timer
          waiting_for_second_snore = false;
        }
      } else {
        // 3 seconds passed, no new snore
        MicroPrintf("No snore in 3 seconds, stop pumping.");
        send_stop_pumping_signal();
        snore_detected = false;
        waiting_for_second_snore = false;
      }
    }
  }
}
  