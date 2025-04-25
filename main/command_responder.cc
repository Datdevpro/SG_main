/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "command_responder.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "data_sender.h"
#include "esp_timer.h"

// The default implementation writes out the name of the recognized command
// to the error console. Real applications will want to take some custom
// action instead, and should implement their own versions of this function.

uint64_t millis() {   // add deplay 2s when detected snoring for 
                      //prevent multiple event in a few seconds
  return esp_timer_get_time() / 2000;
}

void RespondToCommand(int32_t current_time, const char* found_command,
                      int16_t score, bool is_new_command) {
  // static unsigned long lastcommandtime = 0;
  // if (is_new_command && (millis() - lastcommandtime > 2000)) {
  //   lastcommandtime = millis();

  //   MicroPrintf("Heard %s (%d) at %dms", found_command, score, current_time);
  //   uart_send_json(1, found_command, score/15);
  //   uart_receive_response();
  // }
  //if ( is_new_command && score > 128)
  if (score > 128)
  {
      MicroPrintf("Heard %s (%d) at %dms", found_command, score, current_time);
      //uart_send_json(1, found_command, score/15);
      send_flag_start_time(true);
      uart_receive_response();
    }

   else {
      //is_new_command = false; 
      send_flag_start_time(false); // then pack the json
      uart_receive_response();
    }
  
  

}
