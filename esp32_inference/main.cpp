#include <Arduino.h>
#include "esp_log.h"
#include "driver/i2s.h"
#include "esp_dsp.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model.h"  // Model data từ file đã convert

static const char* TAG = "SNORE_DETECTION";

// ===== CẤU HÌNH AUDIO =====
#define SAMPLE_RATE 16000
#define I2S_NUM         I2S_NUM_0
#define I2S_BCK_PIN     GPIO_NUM_4
#define I2S_WS_PIN      GPIO_NUM_5
#define I2S_DATA_PIN    GPIO_NUM_18
#define I2S_BUFFER_LEN  1024

// ===== CẤU HÌNH MEL SPECTROGRAM =====
#define N_MELS          40
#define N_FFT           512
#define HOP_LENGTH      160
#define DURATION_SEC    1.0
#define N_SAMPLES       (int)(SAMPLE_RATE * DURATION_SEC)  // 16000 samples
#define N_FRAMES        ((N_SAMPLES - N_FFT) / HOP_LENGTH + 1)  // ~101 frames

// ===== CẤU HÌNH TENSORFLOW LITE =====
constexpr int kTensorArenaSize = 60 * 1024;  // 60KB cho tensor arena
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

// TensorFlow Lite components
tflite::MicroErrorReporter micro_error_reporter;
tflite::AllOpsResolver resolver;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;

// ===== AUDIO BUFFERS =====
int16_t audio_buffer[N_SAMPLES];
float mel_spectrogram[N_MELS][N_FRAMES];
float normalized_features[N_MELS * N_FRAMES];

// ===== I2S SETUP =====
void setup_i2s() {
    ESP_LOGI(TAG, "Setting up I2S for INMP441...");
    
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = I2S_BUFFER_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DATA_PIN
    };
    
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM, &pin_config));
    ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM));
    
    ESP_LOGI(TAG, "I2S setup complete");
}

// ===== TENSORFLOW LITE SETUP =====
void setup_tflite() {
    ESP_LOGI(TAG, "Setting up TensorFlow Lite...");
    
    // Load model
    model = tflite::GetModel(model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model version mismatch!");
        return;
    }
    
    // Create interpreter
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize, &micro_error_reporter);
    interpreter = &static_interpreter;
    
    // Allocate tensors
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        return;
    }
    
    // Check input tensor dimensions
    TfLiteTensor* input = interpreter->input(0);
    ESP_LOGI(TAG, "Input tensor dims: %d", input->dims->size);
    for (int i = 0; i < input->dims->size; i++) {
        ESP_LOGI(TAG, "  Dim %d: %d", i, input->dims->data[i]);
    }
    
    ESP_LOGI(TAG, "TensorFlow Lite setup complete");
}

// ===== CAPTURE AUDIO =====
bool capture_audio() {
    size_t bytes_read = 0;
    size_t total_bytes = N_SAMPLES * sizeof(int16_t);
    
    ESP_LOGI(TAG, "Capturing %d samples...", N_SAMPLES);
    
    esp_err_t result = i2s_read(I2S_NUM, audio_buffer, total_bytes, &bytes_read, portMAX_DELAY);
    
    if (result != ESP_OK || bytes_read != total_bytes) {
        ESP_LOGE(TAG, "I2S read failed: %s, bytes_read: %d", esp_err_to_name(result), bytes_read);
        return false;
    }
    
    ESP_LOGI(TAG, "Audio captured successfully");
    return true;
}

// ===== MEL SPECTROGRAM EXTRACTION (Simplified) =====
void extract_mel_features() {
    ESP_LOGI(TAG, "Extracting mel features...");
    
    // Đây là implementation đơn giản
    // Trong thực tế, bạn cần implement đầy đủ mel spectrogram extraction
    // hoặc dùng thư viện như ESP-DSP
    
    // Tạm thời tạo features giả để test
    for (int i = 0; i < N_MELS; i++) {
        for (int j = 0; j < N_FRAMES; j++) {
            // Compute simple features từ audio buffer
            float sum = 0;
            int start_idx = (j * HOP_LENGTH) + (i * 10);
            for (int k = 0; k < 32 && (start_idx + k) < N_SAMPLES; k++) {
                sum += abs(audio_buffer[start_idx + k]);
            }
            mel_spectrogram[i][j] = log(sum + 1.0) / 10.0;  // Simple log-mel approximation
        }
    }
    
    // Normalize features
    float mean = 0, variance = 0;
    int total_features = N_MELS * N_FRAMES;
    
    // Tính mean
    for (int i = 0; i < N_MELS; i++) {
        for (int j = 0; j < N_FRAMES; j++) {
            mean += mel_spectrogram[i][j];
        }
    }
    mean /= total_features;
    
    // Tính variance
    for (int i = 0; i < N_MELS; i++) {
        for (int j = 0; j < N_FRAMES; j++) {
            float diff = mel_spectrogram[i][j] - mean;
            variance += diff * diff;
        }
    }
    variance /= total_features;
    float std_dev = sqrt(variance + 1e-8);
    
    // Normalize và flatten
    int idx = 0;
    for (int i = 0; i < N_MELS; i++) {
        for (int j = 0; j < N_FRAMES; j++) {
            normalized_features[idx++] = (mel_spectrogram[i][j] - mean) / std_dev;
        }
    }
    
    ESP_LOGI(TAG, "Mel features extracted and normalized");
}

// ===== INFERENCE =====
void run_inference() {
    ESP_LOGI(TAG, "Running inference...");
    
    // Get input tensor
    TfLiteTensor* input = interpreter->input(0);
    
    // Copy features to input tensor
    if (input->type == kTfLiteFloat32) {
        float* input_data = input->data.f;
        for (int i = 0; i < N_MELS * N_FRAMES; i++) {
            input_data[i] = normalized_features[i];
        }
    } else {
        ESP_LOGE(TAG, "Input tensor type not supported");
        return;
    }
    
    // Run inference
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed");
        return;
    }
    
    // Get output
    TfLiteTensor* output = interpreter->output(0);
    float* output_data = output->data.f;
    
    float snore_confidence = output_data[0];      // snoring class
    float background_confidence = output_data[1]; // background class
    
    // Determine prediction
    bool is_snoring = snore_confidence > background_confidence;
    float max_confidence = max(snore_confidence, background_confidence);
    
    // Log results
    ESP_LOGI(TAG, "=== INFERENCE RESULT ===");
    ESP_LOGI(TAG, "Snoring: %.3f", snore_confidence);
    ESP_LOGI(TAG, "Background: %.3f", background_confidence);
    ESP_LOGI(TAG, "Prediction: %s (%.3f)", 
             is_snoring ? "SNORING" : "BACKGROUND", max_confidence);
    
    // Action based on prediction
    if (is_snoring && max_confidence > 0.7) {  // Threshold 70%
        ESP_LOGW(TAG, "🔴 SNORING DETECTED! 🔴");
        // Trigger action (LED, buzzer, send data, etc.)
    } else {
        ESP_LOGI(TAG, "✅ No snoring detected");
    }
}

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    ESP_LOGI(TAG, "Snore Detection System Starting...");
    
    // Setup I2S microphone
    setup_i2s();
    
    // Setup TensorFlow Lite
    setup_tflite();
    
    ESP_LOGI(TAG, "System ready!");
}

// ===== MAIN LOOP =====
void loop() {
    ESP_LOGI(TAG, "\n--- New Detection Cycle ---");
    
    // Capture audio
    if (!capture_audio()) {
        ESP_LOGE(TAG, "Failed to capture audio");
        delay(1000);
        return;
    }
    
    // Extract features
    extract_mel_features();
    
    // Run inference
    run_inference();
    
    // Wait before next detection
    delay(2000);  // 2 second interval
} 