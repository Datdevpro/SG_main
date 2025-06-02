# Hệ thống phân loại tiếng ngáy với FAISS/CNN và ESP32-S3

## Tổng quan
Dự án này xây dựng hệ thống phân loại tiếng ngáy từ đầu, bao gồm:
- Trích xuất đặc trưng log-mel spectrogram
- Huấn luyện mô hình với FAISS (k-NN) hoặc CNN
- Deploy lên ESP32-S3 với microphone INMP441

## Cấu trúc thư mục
```
snore-recognition/
├── feature_extractor.py      # Trích xuất log-mel spectrogram
├── faiss_classifier.py       # Mô hình FAISS k-NN
├── cnn_classifier.py         # Mô hình CNN (khuyến nghị)
├── train_test_pipeline.py    # Pipeline training hoàn chỉnh
├── requirements.txt          # Dependencies
├── esp32_inference/          # Code ESP32
│   ├── main.cpp             # Code chính ESP32
│   ├── model.h              # Header file model
│   └── platformio.ini       # Cấu hình build
└── README_NEW.md            # Hướng dẫn này
```

## Bước 1: Chuẩn bị môi trường

### 1.1. Cài đặt Python dependencies
```bash
pip install -r requirements.txt
```

### 1.2. Chuẩn bị dataset
Tổ chức dataset theo cấu trúc:
```
your_dataset/
├── snoring/
│   ├── snore1.wav
│   ├── snore2.wav
│   └── ...
└── background/
    ├── noise1.wav
    ├── speech1.wav
    └── ...
```

## Bước 2: Training mô hình

### 2.1. Sử dụng pipeline hoàn chỉnh (Khuyến nghị)
```python
# Chỉnh sửa đường dẫn dataset trong train_test_pipeline.py
DATA_FOLDER = "path/to/your/dataset"

# Chạy training
python train_test_pipeline.py
```

### 2.2. Hoặc train từng bước

#### Trích xuất đặc trưng:
```python
from feature_extractor import LogMelFeatureExtractor

extractor = LogMelFeatureExtractor(
    sample_rate=16000,
    n_mels=40,
    duration=1.0
)

features, labels, label_names = extractor.extract_dataset_features(
    "path/to/dataset", 
    save_path="features.pkl"
)
```

#### Train với FAISS:
```python
from faiss_classifier import FAISSClassifier

classifier = FAISSClassifier(k=5)
classifier.train(X_train, y_train, label_names)
classifier.save_model('snore_faiss_model')
```

#### Train với CNN (Khuyến nghị cho ESP32):
```python
from cnn_classifier import SnoreCNNClassifier

classifier = SnoreCNNClassifier()
classifier.train(X_train, y_train, X_val, y_val)
classifier.save_model('snore_cnn_model')
classifier.convert_to_tflite('snore_cnn_model')  # Convert cho ESP32
```

## Bước 3: Deployment lên ESP32-S3

### 3.1. Chuẩn bị phần cứng
**Kết nối INMP441 với ESP32-S3:**
- VDD → 3.3V
- GND → GND  
- SCK/BCLK → GPIO 4
- WS/LRCK → GPIO 5
- SD/DATA → GPIO 18

### 3.2. Copy model data
Sau khi train CNN, file `snore_cnn_model_model.cc` sẽ được tạo.
Copy nội dung file này vào `esp32_inference/model.cc`

### 3.3. Build và upload
```bash
cd esp32_inference
pio run --target upload --target monitor
```

## Bước 4: Tối ưu hóa và cải thiện

### 4.1. Cải thiện độ chính xác
- **Augment dữ liệu:** Thêm nhiễu, thay đổi âm lượng, pitch
- **Tăng dữ liệu:** Thu thập thêm samples ở khoảng cách khác nhau
- **Fine-tune tham số:** Thử các giá trị k khác nhau (FAISS) hoặc architecture (CNN)

### 4.2. Xử lý nhiễu
```python
from audiomentations import Compose, AddBackgroundNoise, AddGaussianNoise

augmenter = Compose([
    AddBackgroundNoise(sounds_path="noise_folder", p=0.5),
    AddGaussianNoise(min_amplitude=0.001, max_amplitude=0.01, p=0.3)
])
```

### 4.3. Cải thiện feature extraction trên ESP32
Hiện tại ESP32 code dùng feature extraction đơn giản. Để cải thiện:
- Implement đầy đủ mel spectrogram với ESP-DSP
- Sử dụng pre-computed mel filter bank
- Tối ưu hóa FFT computation

## Bước 5: Testing và đánh giá

### 5.1. Test trên file mới
```python
from train_test_pipeline import test_on_new_audio

prediction, confidence = test_on_new_audio(
    "snore_cnn_model", 
    extractor_config, 
    "test_audio.wav"
)
```

### 5.2. Metrics đánh giá
- **Accuracy:** % dự đoán đúng
- **Precision/Recall:** Đặc biệt quan trọng cho class "snoring"
- **False Positive Rate:** Tránh báo động giả
- **Latency:** Thời gian xử lý trên ESP32

## So sánh FAISS vs CNN

| Tiêu chí | FAISS k-NN | CNN |
|----------|------------|-----|
| **Độ chính xác** | Tốt (85-90%) | Rất tốt (90-95%) |
| **Kích thước model** | Lớn (lưu toàn bộ training data) | Nhỏ (chỉ weights) |
| **Tốc độ inference** | Chậm hơn | Nhanh hơn |
| **Phù hợp ESP32** | Kém (cần nhiều RAM) | Tốt (TensorFlow Lite) |
| **Robustness** | Phụ thuộc training data | Tốt hơn với nhiễu |

**Khuyến nghị:** Sử dụng CNN cho deployment thực tế trên ESP32.

## Troubleshooting

### Lỗi thường gặp:
1. **Model quá lớn cho ESP32:** Giảm số neurons hoặc tăng quantization
2. **Accuracy thấp:** Cần thêm dữ liệu hoặc augment data
3. **False positives:** Điều chỉnh threshold detection
4. **I2S không hoạt động:** Kiểm tra wiring và cấu hình pin

### Debug tips:
- Log intermediate features để kiểm tra
- Test model trên PC trước khi deploy
- Sử dụng serial monitor để debug ESP32
- Kiểm tra audio input với oscilloscope

## Tài liệu tham khảo
- [TensorFlow Lite Micro](https://www.tensorflow.org/lite/microcontrollers)
- [ESP32-S3 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [INMP441 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/INMP441.pdf)
- [Mel Spectrogram Theory](https://en.wikipedia.org/wiki/Mel-frequency_cepstrum) 