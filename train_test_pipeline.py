import os
import numpy as np
import pickle
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report
import matplotlib.pyplot as plt
import seaborn as sns

from feature_extractor import LogMelFeatureExtractor
from faiss_classifier import FAISSClassifier

def main():
    """Pipeline hoàn chỉnh từ raw audio đến trained model"""
    
    # ===== BƯỚC 1: TRÍCH XUẤT ĐẶC TRƯNG =====
    print("="*50)
    print("BƯỚC 1: TRÍCH XUẤT ĐẶC TRƯNG")
    print("="*50)
    
    # Cấu hình đường dẫn
    DATA_FOLDER = "path/to/your/dataset"  # Thay đổi đường dẫn này
    FEATURES_PATH = "snore_features.pkl"
    MODEL_PATH = "snore_faiss_model"
    
    # Tạo feature extractor
    extractor = LogMelFeatureExtractor(
        sample_rate=16000,  # Phù hợp với ESP32
        n_mels=40,          # Số mel filters
        n_fft=512,          # FFT size
        hop_length=160,     # Hop length
        duration=1.0,       # 1 giây mỗi đoạn
        normalize=True      # Chuẩn hóa đặc trưng
    )
    
    # Kiểm tra xem features đã được trích xuất chưa
    if not os.path.exists(FEATURES_PATH):
        print("Đang trích xuất đặc trưng từ dataset...")
        features, labels, label_names = extractor.extract_dataset_features(
            DATA_FOLDER, 
            save_path=FEATURES_PATH
        )
    else:
        print("Loading features đã có sẵn...")
        with open(FEATURES_PATH, 'rb') as f:
            data = pickle.load(f)
        features = data['features']
        labels = data['labels']
        label_names = data['label_names']
    
    print(f"Features shape: {features.shape}")
    print(f"Labels shape: {labels.shape}")
    print(f"Classes: {label_names}")
    print(f"Class distribution: {np.bincount(labels)}")
    
    # ===== BƯỚC 2: CHIA DỮ LIỆU TRAIN/VALIDATION/TEST =====
    print("\n" + "="*50)
    print("BƯỚC 2: CHIA DỮ LIỆU")
    print("="*50)
    
    # Chia train/temp (80/20)
    X_train, X_temp, y_train, y_temp = train_test_split(
        features, labels, 
        test_size=0.2, 
        random_state=42, 
        stratify=labels
    )
    
    # Chia temp thành validation/test (10/10)
    X_val, X_test, y_val, y_test = train_test_split(
        X_temp, y_temp, 
        test_size=0.5, 
        random_state=42, 
        stratify=y_temp
    )
    
    print(f"Train set: {len(X_train)} samples")
    print(f"Validation set: {len(X_val)} samples") 
    print(f"Test set: {len(X_test)} samples")
    
    # ===== BƯỚC 3: TRAIN MÔ HÌNH FAISS =====
    print("\n" + "="*50)
    print("BƯỚC 3: TRAIN MÔ HÌNH FAISS")
    print("="*50)
    
    # Thử nghiệm với các giá trị k khác nhau
    k_values = [3, 5, 7, 9]
    best_k = 5
    best_val_acc = 0
    
    print("Tìm kiếm k tối ưu...")
    for k in k_values:
        classifier = FAISSClassifier(k=k, metric='L2')
        classifier.train(X_train, y_train, label_names)
        
        # Đánh giá trên validation set
        val_predictions = classifier.predict(X_val)
        val_accuracy = accuracy_score(y_val, val_predictions)
        
        print(f"k={k}: Validation Accuracy = {val_accuracy:.4f}")
        
        if val_accuracy > best_val_acc:
            best_val_acc = val_accuracy
            best_k = k
    
    print(f"\nBest k: {best_k} (Validation Accuracy: {best_val_acc:.4f})")
    
    # ===== BƯỚC 4: TRAIN MÔ HÌNH CUỐI CÙNG =====
    print("\n" + "="*50)
    print("BƯỚC 4: TRAIN MÔ HÌNH CUỐI CÙNG")
    print("="*50)
    
    # Train lại với best k trên toàn bộ train + validation
    X_train_full = np.concatenate([X_train, X_val])
    y_train_full = np.concatenate([y_train, y_val])
    
    final_classifier = FAISSClassifier(k=best_k, metric='L2')
    final_classifier.train(X_train_full, y_train_full, label_names)
    
    # ===== BƯỚC 5: ĐÁNH GIÁ TRÊN TEST SET =====
    print("\n" + "="*50)
    print("BƯỚC 5: ĐÁNH GIÁ TRÊN TEST SET")
    print("="*50)
    
    test_accuracy = final_classifier.evaluate(X_test, y_test)
    
    # ===== BƯỚC 6: LÊM VÀ DEMO =====
    print("\n" + "="*50)
    print("BƯỚC 6: LƯU MÔ HÌNH VÀ DEMO")
    print("="*50)
    
    # Lưu mô hình
    final_classifier.save_model(MODEL_PATH)
    
    # Demo dự đoán trên một vài samples
    print("\nDemo dự đoán:")
    for i in range(min(5, len(X_test))):
        prediction, probability = final_classifier.predict_single(X_test[i])
        true_label = y_test[i]
        
        print(f"Sample {i+1}:")
        print(f"  True: {label_names[true_label]}")
        print(f"  Predicted: {label_names[prediction]}")
        print(f"  Confidence: {probability}")
        print(f"  Correct: {'✓' if prediction == true_label else '✗'}")
        print()
    
    # ===== BƯỚC 7: THÔNG TIN CHO ESP32 =====
    print("\n" + "="*50)
    print("THÔNG TIN CHO ESP32 DEPLOYMENT")
    print("="*50)
    
    print(f"Feature extractor config:")
    print(f"  - Sample rate: {extractor.sample_rate} Hz")
    print(f"  - N_mels: {extractor.n_mels}")
    print(f"  - N_fft: {extractor.n_fft}")
    print(f"  - Hop length: {extractor.hop_length}")
    print(f"  - Duration: {extractor.duration} seconds")
    print(f"  - Feature dimension: {final_classifier.feature_dim}")
    print(f"  - Model accuracy: {test_accuracy:.4f}")
    
    return final_classifier, extractor

def test_on_new_audio(model_path, extractor_config, audio_file_path):
    """Test mô hình trên file âm thanh mới"""
    
    # Load model
    classifier = FAISSClassifier()
    classifier.load_model(model_path)
    
    # Tạo feature extractor với config giống training
    extractor = LogMelFeatureExtractor(**extractor_config)
    
    # Trích xuất đặc trưng
    feature = extractor.extract_features(audio_file_path)
    
    if feature is not None:
        # Dự đoán
        prediction, probability = classifier.predict_single(feature)
        
        print(f"File: {audio_file_path}")
        print(f"Prediction: {classifier.label_names[prediction]}")
        print(f"Confidence: {probability}")
        
        return prediction, probability
    else:
        print(f"Lỗi khi xử lý file: {audio_file_path}")
        return None, None

if __name__ == "__main__":
    # Chạy pipeline training
    model, extractor = main()
    
    # Test trên file âm thanh mới (nếu có)
    # test_audio = "path/to/test/audio.wav"
    # if os.path.exists(test_audio):
    #     test_on_new_audio("snore_faiss_model", extractor.__dict__, test_audio) 