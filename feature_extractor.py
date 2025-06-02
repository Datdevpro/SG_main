import librosa
import numpy as np
import os
from tqdm import tqdm
import pickle

class LogMelFeatureExtractor:
    def __init__(self, sample_rate=16000, n_mels=40, n_fft=512, hop_length=160, 
                 duration=1.0, normalize=True):
        """
        Log-Mel Spectrogram Feature Extractor
        
        Args:
            sample_rate: Tần số lấy mẫu (Hz)
            n_mels: Số mel filters
            n_fft: Kích thước FFT
            hop_length: Hop length cho STFT
            duration: Độ dài đoạn âm (giây)
            normalize: Có chuẩn hóa không
        """
        self.sample_rate = sample_rate
        self.n_mels = n_mels
        self.n_fft = n_fft
        self.hop_length = hop_length
        self.duration = duration
        self.normalize = normalize
        self.target_length = int(sample_rate * duration)
        
    def extract_features(self, audio_path):
        """Trích xuất log-mel spectrogram từ file âm thanh"""
        try:
            # Load audio file
            audio, sr = librosa.load(audio_path, sr=self.sample_rate)
            
            # Pad hoặc cắt audio về độ dài cố định
            if len(audio) > self.target_length:
                audio = audio[:self.target_length]
            else:
                audio = np.pad(audio, (0, self.target_length - len(audio)), mode='constant')
            
            # Tính mel spectrogram
            mel_spec = librosa.feature.melspectrogram(
                y=audio,
                sr=self.sample_rate,
                n_mels=self.n_mels,
                n_fft=self.n_fft,
                hop_length=self.hop_length
            )
            
            # Chuyển sang log scale
            log_mel_spec = librosa.power_to_db(mel_spec, ref=np.max)
            
            # Chuẩn hóa
            if self.normalize:
                log_mel_spec = (log_mel_spec - np.mean(log_mel_spec)) / (np.std(log_mel_spec) + 1e-8)
            
            return log_mel_spec
            
        except Exception as e:
            print(f"Error processing {audio_path}: {e}")
            return None
    
    def extract_dataset_features(self, data_folder, save_path=None):
        """
        Trích xuất đặc trưng cho toàn bộ dataset
        
        Args:
            data_folder: Thư mục chứa data với cấu trúc:
                data_folder/
                ├── snoring/
                │   ├── file1.wav
                │   └── file2.wav
                └── background/
                    ├── file1.wav
                    └── file2.wav
            save_path: Đường dẫn lưu features (optional)
        
        Returns:
            features: numpy array (n_samples, n_mels, n_frames)
            labels: numpy array (n_samples,)
            label_names: list of class names
        """
        features = []
        labels = []
        label_names = ['snoring', 'background']
        
        for label_idx, class_name in enumerate(label_names):
            class_folder = os.path.join(data_folder, class_name)
            if not os.path.exists(class_folder):
                print(f"Warning: {class_folder} not found!")
                continue
                
            audio_files = [f for f in os.listdir(class_folder) 
                          if f.endswith(('.wav', '.mp3', '.m4a', '.flac'))]
            
            print(f"Processing {len(audio_files)} files from {class_name}...")
            
            for audio_file in tqdm(audio_files):
                audio_path = os.path.join(class_folder, audio_file)
                feature = self.extract_features(audio_path)
                
                if feature is not None:
                    features.append(feature)
                    labels.append(label_idx)
        
        features = np.array(features)
        labels = np.array(labels)
        
        print(f"Extracted features shape: {features.shape}")
        print(f"Labels shape: {labels.shape}")
        print(f"Class distribution: {np.bincount(labels)}")
        
        if save_path:
            data_dict = {
                'features': features,
                'labels': labels,
                'label_names': label_names,
                'extractor_config': {
                    'sample_rate': self.sample_rate,
                    'n_mels': self.n_mels,
                    'n_fft': self.n_fft,
                    'hop_length': self.hop_length,
                    'duration': self.duration
                }
            }
            with open(save_path, 'wb') as f:
                pickle.dump(data_dict, f)
            print(f"Features saved to: {save_path}")
        
        return features, labels, label_names

# Ví dụ sử dụng
if __name__ == "__main__":
    # Tạo feature extractor
    extractor = LogMelFeatureExtractor(
        sample_rate=16000,
        n_mels=40,
        duration=1.0,
        normalize=True
    )
    
    # Trích xuất đặc trưng cho dataset
    # Thay đổi đường dẫn này theo dataset của bạn
    data_folder = "path/to/your/dataset"
    features, labels, label_names = extractor.extract_dataset_features(
        data_folder, 
        save_path="snore_features.pkl"
    ) 