import tensorflow as tf
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
import pickle

class SnoreCNNClassifier:
    def __init__(self, input_shape=(40, 101, 1), num_classes=2):
        """
        CNN Classifier cho phân loại tiếng ngáy
        
        Args:
            input_shape: Shape của log-mel spectrogram (n_mels, n_frames, channels)
            num_classes: Số lượng classes
        """
        self.input_shape = input_shape
        self.num_classes = num_classes
        self.model = None
        self.label_names = None
        
    def build_model(self):
        """Xây dựng CNN model nhẹ cho ESP32"""
        model = tf.keras.Sequential([
            # Input layer
            tf.keras.layers.Input(shape=self.input_shape),
            
            # Conv2D layers
            tf.keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same'),
            tf.keras.layers.MaxPooling2D((2, 2)),
            tf.keras.layers.Dropout(0.2),
            
            tf.keras.layers.Conv2D(32, (3, 3), activation='relu', padding='same'),
            tf.keras.layers.MaxPooling2D((2, 2)),
            tf.keras.layers.Dropout(0.2),
            
            tf.keras.layers.Conv2D(32, (3, 3), activation='relu', padding='same'),
            tf.keras.layers.GlobalAveragePooling2D(),
            
            # Dense layers
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dropout(0.5),
            tf.keras.layers.Dense(self.num_classes, activation='softmax')
        ])
        
        model.compile(
            optimizer='adam',
            loss='sparse_categorical_crossentropy',
            metrics=['accuracy']
        )
        
        self.model = model
        return model
    
    def prepare_data(self, features, labels):
        """Chuẩn bị dữ liệu cho CNN"""
        # Thêm channel dimension nếu cần
        if len(features.shape) == 3:
            features = np.expand_dims(features, axis=-1)
        
        # Chuẩn hóa dữ liệu về [0, 1]
        features = (features - np.min(features)) / (np.max(features) - np.min(features))
        
        return features.astype('float32'), labels.astype('int32')
    
    def train(self, X_train, y_train, X_val=None, y_val=None, 
              epochs=50, batch_size=32, label_names=None):
        """Train model"""
        
        if self.model is None:
            self.build_model()
        
        self.label_names = label_names if label_names else ['Class 0', 'Class 1']
        
        # Chuẩn bị dữ liệu
        X_train, y_train = self.prepare_data(X_train, y_train)
        
        if X_val is not None and y_val is not None:
            X_val, y_val = self.prepare_data(X_val, y_val)
            validation_data = (X_val, y_val)
        else:
            validation_data = None
        
        # Callbacks
        callbacks = [
            tf.keras.callbacks.EarlyStopping(
                monitor='val_accuracy' if validation_data else 'accuracy',
                patience=10,
                restore_best_weights=True
            ),
            tf.keras.callbacks.ReduceLROnPlateau(
                monitor='val_loss' if validation_data else 'loss',
                factor=0.5,
                patience=5,
                min_lr=1e-7
            )
        ]
        
        # Train
        history = self.model.fit(
            X_train, y_train,
            epochs=epochs,
            batch_size=batch_size,
            validation_data=validation_data,
            callbacks=callbacks,
            verbose=1
        )
        
        return history
    
    def evaluate(self, X_test, y_test):
        """Đánh giá model"""
        X_test, y_test = self.prepare_data(X_test, y_test)
        
        # Dự đoán
        predictions = self.model.predict(X_test)
        predicted_classes = np.argmax(predictions, axis=1)
        
        # Tính accuracy
        accuracy = accuracy_score(y_test, predicted_classes)
        
        print(f"Test Accuracy: {accuracy:.4f}")
        print("\nClassification Report:")
        print(classification_report(y_test, predicted_classes, target_names=self.label_names))
        
        # Confusion Matrix
        cm = confusion_matrix(y_test, predicted_classes)
        plt.figure(figsize=(8, 6))
        sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', 
                    xticklabels=self.label_names, yticklabels=self.label_names)
        plt.title('Confusion Matrix')
        plt.ylabel('True Label')
        plt.xlabel('Predicted Label')
        plt.show()
        
        return accuracy
    
    def predict(self, X, return_probabilities=False):
        """Dự đoán"""
        X, _ = self.prepare_data(X, np.zeros(len(X)))
        
        predictions = self.model.predict(X)
        predicted_classes = np.argmax(predictions, axis=1)
        
        if return_probabilities:
            return predicted_classes, predictions
        return predicted_classes
    
    def predict_single(self, single_feature):
        """Dự đoán cho một sample"""
        if len(single_feature.shape) == 2:
            single_feature = np.expand_dims(single_feature, axis=0)
        
        prediction, probability = self.predict(single_feature, return_probabilities=True)
        return prediction[0], probability[0]
    
    def save_model(self, save_path):
        """Lưu model"""
        # Lưu model
        self.model.save(save_path + '.h5')
        
        # Lưu metadata
        metadata = {
            'input_shape': self.input_shape,
            'num_classes': self.num_classes,
            'label_names': self.label_names
        }
        with open(save_path + '_metadata.pkl', 'wb') as f:
            pickle.dump(metadata, f)
        
        print(f"Model saved to {save_path}")
    
    def load_model(self, save_path):
        """Load model"""
        # Load model
        self.model = tf.keras.models.load_model(save_path + '.h5')
        
        # Load metadata
        with open(save_path + '_metadata.pkl', 'rb') as f:
            metadata = pickle.load(f)
        
        self.input_shape = metadata['input_shape']
        self.num_classes = metadata['num_classes'] 
        self.label_names = metadata['label_names']
        
        print(f"Model loaded from {save_path}")
    
    def convert_to_tflite(self, save_path, quantize=True):
        """Convert model sang TensorFlow Lite cho ESP32"""
        if self.model is None:
            raise ValueError("Model chưa được train!")
        
        converter = tf.lite.TFLiteConverter.from_keras_model(self.model)
        
        if quantize:
            # Quantization để giảm kích thước model
            converter.optimizations = [tf.lite.Optimize.DEFAULT]
            # Có thể thêm representative dataset để quantize tốt hơn
        
        tflite_model = converter.convert()
        
        # Lưu TFLite model
        with open(save_path + '.tflite', 'wb') as f:
            f.write(tflite_model)
        
        # Lưu model dạng C array cho ESP32
        with open(save_path + '_model.cc', 'w') as f:
            f.write('#include "model.h"\n\n')
            f.write('// Model data\n')
            f.write('const unsigned char model_data[] = {\n')
            
            # Chuyển bytes thành hex array
            hex_array = ', '.join([f'0x{byte:02x}' for byte in tflite_model])
            
            # Chia thành nhiều dòng cho dễ đọc
            lines = []
            for i in range(0, len(hex_array), 120):
                lines.append('  ' + hex_array[i:i+120])
            
            f.write(',\n'.join(lines))
            f.write('\n};\n\n')
            f.write(f'const int model_data_len = {len(tflite_model)};\n')
        
        print(f"TFLite model saved to {save_path}.tflite")
        print(f"C++ model file saved to {save_path}_model.cc")
        print(f"Model size: {len(tflite_model)} bytes")
        
        return tflite_model

def train_cnn_model():
    """Train CNN model cho snore classification"""
    
    # Load features
    with open('snore_features.pkl', 'rb') as f:
        data = pickle.load(f)
    
    features = data['features']
    labels = data['labels']
    label_names = data['label_names']
    
    print(f"Features shape: {features.shape}")
    print(f"Labels shape: {labels.shape}")
    
    # Split data
    X_train, X_temp, y_train, y_temp = train_test_split(
        features, labels, test_size=0.2, random_state=42, stratify=labels
    )
    X_val, X_test, y_val, y_test = train_test_split(
        X_temp, y_temp, test_size=0.5, random_state=42, stratify=y_temp
    )
    
    print(f"Train: {len(X_train)}, Val: {len(X_val)}, Test: {len(X_test)}")
    
    # Tạo và train model
    classifier = SnoreCNNClassifier(
        input_shape=(*features.shape[1:], 1),
        num_classes=len(label_names)
    )
    
    # Train
    history = classifier.train(
        X_train, y_train,
        X_val, y_val,
        epochs=50,
        batch_size=32,
        label_names=label_names
    )
    
    # Evaluate
    test_accuracy = classifier.evaluate(X_test, y_test)
    
    # Save model
    classifier.save_model('snore_cnn_model')
    
    # Convert to TFLite
    classifier.convert_to_tflite('snore_cnn_model', quantize=True)
    
    return classifier

if __name__ == "__main__":
    model = train_cnn_model() 