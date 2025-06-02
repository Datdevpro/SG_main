import faiss
import numpy as np
import pickle
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns

class FAISSClassifier:
    def __init__(self, k=5, metric='L2'):
        """
        FAISS-based k-NN Classifier
        
        Args:
            k: Số lượng neighbors gần nhất
            metric: Distance metric ('L2' hoặc 'IP' - inner product)
        """
        self.k = k
        self.metric = metric
        self.index = None
        self.train_labels = None
        self.label_names = None
        self.feature_dim = None
        
    def _flatten_features(self, features):
        """Flatten features từ (n_samples, n_mels, n_frames) thành (n_samples, feature_dim)"""
        if len(features.shape) == 3:
            return features.reshape(features.shape[0], -1)
        return features
    
    def train(self, train_features, train_labels, label_names=None):
        """
        Train FAISS index với training data
        
        Args:
            train_features: numpy array (n_samples, n_mels, n_frames)
            train_labels: numpy array (n_samples,)
            label_names: list of class names
        """
        # Flatten features
        train_features_flat = self._flatten_features(train_features)
        self.feature_dim = train_features_flat.shape[1]
        
        # Chuẩn hóa features (quan trọng cho FAISS)
        train_features_flat = train_features_flat.astype('float32')
        
        # Tạo FAISS index
        if self.metric == 'L2':
            self.index = faiss.IndexFlatL2(self.feature_dim)
        elif self.metric == 'IP':
            self.index = faiss.IndexFlatIP(self.feature_dim)
            # Chuẩn hóa vector cho inner product
            faiss.normalize_L2(train_features_flat)
        else:
            raise ValueError("metric phải là 'L2' hoặc 'IP'")
        
        # Thêm vectors vào index
        self.index.add(train_features_flat)
        self.train_labels = train_labels
        self.label_names = label_names if label_names else ['Class 0', 'Class 1']
        
        print(f"FAISS index trained với {len(train_features)} samples")
        print(f"Feature dimension: {self.feature_dim}")
        
    def predict(self, test_features, return_probabilities=False):
        """
        Dự đoán labels cho test data
        
        Args:
            test_features: numpy array (n_samples, n_mels, n_frames)
            return_probabilities: Có trả về xác suất không
            
        Returns:
            predictions: numpy array (n_samples,)
            probabilities (optional): numpy array (n_samples, n_classes)
        """
        if self.index is None:
            raise ValueError("Model chưa được train!")
        
        # Flatten và chuẩn hóa features
        test_features_flat = self._flatten_features(test_features).astype('float32')
        
        if self.metric == 'IP':
            faiss.normalize_L2(test_features_flat)
        
        # Tìm k neighbors gần nhất
        distances, indices = self.index.search(test_features_flat, self.k)
        
        # Voting dựa trên labels của k neighbors
        predictions = []
        probabilities = []
        
        for i in range(len(test_features_flat)):
            neighbor_labels = self.train_labels[indices[i]]
            
            # Đếm votes cho mỗi class
            unique_labels, counts = np.unique(neighbor_labels, return_counts=True)
            
            # Predicted class là class có nhiều votes nhất
            predicted_class = unique_labels[np.argmax(counts)]
            predictions.append(predicted_class)
            
            if return_probabilities:
                # Tính probability dựa trên vote ratio
                prob = np.zeros(len(self.label_names))
                for label, count in zip(unique_labels, counts):
                    prob[label] = count / self.k
                probabilities.append(prob)
        
        predictions = np.array(predictions)
        
        if return_probabilities:
            return predictions, np.array(probabilities)
        return predictions
    
    def predict_single(self, single_feature):
        """Dự đoán cho một sample duy nhất"""
        single_feature = single_feature.reshape(1, *single_feature.shape)
        prediction, probability = self.predict(single_feature, return_probabilities=True)
        return prediction[0], probability[0]
    
    def evaluate(self, test_features, test_labels):
        """Đánh giá model trên test set"""
        predictions = self.predict(test_features)
        accuracy = accuracy_score(test_labels, predictions)
        
        print(f"Test Accuracy: {accuracy:.4f}")
        print("\nClassification Report:")
        print(classification_report(test_labels, predictions, target_names=self.label_names))
        
        # Confusion Matrix
        cm = confusion_matrix(test_labels, predictions)
        plt.figure(figsize=(8, 6))
        sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', 
                    xticklabels=self.label_names, yticklabels=self.label_names)
        plt.title('Confusion Matrix')
        plt.ylabel('True Label')
        plt.xlabel('Predicted Label')
        plt.show()
        
        return accuracy
    
    def save_model(self, save_path):
        """Lưu model"""
        model_data = {
            'k': self.k,
            'metric': self.metric,
            'train_labels': self.train_labels,
            'label_names': self.label_names,
            'feature_dim': self.feature_dim
        }
        
        # Lưu FAISS index
        faiss.write_index(self.index, save_path + '.faiss')
        
        # Lưu metadata
        with open(save_path + '.pkl', 'wb') as f:
            pickle.dump(model_data, f)
        
        print(f"Model saved to {save_path}")
    
    def load_model(self, save_path):
        """Load model"""
        # Load FAISS index
        self.index = faiss.read_index(save_path + '.faiss')
        
        # Load metadata
        with open(save_path + '.pkl', 'rb') as f:
            model_data = pickle.load(f)
        
        self.k = model_data['k']
        self.metric = model_data['metric']
        self.train_labels = model_data['train_labels']
        self.label_names = model_data['label_names']
        self.feature_dim = model_data['feature_dim']
        
        print(f"Model loaded from {save_path}")

# Ví dụ sử dụng
if __name__ == "__main__":
    # Load features đã trích xuất
    with open('snore_features.pkl', 'rb') as f:
        data = pickle.load(f)
    
    features = data['features']
    labels = data['labels']
    label_names = data['label_names']
    
    # Split train/test
    X_train, X_test, y_train, y_test = train_test_split(
        features, labels, test_size=0.2, random_state=42, stratify=labels
    )
    
    print(f"Train set: {len(X_train)} samples")
    print(f"Test set: {len(X_test)} samples")
    
    # Train FAISS classifier
    classifier = FAISSClassifier(k=5, metric='L2')
    classifier.train(X_train, y_train, label_names)
    
    # Evaluate
    accuracy = classifier.evaluate(X_test, y_test)
    
    # Save model
    classifier.save_model('snore_faiss_model') 