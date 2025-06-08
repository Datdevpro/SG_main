import sounddevice as sd
import numpy as np
import tensorflow as tf
from tensorflow import keras
from scipy.signal import butter, lfilter

SAMPLE_RATE = 16000  # Sample rate
DURATION = 1  # Duration in seconds
THRESHOLD = 0.9  # Threshold for snoring detection

# Load your trained model
loaded_model = keras.models.load_model('.\trained_model\model_best.keras')

def bandpass_filter(audio, sr, lowcut=100, highcut=4000, order=5):
    nyq = 0.5 * sr
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return lfilter(b, a, audio)

def preprocess_audio(audio, sample_rate=SAMPLE_RATE):
    # Ensure audio is 1D float32 and length is SAMPLE_RATE * DURATION
    audio = audio.flatten().astype(np.float32)
    if len(audio) < sample_rate * DURATION:
        pad_width = sample_rate * DURATION - len(audio)
        audio = np.pad(audio, (0, pad_width))
    else:
        audio = audio[:sample_rate * DURATION]
    # Apply band-pass filter for noise reduction
    audio = bandpass_filter(audio, sample_rate)
    return tf.convert_to_tensor(audio, dtype=tf.float32)

print("Listening for snoring... (Press Ctrl+C to stop)")
try:
    while True:
        # Record audio for DURATION seconds
        audio = sd.rec(int(DURATION * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1)
        sd.wait()
        audio = preprocess_audio(audio, SAMPLE_RATE)
        # Extract log-mel features
        log_mel_features = get_log_mel(audio)
        log_mel_features = tf.reshape(log_mel_features, [1830])
        # Inference
        prediction = loaded_model(np.expand_dims(log_mel_features, axis=0))
        probability = prediction[0, 0].numpy()
        if probability >= THRESHOLD:
            print(f"Snoring detected! (prob={probability:.2f})")
        else:
            print(f"Non-snoring. (prob={probability:.2f})")
except KeyboardInterrupt:
    print("Stopped listening.")