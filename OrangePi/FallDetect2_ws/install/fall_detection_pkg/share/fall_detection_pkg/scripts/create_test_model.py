#!/usr/bin/env python3
"""
Create a basic test TFLite model for fall detection
This is a simplified model for testing purposes
"""

import tensorflow as tf
import numpy as np
import os

def create_test_model():
    """Create a simple test model for object detection"""
    
    # Create a simple model that outputs dummy detections
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=(300, 300, 3)),
        tf.keras.layers.Conv2D(16, 3, activation='relu'),
        tf.keras.layers.MaxPooling2D(),
        tf.keras.layers.Conv2D(32, 3, activation='relu'),
        tf.keras.layers.GlobalAveragePooling2D(),
        tf.keras.layers.Dense(10, activation='relu'),
        tf.keras.layers.Dense(4, activation='sigmoid')  # 4 values for bounding box
    ])
    
    # Compile model
    model.compile(optimizer='adam', loss='mse')
    
    # Convert to TFLite
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    tflite_model = converter.convert()
    
    # Save model
    os.makedirs('models', exist_ok=True)
    with open('models/mobilenet_ssd.tflite', 'wb') as f:
        f.write(tflite_model)
    
    print("Test model created: models/mobilenet_ssd.tflite")
    print("Note: This is a test model. For production, use a proper pre-trained model.")

def download_pretrained_model():
    """Download a pre-trained MobileNet SSD model"""
    
    import urllib.request
    import zipfile
    
    # URL for MobileNet SSD model
    model_url = "https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip"
    
    print("Downloading pre-trained MobileNet SSD model...")
    
    # Create models directory
    os.makedirs('models', exist_ok=True)
    
    # Download and extract
    zip_path = 'models/model.zip'
    urllib.request.urlretrieve(model_url, zip_path)
    
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall('models')
    
    # Rename the model file
    os.rename('models/detect.tflite', 'models/mobilenet_ssd.tflite')
    os.remove(zip_path)
    
    print("Pre-trained model downloaded: models/mobilenet_ssd.tflite")

def main():
    print("MobileNet SSD TFLite Model Setup")
    print("=================================")
    print("1. Create test model (for testing only)")
    print("2. Download pre-trained model (recommended)")
    
    choice = input("Enter your choice (1 or 2): ").strip()
    
    if choice == "1":
        create_test_model()
    elif choice == "2":
        try:
            download_pretrained_model()
        except Exception as e:
            print(f"Download failed: {e}")
            print("Creating test model instead...")
            create_test_model()
    else:
        print("Invalid choice. Creating test model...")
        create_test_model()

if __name__ == "__main__":
    main() 