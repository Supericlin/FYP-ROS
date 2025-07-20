#!/usr/bin/env python3
"""
Script to download MobileNet SSD TFLite model for person detection
"""

import os
import urllib.request
import zipfile
import shutil
from pathlib import Path

def download_mobilenet_ssd():
    """Download MobileNet SSD TFLite model"""
    
    # Model URLs (multiple sources)
    model_urls = [
        "https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip",
        "https://github.com/tensorflow/models/raw/master/research/object_detection/test_images/mobilenet_ssd.tflite"
    ]
    
    # Local paths
    script_dir = Path(__file__).parent
    models_dir = script_dir.parent / "models"
    models_dir.mkdir(exist_ok=True)
    
    model_path = models_dir / "mobilenet_ssd.tflite"
    
    print(f"Downloading MobileNet SSD TFLite model...")
    print(f"Target location: {model_path}")
    
    # Try downloading from different sources
    for i, url in enumerate(model_urls):
        try:
            print(f"Attempting download from source {i+1}: {url}")
            
            if url.endswith('.zip'):
                # Download and extract zip file
                zip_path = models_dir / "temp_model.zip"
                urllib.request.urlretrieve(url, zip_path)
                
                with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                    # Look for .tflite file in the zip
                    tflite_files = [f for f in zip_ref.namelist() if f.endswith('.tflite')]
                    if tflite_files:
                        zip_ref.extract(tflite_files[0], models_dir)
                        # Rename to standard name
                        extracted_path = models_dir / tflite_files[0]
                        if extracted_path != model_path:
                            shutil.move(extracted_path, model_path)
                        print(f"Successfully extracted: {tflite_files[0]}")
                    else:
                        print("No .tflite file found in zip")
                
                # Clean up zip file
                zip_path.unlink()
                
            else:
                # Direct download
                urllib.request.urlretrieve(url, model_path)
                print("Direct download successful")
            
            if model_path.exists():
                print(f"✅ Model downloaded successfully: {model_path}")
                print(f"File size: {model_path.stat().st_size / (1024*1024):.2f} MB")
                return True
                
        except Exception as e:
            print(f"❌ Download failed from source {i+1}: {str(e)}")
            continue
    
    print("❌ All download attempts failed")
    return False

def create_simple_model():
    """Create a simple placeholder model if download fails"""
    print("Creating a simple placeholder model...")
    
    script_dir = Path(__file__).parent
    models_dir = script_dir.parent / "models"
    models_dir.mkdir(exist_ok=True)
    
    model_path = models_dir / "mobilenet_ssd.tflite"
    
    # Create a simple TensorFlow Lite model for testing
    try:
        import tensorflow as tf
        import numpy as np
        
        # Create a simple model
        model = tf.keras.Sequential([
            tf.keras.layers.Input(shape=(300, 300, 3)),
            tf.keras.layers.Conv2D(16, 3, activation='relu'),
            tf.keras.layers.GlobalAveragePooling2D(),
            tf.keras.layers.Dense(1, activation='sigmoid')
        ])
        
        # Convert to TFLite
        converter = tf.lite.TFLiteConverter.from_keras_model(model)
        tflite_model = converter.convert()
        
        # Save the model
        with open(model_path, 'wb') as f:
            f.write(tflite_model)
        
        print(f"✅ Created placeholder model: {model_path}")
        return True
        
    except ImportError:
        print("❌ TensorFlow not available for creating placeholder model")
        return False

def main():
    """Main function"""
    print("=" * 60)
    print("MobileNet SSD TFLite Model Downloader")
    print("=" * 60)
    
    # Try to download the model
    if download_mobilenet_ssd():
        print("\n🎉 Model download completed successfully!")
        print("You can now run the fall detection system.")
    else:
        print("\n⚠️  Download failed. Trying to create placeholder model...")
        if create_simple_model():
            print("⚠️  Placeholder model created. This is for testing only.")
            print("For production use, please manually download a proper MobileNet SSD model.")
        else:
            print("❌ Failed to create any model.")
            print("\nManual download instructions:")
            print("1. Visit: https://github.com/tensorflow/models/tree/master/research/object_detection")
            print("2. Download a MobileNet SSD TFLite model")
            print("3. Place it in: src/fall_detection_pkg/models/mobilenet_ssd.tflite")
    
    print("\n" + "=" * 60)

if __name__ == "__main__":
    main() 