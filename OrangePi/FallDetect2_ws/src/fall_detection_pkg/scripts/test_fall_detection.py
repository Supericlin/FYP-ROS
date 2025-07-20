#!/usr/bin/env python3
"""
Test script for fall detection with wide-angle lens
This script helps you tune the parameters by showing detection results in real-time
"""

import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import os
import time
from ament_index_python.packages import get_package_share_directory

def test_fall_detection():
    # Get model path
    package_dir = get_package_share_directory('fall_detection_pkg')
    model_path = os.path.join(package_dir, 'models/mobilenet_ssd.tflite')
    
    if not os.path.isfile(model_path):
        print(f"Model not found at {model_path}")
        return
    
    # Initialize TFLite interpreter
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    input_height = input_details[0]['shape'][1]
    input_width = input_details[0]['shape'][2]
    
    print(f"Model input size: {input_width}x{input_height}")
    
    # Open camera (adjust device number if needed)
    cap = cv2.VideoCapture(0)
    
    # Set camera properties for wide-angle lens
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    # Tuning parameters
    confidence_threshold = 0.3  # Lower threshold for wide-angle
    fall_threshold = 0.6  # More lenient for wide-angle distortion
    
    print(f"Confidence threshold: {confidence_threshold}")
    print(f"Fall threshold: {fall_threshold}")
    print("Press 'q' to quit, 'c' to change confidence, 'f' to change fall threshold")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
            
        # Resize and preprocess
        input_img = cv2.resize(frame, (input_width, input_height))
        input_data = np.expand_dims(input_img, axis=0)
        
        if input_details[0]['dtype'] == np.float32:
            input_data = (input_data.astype(np.float32) / 127.5) - 1.0
            
        # Inference
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        
        # Get outputs
        boxes = interpreter.get_tensor(output_details[0]['index'])[0]
        classes = interpreter.get_tensor(output_details[1]['index'])[0]
        scores = interpreter.get_tensor(output_details[2]['index'])[0]
        num_detections = int(interpreter.get_tensor(output_details[3]['index'])[0])
        
        # Process detections
        for i in range(num_detections):
            if scores[i] > confidence_threshold and classes[i] == 0:  # Person
                ymin = int(boxes[i][0] * frame.shape[0])
                xmin = int(boxes[i][1] * frame.shape[1])
                ymax = int(boxes[i][2] * frame.shape[0])
                xmax = int(boxes[i][3] * frame.shape[1])
                
                # Calculate properties
                bbox_height = ymax - ymin
                bbox_width = xmax - xmin
                aspect_ratio = bbox_height / bbox_width if bbox_width > 0 else 0
                bbox_area = bbox_width * bbox_height
                frame_area = frame.shape[0] * frame.shape[1]
                relative_area = bbox_area / frame_area
                
                # Draw bounding box
                color = (0, 255, 0)  # Green for person
                if aspect_ratio < fall_threshold:
                    color = (0, 0, 255)  # Red for potential fall
                
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                
                # Add text
                text = f"Person: {scores[i]:.2f}, Ratio: {aspect_ratio:.2f}"
                cv2.putText(frame, text, (xmin, ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Check fall conditions
                fall_detected = False
                if aspect_ratio < fall_threshold:
                    fall_detected = True
                elif relative_area < 0.01 and aspect_ratio < 0.8:
                    fall_detected = True
                elif ymax > frame.shape[0] * 0.7 and aspect_ratio < 0.7:
                    fall_detected = True
                
                if fall_detected:
                    cv2.putText(frame, "FALL DETECTED!", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Add parameter info
        cv2.putText(frame, f"Confidence: {confidence_threshold}", (10, frame.shape[0]-60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Fall threshold: {fall_threshold}", (10, frame.shape[0]-40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Detections: {num_detections}", (10, frame.shape[0]-20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow('Fall Detection Test', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            confidence_threshold = float(input("Enter new confidence threshold (0.1-1.0): "))
            print(f"Confidence threshold set to: {confidence_threshold}")
        elif key == ord('f'):
            fall_threshold = float(input("Enter new fall threshold (0.1-2.0): "))
            print(f"Fall threshold set to: {fall_threshold}")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_fall_detection() 