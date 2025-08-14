# FYP-ROS

## Intelligent Robotic Navigation for the Blind or Visually Impaired (iRGB)

The Intelligent Robotic Navigation for the Blind or Visually Impaired (iRGB) project aims to transform the indoor navigation experience for visually impaired individuals by developing an artificial intelligence (AI) navigation system. Designed for complex indoor spaces like shopping malls, the system combines an intelligent robotic vehicle with a connected guide cane. The system is equipped with ultrasonic radar, lidar, and AI cameras, enabling real-time obstacle detection, object recognition, and safe navigation.

## System Overview

The iRGB system is developed based on open-source platforms such as Linux and ROS2, focusing on cost-effectiveness, scalability, and user-centric design. Its core features include:

- **Fall Detection**: Advanced fall detection algorithms for user safety
- **Handlebar Pressure Sensing**: Intelligent pressure monitoring for user interaction
- **Handlebar Separation Detection**: Safety feature to detect when user loses contact
- **Multi-function Button**: Versatile control interface for user commands
- **AI Voice Interaction**: Natural language communication between user and robot

All these features prioritize user safety and enhance the overall navigation experience.

## Technical Specifications

- **Platform**: Linux-based with ROS2 (Robot Operating System 2)
- **Sensors**: 
  - Ultrasonic radar
  - LiDAR (Light Detection and Ranging)
  - AI cameras
- **Architecture**: Open-source, cost-effective, and scalable design
- **Communication**: AI voice interaction technology

## Performance Results

Testing in various scenarios, including a simulated shopping mall, demonstrated impressive results:

### Obstacle Detection
- **Static Obstacles**: 98% success rate
- **Dynamic Obstacles**: 92% success rate
- **Navigation Accuracy**: Average of 18 cm

### Fall Detection
- **Normal Conditions**: 95% success rate
- **Low-light Conditions**: 87% success rate

These test results demonstrate the system's ability to help visually impaired individuals navigate complex environments independently and confidently.

## Project Impact

The iRGB project successfully integrates traditional mobility aids with advanced robotics and artificial intelligence technologies, marking a significant step forward in advancing accessible and inclusive technologies. The system can potentially transform the lives of visually impaired individuals, enhancing their daily independence, safety, and confidence.

## Repository Structure

This repository contains various components of the iRGB system:

- **OrangePi**: Fall detection and camera systems
- **RDK-X5**: Navigation and MQTT goal systems
- **xiaozhi_mqtt**: ESP32-based MQTT communication
- **ROS2_ESP32-S3**: Arduino-based ROS2 integration

## Getting Started

For detailed setup and usage instructions, please refer to the specific directories within this repository.

## License

This project is part of a Final Year Project (FYP) focused on developing accessible robotics technology for the visually impaired community.