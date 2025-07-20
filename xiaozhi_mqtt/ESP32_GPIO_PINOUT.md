# ESP32 GPIO Pinout Configuration

## 🔧 Fixed GPIO Configuration for ESP32

### **❌ Problem:**
The original configuration used GPIO pins 25 and 26 which don't exist on regular ESP32 (only on ESP32-S3).

### **✅ Solution:**
Updated GPIO assignments to use valid ESP32 pins.

## 📍 GPIO Pin Assignments

### **🎵 Audio I2S Pins (Simplex Mode):**
| Function | GPIO | Description |
|----------|------|-------------|
| **MIC_WS** | GPIO 4 | Microphone Word Select |
| **MIC_SCK** | GPIO 5 | Microphone Serial Clock |
| **MIC_DIN** | GPIO 18 | Microphone Data In |
| **SPK_DOUT** | GPIO 19 | Speaker Data Out |
| **SPK_BCLK** | GPIO 14 | Speaker Bit Clock |
| **SPK_LRCK** | GPIO 27 | Speaker Left/Right Clock |

### **🔘 Button Pins:**
| Function | GPIO | Description |
|----------|------|-------------|
| **BOOT_BUTTON** | GPIO 0 | Boot/Reset Button |
| **TOUCH_BUTTON** | GPIO 21 | Touch/Record Button |
| **ASR_BUTTON** | GPIO 22 | ASR/Wake Word Button |

### **📱 Display Pins:**
| Function | GPIO | Description |
|----------|------|-------------|
| **DISPLAY_SDA** | GPIO 23 | I2C Data Line |
| **DISPLAY_SCL** | GPIO 15 | I2C Clock Line |

### **💡 LED Pins:**
| Function | GPIO | Description |
|----------|------|-------------|
| **BUILTIN_LED** | GPIO 2 | Built-in LED |

## 🔄 Changes Made

### **Before (ESP32-S3 pins):**
```cpp
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_25  // ❌ Not available on ESP32
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_26  // ❌ Not available on ESP32
#define TOUCH_BUTTON_GPIO       GPIO_NUM_5   // ❌ Conflicts with audio
#define ASR_BUTTON_GPIO         GPIO_NUM_19  // ❌ Conflicts with audio
#define DISPLAY_SDA_PIN         GPIO_NUM_4   // ❌ Conflicts with audio
```

### **After (ESP32 compatible):**
```cpp
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4   // ✅ Available on ESP32
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5   // ✅ Available on ESP32
#define TOUCH_BUTTON_GPIO       GPIO_NUM_21  // ✅ No conflicts
#define ASR_BUTTON_GPIO         GPIO_NUM_22  // ✅ No conflicts
#define DISPLAY_SDA_PIN         GPIO_NUM_23  // ✅ No conflicts
```

## 🚀 Compilation Instructions

### **1. Clean and Rebuild:**
```bash
cd xiaozhi_mqtt
idf.py clean
idf.py build
```

### **2. Flash to ESP32:**
```bash
idf.py flash monitor
```

## ⚠️ Hardware Connection Notes

### **Audio Connections:**
- **Microphone:** Connect to GPIO 4, 5, 18
- **Speaker:** Connect to GPIO 14, 19, 27

### **Button Connections:**
- **Boot Button:** GPIO 0 (usually built-in)
- **Touch Button:** GPIO 21
- **ASR Button:** GPIO 22

### **Display Connections:**
- **SSD1306 OLED:** SDA to GPIO 23, SCL to GPIO 15

### **LED Connection:**
- **Built-in LED:** GPIO 2 (usually built-in)

## 🔍 ESP32 vs ESP32-S3 Pin Differences

| Feature | ESP32 | ESP32-S3 |
|---------|-------|----------|
| **GPIO Range** | 0-39 | 0-48 |
| **GPIO 25** | ❌ Not available | ✅ Available |
| **GPIO 26** | ❌ Not available | ✅ Available |
| **GPIO 32-39** | ✅ Available | ✅ Available |

## ✅ Expected Results

After fixing the GPIO configuration:

1. **Compilation should succeed** without GPIO errors
2. **Audio I2S should work** with the new pin assignments
3. **Buttons should function** without conflicts
4. **Display should initialize** properly
5. **MQTT functionality** should work as expected

## 🛠️ Troubleshooting

### **If compilation still fails:**
```bash
# Check ESP32 target
idf.py set-target esp32

# Clean and rebuild
idf.py clean
idf.py build
```

### **If audio doesn't work:**
- Verify audio hardware connections match new GPIO pins
- Check I2S configuration in menuconfig

### **If buttons don't work:**
- Verify button connections match new GPIO pins
- Check button pull-up/pull-down configuration 