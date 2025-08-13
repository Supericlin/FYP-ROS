#include <esp_log.h>
#include <cstring>
#include "board.h"
#include "boards/bread-compact-wifi/config.h"
#include "iot/thing.h"
#include "mqtt_client.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "application.h"

#define TAG "irgb_mqtt"
#define CONFIG_BROKER_URL "mqtt://192.168.0.224"

extern "C" void TriggerWakeWordFromMQTT();
extern "C" void PlayDestinationReachedSound();
extern "C" void ShowDestinationReachedMessage();

namespace iot {
  void ShowNavigationCompletedMessage();
}

namespace iot {

class iRGBMqtt : public Thing {
 private:
  esp_mqtt_client_handle_t client;
  bool mqtt_connected = false;
  QueueHandle_t mqtt_msg_queue;
  const size_t QUEUE_LENGTH = 10;

  const TickType_t min_publish_interval = pdMS_TO_TICKS(100);
  TickType_t last_publish_tick;
  SemaphoreHandle_t mqtt_mutex;

  void PublishMessageInternal(const char* message) {
    TickType_t now = xTaskGetTickCount();
    if (now - last_publish_tick < min_publish_interval) {
      vTaskDelay(min_publish_interval - (now - last_publish_tick));
    }
    last_publish_tick = xTaskGetTickCount();

    if (client == nullptr) {
      ESP_LOGE(TAG, "MQTT client is not initialized!");
      return;
    }
  
    if (!mqtt_connected) {
      ESP_LOGW(TAG, "MQTT not connected, skipping publishing: %s", message);
      return;
    }
  
    const char* topic = "robot/control";  //publish topic
    int retries = 4;
    int msg_id = -1;
    while (retries-- > 0) {
      msg_id = esp_mqtt_client_publish(client, topic, message, 0, 1, 0);
      if (msg_id >= 0) {
        break;
      }
      ESP_LOGW(TAG, "Publish failed (attempt %d) for message: %s", 4 - retries, message);
      // restart after 60ms
      vTaskDelay(pdMS_TO_TICKS(60));
    }
    if (msg_id >= 0) {
      ESP_LOGI(TAG, "Message sent, msg_id: %d, message: %s", msg_id, message);
    } else {
      ESP_LOGE(TAG, "Failed to publish message: %s after retries", message);
    }
  }

  void PublishLedMessageInternal(const char* message) {
    TickType_t now = xTaskGetTickCount();
    if (now - last_publish_tick < min_publish_interval) {
      vTaskDelay(min_publish_interval - (now - last_publish_tick));
    }
    last_publish_tick = xTaskGetTickCount();

    if (client == nullptr) {
      ESP_LOGE(TAG, "MQTT client is not initialized!");
      return;
    }
  
    if (!mqtt_connected) {
      ESP_LOGW(TAG, "MQTT not connected, skipping publishing: %s", message);
      return;
    }
  
    const char* topic = "robot/status";
    int retries = 4;
    int msg_id = -1;
    while (retries-- > 0) {
      msg_id = esp_mqtt_client_publish(client, topic, message, 0, 1, 0);
      if (msg_id >= 0) {
        break;
      }
      ESP_LOGW(TAG, "Publish failed (attempt %d) for message: %s", 4 - retries, message);
      // restart after 60ms
      vTaskDelay(pdMS_TO_TICKS(60));
    }
    if (msg_id >= 0) {
      ESP_LOGI(TAG, "LED message sent, msg_id: %d, message: %s", msg_id, message);
    } else {
      ESP_LOGE(TAG, "Failed to publish LED message: %s after retries", message);
    }
  }

  static void PublisherTask(void* pvParameters) {
    iRGBMqtt* instance = static_cast<iRGBMqtt*>(pvParameters);
    char* message = nullptr;
    while (1) {
      if (xQueueReceive(instance->mqtt_msg_queue, &message, portMAX_DELAY) == pdTRUE) {
        instance->PublishMessageInternal(message);
        free(message);
      }
    }
  }

  void QueueMqttMessage(const std::string& message) {
    size_t len = message.size() + 1;
    char* msg_copy = (char*)malloc(len);
      if (msg_copy == nullptr) {
    ESP_LOGE(TAG, "Memory allocation for message failed");
    return;
    }
    strncpy(msg_copy, message.c_str(), len);
    if (xQueueSend(mqtt_msg_queue, &msg_copy, pdMS_TO_TICKS(50)) != pdTRUE) {
      ESP_LOGE(TAG, "Message queue is full; dropping message: %s", msg_copy);
    free(msg_copy);
    }
  }
  
  void SendMqttMessage(const std::string& message) {
    QueueMqttMessage(message);
  }
  
  void SendLedMqttMessage(const std::string& message) {
    size_t len = message.size() + 1;
    char* msg_copy = (char*)malloc(len);
    if (msg_copy == nullptr) {
      ESP_LOGE(TAG, "Memory allocation for LED message failed");
      return;
    }
    strncpy(msg_copy, message.c_str(), len);
    
    PublishLedMessageInternal(msg_copy);
    free(msg_copy);
  }
  
  void ProcessWakeupCommand(const char* command) {
    ESP_LOGI(TAG, "Processing wakeup command: %s", command);
    
    // Check if this is a destination_reached wakeup
    if (strstr(command, "destination_reached") != NULL) {
        ESP_LOGI(TAG, "Destination reached wakeup command received");
        
        // Play a notification sound first
        PlayDestinationReachedSound();
        
        // Now wake up the device and connect to websocket
        WakeupDeviceAndConnectWebsocket();
    }
  }
  
	// Add a method to wake up the device and connect to websocket
	void WakeupDeviceAndConnectWebsocket() {
		ESP_LOGI(TAG, "Waking up device and connecting to websocket");
		
		auto& app = Application::GetInstance();
		app.OnDestinationReached();
  }

  // MQTT event handler
  static void mqtt_event_handler(void* handler_args, esp_event_base_t base,
                                 int32_t event_id, void* event_data) {
    iRGBMqtt* instance = static_cast<iRGBMqtt*>(handler_args);
    ESP_LOGD(TAG,
             "Event dispatched from event loop base=%s, event_id=%" PRIi32 "",
             base, event_id);

    // Get the MQTT event info
    esp_mqtt_event_t* event = static_cast<esp_mqtt_event_t*>(event_data);

    switch (event_id) {
      case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected successfully");
        instance->mqtt_connected = true;
        // Re-subscribe when reconnected
        if (instance->client) {
          esp_mqtt_client_subscribe(instance->client, "robot/status", 0); // subscribe topic
          ESP_LOGI(TAG, "Subscribed to robot/status topic");
        }
        break;
      case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT disconnected");
        instance->mqtt_connected = false;
        break;
      case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "Subscribed to topic");
        break;
      case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "Unsubscribed from topic");
        break;
      case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "Message published");
        break;
      case MQTT_EVENT_DATA:
		// Process incoming messages
		if (event->topic && event->data) {
			char* topic = strndup(event->topic, event->topic_len);
			char* data = strndup(event->data, event->data_len);
			
			if (topic && data) {
				ESP_LOGI(TAG, "Received message on topic %s: %s", topic, data);
				
				// Process status messages
				if (strcmp(topic, "robot/status") == 0 && strstr(data, "goalReached")) {
					ESP_LOGI(TAG, "Goal reached notification detected!");
					
					// Call the notification function
					TriggerWakeWordFromMQTT();
				}
				
				// Process command messages, subscribe robot/command topic
				if (strcmp(topic, "robot/command") == 0 && strstr(data, "wakeup:")) {
					ESP_LOGI(TAG, "Wakeup command received!");

					instance->ProcessWakeupCommand(data);
				}
				
				free(topic);
				free(data);
			}
		}
		break;
      case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT event error");
        break;
      default:
        break;
    }
  }

  void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {.address = {.uri = CONFIG_BROKER_URL}},
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == nullptr) {
      ESP_LOGE(TAG, "Failed to initialize MQTT client");
      return;
    }

    esp_mqtt_client_register_event(
        client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
        mqtt_event_handler, this);
    esp_mqtt_client_start(client);
    
    // Subscribe to goalReach topic
    esp_mqtt_client_subscribe(client, "robot/status", 0);
    ESP_LOGI(TAG, "Subscribed to robot/status topic");
  }

 public:
  iRGBMqtt()
    : Thing("iRGBMqtt", "導航机器人：兩輪差速可以移动"), 
    last_publish_tick(0) {
    mqtt_app_start();

    mqtt_msg_queue = xQueueCreate(QUEUE_LENGTH, sizeof(char*));
    if (mqtt_msg_queue == nullptr) {
      ESP_LOGE(TAG, "Failed to create MQTT message queue");
    }

    mqtt_mutex = xSemaphoreCreateMutex();
    if (mqtt_mutex == nullptr) {
      ESP_LOGE(TAG, "Failed to create MQTT mutex");
    }

    xTaskCreate(PublisherTask, "MQTT_Publisher", 4096, this, tskIDLE_PRIORITY + 1, NULL);
    methods_.AddMethod("goto_StudyRoom", "去書房[回覆:開始導航至書房]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("studyroom");
                       });

    methods_.AddMethod("goto_BedRoom", "去睡房[回覆:開始導航至睡房]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("bedroom");
                       });

    methods_.AddMethod("goto_LivingRoom", "去客廳[回覆:開始導航至客廳]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("livingroom");
                       });

    methods_.AddMethod("goto_DiningRoom", "去飯廳[回覆:開始導航至飯廳]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("diningroom");
                       });

    methods_.AddMethod("goto_WashRoom", "去洗手間[回覆:開始導航至洗手間]", ParameterList(),
                      [this](const ParameterList& parameters) {
                         SendMqttMessage("washroom");
                       });

    /* methods_.AddMethod("goto_mcdonald", "去麥當勞[回覆:開始導航至麥當勞]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("mcdonald");
                       });

    methods_.AddMethod("goto_entrance", "去入口[回覆:開始導航至入口]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("entrance");
                       });

    methods_.AddMethod("goto_toilet", "去洗手間[回覆:開始導航至洗手間]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("toilet");
                       });

    methods_.AddMethod("goto_watsons", "去屈臣氏[回覆:開始導航至屈臣氏]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("watsons");
                       });

    methods_.AddMethod("goto_information_counter", "去詢問處[回覆:開始導航至詢問處]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("information_counter");
                       });

    methods_.AddMethod("goto_starbucks", "去星巴克[回覆:開始導航至星巴克]", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("starbucks");
                       }); */

    methods_.AddMethod("NavigationPause", "導航暫停", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("navPause");
                       });

    methods_.AddMethod("NavigationStop", "導航停止", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("navStop");
                       });
    methods_.AddMethod("NavigationContinue", "導航繼續", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("navCon");
                       });

    methods_.AddMethod("TurnOnLedLight", "開 LED 燈", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendLedMqttMessage("LightSen_on");
                       });

    methods_.AddMethod("TurnOffLedLight", "關 LED 燈", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendLedMqttMessage("LightSen_off");
                       });
  }
  
  // Function to simulate wake word detection
  void SimulateWakeWordDetection() {
    ESP_LOGI(TAG, "Simulating wake word detection for navigation completion");
   
    // Publish a notification to the server
    SendMqttMessage("goalReach:Destination reached");
    
    // Try to display a local notification
    ShowNavigationCompletedMessage();
  }
};

// Implementation of the helper function
void ShowNavigationCompletedMessage() {
  ESP_LOGI(TAG, "Navigation completed - destination reached");
}

}

extern "C" void TriggerWakeWordFromMQTT() {
	ESP_LOGI("MQTT_WakeWord", "Destination reached notification received");
	ESP_LOGI("MQTT_WakeWord", "*** DESTINATION REACHED NOTIFICATION ***");
	
	// Play a notification sound if possible
	PlayDestinationReachedSound();
	
	// Display a message if possible
	ShowDestinationReachedMessage();
}

// Helper functions for notification
extern "C" void PlayDestinationReachedSound() {
	ESP_LOGI("MQTT_WakeWord", "Playing destination reached sound");
}

extern "C" void ShowDestinationReachedMessage() {
	ESP_LOGI("MQTT_WakeWord", "Showing destination reached message");
}

DECLARE_THING(iRGBMqtt);