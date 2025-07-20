#include <esp_log.h>
#include <cstring>
#include "board.h"
#include "boards/bread-compact-wifi/config.h"
#include "iot/thing.h"
#include "mqtt_client.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define TAG "du_mqtt"
#define CONFIG_BROKER_URL "mqtt://192.168.0.224"

namespace iot {

class DuMqtt : public Thing {
 private:
  esp_mqtt_client_handle_t client;
  bool mqtt_connected = false;
  // 消息队列，用来缓冲待发布的 MQTT 消息
  QueueHandle_t mqtt_msg_queue;
  const size_t QUEUE_LENGTH = 10; // 队列最大消息数
  // 限制连续发布消息之间的最小间隔，避免过快调用
  const TickType_t min_publish_interval = pdMS_TO_TICKS(100);
  TickType_t last_publish_tick;
  // 互斥锁，用于保护 MQTT 客户端（如有需要）
  SemaphoreHandle_t mqtt_mutex;

  // 内部发布消息函数，执行真正的发布与重试逻辑
  void PublishMessageInternal(const char* message) {
    // 限制连续发布的频率
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
  
    const char* topic = "robot/control";  // 根据需要修改主题
    int retries = 4;
    int msg_id = -1;
    while (retries-- > 0) {
      msg_id = esp_mqtt_client_publish(client, topic, message, 0, 1, 0);
      if (msg_id >= 0) {
        break;
      }
      ESP_LOGW(TAG, "Publish failed (attempt %d) for message: %s", 4 - retries, message);
      // 延时60ms后重试
      vTaskDelay(pdMS_TO_TICKS(60));
    }
    if (msg_id >= 0) {
      ESP_LOGI(TAG, "Message sent, msg_id: %d, message: %s", msg_id, message);
    } else {
      ESP_LOGE(TAG, "Failed to publish message: %s after retries", message);
    }
  }

  // 专门的任务，由消息队列中取出消息进行发送
  static void PublisherTask(void* pvParameters) {
    DuMqtt* instance = static_cast<DuMqtt*>(pvParameters);
    char* message = nullptr;
    while (1) {
      // 阻塞等待消息，下次无限等待直到有消息到达
      if (xQueueReceive(instance->mqtt_msg_queue, &message, portMAX_DELAY) == pdTRUE) {
        instance->PublishMessageInternal(message);
        free(message); // 发布完成后释放内存
      }
    }
  }

  // 通过消息队列发送消息，避免直接在业务线程中做耗时操作
  void QueueMqttMessage(const std::string& message) {
    // 动态分配内存保存消息（记得加1字符空间存放结束符'\0'）
    size_t len = message.size() + 1;
    char* msg_copy = (char*)malloc(len);
      if (msg_copy == nullptr) {
    ESP_LOGE(TAG, "Memory allocation for message failed");
    return;
    }
    strncpy(msg_copy, message.c_str(), len);
    // 若队列满了，则阻塞一定时间
    if (xQueueSend(mqtt_msg_queue, &msg_copy, pdMS_TO_TICKS(50)) != pdTRUE) {
      ESP_LOGE(TAG, "Message queue is full; dropping message: %s", msg_copy);
    free(msg_copy);
    }
  }
  
  // 外部调用的发送消息接口，内部将消息入队
  void SendMqttMessage(const std::string& message) {
    QueueMqttMessage(message);
  }

  // MQTT 事件处理函数
  static void mqtt_event_handler(void* handler_args, esp_event_base_t base,
                                 int32_t event_id, void* event_data) {
    DuMqtt* instance = static_cast<DuMqtt*>(handler_args);
    ESP_LOGD(TAG,
             "Event dispatched from event loop base=%s, event_id=%" PRIi32 "",
             base, event_id);

    switch (event_id) {
      case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected successfully");
        instance->mqtt_connected = true;
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
        // 你可以在此处扩展其他 MQTT 配置项
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
  }

 public:
  DuMqtt()
    : Thing("DuMqtt", "導航机器人：兩輪差速可以移动"), 
    last_publish_tick(0) {
    mqtt_app_start();
    // 创建消息队列
    mqtt_msg_queue = xQueueCreate(QUEUE_LENGTH, sizeof(char*));
    if (mqtt_msg_queue == nullptr) {
      ESP_LOGE(TAG, "Failed to create MQTT message queue");
    }
    // 创建互斥锁（如有需要保护共享资源）
    mqtt_mutex = xSemaphoreCreateMutex();
    if (mqtt_mutex == nullptr) {
      ESP_LOGE(TAG, "Failed to create MQTT mutex");
    }
    // 启动发布任务
    xTaskCreate(PublisherTask, "MQTT_Publisher", 4096, this, tskIDLE_PRIORITY + 1, NULL);
    methods_.AddMethod("goto_StudyRoom", "去書房", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("studyroom");
                       });

    methods_.AddMethod("goto_BedRoom", "去睡房", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("bedroom");
                       });

    methods_.AddMethod("goto_LivingRoom", "去客廳", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("livingroom");
                       });

    methods_.AddMethod("goto_DiningRoom", "去飯廳", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("diningroom");
                       });

    methods_.AddMethod("goto_WashRoom", "去洗手間", ParameterList(),
                       [this](const ParameterList& parameters) {
                         SendMqttMessage("washroom");
                       });

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
  }
};

} // namespace iot

DECLARE_THING(DuMqtt);
