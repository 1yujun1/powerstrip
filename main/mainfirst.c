#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_eap_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "IOT_STRIP_STEPPER";

#define USE_ENTERPRISE_WIFI 0

#if USE_ENTERPRISE_WIFI
#define WIFI_SSID "학교_또는_회사_와이파이"
#define WIFI_EAP_ID "사용자아이디"
#define WIFI_EAP_USERNAME "사용자아이디"
#define WIFI_EAP_PASSWORD "비밀번호"
#else
#define WIFI_SSID "iptime2.4G"
#define WIFI_PASS "86894925"
#endif

#define MQTT_BROKER "mqtt://broker.hivemq.com"
#define TOPIC_STATUS "yujun_powerstrip/status"
#define TOPIC_CONTROL "yujun_powerstrip/control/#"

#define MOTOR1_IN1 18
#define MOTOR1_IN2 19
#define MOTOR1_IN3 21
#define MOTOR1_IN4 22

#define MOTOR2_IN1 13
#define MOTOR2_IN2 12
#define MOTOR2_IN3 14
#define MOTOR2_IN4 27

#define STEP_DELAY_MS 2
#define STEPS_PER_ACTION 2048

#define ADC_CHANNEL ADC_CHANNEL_6

#define OVER_CURRENT_LIMIT 2.0
#define STABLE_CHECK_TIME_MS 5000

float g_current_a = 0.0;
bool g_is_tripped = false;
bool g_motor1_on = false;
bool g_motor2_on = false;
esp_mqtt_client_handle_t mqtt_client = NULL;
adc_oneshot_unit_handle_t adc_handle;

static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

const uint8_t step_sequence[8][4] = {{1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0},
                                     {0, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 1},
                                     {0, 0, 0, 1}, {1, 0, 0, 1}};

void set_motor_step(int motor, int step) {
  if (motor == 1) {
    gpio_set_level(MOTOR1_IN1, step_sequence[step][0]);
    gpio_set_level(MOTOR1_IN2, step_sequence[step][1]);
    gpio_set_level(MOTOR1_IN3, step_sequence[step][2]);
    gpio_set_level(MOTOR1_IN4, step_sequence[step][3]);
  } else if (motor == 2) {
    gpio_set_level(MOTOR2_IN1, step_sequence[step][0]);
    gpio_set_level(MOTOR2_IN2, step_sequence[step][1]);
    gpio_set_level(MOTOR2_IN3, step_sequence[step][2]);
    gpio_set_level(MOTOR2_IN4, step_sequence[step][3]);
  }
}

void step_motor(int motor, int steps, bool forward) {
  ESP_LOGI(TAG, "Motor %d rotating %s (%d steps)", motor,
           forward ? "Forward" : "Backward", steps);

  for (int i = 0; i < steps; i++) {
    for (int s = 0; s < 8; s++) {
      int current_step = forward ? s : (7 - s);
      set_motor_step(motor, current_step);
      vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
    }
  }
  
  if (motor == 1) {
    gpio_set_level(MOTOR1_IN1, 0);
    gpio_set_level(MOTOR1_IN2, 0);
    gpio_set_level(MOTOR1_IN3, 0);
    gpio_set_level(MOTOR1_IN4, 0);
  } else {
    gpio_set_level(MOTOR2_IN1, 0);
    gpio_set_level(MOTOR2_IN2, 0);
    gpio_set_level(MOTOR2_IN3, 0);
    gpio_set_level(MOTOR2_IN4, 0);
  }
}

void update_motors(bool m1_on, bool m2_on) {
  if (g_is_tripped)
    return;

  if (m1_on != g_motor1_on) {
    g_motor1_on = m1_on;
    step_motor(1, STEPS_PER_ACTION, m1_on);
  }

  if (m2_on != g_motor2_on) {
    g_motor2_on = m2_on;
    step_motor(2, STEPS_PER_ACTION, m2_on);
  }

  ESP_LOGI(TAG, "Motor1: %s, Motor2: %s", g_motor1_on ? "ON" : "OFF",
           g_motor2_on ? "ON" : "OFF");
}

float read_current_rms() {
  float ACCurrtntValue = 0;
  float peakVoltage = 0;
  float voltageVirtualValue = 0;
  int raw_val;

  for (int i = 0; i < 5; i++) {
    esp_err_t ret = adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw_val);
    if (ret == ESP_OK) {
      peakVoltage += (float)raw_val;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  peakVoltage = peakVoltage / 5.0;
  voltageVirtualValue = peakVoltage * 0.707;
  voltageVirtualValue = (voltageVirtualValue / 4096.0 * 3.3) / 2.0;
  ACCurrtntValue = voltageVirtualValue * 20.0;

  return ACCurrtntValue;
}

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id,
                               void *event_data) {
  esp_mqtt_event_handle_t event = event_data;
  if (id == MQTT_EVENT_CONNECTED) {
    esp_mqtt_client_subscribe(mqtt_client, TOPIC_CONTROL, 0);
    ESP_LOGI(TAG, "Connected to HiveMQ");
  } else if (id == MQTT_EVENT_DATA) {
    char topic[64];
    char payload[16];
    int topic_len = (event->topic_len >= 64) ? 63 : event->topic_len;
    int data_len = (event->data_len >= 16) ? 15 : event->data_len;

    memcpy(topic, event->topic, topic_len);
    topic[topic_len] = '\0';
    memcpy(payload, event->data, data_len);
    payload[data_len] = '\0';

    ESP_LOGI(TAG, "MQTT RCV: Topic=[%s] Data=[%s]", topic, payload);

    if (strstr(topic, "servo1") || strstr(topic, "motor1")) {
      bool target = (strncmp(payload, "on", 2) == 0);
      update_motors(target, g_motor2_on);
    } else if (strstr(topic, "servo2") || strstr(topic, "motor2")) {
      bool target = (strncmp(payload, "on", 2) == 0);
      update_motors(g_motor1_on, target);
    }
  }
}

static void wifi_handler(void *arg, esp_event_base_t base, int32_t id,
                         void *data) {
  if (id == WIFI_EVENT_STA_START)
    esp_wifi_connect();
  else if (id == WIFI_EVENT_STA_DISCONNECTED)
    esp_wifi_connect();
  else if (id == IP_EVENT_STA_GOT_IP)
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
}

void strip_logic_task(void *pv) {
  uint32_t stable_counter = 0;
  char payload[128];

  while (1) {
    g_current_a = read_current_rms();

    if (g_current_a > OVER_CURRENT_LIMIT && !g_is_tripped) {
      ESP_LOGW(TAG, "!!! TRIP: %.2fA !!!", g_current_a);
      g_is_tripped = true;

      if (g_motor1_on)
        step_motor(1, STEPS_PER_ACTION, false);
      if (g_motor2_on)
        step_motor(2, STEPS_PER_ACTION, false);

      g_motor1_on = false;
      g_motor2_on = false;

      stable_counter = 0;
    }

    if (g_is_tripped) {
      if (g_current_a < OVER_CURRENT_LIMIT) {
        stable_counter += 1000;
        if (stable_counter >= STABLE_CHECK_TIME_MS) {
          ESP_LOGI(TAG, "Current Stable. Recovering...");
          g_is_tripped = false;
          update_motors(g_motor1_on, g_motor2_on);
          stable_counter = 0;
        }
      } else {
        stable_counter = 0;
      }
    }

    if (mqtt_client) {
      snprintf(
          payload, sizeof(payload),
          "{\"current\":%.2f, \"s1\":\"%s\", \"s2\":\"%s\", \"tripped\":%s}",
          g_current_a, g_motor1_on ? "ON" : "OFF", g_motor2_on ? "ON" : "OFF",
          g_is_tripped ? "true" : "false");
      esp_mqtt_client_publish(mqtt_client, TOPIC_STATUS, payload, 0, 1, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void init_all() {
  ESP_ERROR_CHECK(nvs_flash_init());

  int output_pins[] = {MOTOR1_IN1, MOTOR1_IN2, MOTOR1_IN3, MOTOR1_IN4,
                       MOTOR2_IN1, MOTOR2_IN2, MOTOR2_IN3, MOTOR2_IN4};
  for (int i = 0; i < sizeof(output_pins) / sizeof(output_pins[0]); i++) {
    gpio_reset_pin(output_pins[i]);
    gpio_set_direction(output_pins[i], GPIO_MODE_OUTPUT);
    gpio_set_level(output_pins[i], 0);
  }

  esp_netif_init();
  esp_event_loop_create_default();
  s_wifi_event_group = xEventGroupCreate();
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_handler, NULL, NULL);

#if USE_ENTERPRISE_WIFI
  wifi_config_t wifi_cfg = {.sta = {.ssid = WIFI_SSID}};
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);

  ESP_ERROR_CHECK(
      esp_eap_client_set_identity((uint8_t *)WIFI_EAP_ID, strlen(WIFI_EAP_ID)));
  ESP_ERROR_CHECK(esp_eap_client_set_username((uint8_t *)WIFI_EAP_USERNAME,
                                              strlen(WIFI_EAP_USERNAME)));
  ESP_ERROR_CHECK(esp_eap_client_set_password((uint8_t *)WIFI_EAP_PASSWORD,
                                              strlen(WIFI_EAP_PASSWORD)));
  ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());
#else
  wifi_config_t wifi_cfg = {.sta = {.ssid = WIFI_SSID, .password = WIFI_PASS}};
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
#endif

  esp_wifi_start();
  xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                      portMAX_DELAY);

  adc_oneshot_unit_init_cfg_t init_raw = {.unit_id = ADC_UNIT_1};
  adc_oneshot_new_unit(&init_raw, &adc_handle);
  adc_oneshot_chan_cfg_t chan_cfg = {.bitwidth = ADC_BITWIDTH_DEFAULT,
                                     .atten = ADC_ATTEN_DB_12};
  adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg);

  const esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = MQTT_BROKER,
      .network.timeout_ms = 5000,
      .session.keepalive = 60,
      .session.disable_clean_session = false,
  };
  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                 mqtt_event_handler, NULL);
  esp_mqtt_client_start(mqtt_client);

  ESP_LOGI(TAG, "System Initialized");
}

void app_main(void) {
  init_all();
  xTaskCreate(strip_logic_task, "strip_task", 4096, NULL, 5, NULL);
}
