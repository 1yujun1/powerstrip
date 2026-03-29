#include "driver/gpio.h"
#include "driver/i2c.h"
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
#include "pca9685.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "IOT_STRIP_V5";

// --- 설정 정의 ---
// 와이파이 접속 방식 선택 (1: 대학교/회사 등 ID+PW 방식, 0: 일반 가정용 방식)
#define USE_ENTERPRISE_WIFI 0

#if USE_ENTERPRISE_WIFI
#define WIFI_SSID "학교_또는_회사_와이파이"
#define WIFI_EAP_ID "사용자아이디"       // 외부 식별자 (보통 아이디와 동일)
#define WIFI_EAP_USERNAME "사용자아이디" // 내부 식별자
#define WIFI_EAP_PASSWORD "비밀번호"
#else
#define WIFI_SSID "iptime2.4G"
#define WIFI_PASS "86894925"
#endif

#define MQTT_BROKER "mqtt://broker.hivemq.com"

#define TOPIC_STATUS "yujun_powerstrip/status"
#define TOPIC_CONTROL "yujun_powerstrip/control/#"

// PCA9685 설정
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_NUM I2C_NUM_0

#define SERVO_MIN_US 1000 // 0도
#define SERVO_MAX_US 2000 // 180도

// 서보 1번각도 (약)
#define SERVO1_OFF_DEG 140
#define SERVO1_ON_DEG 45

// 서보 2번 각도(강)
#define SERVO2_OFF_DEG 140
#define SERVO2_ON_DEG 48

#define SERVO_INIT_DEG 90

// 전류 센서 핀
#define ADC_CHANNEL ADC_CHANNEL_4

// 상태 표시 핀
#define LED1_PIN 18
#define LED2_PIN 19
#define BUZZER_PIN 27

#define OVER_CURRENT_LIMIT 2.0
#define STABLE_CHECK_TIME_MS 5000

// 상태 변수
float g_current_a = 0.0;
bool g_is_tripped = false;
bool g_servo1_on = false;
bool g_servo2_on = false;
esp_mqtt_client_handle_t mqtt_client = NULL;
adc_oneshot_unit_handle_t adc_handle;
i2c_dev_t pca9685_dev;

static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// 서보 모터 제어

static uint16_t pca9685_us_to_value(i2c_dev_t *dev, uint32_t us) {
  return (uint16_t)((us * 4096) / 20000);
}

uint16_t angle_to_pwm(int angle) {
  if (angle > 180)
    angle = 180;
  if (angle < 0)
    angle = 0;
  uint32_t pulse_us =
      SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US) / 180);
  return pca9685_us_to_value(&pca9685_dev, pulse_us);
}

void set_servo_angle(uint8_t channel, int angle) {
  uint16_t pwm_val = angle_to_pwm(angle);
  esp_err_t ret = pca9685_set_pwm_value(&pca9685_dev, channel, pwm_val);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C 전송 실패 (CH%d, 에러: %d) - 선 연결을 확인하세요!",
             channel, ret);
  } else {
    ESP_LOGI(TAG, "Servo CH%d -> Angle: %d, PWM: %u (전송 성공)", channel,
             angle, pwm_val);
  }
}

void update_servos(bool s1_on, bool s2_on) {
  g_servo1_on = s1_on;
  g_servo2_on = s2_on;

  if (g_is_tripped)
    return;

  set_servo_angle(0, s1_on ? SERVO1_ON_DEG : SERVO1_OFF_DEG);
  set_servo_angle(1, s2_on ? SERVO2_ON_DEG : SERVO2_OFF_DEG);

  // 모터와 LED연동
  gpio_set_level(LED1_PIN, s1_on ? 1 : 0);
  gpio_set_level(LED2_PIN, s2_on ? 1 : 0);

  ESP_LOGI(TAG, "Servo1: %s, Servo2: %s", s1_on ? "ON" : "OFF",
           s2_on ? "ON" : "OFF");
}

// 전류 측정

float read_current_rms() {
  float ACCurrtntValue = 0;
  float peakVoltage = 0;
  float voltageVirtualValue = 0; // Vrms
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

  ESP_LOGI(TAG, "[V5] 원본 로직 전류: %.2fA | Raw 평균: %.1f", ACCurrtntValue,
           peakVoltage);

  return ACCurrtntValue;
}

// --- MQTT & Wi-Fi 핸들러 ---

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

    if (strstr(topic, "servo1")) {
      bool target = (strncmp(payload, "on", 2) == 0);
      update_servos(target, g_servo2_on);
    } else if (strstr(topic, "servo2")) {
      bool target = (strncmp(payload, "on", 2) == 0);
      update_servos(g_servo1_on, target);
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

// --- 메인 모니터링 태스크 ---

void strip_logic_task(void *pv) {
  uint32_t stable_counter = 0;
  char payload[128];

  while (1) {
    g_current_a = read_current_rms();

    // 과부하 차단 (2.0A)
    if (g_current_a > OVER_CURRENT_LIMIT && !g_is_tripped) {
      ESP_LOGW(TAG, "!!! TRIP: %.2fA !!!", g_current_a);
      g_is_tripped = true;
      g_servo1_on = false; // 차단 시 서버 상태도 OFF로 동기화하여 자동 복구 시
                           // 모터가 다시 켜지는 현상 방지
      g_servo2_on = false;
      set_servo_angle(0, SERVO1_OFF_DEG);
      set_servo_angle(1, SERVO2_OFF_DEG);

      // 과부하 시 부저 사이렌 울림 및 LED 차단
      gpio_set_level(LED1_PIN, 0);
      gpio_set_level(LED2_PIN, 0);
      gpio_set_level(BUZZER_PIN, 1);

      stable_counter = 0;
    }

    // 복구 시스템
    if (g_is_tripped) {
      if (g_current_a < OVER_CURRENT_LIMIT) {
        stable_counter += 1000;
        if (stable_counter >= STABLE_CHECK_TIME_MS) {
          ESP_LOGI(TAG, "Current Stable. Recovering...");
          g_is_tripped = false;

          // 부저 알람 해제
          gpio_set_level(BUZZER_PIN, 0);

          update_servos(g_servo1_on, g_servo2_on);
          stable_counter = 0;
        }
      } else {
        stable_counter = 0;
      }
    }

    // MQTT 상태 전송
    if (mqtt_client) {
      snprintf(
          payload, sizeof(payload),
          "{\"current\":%.2f, \"s1\":\"%s\", \"s2\":\"%s\", \"tripped\":%s}",
          g_current_a, g_servo1_on ? "ON" : "OFF", g_servo2_on ? "ON" : "OFF",
          g_is_tripped ? "true" : "false");
      esp_mqtt_client_publish(mqtt_client, TOPIC_STATUS, payload, 0, 1, 0);

      ESP_LOGI(TAG, "[실시간] 전류: %.2fA | S1: %s | S2: %s %s", g_current_a,
               g_servo1_on ? "ON " : "OFF", g_servo2_on ? "ON " : "OFF",
               g_is_tripped ? "(!!!과부하 차단!!!)" : "");
    }

    // 아두이노 원본의 loop() 내 1000ms 딜레이(500ms + 500ms) 복구
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void init_all() {
  ESP_ERROR_CHECK(nvs_flash_init());

  // 새로 추가된 부저 및 LED용 GPIO 핀 초기화
  gpio_reset_pin(LED1_PIN);
  gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED1_PIN, 0); // 기본 꺼짐

  gpio_reset_pin(LED2_PIN);
  gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED2_PIN, 0); // 기본 꺼짐

  gpio_reset_pin(BUZZER_PIN);
  gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(BUZZER_PIN, 0); // 알람 꺼짐

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
  wifi_config_t wifi_cfg = {.sta = {
                                .ssid = WIFI_SSID,
                            }};
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

  // PCA9685 초기화 (100kHz 정석 설정)
  ESP_ERROR_CHECK(i2cdev_init());
  memset(&pca9685_dev, 0, sizeof(i2c_dev_t));
  pca9685_dev.cfg.master.clk_speed = 100000; // 100kHz 정석 설정
  pca9685_dev.cfg.sda_pullup_en = true;      // 내부 풀업 활성화
  pca9685_dev.cfg.scl_pullup_en = true;      // 내부 풀업 활성화
  ESP_ERROR_CHECK(pca9685_init_desc(&pca9685_dev, PCA9685_ADDR_BASE,
                                    I2C_MASTER_NUM, I2C_MASTER_SDA_IO,
                                    I2C_MASTER_SCL_IO));
  ESP_ERROR_CHECK(pca9685_init(&pca9685_dev));
  ESP_ERROR_CHECK(pca9685_restart(&pca9685_dev));
  ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca9685_dev, 50));

  // 서보 모터는 보통 Push-Pull 방식을 사용하므로 이를 명시적으로 설정
  ESP_ERROR_CHECK(pca9685_set_output_open_drain(&pca9685_dev, false));

  // ADC
  adc_oneshot_unit_init_cfg_t init_raw = {.unit_id = ADC_UNIT_1};
  adc_oneshot_new_unit(&init_raw, &adc_handle);
  adc_oneshot_chan_cfg_t chan_cfg = {.bitwidth = ADC_BITWIDTH_DEFAULT,
                                     .atten = ADC_ATTEN_DB_12};
  adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg);

  // MQTT 설정 (타임아웃 연장으로 안정성 강화)
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

  // 부팅 시 서보 모터를 무조건 돌려서 "안전하게 모두 OFF" 상태로 강제 세팅
  g_servo1_on = false;
  g_servo2_on = false;
  set_servo_angle(0, SERVO1_OFF_DEG);
  set_servo_angle(1, SERVO2_OFF_DEG);
  ESP_LOGI(TAG, "System Initialized: All Servos forcefully set to OFF");
}

void app_main(void) {
  init_all();
  xTaskCreate(strip_logic_task, "strip_task", 4096, NULL, 5, NULL);
}