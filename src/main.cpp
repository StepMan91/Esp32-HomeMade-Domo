#include "esp_camera.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

// --- Configuration ---
const char *ssid = "Freebox-77D9E1";
const char *password = "secret";
const char *mqtt_server = "192.168.1.183";

// Topics
const char *topic_set = "home/garage/set";
const char *topic_state = "home/garage/state";
const char *topic_cam = "home/garage/camera";
const char *topic_log = "home/garage/log";

// Pins (AI Thinker ESP32-CAM)
const int RELAY_PIN = 14;    // Free GPIO
const int FLASH_LED_PIN = 4; // Onboard High Power LED
const int BUTTON_PIN = 0;    // BOOT Button

const int STATUS_LED = 33; // Small Red LED on back (Active Low)

// PWM for Flash LED
const int FLASH_LED_CHANNEL =
    1; // Use channel 1 (0 is likely used by Camera or adjust if needed)
const int FLASH_PWM_FREQ = 5000;
const int FLASH_PWM_RES = 8;
const int FLASH_DUTY_TARGET = 77; // ~30% of 255

// Camera Pins (AI THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Globals
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastReconnectAttempt = 0;
bool cameraInitialized = false;

// Mock Image (1x1 Pixel Gray JPEG Header) - Fallback if camera fails
const uint8_t mock_jpeg[] = {
    0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01,
    0x01, 0x01, 0x00, 0x48, 0x00, 0x48, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43,
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0x01, 0x00, 0x01, 0x03, 0x01,
    0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xC4, 0x00, 0x15,
    0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xFF, 0xC4, 0x00, 0x14, 0x10,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xC4, 0x00, 0x14, 0x01, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0xC4, 0x00, 0x14, 0x11, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0xDA, 0x00, 0x0C, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03, 0x11,
    0x00, 0x3F, 0x00, 0xB2, 0xC0, 0x07, 0xFF, 0xD9};

// State Machine
enum GarageState { IDLE, RELAY_Active, PHOTO_Wait, PHOTO_Capture };

GarageState currentState = IDLE;
unsigned long stateStartTime = 0;
int photosTaken = 0;
const int PHOTOS_TO_TAKE = 4;
const unsigned long RELAY_DURATION = 2000;
const unsigned long PHOTO_INTERVAL = 5000; // Faster interval (5s) for testing

// Non-blocking blink helpers
unsigned long lastLedTime = 0;
bool ledState = false;

void updateStatusLed(int interval) {
  unsigned long now = millis();
  if (now - lastLedTime >= interval) {
    lastLedTime = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED,
                 ledState ? LOW : HIGH); // LOW is ON for built-in LED
                                         // typically? (Comment says Active Low)
                                         // If Active Low: LOW=ON, HIGH=OFF.
  }
}

void setupCamera() {
  Serial.println("[Cam] Initializing...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Resolution logic
  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA; // 800x600 (Good balance)
    config.jpeg_quality = 12;           // High Quality
    config.fb_count = 2;
    Serial.println("[Cam] PSRAM Found. Using SVGA/High Quality.");
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    Serial.println("[Cam] No PSRAM. Using VGA/High Quality.");
  }

  // Retry Loop for Camera Init
  int retry = 0;
  while (retry < 3) {
    Serial.printf("[Cam] Init Attempt %d/3...\n", retry + 1);

    // Power Cycle Camera
    if (PWDN_GPIO_NUM != -1) {
      digitalWrite(PWDN_GPIO_NUM, HIGH); // Power OFF
      delay(100);
      digitalWrite(PWDN_GPIO_NUM, LOW); // Power ON
      delay(100);
    }

    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
      Serial.println("[Cam] Initialized Successfully.");
      cameraInitialized = true;

      // Sensor Settings (Flip/Mirror if needed)
      sensor_t *s = esp_camera_sensor_get();
      if (s) {
        s->set_vflip(s, 1);      // Flip vertical
        s->set_hmirror(s, 1);    // Flip horizontal
        s->set_brightness(s, 1); // Up brightness
        s->set_saturation(s, 0);
      }
      return; // Success!
    } else {
      Serial.printf("[Cam] Init Warning: 0x%x. Retrying...\n", err);
      // Verify if we need to deinit before retry?
      // esp_camera_deinit(); // Sometimes causing crash if not init?
      delay(1000);
      retry++;
    }
  }

  Serial.println("[Cam] Init FAILED after 3 attempts. Using MOCK mode.");
  cameraInitialized = false;
}

void triggerAction() {
  if (currentState == IDLE) {
    Serial.println(">>> ACTION: BUTTON/MQTT <<<");
    currentState = RELAY_Active;
    stateStartTime = millis();
    photosTaken = 0;

    // Relay ON
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("[Relay] ON");

    if (client.connected())
      client.publish(topic_state, "ON");
  } else {
    Serial.println("[Logic] Ignored (Busy).");
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("[MQTT] Message Recv on: ");
  Serial.println(topic);
  // Any message on topic_set triggers logic
  triggerAction();
}

void takePhoto() {
  Serial.println("[Cam] Capturing...");

  if (cameraInitialized) {
    // Turn ON Flash at 30%
    ledcWrite(FLASH_LED_CHANNEL, FLASH_DUTY_TARGET);
    delay(150); // Stabilization

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[Cam] Capture Failed! Sending Mock.");
      // Fallback to mock
      if (client.connected()) {
        client.beginPublish(topic_cam, sizeof(mock_jpeg), false);
        client.write(mock_jpeg, sizeof(mock_jpeg));
        client.endPublish();
      }
      return;
    }

    Serial.printf("[Cam] JPG Size: %u bytes\n", fb->len);

    if (client.connected()) {
      if (fb->len > MQTT_MAX_PACKET_SIZE) {
        Serial.printf("[MQTT] Error! Image (%u) > MaxSize (%u)\n", fb->len,
                      MQTT_MAX_PACKET_SIZE);
      } else {
        Serial.println("[MQTT] Publishing...");
        if (client.beginPublish(topic_cam, fb->len, false)) {
          client.write(fb->buf, fb->len);
          client.endPublish();
          Serial.println("[MQTT] Published!");
        } else {
          Serial.println("[MQTT] Publish Failed (Connection lost?)");
        }
      }
    }

    // Turn OFF Flash
    ledcWrite(FLASH_LED_CHANNEL, 0);

    esp_camera_fb_return(fb);
  } else {
    // Camera not init, send mock
    Serial.println("[Cam] Not init. Sending Mock.");
    if (client.connected()) {
      client.beginPublish(topic_cam, sizeof(mock_jpeg), false);
      client.write(mock_jpeg, sizeof(mock_jpeg));
      client.endPublish();
      Serial.println("[MQTT] Mock Published");
    }
  }
}

void reconnect() {
  // Blocking reconnect is annoying. We do non-blocking attempts in loop.
  // But for initial setup we can block a bit.
  if (client.connect("ESP32Garage", NULL, NULL, topic_state, 0, false, "OFF")) {
    Serial.println("[MQTT] Connected!");
    client.publish(topic_state, "OFF");
    client.subscribe(topic_set);
    // blinkStatus(3, 100); // Removed blocking blink
  } else {
    Serial.print("[MQTT] Failed rc=");
    Serial.println(client.state());
  }
}

void setup() {
  // Disable Brownout
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println("\n\n=== ESP32 GARAGE V2.5 (HOLISTIC) ===");

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH); // Off

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  // pinMode(FLASH_LED_PIN, OUTPUT); // Handled by PWM ledcAttachPin
  // digitalWrite(FLASH_LED_PIN, LOW); // FORCE OFF

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // PWM Setup for Flash LED
  ledcSetup(FLASH_LED_CHANNEL, FLASH_PWM_FREQ, FLASH_PWM_RES);
  ledcAttachPin(FLASH_LED_PIN, FLASH_LED_CHANNEL);
  ledcWrite(FLASH_LED_CHANNEL, 0); // Ensure OFF initially

  setupCamera();

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Connecting");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); // Simple toggle
    delay(500);
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[WiFi] Connected IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] Failed! Will retry.");
  }

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setBufferSize(
      60000); // Fixed size to prevent overflow (uint16_t max 65535)
}

void loop() {
  // 1. Connectivity & LED Status
  if (WiFi.status() == WL_CONNECTED) {
    updateStatusLed(2000); // Slow blink (2s) = Connected
  } else {
    updateStatusLed(100); // Fast blink (100ms) = Connecting/Lost

    // Reconnect logic
    static unsigned long lastWiFiReconnect = 0;
    if (millis() - lastWiFiReconnect > 10000) {
      lastWiFiReconnect = millis();
      Serial.println("[WiFi] Reconnecting...");
      WiFi.reconnect();
    }
    return; // Skip MQTT and Logic if no WiFi
  }

  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      reconnect();
    }
  } else {
    client.loop();
  }

  // 2. Heartbeat (Every 5s)
  static unsigned long hb = 0;
  if (millis() - hb > 5000) {
    hb = millis();
    Serial.print("[Sys] Alive. Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" State: ");
    Serial.println(currentState);
  }

  // 3. Button
  static int lastBtnState = HIGH;
  static unsigned long lastBtnTime = 0;
  int reading = digitalRead(BUTTON_PIN);

  if (reading == LOW && lastBtnState == HIGH &&
      (millis() - lastBtnTime > 200)) {
    lastBtnTime = millis();
    Serial.println("[Button] IO0 Pressed");
    triggerAction();
  }
  lastBtnState = reading;

  // 4. State Machine
  unsigned long now = millis();
  switch (currentState) {
  case IDLE:
    break;
  case RELAY_Active:
    if (now - stateStartTime >= RELAY_DURATION) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("[Relay] OFF");
      client.publish(topic_state, "OFF");
      currentState = PHOTO_Wait;
      stateStartTime = now - PHOTO_INTERVAL; // Trigger first immediately
      photosTaken = 0;
    }
    break;
  case PHOTO_Wait:
    if (now - stateStartTime >= PHOTO_INTERVAL) {
      currentState = PHOTO_Capture;
      stateStartTime = now;
    }
    break;
  case PHOTO_Capture:
    Serial.printf("[Seq] Taking Photo %d/%d\n", photosTaken + 1,
                  PHOTOS_TO_TAKE);
    takePhoto();
    photosTaken++;
    if (photosTaken >= PHOTOS_TO_TAKE) {
      currentState = IDLE;
      Serial.println("[Seq] Complete.");
    } else {
      currentState = PHOTO_Wait;
      stateStartTime = now;
    }
    break;
  }
}

