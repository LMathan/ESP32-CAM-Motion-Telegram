#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// WiFi Credentials
const char* ssid = "1";  // Change this to your WiFi SSID
const char* password = "123456778";  // Change this to your WiFi password

// Telegram Bot Credentials
String token = "*******************************";  
String chat_id = "**************";

// Camera Model AI Thinker (ESP32-CAM) Pin Configuration
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4
#define PIR_SENSOR_PIN    13

// Motion Detection Variables
bool motionDetected = false;
unsigned long lastTriggerTime = 0;
unsigned long triggerDelay = 10000; // Avoid continuous triggers (5 sec delay)

// Setup Function
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(115200);
  delay(100);

  // PIR Sensor Setup
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Connect to WiFi
  connectToWiFi();

  // Camera Configuration
  setupCamera();
}

// Loop Function
void loop() {
  checkWiFiConnection(); // Reconnect if WiFi is lost

  int pirValue = digitalRead(PIR_SENSOR_PIN);
  
  if (pirValue == HIGH) {
    if (!motionDetected || (millis() - lastTriggerTime > triggerDelay)) {
      motionDetected = true;
      lastTriggerTime = millis();
      Serial.println("🚨 Motion detected! Capturing and sending image...");
      
      String response = sendPhotoToTelegram(token, chat_id, "🚨 Motion detected!");

      Serial.println("📩 Telegram Response: " + response);
    }
  } else {
    motionDetected = false; // Reset for new triggers
  }

  delay(500); // Reduce false triggers
}

// Function to Connect to WiFi
void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("🔗 Connecting to WiFi");
  
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    Serial.print(".");
    delay(500);
    wifi_attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected to WiFi");
  } else {
    Serial.println("\n❌ Failed to connect to WiFi. Restarting...");
    ESP.restart();
  }
}

// Function to Check and Reconnect WiFi
void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi Disconnected! Reconnecting...");
    WiFi.disconnect();
    WiFi.reconnect();
    delay(5000); // Wait before retrying
  }
}

// Function to Setup ESP32-CAM
void setupCamera() {
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

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;  // Lower frame size for stability
    config.jpeg_quality = 12;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed with error 0x%x\n", err);
    delay(1000);
    ESP.restart();
  }
}

// Function to Capture and Send Photo to Telegram
// Function to Capture and Send Photo to Telegram
String sendPhotoToTelegram(String token, String chat_id, String caption) {
  const char* myDomain = "api.telegram.org";
  String response = "";

  digitalWrite(FLASH_LED_PIN, HIGH); // Turn on flash before capturing
  delay(200); // Small delay to ensure flash is active

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Camera capture failed");
    digitalWrite(FLASH_LED_PIN, LOW); // Turn off flash if capture fails
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  digitalWrite(FLASH_LED_PIN, LOW); // Turn off flash after capturing

  WiFiClientSecure client;
  client.setInsecure();

  if (client.connect(myDomain, 443)) {
    Serial.println("✅ Connected to Telegram API");

    String head = "--boundary\r\n";
    head += "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n" + chat_id + "\r\n";
    head += "--boundary\r\n";
    head += "Content-Disposition: form-data; name=\"caption\"\r\n\r\n" + caption + "\r\n";
    head += "--boundary\r\n";
    head += "Content-Disposition: form-data; name=\"photo\"; filename=\"image.jpg\"\r\n";
    head += "Content-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--boundary--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;

    client.println("POST /bot" + token + "/sendPhoto HTTP/1.1");
    client.println("Host: " + String(myDomain));
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=boundary");
    client.println();
    client.print(head);
    client.write(fb->buf, fb->len);
    client.print(tail);

    esp_camera_fb_return(fb);

    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) { // 5 sec timeout
        Serial.println("⏳ Telegram API response timeout");
        client.stop();
        return "Timeout";
      }
    }

    while (client.available()) {
      char c = client.read();
      response += c;
    }

    Serial.println("📩 Telegram Response:");
    Serial.println(response);
    client.stop();
  } else {
    Serial.println("❌ Failed to connect to Telegram API");
    response = "Connection failed";
  }
  return response;
}
