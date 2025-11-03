#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>


// NOTE: This code need to be compiled in Arduino IDE with the proper header files, open the CameraWebServer example folder and paste it in CameraWebServer.ino

const char* ssid = "erichome";
const char* password = "20031230";

#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

WebServer server(80);

void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  // Send response headers
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Connection", "close");
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");

  // Send image data directly
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  client.stop();

  esp_camera_fb_return(fb);
}

void startServer() {
  server.on("/capture", HTTP_GET, handleCapture);
  server.begin();
  Serial.println("HTTP server started");
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  WiFi.begin(ssid, password);
  Serial.printf("Connecting to %s", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  Serial.print("Camera Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/capture");

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
    config.frame_size = FRAMESIZE_VGA;  
    config.jpeg_quality = 12;           
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  startServer();
}

void loop() {
  server.handleClient();
}
