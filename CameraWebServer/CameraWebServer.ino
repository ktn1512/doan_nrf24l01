#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

const char* ssid_ap = "ESP32-CAM-SECURITY"; 
const char* password_ap = "12345678";       

IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);


#define MQ6_PIN 15
#define FLASH_PIN 4     

WebServer server(80);

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

int gas_state = 1;
int flash_state = 0;  

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Hệ Thống An Toàn (AP Mode)</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background-color: #222; color: #eee; margin: 0; padding-top: 10px; }
    
    /* Khung Camera */
    #cam-box { position: relative; display: inline-block; width: 95%; max-width: 640px; }
    img { width: 100%; height: auto; border: 4px solid #444; border-radius: 8px; display: block; }
    
    /* Thông số */
    .dashboard { display: flex; justify-content: center; gap: 10px; margin-top: 15px; flex-wrap: wrap; }
    .card { background: #333; padding: 10px 20px; border-radius: 5px; border: 1px solid #555; min-width: 100px; }
    
    /* Cảnh báo */
    .alert-banner { display: none; background: red; color: white; font-weight: bold; padding: 10px; font-size: 1.5rem; margin-bottom: 10px; animation: blink 0.5s infinite; }
    .alert-border { border-color: red !important; box-shadow: 0 0 15px red; }
    .status-danger { color: red; font-weight: bold; }
    .status-safe { color: #0f0; font-weight: bold; }
    
    /* Nút Flash */
    .button-group { margin-top: 15px; }
    .flash-btn { 
      padding: 12px 30px; 
      font-size: 1.1rem; 
      border: none; 
      border-radius: 5px; 
      cursor: pointer;
      transition: all 0.3s;
      font-weight: bold;
    }
    .flash-off { background-color: #444; color: #fff; }
    .flash-on { background-color: #ffcc00; color: #000; box-shadow: 0 0 20px #ffcc00; }
    .flash-btn:hover { transform: scale(1.05); }
    
    @keyframes blink { 50% { opacity: 0.5; } }
  </style>
</head>
<body>

  <div id="alert" class="alert-banner">⚠️ CẢNH BÁO KHÍ GAS! ⚠️</div>

  <div id="cam-box">
    <img id="camera-feed" src="">
  </div>

  <div class="dashboard">
    <div class="card">🛡️ <span id="gas">--</span></div>
  </div>

  <div class="button-group">
    <button class="flash-btn flash-off" id="flashBtn" onclick="toggleFlash()">💡 Flash OFF</button>
  </div>

<script>
  const img = document.getElementById("camera-feed");
  const flashBtn = document.getElementById("flashBtn");
  
  // Hàm lấy ảnh liên tục
  function refreshImage() {
    img.src = "/capture?t=" + new Date().getTime();
  }
  
  img.onload = function() { refreshImage(); }
  img.onerror = function() { setTimeout(refreshImage, 1000); }
  refreshImage();

  // Hàm điều khiển Flash
  function toggleFlash() {
    fetch("/flash?toggle=1")
      .then(r => r.json())
      .then(d => {
        updateFlashButton(d.flash_state);
      });
  }

  // Cập nhật trạng thái nút Flash
  function updateFlashButton(state) {
    if(state == 1) {
      flashBtn.innerText = "💡 Flash ON";
      flashBtn.className = "flash-btn flash-on";
    } else {
      flashBtn.innerText = "💡 Flash OFF";
      flashBtn.className = "flash-btn flash-off";
    }
  }

  // Hàm lấy số liệu cảm biến
  setInterval(() => {
    fetch("/read?nocache=" + Date.now())
      .then(r => r.json())
      .then(d => {
        updateFlashButton(d.flash_state);
        
        if(d.gas == 0) {
          document.getElementById("gas").innerText = "NGUY HIỂM";
          document.getElementById("gas").className = "status-danger";
          document.getElementById("alert").style.display = "block";
          img.className = "alert-border";
          document.body.style.backgroundColor = "#500";
        } else {
          document.getElementById("gas").innerText = "An toàn";
          document.getElementById("gas").className = "status-safe";
          document.getElementById("alert").style.display = "none";
          img.className = "";
          document.body.style.backgroundColor = "#222";
        }
      });
  }, 1000);
</script>

</body>
</html>
)rawliteral";

void handle_capture() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera Capture Failed");
    return;
  }
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handle_sensor_data() {
  gas_state = digitalRead(MQ6_PIN);
  
  String json = "{\"gas\":" + String(gas_state) + ", \"flash_state\":" + String(flash_state) + "}";
  server.send(200, "application/json", json);
}

void handle_flash() {
  flash_state = (flash_state == 1) ? 0 : 1;
  digitalWrite(FLASH_PIN, flash_state);
  
  String json = "{\"flash_state\":" + String(flash_state) + "}";
  server.send(200, "application/json", json);
}

void handle_root() { 
  server.send(200, "text/html; charset=utf-8", index_html); 
}

void setup() {
  Serial.begin(115200);
  
  pinMode(MQ6_PIN, INPUT_PULLUP);
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);  
  
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA; 
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_camera_init(&config);
  
  sensor_t * s = esp_camera_sensor_get();
  s->set_hmirror(s, 1);
  s->set_vflip(s, 0);

  Serial.println("Khoi tao AP Mode...");
  
  WiFi.disconnect(); 
  
  WiFi.softAPConfig(local_ip, gateway, subnet);
  
  WiFi.softAP(ssid_ap, password_ap);

  Serial.print("Da phat WiFi: ");
  Serial.println(ssid_ap);
  Serial.print("Dia chi IP truy cap: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handle_root);
  server.on("/capture", handle_capture);
  server.on("/read", handle_sensor_data);
  server.on("/flash", handle_flash);
  
  server.begin();
  Serial.println("Server da san sang!");
}

void loop() { 
  server.handleClient(); 
}