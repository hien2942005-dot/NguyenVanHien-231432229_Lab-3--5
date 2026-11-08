/*
 * Lab 3 - De 5: Fall Detection voi FreeRTOS - ESP32 + MPU6050 + MQTT (HiveMQ)
 * Ho ten - Lop: Nguyen Van Hien - 231432229
 
 * Kien truc:
 *   Task doc data   (Core 1) -> dataQueue  -> Task truyen data (Core 0) -> MQTT
 *                            -> alertQueue -> Task canh bao    (Core 1) -> LED/Buzzer
 *   Bien dung chung (state, led, buzzer, reset) duoc bao ve bang Mutex.
 */
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>

// ---------- Chan ket noi ----------
#define SDA_PIN    21
#define SCL_PIN    22
#define LED_PIN    26
#define BUZZER_PIN 27

// ---------- WiFi / MQTT ----------
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";
const char* MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;

// Tien to topic
#define BASE "nguyenvanhien_231432229/lab3/"
const char* T_ACCEL   = BASE "accel";
const char* T_TILT    = BASE "tilt";
const char* T_STATUS  = BASE "status";
const char* T_LED_ST  = BASE "led/state";
const char* T_BUZ_ST  = BASE "buzzer/state";
const char* T_LED_CMD = BASE "led/cmd";
const char* T_BUZ_CMD = BASE "buzzer/cmd";
const char* T_RESET   = BASE "reset";

// ---------- Nguong thuat toan ----------
const float    IMPACT_G        = 2.5;    // |A| vuot nguong => va cham
const float    TILT_JUMP_DEG   = 50.0;   // goc nghieng doi qua nhanh trong 300 ms
const float    TILT_FALL_DEG   = 60.0;   // goc nghieng coi la dang nam
const float    LYING_MIN_G     = 0.7;    // nam yen: |A| trong khoang 0.7 - 1.3 g
const float    LYING_MAX_G     = 1.3;
const uint32_t LYING_HOLD_MS   = 1500;   // phai nam yen lien tuc
const uint32_t CHECK_WINDOW_MS = 8000;   // cua so kiem tra sau khi kich hoat

// ---------- Kieu du lieu ----------
typedef struct {
  float accG;    // do lon gia toc tong (g)
  float tilt;    // goc nghieng so voi phuong thang dung (do)
} SensorData;

enum State { NORMAL, CHECKING, FALL };
const char* stateName(State s) {
  return s == NORMAL ? "NORMAL" : (s == CHECKING ? "CHECKING" : "FALL");
}

// ---------- Doi tuong RTOS ----------
QueueHandle_t     dataQueue;    // Task doc data -> Task truyen data
QueueHandle_t     alertQueue;   // Task doc data -> Task canh bao
SemaphoreHandle_t stateMutex;   // bao ve cac bien dung chung ben duoi

// Bien dung chung (chi truy cap khi da giu stateMutex)
State g_state    = NORMAL;
bool  g_ledOn    = false;
bool  g_buzOn    = false;
bool  g_resetReq = false;

WiFiClient   espClient;
PubSubClient client(espClient);
const uint8_t MPU_ADDR = 0x68;

#define LOCK()   xSemaphoreTake(stateMutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(stateMutex)

// ================= MPU6050 =================
bool mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);            // danh thuc cam bien
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x10);            // gia toc +-8g (4096 LSB/g)
  Wire.endTransmission();
  return true;
}

bool readMpu(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                              // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true) != 6) return false;
  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();
  ax = rx / 4096.0; ay = ry / 4096.0; az = rz / 4096.0;
  return true;
}

// ================= TASK DOC DATA (Core 1) =================
void taskReadData(void *pv) {
  TickType_t last = xTaskGetTickCount();
  uint8_t cnt = 0;
  for (;;) {
    float ax, ay, az;
    if (readMpu(ax, ay, az)) {
      SensorData d;
      d.accG = sqrt(ax * ax + ay * ay + az * az);
      d.tilt = atan2(sqrt(ax * ax + ay * ay), az) * 180.0 / PI;

      xQueueSend(alertQueue, &d, 0);             // moi 50 ms -> Task canh bao

      if (++cnt >= 10) {                         // moi 500 ms
        cnt = 0;
        xQueueSend(dataQueue, &d, 0);            // -> Task truyen data
        LOCK(); State s = g_state; UNLOCK();
        Serial.printf("[Doc data  | core %d] accel=%.2f g | tilt=%.1f deg | status=%s\n",
                      xPortGetCoreID(), d.accG, d.tilt, stateName(s));
      }
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));   // chu ky co dinh 50 ms
  }
}

// ================= TASK CANH BAO (Core 1) =================
static State    aState = NORMAL;     // trang thai thuat toan (chi Task canh bao ghi)
static uint32_t tCheckStart = 0, lyingSince = 0;

void detectFall(const SensorData &d, uint32_t now) {
  // Theo doi goc nghieng thay doi dot ngot (so voi 300 ms truoc)
  static float tiltRef = 0; static uint32_t tRef = 0;
  bool tiltJump = false;
  if (now - tRef >= 300) {
    tiltJump = fabs(d.tilt - tiltRef) > TILT_JUMP_DEG;
    tiltRef = d.tilt; tRef = now;
  }

  if (aState == NORMAL) {
    if (d.accG > IMPACT_G || tiltJump) {         // dau hieu 1: va cham / nghieng dot ngot
      aState = CHECKING; tCheckStart = now; lyingSince = 0;
      Serial.println("[Canh bao  | core 1] Nghi te nga, dang kiem tra tu the...");
    }
  } else if (aState == CHECKING) {
    bool lying = d.tilt > TILT_FALL_DEG && d.accG > LYING_MIN_G && d.accG < LYING_MAX_G;
    if (lying) {                                 // dau hieu 2: nam yen, goc nghieng lon
      if (!lyingSince) lyingSince = now;
      if (now - lyingSince >= LYING_HOLD_MS) {
        aState = FALL;
        LOCK(); g_ledOn = true; g_buzOn = true; UNLOCK();
        Serial.println("[Canh bao  | core 1] !!! PHAT HIEN TE NGA !!!");
      }
    } else lyingSince = 0;

    if (aState == CHECKING && now - tCheckStart > CHECK_WINDOW_MS) {
      aState = NORMAL;                           // bao dong gia
      Serial.println("[Canh bao  | core 1] Khong phai te nga, ve NORMAL");
    }
  }
}

void applyOutputs() {
  LOCK();
  if (g_resetReq) {                              // lenh reset tu Web
    aState = NORMAL; lyingSince = 0;
    g_ledOn = false; g_buzOn = false; g_resetReq = false;
  }
  g_state = aState;
  bool led = g_ledOn, buz = g_buzOn;
  UNLOCK();

  bool blink = (millis() / 250) % 2;
  digitalWrite(LED_PIN, led && (aState != FALL || blink));
  if (buz) tone(BUZZER_PIN, (aState == FALL && blink) ? 1000 : 2000);
  else     noTone(BUZZER_PIN);
}

void taskAlert(void *pv) {
  for (;;) {
    SensorData d;
    // Cho toi da 50 ms; het han van chay tiep de cap nhat nhap nhay + lenh tu Web
    if (xQueueReceive(alertQueue, &d, pdMS_TO_TICKS(50)) == pdTRUE) {
      detectFall(d, millis());
    }
    applyOutputs();
  }
}

// ================= TASK TRUYEN DATA (Core 0) =================
void onMessage(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim(); msg.toUpperCase();
  String t = topic;

  LOCK();
  if (t == T_LED_CMD)      g_ledOn = (msg == "ON");
  else if (t == T_BUZ_CMD) g_buzOn = (msg == "ON");
  else if (t == T_RESET)   g_resetReq = true;
  UNLOCK();
  Serial.printf("[Truyen data | core %d] nhan lenh %s = %s\n", xPortGetCoreID(), topic, msg.c_str());
}

void reconnectMqtt() {
  static uint32_t last = 0;
  if (millis() - last < 3000) return;            // thu lai moi 3 giay
  last = millis();
  String id = "esp32-fall-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (client.connect(id.c_str())) {
    Serial.println("[Truyen data] MQTT connected");
    client.subscribe(T_LED_CMD);
    client.subscribe(T_BUZ_CMD);
    client.subscribe(T_RESET);
  } else {
    Serial.printf("[Truyen data] MQTT loi, rc=%d\n", client.state());
  }
}

void publishAll(const SensorData &d) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", d.accG); client.publish(T_ACCEL, buf);
  snprintf(buf, sizeof(buf), "%.1f", d.tilt); client.publish(T_TILT, buf);
  LOCK(); State s = g_state; bool led = g_ledOn, buz = g_buzOn; UNLOCK();
  client.publish(T_STATUS, stateName(s));
  client.publish(T_LED_ST, led ? "ON" : "OFF");
  client.publish(T_BUZ_ST, buz ? "ON" : "OFF");
}

void taskComm(void *pv) {
  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(onMessage);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
  Serial.println("[Truyen data] Dang ket noi WiFi...");
  bool wifiReported = false;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {         // chua co WiFi: khong lam gi them
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    if (!wifiReported) {
      wifiReported = true;
      Serial.printf("[Truyen data | core %d] WiFi OK\n", xPortGetCoreID());
    }
    if (!client.connected()) reconnectMqtt();
    client.loop();

    SensorData d;                                // WiFi OK -> doc dataQueue va gui MQTT
    if (xQueueReceive(dataQueue, &d, pdMS_TO_TICKS(50)) == pdTRUE && client.connected()) {
      publishAll(d);
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  if (!mpuInit()) Serial.println("Khong tim thay MPU6050!");

  stateMutex = xSemaphoreCreateMutex();
  dataQueue  = xQueueCreate(10, sizeof(SensorData));
  alertQueue = xQueueCreate(10, sizeof(SensorData));

  //                 ham          ten                stack uu tien handle core
  xTaskCreatePinnedToCore(taskComm,     "Task truyen data", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskReadData, "Task doc data",    4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskAlert,    "Task canh bao",    4096, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));               // moi viec do cac task dam nhiem
}
