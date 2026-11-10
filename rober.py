#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <HTTPClient.h>

// ===================== RED =====================
const char* WIFI_SSID = "moto sam";         
const char* WIFI_PASS = "Samlopez246";     

const uint16_t PUERTO_UDP = 4210;
WiFiUDP udp;
bool udpIniciado = false;

IPAddress ipPC(10, 181, 62, 59);  // IP fija de tu PC

unsigned long ultimoMov = 0;
unsigned long ultimoIntentoWifi = 0;
unsigned long ultimaBaliza = 0;

const unsigned long TIMEOUT_MOV_MS = 400;      
const unsigned long PERIODO_BALIZA_MS = 1000;

// ===================== PINES TB6612 =====================
const int PWMA1 = 4,  AIN1 = 16, AIN2 = 17;
const int PWMB1 = 25, BIN1 = 26, BIN2 = 27;

// ===================== ENCODERS Y RUEDAS =====================
const int ENCODER_M1_PIN = 35;
const int ENCODER_M2_PIN = 34;
const unsigned long DEBOUNCE_US = 50;  

volatile long pulsosM1 = 0, pulsosM2 = 0;
volatile unsigned long tUltM1 = 0, tUltM2 = 0;

int sentidoEncM1 = 1, sentidoEncM2 = 1;  
bool movimientoActivo = false;

const float DIAMETRO_RUEDA = 0.049;  // Metros
const float PERIMETRO_RUEDA = PI * DIAMETRO_RUEDA;
const int PPR = 370;
const float DISTANCIA_ENTRE_RUEDAS = 0.15;  // Metros

float velocidadM1 = 0, velocidadM2 = 0;
float orientacionZ = 0; // Ángulo Yaw acumulado (grados)
float posX = 0, posY = 0; // Posición cartesiana en cm

// ===================== MPU6050 =====================
const uint8_t MPU_ADDR = 0x68;
float sesgoGiroZ = 0;

TaskHandle_t TaskRed;
SemaphoreHandle_t xMutex;

// Interrupciones ultrarrápidas de Encoders
void IRAM_ATTR contarM1() {
  unsigned long t = micros();
  if (t - tUltM1 >= DEBOUNCE_US) { pulsosM1++; tUltM1 = t; }
}
void IRAM_ATTR contarM2() {
  unsigned long t = micros();
  if (t - tUltM2 >= DEBOUNCE_US) { pulsosM2++; tUltM2 = t; }
}

// Configuración MPU6050
void inicializarMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(true); // ±500 °/s
}

bool leerGiroZ(float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true) != 2) return false;
  gz = (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5f;
  return true;
}

void calibrarGiro() {
  float suma = 0; int n = 0;
  for (int i = 0; i < 150; i++) {
    float g; if (leerGiroZ(g)) { suma += g; n++; }
    delay(3);
  }
  if (n > 0) sesgoGiroZ = suma / n;
}

// Control de Motores
void moverMotores(int dirM1, int pwm1, int dirM2, int pwm2) {
  pwm1 = constrain(pwm1, 0, 255);
  pwm2 = constrain(pwm2, 0, 255);

  if (dirM1 > 0)      { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  sentidoEncM1 = 1; }
  else if (dirM1 < 0) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); sentidoEncM1 = -1; }
  else                { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  pwm1 = 0; }
  analogWrite(PWMA1, pwm1);

  if (dirM2 > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  sentidoEncM2 = 1; }
  else if (dirM2 < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); sentidoEncM2 = -1; }
  else                { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  pwm2 = 0; }
  analogWrite(PWMB1, pwm2);

  movimientoActivo = (dirM1 != 0 || dirM2 != 0);
}

// Procesar comandos UDP del Guante
void procesarMensaje(char* buf) {
  int len = strlen(buf);
  while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r' || buf[len - 1] == ' ')) buf[--len] = 0;

  bool hayMov = false;
  char mov[12] = "";
  int pwm = 200;

  char* ctx = NULL;
  for (char* tok = strtok_r(buf, ",", &ctx); tok; tok = strtok_r(NULL, ",", &ctx)) {
    char* sep = strchr(tok, ':');
    if (!sep) continue;
    *sep = 0;
    char* clave = tok; char* valor = sep + 1;

    if (strcmp(clave, "MOV") == 0) {
      strncpy(mov, valor, sizeof(mov) - 1); hayMov = true;
    } else if (strcmp(clave, "PWM") == 0) {
      pwm = constrain(atoi(valor), 0, 255);
    }
  }

  if (hayMov) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      // Mapeo corregido de comandos del guante
      if      (strcmp(mov, "DER") == 0)      moverMotores(1, pwm, 1, pwm);   // Adelante
      else if (strcmp(mov, "IZQ") == 0)      moverMotores(-1, pwm, -1, pwm); // Atrás
      else if (strcmp(mov, "ADELANTE") == 0) moverMotores(1, pwm, -1, pwm);  // Giro Derecha
      else if (strcmp(mov, "ATRAS") == 0)    moverMotores(-1, pwm, 1, pwm);  // Giro Izquierda
      else                                   moverMotores(0, 0, 0, 0);
      xSemaphoreGive(xMutex);
    }
    ultimoMov = millis();
  }
}

void recibirMensajes() {
  while (udp.parsePacket() > 0) {
    char buf[128];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len <= 0) continue;
    buf[len] = 0;
    procesarMensaje(buf);
  }
}

// Detección inteligente del estado del robot (giros y movimientos)
const char* determinarEstado(float gz) {
  if (!movimientoActivo) return "detenido";
  if (gz > 15.0) return "girando_izq";
  if (gz < -15.0) return "girando_der";
  if (sentidoEncM1 > 0 && sentidoEncM2 > 0) return "avanzando";
  return "retrocediendo";
}

// Envío ultrarrápido de telemetría HTTP en Core 0
void enviarMuestraHTTP(float v1, float v2, float x, float y, float yaw, const char* estado) {
  static char payload[250];
  int n = snprintf(payload, sizeof(payload), 
      "{\"samples\":[{\"t\":%lu,\"vl\":%.1f,\"vr\":%.1f,\"x\":%.1f,\"y\":%.1f,\"yaw\":%.1f,\"batt\":0.0,\"state\":\"%s\"}]}",
      millis(), v1, v2, x, y, yaw, estado);

  HTTPClient http;
  http.setTimeout(500);          
  http.setConnectTimeout(500);   
  http.begin(String("http://") + ipPC.toString() + ":5000/api/telemetry");
  http.addHeader("Content-Type", "application/json");
  http.POST((uint8_t*)payload, n);
  http.end();
}

// Tarea dedicada exclusivamente a la Red (Core 0)
void tareaRed(void * pvParameters) {
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!udpIniciado) { udp.begin(PUERTO_UDP); udpIniciado = true; }
      recibirMensajes();
      if (millis() - ultimaBaliza >= PERIODO_BALIZA_MS) {
        ultimaBaliza = millis();
        udp.beginPacket(WiFi.broadcastIP(), PUERTO_UDP);
        udp.print("BCN,CAR");
        udp.endPacket();
      }
    } else {
      WiFi.reconnect();
    }
    vTaskDelay(5 / portTICK_PERIOD_MS); 
  }
}

unsigned long ultimoUs = 0;

void setup() {
  Serial.begin(115200);
  xMutex = xSemaphoreCreateMutex();

  pinMode(PWMA1, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB1, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  moverMotores(0, 0, 0, 0);

  pinMode(ENCODER_M1_PIN, INPUT); pinMode(ENCODER_M2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_M1_PIN), contarM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_M2_PIN), contarM2, RISING);

  Wire.begin(21, 22); Wire.setClock(400000); Wire.setTimeOut(10);
  inicializarMPU();
  calibrarGiro();

  WiFi.mode(WIFI_STA); WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(200); }

  // Lanzar red en Core 0
  xTaskCreatePinnedToCore(tareaRed, "TaskRed", 4096, NULL, 2, &TaskRed, 0);
  ultimoUs = micros();
}

void loop() {
  unsigned long ahoraUs = micros();
  float dt = (ahoraUs - ultimoUs) / 1e6f;
  if (dt >= 0.1) { // Muestreo rígido a 10 Hz (cada 100ms) en Core 1
    ultimoUs = ahoraUs;

    noInterrupts();
    long p1 = pulsosM1; long p2 = pulsosM2;
    pulsosM1 = 0; pulsosM2 = 0;
    interrupts();

    float d1 = sentidoEncM1 * ((float)p1 / PPR) * PERIMETRO_RUEDA;
    float d2 = sentidoEncM2 * ((float)p2 / PPR) * PERIMETRO_RUEDA;
    velocidadM1 = (d1 / dt) * 100.0f; // cm/s
    velocidadM2 = (d2 / dt) * 100.0f; // cm/s

    float gz = 0;
    if (leerGiroZ(gz)) {
      gz -= sesgoGiroZ;
      if (abs(gz) < 0.5) gz = 0; 
      orientacionZ += gz * dt;   
    }

    float distanciaPasoCm = (d1 + d2) / 2.0f * 100.0f;
    float rad = orientacionZ * (PI / 180.0f);
    posX += distanciaPasoCm * cos(rad);
    posY += distanciaPasoCm * sin(rad);

    const char* estado = determinarEstado(gz);
    enviarMuestraHTTP(velocidadM1, velocidadM2, posX, posY, orientacionZ, estado);
  }

  // Failsafe de seguridad por pérdida de señal del guante
  if (millis() - ultimoMov > TIMEOUT_MOV_MS) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      moverMotores(0, 0, 0, 0);
      xSemaphoreGive(xMutex);
    }
  }
  delay(1);
}
