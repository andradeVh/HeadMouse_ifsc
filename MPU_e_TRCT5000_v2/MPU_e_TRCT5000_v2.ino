#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <BleMouse.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp32-hal-cpu.h>

// ─── Pinos ────────────────────────────────────────────────
const int sensorPin   = 34;
const int pino_sample = 15;

// ─── BLE Mouse ───────────────────────────────────────────
BleMouse bleMouse("HeadMouse ESP32", "DIY", 100);

// ─── MPU6050 ─────────────────────────────────────────────
MPU6050 mpu;
int16_t gx, gy, gz;
int vx, vy;

// ─── Sensor IR ───────────────────────────────────────────
int sensorValueOn  = 0;
int sensorValueOff = 0; // leitura off não funciona com módulo TCRT5000, só com sensor IR puro

// ─── Filtro de média móvel ────────────────────────────────
#define NUM_LEITURAS 50
unsigned int vetorLeituras[NUM_LEITURAS];
int indexLeituras     = 0;
bool bufferPreenchido = false;
float g_mediaAtual    = 0;
float g_valorBaseLine = -1;

// ─── Parâmetros de detecção de piscada ───────────────────
#define DELTA_PISCADA      20
#define FATOR_RELEASE      0.35
#define TIMEOUT_PISCADA_MS 900
#define ALPHA_BASELINE     0.05

// ─── Estado da piscada ────────────────────────────────────
bool emPiscada = false;
unsigned long tempoInicioPiscada = 0;

// ─── Temporização das fases IR ────────────────────────────
unsigned long ultimoSampleMicros = 0;
const int INTERVALO_FASE_US = 600;
int fase = 1;

// ─── Controle de inatividade / deep sleep ────────────────
#define TEMPO_SEM_BT_MS 60000

unsigned long tempoSemConexao = 0;
bool btConectadoAntes = false;

// ─── Temporização do MPU ──────────────────────────────────
unsigned long ultimoMPU = 0;
const int INTERVALO_MPU_MS = 10;

// ─── Debug ───────────────────────────────────────────────
unsigned long ultimoDebug = 0;

// ─────────────────────────────────────────────────────────

void configurarEnergia() {
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  // 80MHz é o mínimo estável para o stack BLE
  setCpuFrequencyMhz(80);
}

void setCalibration() {
  mpu.CalibrateGyro(7);
}

// ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  configurarEnergia();

  // I2C para o MPU6050
  Wire.begin(21, 22);

  pinMode(pino_sample, OUTPUT);
  digitalWrite(pino_sample, LOW);

  memset(vetorLeituras, 0, sizeof(vetorLeituras));

  // MPU6050 antes do BLE (menos conflito de memória)
  Serial.println("Iniciando MPU6050...");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050: OK");
  } else {
    Serial.println("MPU6050: Erro de conexão!");
  }
  setCalibration();

  Serial.println("Iniciando BLE...");
  bleMouse.begin();

  tempoSemConexao = millis();
  ultimoMPU       = millis();

  Serial.println("HeadMouse iniciado — CPU: 80MHz");
}

// ─────────────────────────────────────────────────────────

void loop() {
  unsigned long agora      = micros();
  unsigned long agoraMilis = millis();

  bool btConectado = bleMouse.isConnected();

  // ── Deep sleep por perda de conexão BLE ───────────────
  if (btConectado) {
    tempoSemConexao  = agoraMilis;
    btConectadoAntes = true;
  } else if (btConectadoAntes && (agoraMilis - tempoSemConexao > TEMPO_SEM_BT_MS)) {
    Serial.println("Sem BLE por 60s — deep sleep de 10s");
    Serial.flush();
    digitalWrite(pino_sample, LOW);
    esp_sleep_enable_timer_wakeup(10ULL * 1000000);
    esp_deep_sleep_start();
  }

  // ── Movimento do mouse via MPU6050 ────────────────────
  if (btConectado && (agoraMilis - ultimoMPU >= INTERVALO_MPU_MS)) {
    ultimoMPU = agoraMilis;

    int16_t _gx, _gy, _gz;
    mpu.getRotation(&_gx, &_gy, &_gz);

    vx = _gy / 200;
    vy = _gz / 200;

    if (abs(vx) < 2) vx = 0;
    if (abs(vy) < 2) vy = 0;

    if (vx != 0 || vy != 0) {
      bleMouse.move(vx, vy);
    }
  }

  // ── Fases do sensor IR (detecção de piscada) ──────────
  if (agora - ultimoSampleMicros >= (unsigned long)INTERVALO_FASE_US) {
    ultimoSampleMicros = agora;

    switch (fase) {
      case 1:
        digitalWrite(pino_sample, HIGH);
        break;
      case 2:
        sensorValueOn = analogRead(sensorPin);
        digitalWrite(pino_sample, LOW);
        break;
      case 3:
        sensorValueOff = analogRead(sensorPin);
        processarLeituraIR(btConectado);
        break;
    }
    fase = (fase >= 3) ? 1 : fase + 1;
  }

  // ── Cede tempo ao FreeRTOS / stack BLE ────────────────
  delay(1);
}

// ─────────────────────────────────────────────────────────

void processarLeituraIR(bool btConectado) {
  int diff = sensorValueOn - sensorValueOff;
  if (diff < 0) diff = 0;

  vetorLeituras[indexLeituras] = diff;
  indexLeituras = (indexLeituras + 1) % NUM_LEITURAS;
  if (!bufferPreenchido && indexLeituras == 0) bufferPreenchido = true;

  unsigned long soma = 0;
  for (int i = 0; i < NUM_LEITURAS; i++) soma += vetorLeituras[i];
  g_mediaAtual = (float)soma / NUM_LEITURAS;

  if (g_valorBaseLine < 0 && bufferPreenchido) {
    g_valorBaseLine = g_mediaAtual;
    Serial.print("Baseline inicializado: ");
    Serial.println(g_valorBaseLine);
    return;
  }
  if (g_valorBaseLine < 0) return;

  // ── Debug ──────────────────────────────────────────────
  if (millis() - ultimoDebug > 150) {
    Serial.print("On:"); Serial.print(sensorValueOn);
    Serial.print(" Off:"); Serial.print(sensorValueOff);
    Serial.print(" Diff:"); Serial.print(diff);
    Serial.print(" Media:"); Serial.print(g_mediaAtual, 1);
    Serial.print(" Base:"); Serial.print(g_valorBaseLine, 1);
    Serial.print(" Estado:"); Serial.print(emPiscada ? "PISCADA" : "normal");
    Serial.print(" BT:"); Serial.println(btConectado ? "OK" : "---");
    ultimoDebug = millis();
  }

  // ── Detecção ───────────────────────────────────────────
  if (!emPiscada) {
    g_valorBaseLine = (g_valorBaseLine * (1.0 - ALPHA_BASELINE))
                    + (g_mediaAtual * ALPHA_BASELINE);

    if (g_mediaAtual < (g_valorBaseLine - DELTA_PISCADA)) {
      emPiscada          = true;
      tempoInicioPiscada = millis();
      if (btConectado) {
        bleMouse.press(MOUSE_LEFT);
        Serial.println(">>> PRESS");
      }
    }
  } else {
    bool voltouAoNormal = (g_mediaAtual > (g_valorBaseLine - DELTA_PISCADA * FATOR_RELEASE));
    bool timeout        = (millis() - tempoInicioPiscada > TIMEOUT_PISCADA_MS);

    if (voltouAoNormal || timeout) {
      emPiscada = false;
      if (btConectado) {
        bleMouse.release(MOUSE_LEFT);
        Serial.println("<<< RELEASE");
      }
    }
  }
}
