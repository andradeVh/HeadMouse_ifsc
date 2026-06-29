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
const int PIN_BOOT    = 0;   // Botão BOOT do ESP32

// ─── Modos de operação ────────────────────────────────────
// 1 = só movimento (MPU)   2 = só piscada (IR)   3 = tudo
enum Modo { MODO_MOVIMENTO = 1, MODO_PISCADA = 2, MODO_COMPLETO = 3 };
Modo modoAtual = MODO_COMPLETO;

const char* nomeModo[] = { "", "SO MOVIMENTO (MPU)", "SO PISCADA (IR)", "COMPLETO" };

// ─── Debounce do botão BOOT ───────────────────────────────
unsigned long ultimoPressBoot = 0;
const int DEBOUNCE_MS = 300;

// ─── BLE Mouse ───────────────────────────────────────────
BleMouse bleMouse("HeadMouse ESP32", "DIY", 100);

// ─── MPU6050 ─────────────────────────────────────────────
MPU6050 mpu;
int16_t gx, gy, gz;
int vx, vy;

// ─── Sensor IR ───────────────────────────────────────────
int sensorValueOn  = 0;
int sensorValueOff = 0;

// ─── Filtro de média móvel ────────────────────────────────
#define NUM_LEITURAS 50
unsigned int vetorLeituras[NUM_LEITURAS];
int indexLeituras     = 0;
bool bufferPreenchido = false;
float g_mediaAtual    = 0;
float g_valorBaseLine = -1;

// ─── Parâmetros de detecção de piscada ───────────────────
#define DELTA_PISCADA      15
#define FATOR_RELEASE      0.35
#define TIMEOUT_PISCADA_MS 900
#define ALPHA_BASELINE     0.1

// ─── Estado da piscada ────────────────────────────────────
bool emPiscada = false;
unsigned long tempoInicioPiscada = 0;

// ─── Temporização das fases IR ────────────────────────────
unsigned long ultimoSampleMicros = 0;
const int INTERVALO_FASE_US = 600;
int fase = 1;

// ─── Controle de deep sleep ───────────────────────────────
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
  setCpuFrequencyMhz(80);
}

void setCalibration() {
  mpu.CalibrateGyro(7);
}

void avancarModo() {
  // Cicla 1 → 2 → 3 → 1
  modoAtual = (Modo)(modoAtual % 3 + 1);

  // Se saiu do modo piscada, garante LED IR desligado
  if (modoAtual == MODO_MOVIMENTO) {
    digitalWrite(pino_sample, LOW);
    emPiscada = false;
  }

  Serial.print(">>> MODO: ");
  Serial.println(nomeModo[modoAtual]);
}

// ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  configurarEnergia();

  Wire.begin(21, 22);

  pinMode(pino_sample, OUTPUT);
  digitalWrite(pino_sample, LOW);

  pinMode(PIN_BOOT, INPUT_PULLUP);

  memset(vetorLeituras, 0, sizeof(vetorLeituras));

  Serial.println("Iniciando MPU6050...");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050: OK");
  } else {
    Serial.println("MPU6050: Erro de conexao!");
  }
  setCalibration();

  Serial.println("Iniciando BLE...");
  bleMouse.begin();

  tempoSemConexao = millis();
  ultimoMPU       = millis();

  Serial.print("HeadMouse iniciado — CPU: 80MHz — Modo inicial: ");
  Serial.println(nomeModo[modoAtual]);
}

// ─────────────────────────────────────────────────────────

void loop() {
  unsigned long agora      = micros();
  unsigned long agoraMilis = millis();

  bool btConectado = bleMouse.isConnected();

  // ── Botão BOOT: troca de modo ─────────────────────────
  if (digitalRead(PIN_BOOT) == LOW) {
    if (agoraMilis - ultimoPressBoot > DEBOUNCE_MS) {
      ultimoPressBoot = agoraMilis;
      avancarModo();
    }
  }

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
  bool movimentoAtivo = (modoAtual == MODO_MOVIMENTO || modoAtual == MODO_COMPLETO);
  if (btConectado && movimentoAtivo && (agoraMilis - ultimoMPU >= INTERVALO_MPU_MS)) {
    ultimoMPU = agoraMilis;

    int16_t _gx, _gy, _gz;
    mpu.getRotation(&_gx, &_gy, &_gz);

    vx = -_gx / 150;
    vy = _gz / 150;

    if (abs(vx) < 2) vx = 0;
    if (abs(vy) < 2) vy = 0;

    if (vx != 0 || vy != 0) {
      bleMouse.move(vx, vy);
    }
  }

  // ── Fases do sensor IR (detecção de piscada) ──────────
  bool piscadaAtiva = (modoAtual == MODO_PISCADA || modoAtual == MODO_COMPLETO);
  if (piscadaAtiva && (agora - ultimoSampleMicros >= (unsigned long)INTERVALO_FASE_US)) {
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
  } else if (!piscadaAtiva) {
    // Garante LED IR apagado quando modo piscada está desativado
    digitalWrite(pino_sample, LOW);
  }

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
    Serial.print(" Modo:"); Serial.print(nomeModo[modoAtual]);
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
