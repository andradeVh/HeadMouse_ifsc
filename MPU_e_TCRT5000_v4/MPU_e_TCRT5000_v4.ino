#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <BleMouse.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp32-hal-cpu.h>

// ─── Pinos ────────────────────────────────────────────────
const int sensorPin = 35;   // A0 do módulo TCRT5000 → ADC1
const int PIN_BOOT  = 0;    // Botão BOOT do ESP32

// ─── Modos de operação ────────────────────────────────────
enum Modo { MODO_COMPLETO = 1, MODO_PISCADA = 2, MODO_MOVIMENTO = 3 };
Modo modoAtual = MODO_PISCADA;
const char* nomeModo[] = { "", "SO MOVIMENTO", "SO PISCADA", "COMPLETO" };

// ─── Debounce do botão BOOT ───────────────────────────────
unsigned long ultimoPressBoot = 0;
const int DEBOUNCE_MS = 300;

// ─── BLE Mouse ───────────────────────────────────────────
BleMouse bleMouse("HeadMouse ESP32", "DIY", 100);

// ─── MPU6050 ─────────────────────────────────────────────
MPU6050 mpu;
int16_t gx, gy, gz;
int vx, vy;

// ─── Buffers de leituras e médias ────────────────────────
// Dois níveis: leituras brutas → médias → derivada
#define NUM_LEITURAS 50
#define NUM_MEDIAS   50

unsigned int  vetorLeituras[NUM_LEITURAS];
unsigned long vetorMedias[NUM_MEDIAS];
int indexLeituras = 0;
int indexMedias   = 0;

float g_mediaAtual    = 0;
float g_mediaAnterior = 0;
float g_valorBaseLine = 0;

// ─── Parâmetros de detecção ───────────────────────────────
#define DELTA_PISCADA          20
#define FATOR_RELEASE          0.35
#define TIMEOUT_PISCADA_MS     900
#define VELOCIDADE_MIN_PISCADA 11.0   // Queda mínima entre médias consecutivas

// Alpha adaptativo: baseline atualiza devagar em repouso,
// rápido durante mudanças bruscas de luz ambiente
#define ALPHA_MIN  0.03
#define ALPHA_MAX  0.30
#define ERRO_MIN   5.0
#define ERRO_MAX   60.0

// Tempos da máquina de estados
#define TEMPO_AQUECIMENTO_MS   5000   // Descarta leituras no boot
#define TEMPO_REPOUSO_MS       300    // Repouso mínimo entre piscadas
#define TIMEOUT_RESET_BASE_MS  700   // Repouso longo → re-ancora baseline

// ─── Máquina de estados do sensor IR ─────────────────────
enum EstadoIR {
  IR_AQUECENDO,
  IR_CALIBRANDO,
  IR_REPOUSO,       // Exige repouso confirmado antes de liberar detecção
  IR_DETECTANDO,    // Monitora condições de início de piscada
  IR_PISCADA        // Aguarda fim da piscada
};
EstadoIR estadoIR = IR_AQUECENDO;

unsigned long tempoInicioAquecimento = 0;
unsigned long tempoInicioRepouso     = 0;
unsigned long tempoInicioPiscada     = 0;
unsigned long tempoUltimoRepouso     = 0;

// ─── Controle geral ───────────────────────────────────────
#define TEMPO_SEM_BT_MS    60000
unsigned long tempoSemConexao = 0;
bool btConectadoAntes = false;

unsigned long ultimoMPU   = 0;
const int INTERVALO_MPU_MS = 10;

// Leitura IR: ciclo simples de 2 fases (LED ligado → lê → processa)
// Fase OFF removida: módulo TCRT5000 precisa de VCC fixo,
// sensorValueOff sempre zero. Leitura contínua com baseline adaptativo.
unsigned long ultimoSampleUs = 0;
const int INTERVALO_US = 1800;

unsigned long ultimoDebug = 0;

// ─────────────────────────────────────────────────────────

void configurarEnergia() {
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  setCpuFrequencyMhz(80);
}

void avancarModo() {
  modoAtual = (Modo)(modoAtual % 3 + 1);
  Serial.print(">>> MODO: ");
  Serial.println(nomeModo[modoAtual]);
}

// ─── Calcula média e salva no buffer de médias ───────────
float calcularMedia() {
  unsigned long soma = 0;
  for (int i = 0; i < NUM_LEITURAS; i++) soma += vetorLeituras[i];
  float media = (float)soma / NUM_LEITURAS;
  vetorMedias[indexMedias] = (unsigned long)media;
  indexMedias = (indexMedias + 1) % NUM_MEDIAS;
  return media;
}

void setup() {
  Serial.begin(115200);
  configurarEnergia();

  Wire.begin(21, 22);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  memset(vetorLeituras, 0, sizeof(vetorLeituras));
  memset(vetorMedias,   0, sizeof(vetorMedias));

  Serial.println("Iniciando MPU6050...");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050: OK");
  } else {
    Serial.println("MPU6050: ERRO!");
  }
  mpu.CalibrateGyro(7);

  bleMouse.begin();

  tempoInicioAquecimento = millis();
  tempoSemConexao        = millis();
  ultimoMPU              = millis();

  Serial.print("HeadMouse iniciado — CPU: 80MHz — Modo: ");
  Serial.println(nomeModo[modoAtual]);
}

// ─────────────────────────────────────────────────────────

void loop() {
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
  } else if (btConectadoAntes && agoraMilis - tempoSemConexao > TEMPO_SEM_BT_MS) {
    Serial.println("Sem BLE 60s — deep sleep 10s");
    Serial.flush();
    esp_sleep_enable_timer_wakeup(10ULL * 1000000);
    esp_deep_sleep_start();
  }

  // ── Sensor IR ─────────────────────────────────────────
  bool piscadaAtiva = (modoAtual == MODO_PISCADA || modoAtual == MODO_COMPLETO);

  if (piscadaAtiva) {
    unsigned long agoraUs = micros();
    if (agoraUs - ultimoSampleUs >= (unsigned long)INTERVALO_US) {
      ultimoSampleUs = agoraUs;
      int leitura = analogRead(sensorPin);
      processarLeituraIR(leitura, btConectado, agoraMilis);
    }
  }

  // ── MPU6050 ───────────────────────────────────────────
  bool movimentoAtivo = (modoAtual == MODO_MOVIMENTO || modoAtual == MODO_COMPLETO);

  if (btConectado && movimentoAtivo) {
    if (agoraMilis - ultimoMPU >= INTERVALO_MPU_MS) {
      ultimoMPU = agoraMilis;

      int16_t _gx, _gy, _gz;
      mpu.getRotation(&_gx, &_gy, &_gz);

      vx = -_gx / 100;
      vy =  _gz / 100;

      if (abs(vx) < 2) vx = 0;
      if (abs(vy) < 2) vy = 0;

      if (vx != 0 || vy != 0) {
        bleMouse.move(vx, vy); // ← movimento reativado
      }
    }
  }

  delay(1);
}

// ─────────────────────────────────────────────────────────

void processarLeituraIR(int leitura, bool btConectado, unsigned long agora) {

  // ── IR_AQUECENDO: descarta 500ms iniciais ────────────
  if (estadoIR == IR_AQUECENDO) {
    if (agora - tempoInicioAquecimento < TEMPO_AQUECIMENTO_MS) return;
    estadoIR = IR_CALIBRANDO;
    Serial.println("Aquecimento OK — calibrando...");
  }

  // ── Atualiza buffer de leituras e calcula média ───────
  vetorLeituras[indexLeituras] = leitura;
  indexLeituras = (indexLeituras + 1) % NUM_LEITURAS;

  g_mediaAnterior = g_mediaAtual;
  g_mediaAtual    = calcularMedia();

  // ── IR_CALIBRANDO: aguarda buffers estabilizarem ─────
  if (estadoIR == IR_CALIBRANDO) {
    static int ciclos = 0;
    g_valorBaseLine = g_mediaAtual;
    if (++ciclos < NUM_LEITURAS + NUM_MEDIAS) return;
    ciclos       = 0;
    estadoIR     = IR_REPOUSO;
    tempoInicioRepouso  = agora;
    tempoUltimoRepouso  = agora;
    Serial.print("Sensor ativo — baseline: ");
    Serial.println(g_valorBaseLine);
  }

  // ── Debug ─────────────────────────────────────────────
  if (agora - ultimoDebug > 150) {
    float vel = g_mediaAnterior - g_mediaAtual;
    Serial.print("Leit:"); Serial.print(leitura);
    Serial.print(" Med:");  Serial.print(g_mediaAtual, 1);
    Serial.print(" Base:"); Serial.print(g_valorBaseLine, 1);
    Serial.print(" Vel:");  Serial.print(vel, 1);
    Serial.print(" IR:");   Serial.print(estadoIR);
    Serial.print(" BT:");   Serial.println(btConectado ? "OK" : "---");
    ultimoDebug = agora;
  }

  // ── Máquina de estados IR ────────────────────────────
  switch (estadoIR) {

    case IR_REPOUSO:
      // Exige repouso confirmado antes de liberar detecção.
      // Evita que uma piscada longa dispare múltiplos eventos.
      if (agora - tempoInicioRepouso > TEMPO_REPOUSO_MS) {
        g_valorBaseLine = g_mediaAtual; // Ancora baseline no repouso
        tempoUltimoRepouso = agora;
        estadoIR = IR_DETECTANDO;
      }
      break;

    case IR_DETECTANDO: {
      // Baseline adaptativo: atualiza devagar em repouso,
      // mais rápido se houver deriva lenta de luz ambiente
      float erro = abs(g_mediaAtual - g_valorBaseLine);
      float t = constrain((erro - ERRO_MIN) / (ERRO_MAX - ERRO_MIN), 0.0, 1.0);
      float alpha = ALPHA_MIN + t * (ALPHA_MAX - ALPHA_MIN);
      g_valorBaseLine = g_valorBaseLine * (1.0 - alpha) + g_mediaAtual * alpha;

      // Condição 1: queda suficiente em relação ao baseline
      bool quedaSuficiente = (g_mediaAtual < (g_valorBaseLine - DELTA_PISCADA));

      // Condição 2: queda brusca — diferencia piscada de mudança lenta de luz
      float velocidade = g_mediaAnterior - g_mediaAtual;
      bool quedaBrusca = (velocidade > VELOCIDADE_MIN_PISCADA);

      if (quedaSuficiente && quedaBrusca) {
        estadoIR           = IR_PISCADA;
        tempoInicioPiscada = agora;
        if (btConectado) {
          bleMouse.press(MOUSE_LEFT);
          Serial.print(">>> PRESS  vel:");
          Serial.println(velocidade);
        }
      } else {
        // Repouso prolongado → re-ancora baseline e volta ao estado de repouso
        if (agora - tempoUltimoRepouso > TIMEOUT_RESET_BASE_MS) {
          g_valorBaseLine    = g_mediaAtual;
          tempoInicioRepouso = agora;
          tempoUltimoRepouso = agora;
          estadoIR           = IR_REPOUSO;
        }
      }
      break;
    }

    case IR_PISCADA: {
      // Histerese de 70%: solta quando média volta a 70% do delta
      // (extraído do código de referência — evita release prematuro)
      bool voltouAoNormal = (g_mediaAtual > (g_valorBaseLine - DELTA_PISCADA * FATOR_RELEASE));
      bool timeout        = (agora - tempoInicioPiscada > TIMEOUT_PISCADA_MS);

      if (voltouAoNormal || timeout) {
        estadoIR           = IR_REPOUSO;
        tempoInicioRepouso = agora;
        tempoUltimoRepouso = agora;

        if (btConectado) {
          bleMouse.release(MOUSE_LEFT);
          Serial.println(timeout ? "<<< RELEASE (timeout)" : "<<< RELEASE");
        }
      }
      break;
    }

    default:
      break;
  }
}
