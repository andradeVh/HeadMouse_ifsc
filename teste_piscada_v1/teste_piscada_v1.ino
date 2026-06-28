#include <BleMouse.h>
#include <esp_bt.h>
#include <esp_wifi.h>

// ─── Pinos ───────────────────────────────────────────────
const int PIN_BT      = 14;      // Botão físico
const int sensorPin   = 34;      // ← MUDANÇA: ADC1 (GPIO 34, 35, 36, 39 são seguros com BT)
const int pino_sample = 15;      // Controla o LED IR do TCRT5000

// ─── BLE Mouse ───────────────────────────────────────────
BleMouse bleMouse("HeadMouse ESP32", "DIY", 100);

// ─── Sensor IR ───────────────────────────────────────────
int sensorValueOn  = 0;
int sensorValueOff = 0;

// ─── Filtro de média móvel ────────────────────────────────
// MUDANÇA: aumentei para 50 — mais resistente a ruído sem perder responsividade
#define NUM_LEITURAS 50
unsigned int vetorLeituras[NUM_LEITURAS];
int indexLeituras       = 0;
bool bufferPreenchido   = false;   // ← NOVO: evita baseline falso no boot
float g_mediaAtual      = 0;
float g_valorBaseLine   = -1;      // -1 = não inicializado ainda

// ─── Parâmetros de detecção ───────────────────────────────
// Ajuste DELTA conforme seu hardware (comece com 15 e suba se der falsos clicks)
#define DELTA_PISCADA       20     
#define FATOR_RELEASE       0.35   // Solta o botão quando média cai abaixo de (baseline + delta * FATOR)
#define TIMEOUT_PISCADA_MS  900    // Máximo de tempo que uma "piscada" pode durar
#define ALPHA_BASELINE      0.05   // Velocidade de adaptação do baseline (menor = mais lento = mais estável)

// ─── Estado da piscada ────────────────────────────────────
bool emPiscada              = false;
unsigned long tempoInicioPiscada = 0;

// ─── Temporização das fases IR ────────────────────────────
// MUDANÇA: 3 fases apenas (1→3), sem fase 4 morta que atrasava o ciclo
unsigned long ultimoSampleMicros = 0;
const int INTERVALO_FASE_US      = 600; // 600µs por fase → ciclo total ~1.8ms
int fase = 1;

// ─── Debug ───────────────────────────────────────────────
unsigned long ultimoDebug = 0;

// ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  // Libera memória do BT clássico e desliga WiFi
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  esp_wifi_stop();
  esp_wifi_deinit();

  pinMode(PIN_BT, INPUT_PULLUP);
  pinMode(pino_sample, OUTPUT);
  digitalWrite(pino_sample, LOW);

  // Limpa o buffer de leituras
  memset(vetorLeituras, 0, sizeof(vetorLeituras));

  bleMouse.begin();
  Serial.println("HeadMouse iniciado. Aguardando conexão BLE...");
}

void loop() {
  unsigned long agora = micros();

  // ── Lógica de fases do sensor IR ──────────────────────
  if (agora - ultimoSampleMicros >= INTERVALO_FASE_US) {
    ultimoSampleMicros = agora;

    switch (fase) {
      case 1:
        digitalWrite(pino_sample, HIGH); // Liga LED IR — aguarda estabilizar
        break;

      case 2:
        sensorValueOn = analogRead(sensorPin); // Lê com IR ligado
        digitalWrite(pino_sample, LOW);        // Desliga IR
        break;

      case 3:
        sensorValueOff = analogRead(sensorPin); // Lê luz ambiente (sem IR)
        processarLeituraIR();
        break;
    }

    // MUDANÇA: ciclo correto 1→2→3→1, sem fase 4 inútil
    fase = (fase >= 3) ? 1 : fase + 1;
  }

  // ── Botão físico ───────────────────────────────────────
  if (digitalRead(PIN_BT) == LOW) {
    if (bleMouse.isConnected()) bleMouse.click(MOUSE_LEFT);
    delay(200); // Debounce simples
  }
}

// ─────────────────────────────────────────────────────────

void processarLeituraIR() {
  // Subtrai a luz ambiente para isolar apenas o sinal IR refletido
  // MUDANÇA: sensorValueOn - sensorValueOff (não abs()) — o olho aberto reflete
  //           mais IR que o fechado, então o diff deve CAIR na piscada.
  //           Se no seu hardware aumentar, troque por (sensorValueOn - sensorValueOff).
  int diff = sensorValueOn - sensorValueOff;
  if (diff < 0) diff = 0; // Garante que nunca seja negativo

  // ── Atualiza média móvel ────────────────────────────────
  vetorLeituras[indexLeituras] = diff;
  indexLeituras = (indexLeituras + 1) % NUM_LEITURAS;

  if (!bufferPreenchido && indexLeituras == 0) {
    bufferPreenchido = true; // Buffer completo pela primeira vez
  }

  unsigned long soma = 0;
  for (int i = 0; i < NUM_LEITURAS; i++) soma += vetorLeituras[i];
  g_mediaAtual = (float)soma / NUM_LEITURAS;

  // ── Inicializa baseline com a primeira janela completa ──
  // MUDANÇA: evita que o baseline comece em 0 e dispare falsos clicks no boot
  if (g_valorBaseLine < 0 && bufferPreenchido) {
    g_valorBaseLine = g_mediaAtual;
    Serial.print("Baseline inicializado: ");
    Serial.println(g_valorBaseLine);
    return;
  }
  if (g_valorBaseLine < 0) return; // Ainda coletando amostras iniciais

  // ── Debug serial ────────────────────────────────────────
  if (millis() - ultimoDebug > 150) {
    Serial.print("On:"); Serial.print(sensorValueOn);
    Serial.print(" Off:"); Serial.print(sensorValueOff);
    Serial.print(" Diff:"); Serial.print(diff);
    Serial.print(" Media:"); Serial.print(g_mediaAtual, 1);
    Serial.print(" Base:"); Serial.print(g_valorBaseLine, 1);
    Serial.print(" Estado:"); Serial.print(emPiscada ? "PISCADA" : "normal");
    Serial.print(" BT:"); Serial.println(bleMouse.isConnected() ? "OK" : "---");
    ultimoDebug = millis();
  }

  // ── Detecção de piscada ─────────────────────────────────
  if (!emPiscada) {
    // Baseline só atualiza quando o olho está ABERTO (estado normal)
    // MUDANÇA: alpha menor (0.05) = baseline mais estável, menos suscetível a derive
    g_valorBaseLine = (g_valorBaseLine * (1.0 - ALPHA_BASELINE)) + (g_mediaAtual * ALPHA_BASELINE);

    // Detecta início da piscada: diff CAI abaixo do baseline (pálpebra bloqueia IR)
    // ATENÇÃO: dependendo da posição do sensor, pode ser ">" — observe no Serial Plotter
    if (g_mediaAtual < (g_valorBaseLine - DELTA_PISCADA)) {
      emPiscada = true;
      tempoInicioPiscada = millis();
      if (bleMouse.isConnected()) {
        bleMouse.press(MOUSE_LEFT);
        Serial.println(">>> PRESS");
      } else {
        Serial.println(">>> Piscada detectada (BT desconectado)");
      }
    }

  } else {
    // ── Aguarda fim da piscada ───────────────────────────
    bool voltouAoNormal = (g_mediaAtual > (g_valorBaseLine - DELTA_PISCADA * FATOR_RELEASE));
    bool timeout        = (millis() - tempoInicioPiscada > TIMEOUT_PISCADA_MS);

    if (voltouAoNormal || timeout) {
      emPiscada = false;
      if (bleMouse.isConnected()) {
        bleMouse.release(MOUSE_LEFT);
        Serial.println("<<< RELEASE");
      }
      if (timeout) Serial.println("(por timeout)");
    }
  }
}
