#include <BleMouse.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp32-hal-cpu.h>

// ─── Pinos ─────────────r──────────────────────────────────
const int PIN_BT      = 14;
const int sensorPin   = 34;
const int pino_sample = 15;

// ─── BLE Mouse ───────────────────────────────────────────
BleMouse bleMouse("HeadMouse ESP32", "DIY", 100);

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

// ─── Parâmetros de detecção ───────────────────────────────
#define DELTA_PISCADA      20
#define FATOR_RELEASE      0.35
#define TIMEOUT_PISCADA_MS 900
#define ALPHA_BASELINE     0.05

// ─── Estado da piscada ────────────────────────────────────
bool emPiscada = false;
unsigned long tempoInicioPiscada = 0;

// ─── Temporização das fases IR ────────────────────────────
unsigned long ultimoSampleMicros = 0;
const int INTERVALO_FASE_US      = 600;
int fase = 1;

// ─── Controle de inatividade ──────────────────────────────
// Modo econômico: aumenta intervalo de amostragem após inatividade,
// reduzindo o tempo que o LED IR fica ligado.
// O deep sleep só é acionado se não houver conexão BLE por muito tempo.
#define TEMPO_INATIVO_MS      5000
#define INTERVALO_ECONOMICO_US 3000
#define TEMPO_SEM_BT_MS       60000

unsigned long ultimaAtividade = 0;
unsigned long tempoSemConexao = 0;
bool modoEconomico    = false;
bool btConectadoAntes = false;

// ─── Debug ───────────────────────────────────────────────
unsigned long ultimoDebug = 0;

// ─────────────────────────────────────────────────────────

void configurarEnergia() {
  // Desliga WiFi permanentemente
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  // Reduz CPU de 240MHz → 80MHz
  // Abaixo de 80MHz o stack BLE pode ficar instável
  setCpuFrequencyMhz(80);

  // REMOVIDO: esp_pm_configure com light_sleep_enable
  // REMOVIDO: esp_light_sleep_start() manual no loop
  // Ambos conflitam com o gerenciamento interno da BleMouse e causam reset.
  // O FreeRTOS já coloca a CPU em idle automaticamente entre tarefas — 
  // não precisamos gerenciar isso manualmente.
}

void setup() {
  Serial.begin(115200);

  configurarEnergia();

  pinMode(PIN_BT, INPUT_PULLUP);
  pinMode(pino_sample, OUTPUT);
  digitalWrite(pino_sample, LOW);

  memset(vetorLeituras, 0, sizeof(vetorLeituras));

  bleMouse.begin();

  ultimaAtividade = millis();
  tempoSemConexao = millis();

  Serial.println("HeadMouse iniciado — CPU: 80MHz");
}

void loop() {
  unsigned long agora      = micros();
  unsigned long agoraMilis = millis();

  // ── Estado BLE capturado uma única vez por ciclo ───────
  bool btConectado = bleMouse.isConnected();

  // ── Gerencia deep sleep por falta de conexão ──────────
  // Só entra em deep sleep se nunca conectou OU se conectou e perdeu.
  // Isso evita dormir durante o advertising inicial no boot.
  if (btConectado) {
    tempoSemConexao  = agoraMilis;
    btConectadoAntes = true;
  } else if (btConectadoAntes && (agoraMilis - tempoSemConexao > TEMPO_SEM_BT_MS)) {
    Serial.println("Sem conexão BLE por 30s — deep sleep de 10s");
    Serial.flush();
    digitalWrite(pino_sample, LOW);
    // Acorda após 10s para tentar reconectar
    esp_sleep_enable_timer_wakeup(10ULL * 1000000);
    esp_deep_sleep_start();
    // Deep sleep faz reset completo — código abaixo nunca executa
  }

  // ── Modo econômico por inatividade ────────────────────
  // Aumenta o intervalo entre amostras quando não há piscadas.
  // Efeito direto: LED IR ligado por menos tempo → menos consumo.
  int intervaloAtual = INTERVALO_FASE_US;
  if (agoraMilis - ultimaAtividade > TEMPO_INATIVO_MS) {
    if (!modoEconomico) {
      modoEconomico = true;
      Serial.println("Modo econômico ativado");
    }
    intervaloAtual = INTERVALO_ECONOMICO_US;
  } else {
    if (modoEconomico) {
      modoEconomico = false;
      Serial.println("Modo econômico desativado");
    }
  }

  // ── Fases do sensor IR ─────────────────────────────────
  if (agora - ultimoSampleMicros >= (unsigned long)intervaloAtual) {
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

  // ── Botão físico ───────────────────────────────────────
  if (digitalRead(PIN_BT) == LOW) {
    if (btConectado) bleMouse.click(MOUSE_LEFT);
    ultimaAtividade = agoraMilis;
    delay(200);
  }

  // ── Cede tempo ao FreeRTOS ─────────────────────────────
  // delay(1) libera a CPU para outras tasks (incluindo o stack BLE)
  // sem comprometer o timing do sensor — o próximo sample ainda é
  // controlado pelo check de micros() no início do loop.
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
    Serial.print(modoEconomico ? " [ECO]" : " [NRM]");
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
      ultimaAtividade    = millis();
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
