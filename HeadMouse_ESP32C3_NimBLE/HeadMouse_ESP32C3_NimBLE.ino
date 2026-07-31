/*
 * HeadMouse ESP32-C3 Super Mini — NimBLE HID Mouse
 * ─────────────────────────────────────────────────
 * Requisitos:
 *   - Biblioteca "NimBLE-Arduino" (h2zero), versão 2.x recomendada.
 *     Se estiver usando a 1.4.x, os nomes de callback do NimBLEServerCallbacks
 *     mudam (sem NimBLEConnInfo) — ajuste onConnect/onDisconnect conforme a
 *     versão instalada.
 *
 * Pinagem ESP32-C3 Super Mini:
 *   - TCRT5000 (saída analógica) -> GPIO4  (ADC1_CH4, livre de strapping)
 *   - MPU6050  SDA               -> GPIO8
 *   - MPU6050  SCL               -> GPIO5  (GPIO9 evitado: conflita com BOOT/strapping)
 *
 * Modo de operação: único e fixo (movimento + piscada sempre ativos).
 * A opção de alternar modos via botão BOOT foi removida.
 */

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <esp_wifi.h>
#include <esp32-hal-cpu.h>

// ─── Pinos ────────────────────────────────────────────────
const int sensorPin = 0;   // TCRT5000 -> ADC1_CH4 (GPIO4)

// ─── BLE HID (NimBLE) ─────────────────────────────────────
NimBLEServer*        pServer      = nullptr;
NimBLEHIDDevice*      hid          = nullptr;
NimBLECharacteristic* inputMouse   = nullptr;
bool deviceConnected = false;
bool btConectadoAntes = false;

#define MOUSE_LEFT   0x01
#define MOUSE_RIGHT  0x02
#define MOUSE_MIDDLE 0x04
uint8_t mouseButtons = 0;

// Report map padrão de mouse HID (botões + X + Y + wheel, report ID 1)
static const uint8_t reportMap[] = {
  0x05, 0x01,       // USAGE_PAGE (Generic Desktop)
  0x09, 0x02,       // USAGE (Mouse)
  0xA1, 0x01,       // COLLECTION (Application)
  0x09, 0x01,       //   USAGE (Pointer)
  0xA1, 0x00,       //   COLLECTION (Physical)
  0x85, 0x01,       //     REPORT_ID (1)
  0x05, 0x09,       //     USAGE_PAGE (Button)
  0x19, 0x01,       //     USAGE_MINIMUM (Button 1)
  0x29, 0x03,       //     USAGE_MAXIMUM (Button 3)
  0x15, 0x00,       //     LOGICAL_MINIMUM (0)
  0x25, 0x01,       //     LOGICAL_MAXIMUM (1)
  0x95, 0x03,       //     REPORT_COUNT (3)
  0x75, 0x01,       //     REPORT_SIZE (1)
  0x81, 0x02,       //     INPUT (Data,Var,Abs)
  0x95, 0x01,       //     REPORT_COUNT (1)
  0x75, 0x05,       //     REPORT_SIZE (5)
  0x81, 0x03,       //     INPUT (Const,Var,Abs)  -> padding
  0x05, 0x01,       //     USAGE_PAGE (Generic Desktop)
  0x09, 0x30,       //     USAGE (X)
  0x09, 0x31,       //     USAGE (Y)
  0x09, 0x38,       //     USAGE (Wheel)
  0x15, 0x81,       //     LOGICAL_MINIMUM (-127)
  0x25, 0x7F,       //     LOGICAL_MAXIMUM (127)
  0x75, 0x08,       //     REPORT_SIZE (8)
  0x95, 0x03,       //     REPORT_COUNT (3)
  0x81, 0x06,       //     INPUT (Data,Var,Rel)
  0xC0,             //   END_COLLECTION
  0xC0              // END_COLLECTION
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.println("BLE conectado");
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.println("BLE desconectado — reiniciando advertising");
    NimBLEDevice::startAdvertising();
  }
};

void setupBLE() {
  NimBLEDevice::init("HeadMouse ESP32");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(false, false, true); // apenas bonding, sem MITM/SC

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  hid = new NimBLEHIDDevice(pServer);
  inputMouse = hid->getInputReport(1); // report ID 1

  hid->setManufacturer("DIY");
  hid->setPnp(0x02, 0x05ac, 0x820a, 0x0210);
  hid->setHidInfo(0x00, 0x01);
  hid->setReportMap((uint8_t*)reportMap, sizeof(reportMap));
  hid->setBatteryLevel(100);
  hid->startServices();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setAppearance(HID_MOUSE);
  pAdvertising->addServiceUUID(hid->getHidService()->getUUID());
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();

  Serial.println("NimBLE HID Mouse pronto, aguardando conexao...");
}

// Envia o report atual (botões + deslocamento X/Y)
void enviarReportMouse(int8_t x, int8_t y) {
  if (!deviceConnected) return;
  uint8_t report[4] = { mouseButtons, (uint8_t)x, (uint8_t)y, 0x00 };
  inputMouse->setValue(report, sizeof(report));
  inputMouse->notify();
}

void moverMouse(int x, int y) {
  int8_t xc = (int8_t)constrain(x, -127, 127);
  int8_t yc = (int8_t)constrain(y, -127, 127);
  enviarReportMouse(xc, yc);
}

void pressionarBotao(uint8_t botao) {
  mouseButtons |= botao;
  enviarReportMouse(0, 0);
}

void soltarBotao(uint8_t botao) {
  mouseButtons &= ~botao;
  enviarReportMouse(0, 0);
}

// ─── MPU6050 ─────────────────────────────────────────────
MPU6050 mpu;
int vx, vy;

// ─── Buffers de leituras e médias ────────────────────────
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
#define DELTA_PISCADA          15
#define FATOR_RELEASE          0.35
#define TIMEOUT_PISCADA_MS     800
#define VELOCIDADE_MIN_PISCADA 8.0

#define ALPHA_MIN  0.03
#define ALPHA_MAX  0.30
#define ERRO_MIN   5.0
#define ERRO_MAX   60.0

#define TEMPO_AQUECIMENTO_MS   500
#define TEMPO_REPOUSO_MS       20

#define TIMEOUT_RESET_BASE_MS  100

// ─── Máquina de estados do sensor IR ─────────────────────
enum EstadoIR {
  IR_AQUECENDO,
  IR_CALIBRANDO,
  IR_REPOUSO,
  IR_DETECTANDO,
  IR_PISCADA
};
EstadoIR estadoIR = IR_AQUECENDO;

unsigned long tempoInicioAquecimento = 0;
unsigned long tempoInicioRepouso     = 0;
unsigned long tempoInicioPiscada     = 0;
unsigned long tempoUltimoRepouso     = 0;

// ─── Controle geral ───────────────────────────────────────
#define TEMPO_SEM_BT_MS    60000
unsigned long tempoSemConexao = 0;

unsigned long ultimoMPU   = 0;
const int INTERVALO_MPU_MS = 10;

unsigned long ultimoSampleUs = 0;
const int INTERVALO_US = 1800;

unsigned long ultimoDebug = 0;

// ─────────────────────────────────────────────────────────

void configurarEnergia() {
  esp_wifi_stop();
  esp_wifi_deinit();
  setCpuFrequencyMhz(80);
}

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

  Wire.begin(5,6);

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
  
  setupBLE();

  tempoInicioAquecimento = millis();
  tempoSemConexao        = millis();
  ultimoMPU              = millis();

  Serial.println("HeadMouse ESP32-C3 iniciado — CPU: 80MHz — Modo unico (movimento + piscada)");
}

// ─────────────────────────────────────────────────────────

void loop() {
  unsigned long agoraMilis = millis();

  // ── Deep sleep por perda de conexão BLE ───────────────
  if (deviceConnected) {
    tempoSemConexao  = agoraMilis;
    btConectadoAntes = true;
  } else if (btConectadoAntes && agoraMilis - tempoSemConexao > TEMPO_SEM_BT_MS) {
    Serial.println("Sem BLE 60s — deep sleep 10s");
    Serial.flush();
    esp_sleep_enable_timer_wakeup(10ULL * 1000000);
    esp_deep_sleep_start();
  }

  // ── Sensor IR (piscada) ───────────────────────────────
  unsigned long agoraUs = micros();
  if (agoraUs - ultimoSampleUs >= (unsigned long)INTERVALO_US) {
    ultimoSampleUs = agoraUs;
    int leitura = analogRead(sensorPin);
    processarLeituraIR(leitura, deviceConnected, agoraMilis);
  }

  // ── MPU6050 (movimento) ───────────────────────────────
  if (deviceConnected) {
    if (agoraMilis - ultimoMPU >= INTERVALO_MPU_MS) {
      ultimoMPU = agoraMilis;

      int16_t _gx, _gy, _gz;
      mpu.getRotation(&_gx, &_gy, &_gz);

      vx = -_gx / 100;
      vy =  _gz / 100;

      if (abs(vx) < 2) vx = 0;
      if (abs(vy) < 2) vy = 0;

      if (vx != 0 || vy != 0) {
        moverMouse(vx, vy);
      }
    }
  }

  delay(1);
}

// ─────────────────────────────────────────────────────────

void processarLeituraIR(int leitura, bool btConectado, unsigned long agora) {

  if (estadoIR == IR_AQUECENDO) {
    if (agora - tempoInicioAquecimento < TEMPO_AQUECIMENTO_MS) return;
    estadoIR = IR_CALIBRANDO;
    Serial.println("Aquecimento OK — calibrando...");
  }

  vetorLeituras[indexLeituras] = leitura;
  indexLeituras = (indexLeituras + 1) % NUM_LEITURAS;

  g_mediaAnterior = g_mediaAtual;
  g_mediaAtual    = calcularMedia();

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

  switch (estadoIR) {

    case IR_REPOUSO:
      if (agora - tempoInicioRepouso > TEMPO_REPOUSO_MS) {
        g_valorBaseLine = g_mediaAtual;
        tempoUltimoRepouso = agora;
        estadoIR = IR_DETECTANDO;
      }
      break;

    case IR_DETECTANDO: {
      float erro = abs(g_mediaAtual - g_valorBaseLine);
      float t = constrain((erro - ERRO_MIN) / (ERRO_MAX - ERRO_MIN), 0.0, 1.0);
      float alpha = ALPHA_MIN + t * (ALPHA_MAX - ALPHA_MIN);
      g_valorBaseLine = g_valorBaseLine * (1.0 - alpha) + g_mediaAtual * alpha;

      bool quedaSuficiente = (g_mediaAtual < (g_valorBaseLine - DELTA_PISCADA));
      float velocidade = g_mediaAnterior - g_mediaAtual;
      bool quedaBrusca = (velocidade > VELOCIDADE_MIN_PISCADA);

      if (quedaSuficiente && quedaBrusca) {
        estadoIR           = IR_PISCADA;
        tempoInicioPiscada = agora;
        if (btConectado) {
          pressionarBotao(MOUSE_LEFT);
          Serial.print(">>> PRESS  vel:");
          Serial.println(velocidade);
        }
      } else {
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
      bool voltouAoNormal = (g_mediaAtual > (g_valorBaseLine - DELTA_PISCADA * FATOR_RELEASE));
      bool timeout        = (agora - tempoInicioPiscada > TIMEOUT_PISCADA_MS);

      if (voltouAoNormal || timeout) {
        estadoIR           = IR_REPOUSO;
        tempoInicioRepouso = agora;
        tempoUltimoRepouso = agora;

        if (btConectado) {
          soltarBotao(MOUSE_LEFT);
          Serial.println(timeout ? "<<< RELEASE (timeout)" : "<<< RELEASE");
        }
      }
      break;
    }

    default:
      break;
  }
}
