/*
  HeadMouse - ESP32-C3
  BLE HID via BLEDevice.h + MPU6050 (movimento) + TCRT5000 (piscada/clique)
  Modos: 1=Movimento  2=Piscada  3=Completo (botão BOOT alterna)

  Bibliotecas necessárias:
    - I2Cdevlib-MPU6050 (I2Cdev.h + MPU6050.h)

  Ajustes feitos em relação à versão ESP32 DevKit:
    - sensorPin: GPIO34 nao existe no C3 -> usando GPIO4 (ADC1, livre de strapping)
    - I2C: Wire.begin(21,22) -> Wire.begin(8,5)
      (GPIO9 evitado no SCL: e o pino fisico do botao BOOT, usado p/ troca de modo)
    - esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) removido:
      o C3 nao tem Bluetooth classico, essa chamada nao se aplica
    - bleMouse.move()/press()/release() substituidos por enviarReport(),
      usando BLEHIDDevice diretamente (mesma base ja validada como estavel)
    - TCRT5000 agora alimentado diretamente em 3.3V (VCC fixo, LED sempre aceso):
      removido o controle de pino_sample e a leitura em fases (on/off + diff).
      Leitura passa a ser continua, direto do sensorPin.
*/

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <esp_wifi.h>
#include <esp32-hal-cpu.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>

// ─── Pinos ────────────────────────────────────────────────
const int sensorPin = 0;   // ADC1 - TCRT5000 (VCC direto em 3.3V, LED sempre aceso)

const int pinoSDA = 5;
const int pinoSCL = 6;     // GPIO9 evitado: conflita com o botao BOOT

const int PIN_BOOT = 2;   //GPIO02

// ─── Modos de operação ────────────────────────────────────
enum Modo { MODO_MOVIMENTO = 1, MODO_PISCADA = 2, MODO_COMPLETO = 3 };
Modo modoAtual = MODO_COMPLETO;
const char* nomeModo[] = { "", "SO MOVIMENTO", "SO PISCADA", "COMPLETO" };

// ─── Debounce do botão BOOT ───────────────────────────────
unsigned long ultimoPressBoot = 0;
const int DEBOUNCE_MS = 300;

void avancarModo() {
  modoAtual = (Modo)(modoAtual % 3 + 1);
  Serial.print(">>> MODO: ");
  Serial.println(nomeModo[modoAtual]);
}

// ─── BLE HID (BLEDevice.h) ────────────────────────────────
BLEHIDDevice* hid;
BLECharacteristic* inputMouse;
BLEServer* pServer;
bool conectado = false;
uint8_t botoesAtuais = 0;

static const uint8_t reportMap[] = {
  0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
  0x85, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00,
  0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01,
  0x75, 0x05, 0x81, 0x03, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
  0x09, 0x38, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03,
  0x81, 0x06, 0xC0, 0xC0
};

class MinhasServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) {
    conectado = true;
    Serial.println("Conectado!");
  }
  void onDisconnect(BLEServer* server) {
    conectado = false;
    Serial.println("Desconectado! Reiniciando advertising...");
    delay(200);
    pServer->getAdvertising()->start();
  }
};

int8_t clamp127(int v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return (int8_t)v;
}

void enviarReport(uint8_t botoes, int8_t x, int8_t y) {
  if (!conectado) return;
  uint8_t report[4] = { botoes, (uint8_t)x, (uint8_t)y, 0 };
  inputMouse->setValue(report, 4);
  inputMouse->notify();
}

void setupBLE() {
  BLEDevice::init("HeadMouse ESP32 teste");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MinhasServerCallbacks());

  hid = new BLEHIDDevice(pServer);
  inputMouse = hid->inputReport(1);

  hid->manufacturer()->setValue("DIY");
  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->hidInfo(0x00, 0x01);

  BLESecurity* pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

  hid->reportMap((uint8_t*)reportMap, sizeof(reportMap));
  hid->startServices();

  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->setAppearance(HID_MOUSE);
  pAdvertising->addServiceUUID(hid->hidService()->getUUID());
  pAdvertising->start();

  hid->setBatteryLevel(100);
}

// ─── MPU6050 ─────────────────────────────────────────────
MPU6050 mpu;
int vx, vy;
unsigned long ultimoMPU = 0;
const int INTERVALO_MPU_MS = 10;

// ─── Estabilidade do cursor ───────────────────────────────
#define JANELA_ESTABILIDADE_MS  300   // Janela de observação
#define LIMITE_MOVIMENTO     60    // Movimento máximo permitido na janela
#define TEMPO_MINIMO_ESTAVEL_MS 100   // Tempo mínimo consecutivo estável

// Buffer circular para armazenar os deltas de movimento
#define NUM_CICLOS_ESTABILIDADE (JANELA_ESTABILIDADE_MS / INTERVALO_MPU_MS) // 300/10 = 30
int bufferMovimento[NUM_CICLOS_ESTABILIDADE];
int indexMovimento        = 0;
int acumuladoMovimento    = 0;         // Soma corrente — atualizada incrementalmente
unsigned long tempoEstavelInicio = 0;  // Quando o cursor começou a ficar estável
bool cursorEstavel        = false;

// ─── Sensor IR (leitura contínua, sem fases) ─────────────
#define NUM_LEITURAS 50
unsigned int vetorLeituras[NUM_LEITURAS];
int indexLeituras     = 0;
bool bufferPreenchido = false;
float g_mediaAtual    = 0;
float g_valorBaseLine = -1;
float g_mediaAnterior = 0;

#define DELTA_PISCADA      10
#define FATOR_RELEASE      0.35
#define TIMEOUT_PISCADA_MS 900
#define VELOCIDADE_MIN_PISCADA  6.0

#define ALPHA_MIN       0.03
#define ALPHA_MAX       0.30
#define ERRO_MIN        5.0
#define ERRO_MAX        60.0

bool emPiscada = false;
unsigned long tempoInicioPiscada = 0;

unsigned long ultimoSampleUs = 0;
const int INTERVALO_US = 5000; // intervalo entre leituras continuas do TCRT5000

#define TEMPO_SEM_BT_MS 60000
unsigned long tempoSemConexao = 0;
bool btConectadoAntes = false;


unsigned long ultimoDebug = 0;

// ─────────────────────────────────────────────────────────

void configurarEnergia() {
  esp_wifi_stop();
  esp_wifi_deinit();
  setCpuFrequencyMhz(50);
}

void setCalibration() {
  mpu.CalibrateGyro(7);
}

// ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  configurarEnergia();

  Wire.begin(pinoSDA, pinoSCL);
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
  setupBLE();

  tempoSemConexao = millis();
  ultimoMPU       = millis();

  Serial.print("HeadMouse iniciado - CPU: 80MHz - Modo: ");
  Serial.println(nomeModo[modoAtual]);
}

// ─────────────────────────────────────────────────────────

void loop() {
  unsigned long agoraMilis = millis();

  // ── Botão BOOT: troca de modo ─────────────────────────
  if (digitalRead(PIN_BOOT) == LOW) {
    if (agoraMilis - ultimoPressBoot > DEBOUNCE_MS) {
      ultimoPressBoot = agoraMilis;
      avancarModo();
    }
  }

  // ── Deep sleep por perda de conexão BLE ───────────────
  if (conectado) {
    tempoSemConexao  = agoraMilis;
    btConectadoAntes = true;
  } else if (btConectadoAntes && (agoraMilis - tempoSemConexao > TEMPO_SEM_BT_MS)) {
    Serial.println("Sem BLE por 60s - deep sleep de 10s");
    Serial.flush();
    esp_sleep_enable_timer_wakeup(10ULL * 1000000);
    esp_deep_sleep_start();
  }

  // ── Sensor IR — leitura contínua ──────────────────────
  bool piscadaAtiva = (modoAtual == MODO_PISCADA || modoAtual == MODO_COMPLETO);

  if (piscadaAtiva) {
    unsigned long agoraUs = micros();
    if (agoraUs - ultimoSampleUs >= (unsigned long)INTERVALO_US) {
      ultimoSampleUs = agoraUs;
      int leitura = analogRead(sensorPin);
      processarLeituraIR(leitura, conectado);
    }
  }

  // ── MPU6050 ────────────────────────────────────────────
  bool movimentoAtivo = (modoAtual == MODO_MOVIMENTO || modoAtual == MODO_COMPLETO);

  if (conectado && movimentoAtivo) {
    if (agoraMilis - ultimoMPU >= INTERVALO_MPU_MS) {
      ultimoMPU = agoraMilis;

      int16_t _gx, _gy, _gz;
      mpu.getRotation(&_gx, &_gy, &_gz);

      vx = -_gx / 150;
      vy = _gz / 150;

      atualizarEstabilidade(vx, vy);
      
      if (abs(vx) < 2) vx = 0;
      if (abs(vy) < 2) vy = 0;

      if (vx != 0 || vy != 0) {
        enviarReport(botoesAtuais, clamp127(vx), clamp127(vy));
      }
    }
  }
  delay(1);
}

// ─────────────────────────────────────────────────────────

void processarLeituraIR(int leitura, bool btConectado) {
  vetorLeituras[indexLeituras] = leitura;
  indexLeituras = (indexLeituras + 1) % NUM_LEITURAS;
  if (!bufferPreenchido && indexLeituras == 0) bufferPreenchido = true;

  unsigned long soma = 0;
  for (int i = 0; i < NUM_LEITURAS; i++) soma += vetorLeituras[i];
  g_mediaAtual = (float)soma / NUM_LEITURAS;

  if (g_valorBaseLine < 0 && bufferPreenchido) {
    g_valorBaseLine = g_mediaAtual;
    g_mediaAnterior = g_mediaAtual;
    Serial.print("Baseline inicializado: ");
    Serial.println(g_valorBaseLine);
    return;
  }
  
  if (g_valorBaseLine < 0) return;

  if (millis() - ultimoDebug > 150) {
    float vel = g_mediaAnterior - g_mediaAtual;
    Serial.print("Leit:");   Serial.print(leitura);
    Serial.print(" Media:"); Serial.print(g_mediaAtual, 1);
    Serial.print(" Base:");  Serial.print(g_valorBaseLine, 1);
    Serial.print(" Vel:");   Serial.print(vel, 1);
    Serial.print(" Estado:"); Serial.print(emPiscada ? "PISCADA" : "normal");
    Serial.print(" Cursor:"); Serial.print(cursorEstavel);
    Serial.print(" Acumulado cursor:"); Serial.print(acumuladoMovimento);
    Serial.print(" Modo:");  Serial.print(nomeModo[modoAtual]);
    Serial.print(" Valor Ax:");  Serial.print(mpu.getAccelerationX());
    Serial.print(" BT:");    Serial.println(btConectado ? "OK" : "---");
    ultimoDebug = millis();
  }

  
  if (!emPiscada) {
    float erro = abs(g_mediaAtual - g_valorBaseLine);
    float t = (erro - ERRO_MIN) / (ERRO_MAX - ERRO_MIN);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    float alphaAdaptativo = ALPHA_MIN + t * (ALPHA_MAX - ALPHA_MIN);

    g_valorBaseLine = (g_valorBaseLine * (1.0 - alphaAdaptativo))
                    + (g_mediaAtual    *  alphaAdaptativo);
                    
    bool quedaSuficiente = (g_mediaAtual < (g_valorBaseLine - DELTA_PISCADA));

    
    // frequencia de medias iguais, entao a velocidade e a diferençã das medias
    float velocidadeQueda = g_mediaAnterior - g_mediaAtual; 
    bool quedaBrusca = (velocidadeQueda > VELOCIDADE_MIN_PISCADA);

    //Cursor estabilizado
    bool cursorOk = cursorEstavelConfirmado();
    
    if (quedaSuficiente && quedaBrusca && cursorOk) {
      emPiscada = true;
      tempoInicioPiscada = millis();
      botoesAtuais = 0x01;
      enviarReport(botoesAtuais, 0, 0);
      Serial.println(">>> PRESS");
      Serial.print("velocidade no momento: ");
      Serial.println(velocidadeQueda);
      Serial.print("Leit no momento:");   Serial.println(leitura);
      Serial.print("Media no momento:"); Serial.println(g_mediaAtual);
      Serial.print("Base no momento:");  Serial.println(g_valorBaseLine);
    }

  } else {
    //se a media volta 
    bool voltouAoNormal = (g_mediaAtual > (g_valorBaseLine - DELTA_PISCADA* FATOR_RELEASE));
    bool timeout = (millis() - tempoInicioPiscada > TIMEOUT_PISCADA_MS);

    if (voltouAoNormal || timeout) {
      emPiscada = false;
      botoesAtuais = 0x00;
      enviarReport(botoesAtuais, 0, 0);

      if (timeout) {
        Serial.println("<<< TimeOut");
      } else {
        Serial.println("<<< RELEASE");
      }
    }
  }

  g_mediaAnterior = g_mediaAtual;
}

void atualizarEstabilidade(int vx, int vy) {
  // Acumulador
  int acumulador = abs(vx) + abs(vy);

  // Atualização incremental: subtrai o valor mais antigo, soma o novo
  // Evita recalcular a soma inteira a cada ciclo
  
  acumuladoMovimento -= bufferMovimento[indexMovimento];
  acumuladoMovimento += acumulador;

  // Guarda o novo valor no buffer circular
  bufferMovimento[indexMovimento] = acumulador;
  indexMovimento = (indexMovimento + 1) % NUM_CICLOS_ESTABILIDADE;

  // Verifica se está abaixo do threshold
  bool abaixoDoLimite = (acumuladoMovimento < LIMITE_MOVIMENTO);

  unsigned long agora = millis();

  if (abaixoDoLimite) {
    if (!cursorEstavel) {
      // Acabou de entrar no estado estável — registra o início
      tempoEstavelInicio = agora;
      cursorEstavel = true;
    }
    // cursorEstavel permanece true enquanto acumulado < threshold
  } else {
    // Movimento detectado — reseta tudo
    cursorEstavel = false;
    tempoEstavelInicio = agora;
  }
}

bool cursorEstavelConfirmado() {
  if (!cursorEstavel) return false;
  return (millis() - tempoEstavelInicio >= TEMPO_MINIMO_ESTAVEL_MS);
}
