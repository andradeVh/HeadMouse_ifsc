/*
  HX710B (sensor de pressão) + ESP32-C3 Super Mini + BLE HID Mouse
  Stack BLE: Bluedroid clássica (BLEDevice.h / BLEHIDDevice) - mesma base
  usada no HeadMouse que já conecta corretamente.

  Bibliotecas necessárias (Library Manager):
    - ESP32 BLE Arduino (embutida no core ESP32, via BLEDevice.h)
*/

#include <Arduino.h>
#include <esp_wifi.h>
#include <esp32-hal-cpu.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>

// ================== PINOS (ESP32-C3 Super Mini) ==================
const int PIN_OUT = 2; // GPIO 2 (Entrada de dados - DOUT do HX710B)
const int PIN_SCK = 3; // GPIO 3 (Saída de clock - SCK do HX710B)

// ================== PARÂMETROS DE DETECÇÃO ==================
const long DELTA = 20000;              // Limiar de variação em relação à média
const int TAMANHO_BUFFER = 20;         // Tamanho da janela de média móvel
// Tempo mínimo entre dois cliques - serve só para filtrar ruído/repique
// elétrico, NÃO para separar cliques intencionais (a detecção por borda
// já garante 1 clique por pressão real). Se estiver alto demais, engole
// o segundo clique de um double click. 80-120ms costuma ser suficiente.
const unsigned long DEBOUNCE_MS = 200
;

// Trava máxima do estado "pressionado": mesmo que o sinal não volte a ficar
// abaixo do DELTA, o estado é destravado à força após esse tempo, liberando
// o próximo clique. Evita ficar "preso" esperando a média se ajustar.
const unsigned long TIMEOUT_PRESSAO_MS = 500;

// Libera o estado "pressionado" assim que a diferença cair para esta fração
// do DELTA, sem precisar voltar 100% ao normal. Ex: 0.75 = libera quando a
// diferença já caiu 25% em relação ao pico (mais rápido que esperar zerar).
const float FATOR_RELEASE = 0.35;

// ================== BUFFER DE MÉDIA MÓVEL ==================
long leituras[TAMANHO_BUFFER];
int indiceBuffer = 0;
int totalLeituras = 0;
bool bufferCheio = false;

// ================== ESTADO DO CLIQUE / DEBOUNCE ==================
bool emEstadoPressionado = false;      // true enquanto |leitura-media| > DELTA
unsigned long ultimoCliqueMillis = 0;  // instante do último clique enviado
unsigned long inicioPressaoMillis = 0; // instante em que entrou no estado pressionado

// ================== BLE HID (Bluedroid) ==================
BLEHIDDevice* hid;
BLECharacteristic* inputMouse;
BLEServer* pServer;
bool conectado = false;

// Report Map padrão de mouse (botões + X/Y/Wheel relativos) - igual ao HeadMouse
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
  0x81, 0x03,       //     INPUT (Cnst,Var,Abs)  -- padding
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

void setupBLE() {
  BLEDevice::init("HeadMouse-Pressao");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MinhasServerCallbacks());

  hid = new BLEHIDDevice(pServer);
  inputMouse = hid->inputReport(1); // Report ID 1 (mouse)

  hid->manufacturer()->setValue("Francisco");
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

  Serial.println("BLE HID Mouse pronto. Aguardando conexão...");
}

// Envia um report de mouse (botões, deslocamento X/Y, wheel)
void enviarReport(uint8_t botoes, int8_t x, int8_t y) {
  if (!conectado) return;
  uint8_t report[4] = { botoes, (uint8_t)x, (uint8_t)y, 0 };
  inputMouse->setValue(report, 4);
  inputMouse->notify();
}

// Envia um clique completo do botão esquerdo (press + release)
void enviarClique() {
  if (!conectado) {
    Serial.println("Clique detectado, mas nenhum dispositivo BLE conectado.");
    return;
  }
  enviarReport(0x01, 0, 0); // botão esquerdo pressionado
  delay(20);                // tempo mínimo para o host registrar o botão pressionado
  enviarReport(0x00, 0, 0); // botão solto
  Serial.println(">>> CLIQUE ENVIADO <<<");
}

void setup() {
  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- HX710B + ESP32-C3 Super Mini + BLE Mouse (Bluedroid) ---");

  // Desliga wifi/limpa recursos do wifi (economia de energia, igual ao HeadMouse)
  esp_wifi_stop();
  esp_wifi_deinit();
  setCpuFrequencyMhz(80);

  Serial.println("Aguardando preenchimento do buffer (20 leituras)...");
  Serial.println("Iniciando Bluetooth...");
  setupBLE();
}

// Função de leitura por hardware do HX710B
long IRAM_ATTR lerHX710B() {
  unsigned long timeout = millis();
  while (digitalRead(PIN_OUT) == HIGH) {
    if (millis() - timeout > 500) {
      return 0;
    }
    yield();
  }
  long resultado = 0;
  portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&myMutex);
  for (int i = 0; i < 24; i++) {
    digitalWrite(PIN_SCK, HIGH);
    esp_rom_delay_us(1);
    resultado = resultado << 1;
    digitalWrite(PIN_SCK, LOW);
    esp_rom_delay_us(1);
    if (digitalRead(PIN_OUT) == HIGH) {
      resultado++;
    }
  }
  digitalWrite(PIN_SCK, HIGH);
  esp_rom_delay_us(1);
  digitalWrite(PIN_SCK, LOW);
  esp_rom_delay_us(1);
  portEXIT_CRITICAL(&myMutex);
  if (resultado & 0x800000) {
    resultado |= 0xFF000000;
  }
  return resultado;
}

void loop() {
  long ultimaLeitura = lerHX710B();

  if (ultimaLeitura != 0) {
    leituras[indiceBuffer] = ultimaLeitura;
    indiceBuffer = (indiceBuffer + 1) % TAMANHO_BUFFER;

    if (!bufferCheio) {
      totalLeituras++;
      Serial.print("Carregando Buffer: ");
      Serial.print(totalLeituras);
      Serial.println("/20");

      if (totalLeituras >= TAMANHO_BUFFER) {
        bufferCheio = true;
        Serial.println("--> Buffer cheio! Iniciando monitoramento de média.");
      }
    }

    if (bufferCheio) {
      long soma = 0;
      for (int i = 0; i < TAMANHO_BUFFER; i++) {
        soma += leituras[i];
      }
      long media = soma / TAMANHO_BUFFER;

      Serial.print("Leitura Atual: ");
      Serial.print(ultimaLeitura);
      Serial.print(" | Média (Últimas 20): ");
      Serial.print(media);

      long diferenca = abs(ultimaLeitura - media);
      bool acimaDoLimiar = (diferenca > DELTA);
      Serial.print(" | Diferença: ");
      Serial.println(diferenca);

      // ---- Lógica de borda + debounce + timeout de destrave ----
      // Dispara o clique na TRANSIÇÃO de repouso -> pressionado,
      // respeitando o DEBOUNCE_MS entre cliques.
      unsigned long agora = millis();

      if (acimaDoLimiar && !emEstadoPressionado) {
        if (agora - ultimoCliqueMillis > DEBOUNCE_MS) {
          Serial.println("Alerta: Botao pressionado");
          enviarClique();
          ultimoCliqueMillis = agora;
        }
        emEstadoPressionado = true;
        inicioPressaoMillis = agora; // marca início da trava
      } else if (emEstadoPressionado) {
        // Libera antes de a diferença voltar totalmente a zero: basta cair
        // para FATOR_RELEASE do DELTA (recuperação parcial).
        bool voltouAoNormal = (diferenca < (DELTA * FATOR_RELEASE));
        bool estourouTimeout = (agora - inicioPressaoMillis > TIMEOUT_PRESSAO_MS);

        // Rearma tanto se o sinal recuperou o suficiente quanto se o timeout
        // máximo de trava foi atingido (mesmo com o sinal ainda alto).
        if (voltouAoNormal || estourouTimeout) {
          emEstadoPressionado = false;
        }
      }
    }
  }

  // O próprio lerHX710B() já paceia o loop de acordo com a taxa de saída
  // do sensor (DRDY). Um delay pequeno aqui só evita busy-loop excessivo.
  delay(5);
}
