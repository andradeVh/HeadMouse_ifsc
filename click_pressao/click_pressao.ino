#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

// ================== PINOS (ESP32-C3 Super Mini) ==================
const int PIN_OUT = 2; // GPIO 2 (Entrada de dados - DOUT do HX710B)
const int PIN_SCK = 3; // GPIO 3 (Saída de clock - SCK do HX710B)

// ================== PARÂMETROS DE DETECÇÃO ==================
const long DELTA = 20000;          // Limiar de variação em relação à média
const int TAMANHO_BUFFER = 20;     // Tamanho da janela de média móvel
const unsigned long DEBOUNCE_MS = 100; // Tempo mínimo entre dois cliques

// ================== BUFFER DE MÉDIA MÓVEL ==================
long leituras[TAMANHO_BUFFER];
int indiceBuffer = 0;
int totalLeituras = 0;
bool bufferCheio = false;

// ================== ESTADO DO CLIQUE / DEBOUNCE ==================
bool emEstadoPressionado = false;      // true enquanto |leitura-media| > DELTA
unsigned long ultimoCliqueMillis = 0;  // instante do último clique enviado

// ================== BLE HID (NimBLE) ==================
NimBLEHIDDevice* hid;
NimBLECharacteristic* inputMouse;
bool deviceConnected = false;

// Report Map padrão de mouse (botões + X/Y/Wheel relativos)
static const uint8_t reportMap[] = {
  0x05, 0x01,       // USAGE_PAGE (Generic Desktop)
  0x09, 0x02,       // USAGE (Mouse)
  0xa1, 0x01,       // COLLECTION (Application)
  0x09, 0x01,       //   USAGE (Pointer)
  0xa1, 0x00,       //   COLLECTION (Physical)
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
  0x81, 0x03,       //     INPUT (Cnst,Var,Abs)
  0x05, 0x01,       //     USAGE_PAGE (Generic Desktop)
  0x09, 0x30,       //     USAGE (X)
  0x09, 0x31,       //     USAGE (Y)
  0x09, 0x38,       //     USAGE (Wheel)
  0x15, 0x81,       //     LOGICAL_MINIMUM (-127)
  0x25, 0x7f,       //     LOGICAL_MAXIMUM (127)
  0x75, 0x08,       //     REPORT_SIZE (8)
  0x95, 0x03,       //     REPORT_COUNT (3)
  0x81, 0x06,       //     INPUT (Data,Var,Rel)
  0xc0,             //   END_COLLECTION
  0xc0              // END_COLLECTION
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.println("BLE conectado.");
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.println("BLE desconectado. Reiniciando advertising...");
    NimBLEDevice::startAdvertising();
  }
};

void setupBLE() {
  NimBLEDevice::init("HeadMouse-Pressao");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  hid = new NimBLEHIDDevice(server);
  inputMouse = hid->getInputReport(1); // Report ID 1 (mouse)

  hid->setManufacturer("DIY");
  hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->setHidInfo(0x00, 0x01);
  hid->setReportMap((uint8_t*)reportMap, sizeof(reportMap));
  hid->startServices();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->setAppearance(HID_MOUSE);
  advertising->addServiceUUID(hid->getHidService()->getUUID());
  advertising->enableScanResponse(true);
  advertising->start();

  hid->setBatteryLevel(100);

  Serial.println("BLE HID Mouse pronto. Aguardando conexão...");
}

// Envia um clique completo do botão esquerdo (press + release)
void enviarClique() {
  if (!deviceConnected) {
    Serial.println("Clique detectado, mas nenhum dispositivo BLE conectado.");
    return;
  }

  uint8_t report[4] = {0x01, 0x00, 0x00, 0x00}; // botão esquerdo pressionado
  inputMouse->setValue(report, sizeof(report));
  inputMouse->notify();

  delay(20); // tempo mínimo para o host registrar o botão pressionado

  uint8_t reportSolto[4] = {0x00, 0x00, 0x00, 0x00}; // botão solto
  inputMouse->setValue(reportSolto, sizeof(reportSolto));
  inputMouse->notify();

  Serial.println(">>> CLIQUE ENVIADO <<<");
}

void setup() {
  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- HX710B + ESP32-C3 Super Mini + BLE Mouse (NimBLE) ---");
  Serial.println("Aguardando preenchimento do buffer (20 leituras)...");

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

      // ---- Lógica de borda + debounce ----
      // Só dispara o clique na TRANSIÇÃO de repouso -> pressionado,
      // e apenas se já tiver passado DEBOUNCE_MS desde o último clique.
      if (acimaDoLimiar && !emEstadoPressionado) {
        unsigned long agora = millis();
        if (agora - ultimoCliqueMillis > DEBOUNCE_MS) {
          Serial.println("Alerta: Botao pressionado");
          enviarClique();
          ultimoCliqueMillis = agora;
        }
        emEstadoPressionado = true; // trava até o sinal voltar ao normal
      } else if (!acimaDoLimiar && emEstadoPressionado) {
        // Sinal voltou abaixo do limiar: rearma para o próximo evento
        emEstadoPressionado = false;
      }
    }
  }

  delay(5); 
}
