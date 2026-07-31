#include <Arduino.h>

// Definição dos pinos para o ESP32-C3 Super Mini
const int PIN_OUT = 2; // GPIO 2 (Entrada de dados)
const int PIN_SCK = 3; // GPIO 3 (Saída de clock)
const int DELTA = 20000;
// Configurações do Buffer de Média
const int TAMANHO_BUFFER = 20;
long leituras[TAMANHO_BUFFER]; // Vetor para armazenar as últimas 20 leituras
int indiceBuffer = 0;
int totalLeituras = 0;         // Contador para verificar se o buffer encheu
bool bufferCheio = false;

void setup() {
  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW);
  
  Serial.begin(115200); 
  delay(1000);
  Serial.println("\n--- HX710B + ESP32-C3 Super Mini Inicializado ---");
  Serial.println("Aguardando preenchimento do buffer (20 leituras)...");
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

  // Se a leitura falhar devido ao timeout, ignora o ciclo atual
  if (ultimaLeitura != 0) {
    
    // Adiciona a nova leitura no vetor (substituindo a mais antiga)
    leituras[indiceBuffer] = ultimaLeitura;
    indiceBuffer = (indiceBuffer + 1) % TAMANHO_BUFFER;

    // Controla o estado de inicialização do buffer
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

    // Só processa a média e os alertas após o buffer estar completamente cheio
    if (bufferCheio) {
      // Calcula a soma de todos os elementos do vetor
      long soma = 0;
      for (int i = 0; i < TAMANHO_BUFFER; i++) {
        soma += leituras[i];
      }
      
      // Calcula a média aritmética
      long media = soma / TAMANHO_BUFFER;

      // Exibe os dados atuais no terminal de forma organizada
      Serial.print("Leitura Atual: ");
      Serial.print(ultimaLeitura);
      Serial.print(" | Média (Últimas 20): ");
      Serial.println(media);

      // Condição solicitada: se a última leitura cair abaixo da média móvel
      if (abs(ultimaLeitura-media)> DELTA) {
        Serial.println("⚠️ Alerta:Botao pressionado");
      }
    }
  }
  
  delay(250); // Intervalo de 250ms entre medições
}
