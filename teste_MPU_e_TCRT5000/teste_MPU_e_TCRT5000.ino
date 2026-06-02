#include <BleMouse.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_pm.h>
#include "driver/rtc_io.h"

// Configurações de Pinos
const int PIN_BT = 14;        // Botão físico
const int sensorPin = 4;      // Sensor IR TCRT5000 (GPIO4)
const int pino_sample = 15;   // Controle do LED IR do sensor

// Configurações do Mouse e Sono
BleMouse bleMouse("ESP32 Blink Mouse", "Espressif", 100);
const unsigned long TEMP_MAX_INATIVO = 60000;
unsigned long tempoUltimaAtividade = 0;

// Variáveis do Sensor IR
int sensorValueOn = 0;
int sensorValueOff = 0;
float g_mediaJanelaAtual = 0;
float g_valorBaseLine = 0.0;
unsigned long tempoInicioPiscada = 0;

// Filtros e Médias
#define NUM_LEITURAS_JANELA 50
#define DELTA_BASELINE_PADRAO 20 // Sensibilidade
unsigned int vetorLeituras[NUM_LEITURAS_JANELA];
int indexLeituras = 0;
bool emPiscada = false;

unsigned long ultimoSampleMicros = 0;
const int INTERVALO_FASE_MICROS = 200; 
int fase = 1;

void startDeepSleep();

void setup() {
  Serial.begin(115200);

  // Limpeza de recursos para economia de energia
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  esp_wifi_stop();
  esp_wifi_deinit();

  if(rtc_gpio_is_valid_gpio((gpio_num_t)PIN_BT)){
    rtc_gpio_deinit((gpio_num_t)PIN_BT); 
  }

  pinMode(PIN_BT, INPUT_PULLUP);
  pinMode(pino_sample, OUTPUT);
  digitalWrite(pino_sample, LOW);

  // Configuração de energia
  esp_pm_config_esp32_t pm_config;
  pm_config.max_freq_mhz = 240;
  pm_config.min_freq_mhz = 80;
  pm_config.light_sleep_enable = true;
  esp_pm_configure(&pm_config);

  bleMouse.begin();
  tempoUltimaAtividade = millis();
}

void loop() {
  // 1. Lógica do Sensor IR (Amostragem Diferencial)
  unsigned long atualMicros = micros();
  if (atualMicros - ultimoSampleMicros >= INTERVALO_FASE_MICROS) {
    ultimoSampleMicros = atualMicros;
    
    switch (fase) {
      case 1:
        digitalWrite(pino_sample, HIGH); // Liga IR
        break;
      case 2:
        sensorValueOn = analogRead(sensorPin); // Lê reflexão
        digitalWrite(pino_sample, LOW); // Desliga IR
        break;
      case 3:
        sensorValueOff = analogRead(sensorPin); // Lê luz ambiente
        processarLeituraIR();
        break;
    }
    fase = (fase >= 5) ? 1 : fase + 1;
  }

  // 2. Lógica do Botão Físico
  if(digitalRead(PIN_BT) == LOW){
    if(bleMouse.isConnected()) {
      bleMouse.click(MOUSE_LEFT);
      Serial.println("Botão: Click");
    }
    tempoUltimaAtividade = millis();
    while(digitalRead(PIN_BT) == LOW) { delay(10); }
  }

  // 3. Verificação de Inatividade para Deep Sleep
  if(millis() - tempoUltimaAtividade > TEMP_MAX_INATIVO){
    Serial.println("Inativo. Entrando em Deep Sleep...");
    delay(100);
    startDeepSleep();
  }

  delay(1); 

void processarLeituraIR() {
  int sensorValue = abs(sensorValueOff - sensorValueOn);
  
  // Média Móvel
  vetorLeituras[indexLeituras] = sensorValue;
  indexLeituras = (indexLeituras + 1) % NUM_LEITURAS_JANELA;
  
  unsigned long soma = 0;
  for (int i = 0; i < NUM_LEITURAS_JANELA; i++) soma += vetorLeituras[i];
  g_mediaJanelaAtual = (float)soma / NUM_LEITURAS_JANELA;

  // Lógica de Detecção de Piscada
  if (!emPiscada) {
    g_valorBaseLine = g_mediaJanelaAtual;
    if (g_mediaJanelaAtual > (g_valorBaseLine + DELTA_BASELINE_PADRAO)) {
      if (bleMouse.isConnected()) {
        bleMouse.press(MOUSE_LEFT);
        Serial.println("Piscada: Press");
      }
      emPiscada = true;
      tempoInicioPiscada = millis();
      tempoUltimaAtividade = millis();
    }
  } else {
    // Solta o clique se o valor baixar ou por timeout de 1 segundo
    if (g_mediaJanelaAtual < (g_valorBaseLine + (DELTA_BASELINE_PADRAO * 0.5)) || (millis() - tempoInicioPiscada > 1000)) {
      if (bleMouse.isConnected()) {
        bleMouse.release(MOUSE_LEFT);
        Serial.println("Piscada: Release");
      }
      emPiscada = false;
      tempoUltimaAtividade = millis();
    }
  }
}

void startDeepSleep(){
  digitalWrite(pino_sample, LOW); // Garante sensor desligado
  rtc_gpio_init((gpio_num_t)PIN_BT);
  rtc_gpio_set_direction((gpio_num_t)PIN_BT, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en((gpio_num_t)PIN_BT);
  
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BT, 0); // Acorda pelo botão no LOW
  esp_deep_sleep_start();
}
