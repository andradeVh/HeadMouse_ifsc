#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#include <BleMouse.h>
#include <esp_pm.h>
#include <esp_wifi.h>

MPU6050 mpu;
BleMouse bleMouse("BLE_mouse","DIY",100);

int16_t ax, ay, az, gx, gy, gz;
int vx, vy;

void setup() {
  // Inicia Serial primeiro para debug
  Serial.begin(115200);
  delay(1000); 
  
  //desliga wifi/limpa recursos do wifi
  esp_wifi_stop();
  esp_wifi_deinit();
  // Inicia I2C explicitamente nos pinos padrão
  Wire.begin(21, 22);
  //Wire.setClock(400000); // Aumenta velocidade do I2C para 400kHz
  // Configura a gestão de energia
  esp_pm_config_esp32_t pm_config;
  pm_config.max_freq_mhz = 240; // Frequência máxima
  pm_config.min_freq_mhz = 80;  // Frequência mínima (pode baixar para economizar)
  //pm_config.light_sleep_enable = true; // Habilita o sono leve automático

  esp_pm_configure(&pm_config);
  // Inicializa o MPU6050 ANTES do Bluetooth
  Serial.println("Iniciando MPU6050");
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050: Conectado!");
  } else {
    Serial.println("MPU6050: Erro de conexao!");
    
  }

  // Aplica os Offsets
  setCalibration();



  // Inicia o Bluetooth por último (consome muita memória/energia)
  Serial.println("Iniciando Bluetooth...");
  bleMouse.begin();
}

void loop() {
  if (bleMouse.isConnected()) {
    handle();
  }
  delay(10); 
}

void setCalibration() {
  mpu.CalibrateGyro(7);
//mpu.setXAccelOffset(-1738);
//  mpu.setYAccelOffset(-796);
//  mpu.setZAccelOffset(617);
//  mpu.setXGyroOffset(32);
//  mpu.setYGyroOffset(-36);
//  mpu.setZGyroOffset(-5);
}

void handle() {
  mpu.getRotation(&gx, &gy, &gz); // Usamos apenas rotação para o mouse

  // Sensibilidade
  vx = -gy / 100;
  vy = -gz /100; 

  // Zona morta para ignorar pequenos ruídos
  if (abs(vx) < 2) vx = 0;
  if (abs(vy) < 2) vy = 0;

  if (vx != 0 || vy != 0) {
    bleMouse.move(vx, vy);
  }
}
