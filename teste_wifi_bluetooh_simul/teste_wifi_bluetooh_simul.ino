#include <WiFi.h>
#include <WebServer.h>
#include <BleMouse.h>
#include <esp_bt.h>
#include <esp_wifi.h>

// Configurações do Wi-Fi
const char* ssid = "ESP32_Controle";
const char* password = "12345678password";

WebServer server(80);
BleMouse bleMouse("ESP32 Mouse Remoto");

// Página HTML otimizada
String getHTML() {
  return "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
         "<style>body{text-align:center;font-family:sans-serif;} button { width: 90%; height: 200px; font-size: 40px; background-color: #4CAF50; color: white; border: none; border-radius: 20px; touch-action: manipulation; }"
         "</style></head><body>"
         "<h1>Controle Remoto</h1>"
         "<button onmousedown='fetch(\"/click\")'>CLICAR NO PC</button>"
         "<p>Status: <span id='st'>Verificando...</span></p>"
         "<script>setInterval(()=>{fetch('/status').then(r=>r.text()).then(t=>document.getElementById('st').innerText=t)},2000);</script>"
         "</body></html>";
}

void setup() {
  Serial.begin(115200);
  
  // --- AJUSTES DE COEXISTÊNCIA (Telecom) ---
  // Libera memória do BT clássico para focar no BLE (Mouse)
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  
  // Inicia o Mouse Bluetooth
  bleMouse.begin();

  // Inicia o Wi-Fi como Access Point
  WiFi.softAP(ssid, password);
  
  // Reduz agressividade do Wi-Fi para dar janelas de tempo ao Bluetooth
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM); 

  Serial.println("Sistema Iniciado!");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // Rotas
  server.on("/", []() {
    server.send(200, "text/html", getHTML());
  });

  server.on("/status", []() {
    server.send(200, "text/plain", bleMouse.isConnected() ? "Bluetooth Conectado" : "Aguardando Bluetooth...");
  });

  server.on("/click", []() {
    if(bleMouse.isConnected()) {
      bleMouse.click(MOUSE_LEFT);
      server.send(204); // Resposta ultra-rápida (No Content)
    } else {
      server.send(400, "text/plain", "Desconectado");
    }
  });

  server.begin();
}

void loop() {
  server.handleClient();
  
  // Pequena pausa para o processador respirar e gerenciar os rádios
  delay(2); 
}
