// Define o pino do LED azul do ESP32-C3 SuperMini
const int LED_PIN = 8; 

void setup() {
  // Configura o pino como saída
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Lógica invertida: LOW acende o LED
  digitalWrite(LED_PIN, LOW);   
  delay(1000);                  
  
  // HIGH apaga o LED
  digitalWrite(LED_PIN, HIGH);  
  delay(1000);                  
}
