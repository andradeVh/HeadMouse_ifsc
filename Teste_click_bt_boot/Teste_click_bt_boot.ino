#include <BleMouse.h>

BleMouse bleMouse("Mouse do Francisco");
const int pinoBotao = 0; // Botão BOOT da placa

void setup() {
  Serial.begin(115200);
  pinMode(pinoBotao, INPUT_PULLUP); // Ativa o resistor interno
  bleMouse.begin();
}

void loop() {
  if (bleMouse.isConnected()) {
    // Se o botão for pressionado (valor 0)
    if (digitalRead(pinoBotao) == LOW) {
      Serial.println("Botão pressionado: Clicando!");
      bleMouse.click(MOUSE_LEFT);
      delay(50); // Debounce: evita múltiplos cliques por um único toque
    }
  }
}
